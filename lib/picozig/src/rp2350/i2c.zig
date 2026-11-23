//! I2C master (Synopsys DW_apb_i2c).
//!
//! **Status: proven against a real device, 2026-07-28** — an e-paper HAT's ID
//! EEPROM. Two bugs surfaced within the hour: a
//! write that reported
//! success before the bus had moved, and a target change that tore down a
//! transfer still in flight. Neither was reachable by host tests.
//!
//! ## The awkward parts of this IP, up front
//!
//! It is not a simple shift register with a start bit. Three things surprise
//! people, all of them silent:
//!
//!   1. **Almost every register is read-only while the block is enabled.**
//!      Changing the target address or timing without clearing `IC_ENABLE`
//!      first appears to work and does nothing.
//!   2. **Start and stop are per-byte flags in `IC_DATA_CMD`**, not separate
//!      operations. A transfer is expressed by tagging its last byte with STOP
//!      and, for a repeated start, tagging the first byte of the next phase
//!      with RESTART.
//!   3. **Reads are two-sided**: writing a CMD=read byte queues a read, and the
//!      data appears later in the RX FIFO. Queueing and draining are separate
//!      loops, and conflating them deadlocks once the FIFO depth is exceeded.

const std = @import("std");
const hw = @import("hw.zig");
const gpio = @import("gpio.zig");
const timer = @import("timer.zig");

/// GPIO function 3 is I2C.
const func_i2c: u32 = 3;

pub const Instance = enum(u1) { i2c0 = 0, i2c1 = 1 };

pub fn base(inst: Instance) u32 {
    return switch (inst) {
        .i2c0 => hw.i2c0_base,
        .i2c1 => hw.i2c1_base,
    };
}

const Reg = struct {
    const con = 0x00;
    const tar = 0x04;
    const data_cmd = 0x10;
    const ss_scl_hcnt = 0x14;
    const ss_scl_lcnt = 0x18;
    const fs_scl_hcnt = 0x1c;
    const fs_scl_lcnt = 0x20;
    const raw_intr_stat = 0x34;
    const rx_tl = 0x38;
    const tx_tl = 0x3c;
    const clr_intr = 0x40;
    const clr_tx_abrt = 0x54;
    const enable = 0x6c;
    const status = 0x70;
    const txflr = 0x74;
    const rxflr = 0x78;
    const sda_hold = 0x7c;
    const tx_abrt_source = 0x80;
    const dma_cr = 0x88;
    const enable_status = 0x9c;
    const fs_spklen = 0xa0;
};

// IC_CON
const con_master_mode: u32 = 1 << 0;
const con_speed_fast: u32 = 2 << 1;
const con_restart_en: u32 = 1 << 5;
const con_slave_disable: u32 = 1 << 6;
const con_tx_empty_ctrl: u32 = 1 << 8;

// IC_DATA_CMD
const cmd_read: u32 = 1 << 8;
const cmd_stop: u32 = 1 << 9;
const cmd_restart: u32 = 1 << 10;

// IC_STATUS
const status_mst_activity: u32 = 1 << 5;

// IC_RAW_INTR_STAT
/// **"The byte has left the shift register", not "there is room in the FIFO".**
///
/// Only because `init` sets `IC_CON.TX_EMPTY_CTRL` and `IC_TX_TL` to 0. The
/// datasheet spells out the difference: with `TX_EMPTY_CTRL` = 0 this is a FIFO
/// level, and with it 1 it is the level *and* "the transmission of the
/// address/data from the internal shift register for the most recently popped
/// command is completed". The second is the only one worth waiting for, and it
/// is what the pico-sdk waits for after every byte it pushes.
const intr_tx_empty: u32 = 1 << 4;
const intr_tx_abrt: u32 = 1 << 6;
/// **Whether the next transfer must open with a repeated START.**
///
/// The pico-sdk carries exactly this state (`i2c->restart_on_next`), set to the
/// `nostop` of the transfer that just finished and consumed by the first
/// command of the next one. We did without it for as long as `setTarget`
/// cached `IC_TAR`: a combined register read keeps the same address, the cache
/// returned early, the block was never disabled, and `IC_CON.RESTART_EN` made
/// the controller emit the restart by itself when the direction changed.
///
/// **Removing that cache on 2026-08-26 removed the thing holding it together.**
/// `setTarget` now takes `IC_ENABLE` low on every transfer, as the SDK does,
/// and that returns the state machine to idle — so a held bus does not survive
/// into the read, and "the controller emits one automatically" stops being
/// true. The SDK is safe doing the same disable precisely *because* it carries
/// this flag and sets the RESTART bit explicitly.
var restart_on_next: [2]bool = .{ false, false };

/// The RESTART/STOP flags for command `i` of a `len`-byte transfer.
///
/// **Factored out to be testable.** `read` and `write` differ only in what they
/// OR these onto — `cmd_read` and the data byte respectively — and the
/// condition itself is the part that is easy to get wrong and impossible to
/// exercise from a host test while it lives inside a hardware poll loop.
/// `write`'s `stop` parameter sat unexercised in the signature for months and
/// was wrong when the first caller finally passed `false`; that is the failure
/// mode this shape is meant to prevent.
inline fn cmdFlags(i: usize, len: usize, restart: bool, stop: bool) u32 {
    var f: u32 = 0;
    if (i == 0 and restart) f |= cmd_restart;
    if (i + 1 == len and stop) f |= cmd_stop;
    return f;
}

const dma_cr_rdmae: u32 = 1 << 0;
const dma_cr_tdmae: u32 = 1 << 1;
const intr_stop_det: u32 = 1 << 9;

pub const Speed = enum { standard_100k, fast_400k };

pub const Config = struct {
    sda: gpio.Pin,
    scl: gpio.Pin,
    /// clk_sys, which paces SCL.
    clk_hz: u32,
    speed: Speed = .standard_100k,
};

pub const Error = error{
    /// The device did not acknowledge — usually wrong address or absent device.
    NoAcknowledge,
    /// The bus did not make progress in a reasonable time.
    Timeout,
};

/// SCL high/low counts for a target bit rate.
///
/// Split roughly 40/60 high/low, which respects the standard's requirement that
/// the low period exceed the high one. The `+ 1` terms match the IP's internal
/// offsets — the hardware adds cycles of its own, and omitting them yields a bus
/// that runs measurably fast, which usually still works and is exactly the kind
/// of marginal setup that fails only against one fussy device.
/// **No longer used by `init`, and kept only as a record.**
///
/// This computed SS-mode counters for standard speed. As of 2026-08-26 `init`
/// always configures the block in **fast** mode with the FS counters, whatever
/// baudrate is asked for, because that is what the pico-sdk does
/// unconditionally and because the standard-mode configuration did not work:
/// a transfer to an absent address never started at all — no `START_DET`, no
/// `TX_ABRT`, the command left sitting in the TX FIFO.
///
/// The arithmetic differs from the SDK's in more than the mode: this splits the
/// period 40/60 and then subtracts 7 and 1, where the SDK splits 3:5 and writes
/// the counts directly. Kept so the two can be compared if the question ever
/// comes back, and because its tests pin real facts about the SS registers.
pub const Timing = struct {
    hcnt: u16,
    lcnt: u16,

    pub fn compute(clk_hz: u32, target_hz: u32) Timing {
        const period = clk_hz / target_hz;
        const hi = (period * 2) / 5; // 40%
        const lo = period - hi; // 60%
        return .{
            .hcnt = @intCast(@max(hi, 8) - 7),
            .lcnt = @intCast(@max(lo, 8) - 1),
        };
    }

    pub fn actualHz(self: Timing, clk_hz: u32) u32 {
        return clk_hz / (@as(u32, self.hcnt) + 7 + @as(u32, self.lcnt) + 1);
    }
};

/// **Free a bus that a target is holding, before `init`.** Returns whether SDA
/// is released afterwards.
///
/// A target that was mid-read when *this* chip reset is still clocking out a
/// byte, and holds SDA low waiting for the SCL edges that will never come.
/// Nothing on the controller side can reset it — resetting the I²C block, or
/// the whole RP2350, leaves a separately powered target exactly where it was —
/// so every transaction afterwards times out until the target loses power. It
/// looks like a dead device, and a power cycle "fixes" it.
///
/// The standard cure: clock SCL until the target has shifted out its byte and
/// releases SDA (at most nine edges), then issue a STOP so it returns to idle.
/// Open-drain throughout — a line is pulled low by driving it, and released by
/// making it an input with its pull-up — so nothing here can fight a target
/// that is driving.
///
/// **Seen on a signal generator**, where a PCM5122 stopped answering and stayed that
/// way across resets until a boot ran this. The cause was not reproduced — a
/// hundred resets at random points in continuous polling left the bus idle
/// every time — so do not read that case as proof of the mid-read mechanism
/// above, which is the textbook one rather than the observed one.
/// What `recoverBus` found. **Reported, not just acted on**: a boot that says
/// "bus free" cannot tell a bus that was never held from one this freed, and
/// only the second is evidence the recovery is doing anything.
pub const Recovery = struct {
    /// SDA was low on entry — a target was holding the bus.
    held: bool,
    /// SCL edges it took to release it.
    pulses: u8,
    /// SDA is high afterwards.
    free: bool,
};

pub fn recoverBus(sda: gpio.Pin, scl: gpio.Pin) Recovery {
    // 100 kHz is 5000 us a period, so 5 us per half-period is ten times
    // slower than standard speed, well inside what any target accepts.
    const half_us = 5;
    gpio.initInput(sda, .up);
    gpio.initInput(scl, .up);
    timer.delayUs(half_us);

    const held = !gpio.get(sda);
    var pulses: u8 = 0;
    while (!gpio.get(sda) and pulses < 16) : (pulses += 1) {
        gpio.initOutput(scl); // low
        timer.delayUs(half_us);
        gpio.initInput(scl, .up); // released, high
        timer.delayUs(half_us);
    }

    // STOP: SDA rises while SCL is high. Taken whether or not SDA was stuck,
    // because a target that had merely latched a start is also left waiting.
    gpio.initOutput(scl);
    timer.delayUs(half_us);
    gpio.initOutput(sda);
    timer.delayUs(half_us);
    gpio.initInput(scl, .up);
    timer.delayUs(half_us);
    gpio.initInput(sda, .up);
    timer.delayUs(half_us);
    return .{ .held = held, .pulses = pulses, .free = gpio.get(sda) };
}

pub fn init(inst: Instance, cfg: Config) void {
    // **Reset the block, do not merely unreset it.** `i2c_init` opens with
    // `i2c_reset(i2c); i2c_unreset(i2c);` — an unreset alone is a no-op on a
    // block that is already out of reset, so re-initialising inherits every
    // scrap of state the previous run left in the FIFOs and the state machine.
    hw.Resets.resetCycle(switch (inst) {
        .i2c0 => hw.Resets.i2c0,
        .i2c1 => hw.Resets.i2c1,
    });
    const b = base(inst);

    // Disabled while configuring — see the header; most of these registers are
    // read-only otherwise, and writes are silently dropped.
    hw.write(b + Reg.enable, 0);

    // **`SPEED` is always FAST, and the FS counters always carry the timing —
    // even at 100 kHz.** That is not an optimisation, it is what the pico-sdk
    // does unconditionally (`i2c_set_baudrate` writes
    // `I2C_IC_CON_SPEED_VALUE_FAST` for every baudrate) and it is the
    // difference that mattered.
    //
    // **Found 2026-08-26 by diffing the two configurations register by
    // register on the same board.** With `SPEED` = standard and the SS
    // counters, a transfer to an *absent* address does nothing at all: the
    // command sits in the TX FIFO, `IC_TXFLR` stays 1, `MST_ACTIVITY` never
    // asserts and **`IC_RAW_INTR_STAT` reads 0 for the whole poll loop** — no
    // `TX_EMPTY`, no `START_DET`, and crucially no `TX_ABRT`. So the NACK that
    // should end the transfer in microseconds never arrives, every absent
    // address burns its full timeout, and the abandoned transfer wedges every
    // device on the bus until power is removed.
    //
    // Transfers to a *present* address worked, which is why every driver in
    // this workspace was fine and only a bus scan ever showed it.
    const con: u32 = con_master_mode | con_slave_disable | con_restart_en |
        con_tx_empty_ctrl | con_speed_fast;
    hw.write(b + Reg.con, con);

    const target: u32 = switch (cfg.speed) {
        .standard_100k => 100_000,
        .fast_400k => 400_000,
    };

    // The SDK's arithmetic, kept in its shape so the two can be compared:
    // a 3:2 low-to-high split of the period, and a spike-suppression length
    // taken from the low count.
    const period = (cfg.clk_hz + target / 2) / target;
    const lcnt = period * 3 / 5;
    const hcnt = period - lcnt;
    hw.write(b + Reg.fs_scl_hcnt, hcnt);
    hw.write(b + Reg.fs_scl_lcnt, lcnt);
    hw.write(b + Reg.fs_spklen, if (lcnt < 16) 1 else lcnt / 16);

    // **`IC_SDA_HOLD` was never written at all**, so it sat at its reset value
    // of 1. The SDK computes a transmit hold from clk_sys; leaving it at 1 is
    // the other half of a configuration that the block would not run.
    const sda_tx_hold: u32 = if (target < 1_000_000)
        ((cfg.clk_hz * 3) / 10_000_000) + 1
    else
        ((cfg.clk_hz * 3) / 25_000_000) + 1;
    hw.write(b + Reg.sda_hold, sda_tx_hold);
    hw.write(b + Reg.tx_tl, 0);
    hw.write(b + Reg.rx_tl, 0);
    // "Always enable the DREQ signalling -- harmless if DMA isn't listening",
    // in the SDK's words. Kept so the two configurations are register-for-
    // register identical and a future diff has nothing to explain away.
    hw.write(b + Reg.dma_cr, dma_cr_tdmae | dma_cr_rdmae);

    // Pins: I2C is open-drain and needs pull-ups. The internal ones are weak
    // (~50 kOhm) and only adequate for a short bus at 100 kHz — fit external
    // 2.2-4.7 kOhm resistors for anything real. Pull-UP is also the direction
    // erratum E9 leaves working on A2 silicon (see silicon.zig).
    gpio.initFunctionInput(cfg.sda, func_i2c, .up);
    gpio.initFunctionInput(cfg.scl, func_i2c, .up);
    // Both lines must also be able to drive low.
    hw.clrBits(hw.PadsBank0.pad(cfg.sda), hw.PadsBank0.od);
    hw.clrBits(hw.PadsBank0.pad(cfg.scl), hw.PadsBank0.od);

    hw.write(b + Reg.enable, 1);
    restart_on_next[@backingInt(inst)] = false;
    // A fresh configuration invalidates whatever IC_TAR held.
}

/// Point the master at a device.
///
/// **Setting `IC_TAR` means disabling the block, and disabling it mid-transfer
/// destroys the transfer** — the FIFOs flush and the bus is dropped wherever it
/// had got to.
///
/// That is what broke an EEPROM on 2026-07-28, and the reason it was
/// intermittent rather than dead. `write` used to return as soon as the byte
/// was in the TX FIFO, so the address write was still being clocked out when
/// `read` called this and pulled `IC_ENABLE` low underneath it. Whether the
/// write survived came down to how long the intervening code took — it worked
/// on the very first run, when the code was being fetched from flash cold, and
/// never again once XIP had it cached. **An "it worked once" bug with no code
/// change between runs is a timing race until proven otherwise.**
///
/// Two fixes came out of that. `write` waiting for the bus (see `waitIdle`) was
/// the right one and still stands. The second — returning early here when the
/// address was unchanged, so a combined transaction never disabled the block —
/// **was the wrong one, and was removed on 2026-08-26.** It worked, but it made
/// correctness depend on a cache: every caller doing a repeated start was
/// silently relying on this function *not* running. The SDK disables the block
/// on every transfer and stays safe because it carries `restart_on_next` and
/// sets `IC_DATA_CMD.RESTART` explicitly. That is now what this module does.
fn setTarget(inst: Instance, addr: u7) void {
    const b = base(inst);
    // **Unconditional, even when the address has not changed.** The SDK opens
    // every single transfer with this sequence and never caches it. Skipping
    // it when the target is unchanged looks like a free optimisation and is
    // not: taking `IC_ENABLE` low **flushes both FIFOs and returns the state
    // machine to idle**, so the SDK begins each transfer on a clean block.
    // Cached, a run of transfers to one device — which is every register read
    // a driver does — inherits whatever the last one left behind.
    hw.write(b + Reg.enable, 0);
    hw.write(b + Reg.tar, addr);
    hw.write(b + Reg.enable, 1);
}

/// Why the last transfer aborted, latched for a caller that wants to report it.
///
/// **`IC_CLR_TX_ABRT` clears `IC_TX_ABRT_SOURCE` as a side effect of being
/// read**, so the reason has to be captured before the flag is cleared or it is
/// gone. The first version of `checkAbort` cleared without reading, which threw
/// away the one register that says what went wrong — every failure arrived as
/// `NoAcknowledge` whether it was a missing device, arbitration loss or a
/// 10-bit addressing mistake.
pub var last_abort_source: u32 = 0;

/// Bit 0 of `IC_TX_ABRT_SOURCE`: nobody acknowledged the address.
pub const abrt_7b_addr_noack: u32 = 1 << 0;
/// Bit 3: the address was acknowledged but a data byte was not.
pub const abrt_txdata_noack: u32 = 1 << 3;

fn checkAbort(inst: Instance) Error!void {
    const b = base(inst);
    if (hw.read(b + Reg.raw_intr_stat) & intr_tx_abrt != 0) {
        last_abort_source = hw.read(b + Reg.tx_abrt_source);
        _ = hw.read(b + Reg.clr_tx_abrt);
        return error.NoAcknowledge;
    }
}

/// Wait until the master has finished what it was told to do.
///
/// **Without this a write reports success before the bus has moved.** Pushing a
/// byte into the TX FIFO takes nanoseconds; clocking it out at 100 kHz takes
/// about 90 µs, and the NAK that says nobody is listening cannot possibly have
/// happened yet. `write` used to check for an abort immediately after the FIFO
/// push and return — so it returned `void` for a device that was not there, and
/// `probe` built on it answered "yes" for every address on the bus.
///
/// Found on 2026-07-28, when an EEPROM that had read correctly the day before
/// reported `device at 0x50` and then returned 29 bytes of 0xFF. The probe was
/// never evidence.
fn waitIdle(inst: Instance, timeout_polls: u32) Error!void {
    const b = base(inst);
    var polls: u32 = 0;
    while (hw.read(b + Reg.txflr) != 0 or hw.read(b + Reg.status) & status_mst_activity != 0) {
        try checkAbort(inst);
        polls += 1;
        if (polls > timeout_polls) return error.Timeout;
    }
    try checkAbort(inst);
}

/// Write bytes to `addr`. `stop` issues a STOP; pass false to hold the bus for a
/// repeated start.
pub fn write(inst: Instance, addr: u7, data: []const u8, stop: bool, timeout_polls: u32) Error!void {
    const b = base(inst);
    setTarget(inst, addr);

    // Read and re-arm up front so **every** exit path leaves it correct,
    // including the error ones. The SDK assigns it unconditionally after the
    // loop, abort or no abort.
    const restart = restart_on_next[@backingInt(inst)];
    restart_on_next[@backingInt(inst)] = !stop;

    for (data, 0..) |byte, i| {
        const cmd: u32 = byte | cmdFlags(i, data.len, restart, stop);
        hw.write(b + Reg.data_cmd, cmd);

        // **Wait for the byte to be transmitted, not merely accepted — and
        // this is the half of the 2026-08-26 port that was left out.**
        //
        // This used to wait only for room in the TX FIFO, which is true the
        // instant the push returns. The byte then takes about 90 µs to clock
        // out at 100 kHz, and `write` was back in its caller long before that.
        // Harmless with `stop = true`, because `waitIdle` below catches up.
        // **Not harmless with `stop = false`**, which is the whole repeated-
        // start path: the caller's very next act is `read`, whose `setTarget`
        // takes `IC_ENABLE` low — and the datasheet says that flushes the TX
        // FIFO and holds it in reset. So the register-pointer write was
        // destroyed mid-flight, on every single combined transaction, and the
        // read that followed returned whatever the device's pointer happened
        // to be sitting on. No abort, no timeout, no NAK: fifteen plausible
        // bytes from the wrong registers.
        //
        // **It cost a touchscreen controller for eight days.** Its driver was
        // then the only caller that passed `stop = false`, and the
        // RV-3028 this port was proven against does not — it writes with a
        // STOP and reads as a separate transaction, like every other driver
        // here. A reference part that never exercises the path under test
        // cannot confirm it.
        //
        // The pico-sdk has always waited here; it is the comment in
        // `i2c_write_blocking_internal` beginning "Wait until the transmission
        // of the address/data from the internal shift register has completed".
        // Porting `read` and `init` faithfully and leaving `write` as it was
        // is what opened the gap.
        var polls: u32 = 0;
        while (hw.read(b + Reg.raw_intr_stat) & intr_tx_empty == 0) : (polls += 1) {
            if (polls > timeout_polls) return error.Timeout;
            try checkAbort(inst);
        }
        try checkAbort(inst);
    }
    // **Only wait for idle if a STOP was actually issued.**
    //
    // `waitIdle` spins until MST_ACTIVITY clears, and with `stop = false` the
    // master is *deliberately* still holding the bus — that is the whole
    // meaning of "hold it for a repeated start". The bit cannot clear, so the
    // wait can only ever end in `error.Timeout`, and the caller sees a device
    // that acknowledges its address and then fails every transfer.
    //
    // **Found 2026-08-17 by the first caller ever to pass `false`**, reading an
    // FT6236: `probe` returned true, and every 15-byte read timed out. The
    // parameter had been in the signature since the module was written and
    // nothing had exercised it — an untested branch of a tested function,
    // which is the kind this repo keeps finding.
    //
    // The restart itself is `restart_on_next`'s job: this call has just armed
    // it, and the first command of the next transfer sets `IC_DATA_CMD.RESTART`
    // explicitly. **That used to read "the controller emits one automatically
    // when the direction changes" and it was true only while `setTarget`
    // cached `IC_TAR` and so never disabled the block between the two halves.**
    if (!stop) return checkAbort(inst);

    // The transfer has been *queued*, not performed. Waiting for the master to
    // go idle is what makes a NAK observable — see `waitIdle`.
    return waitIdle(inst, timeout_polls);
}

/// Read `out.len` bytes from `addr`.
///
/// Queueing and draining are deliberately interleaved rather than done as two
/// passes: the RX FIFO is 16 deep, so a naive "queue everything, then read
/// everything" deadlocks for longer transfers.
pub fn read(inst: Instance, addr: u7, out: []u8, stop: bool, timeout_polls: u32) Error!void {
    const b = base(inst);
    setTarget(inst, addr);

    const restart = restart_on_next[@backingInt(inst)];
    restart_on_next[@backingInt(inst)] = !stop;

    // **A faithful port of the pico-sdk's `i2c_read_blocking_internal`.**
    //
    // Reasoning about the differences between the two implementations produced
    // four wrong answers on 2026-08-26; porting the reference produced the
    // right one for `init` in one attempt. The shape below is deliberately the
    // SDK's, not a tidier equivalent:
    //
    //   * **wait for room in the TX FIFO *before* queueing**, rather than
    //     queueing conditionally inside the same loop that drains RX;
    //   * **read `IC_TX_ABRT_SOURCE` directly** and treat any non-zero value as
    //     an abort, rather than testing `RAW_INTR_STAT`'s TX_ABRT bit first;
    //   * **one tight loop per byte**, exited only on an abort or on data
    //     actually being available.
    for (out, 0..) |*dst, i| {
        var polls: u32 = 0;
        while (hw.read(b + Reg.txflr) >= 16) : (polls += 1) {
            if (polls > timeout_polls) return error.Timeout;
        }

        const cmd: u32 = cmd_read | cmdFlags(i, out.len, restart, stop);
        hw.write(b + Reg.data_cmd, cmd);

        polls = 0;
        while (true) {
            // **`IC_TX_ABRT_SOURCE` supplies the reason; `IC_RAW_INTR_STAT`
            // decides whether there was one.** The two are not
            // interchangeable: the source register holds its value until
            // `IC_CLR_TX_ABRT` is read, so testing it directly makes a stale
            // reason abort the *next* transfer. Written the other way round on
            // 2026-08-26 and it cost a bench session.
            const abrt = hw.read(b + Reg.tx_abrt_source);
            if (hw.read(b + Reg.raw_intr_stat) & intr_tx_abrt != 0) {
                last_abort_source = abrt;
                _ = hw.read(b + Reg.clr_tx_abrt);
                return error.NoAcknowledge;
            }
            if (hw.read(b + Reg.rxflr) > 0) break;
            polls += 1;
            if (polls > timeout_polls) return error.Timeout;
        }

        dst.* = @truncate(hw.read(b + Reg.data_cmd));
    }
}

/// The controller's own account of itself, for a board with no console.
///
/// **`error.Timeout` says the transfer did not finish and nothing else.** It
/// cannot distinguish a block that was never enabled, a byte still sitting in
/// the TX FIFO, a slave stretching the clock, or a master that never won the
/// bus — and those want completely different fixes. These five registers
/// separate them, and on a board whose only output is an SWD memory read there
/// is no other way to ask.
pub const Diag = struct {
    /// IC_ENABLE. 0 here and the block is off: every queued byte sits in the
    /// FIFO for ever and every call times out with no abort.
    enable: u32,
    /// IC_ENABLE_STATUS, **which is not IC_ENABLE**. Bit 0 `IC_EN` is what the
    /// block actually did with the request. Disabling this IP mid-transfer is
    /// not instantaneous, and a re-enable while it is still unwinding can leave
    /// `IC_ENABLE` reading 1 with the controller still down — writes then queue
    /// into a FIFO nobody is draining, which times out with no abort and looks
    /// exactly like a dead slave.
    enable_status: u32,
    /// IC_STATUS. Bit 0 ACTIVITY, bit 1 TFNF, bit 2 TFE, bit 3 RFNE,
    /// bit 5 MST_ACTIVITY.
    status: u32,
    /// IC_RAW_INTR_STAT. Bit 6 TX_ABRT is the one `checkAbort` reads.
    raw_intr_stat: u32,
    /// IC_TX_ABRT_SOURCE, **live rather than latched** — `last_abort_source`
    /// holds the value from the last abort that was consumed, which may be old.
    tx_abrt_source: u32,
    txflr: u32,
    rxflr: u32,
};

pub fn diagnostics(inst: Instance) Diag {
    const b = base(inst);
    return .{
        .enable = hw.read(b + Reg.enable),
        .enable_status = hw.read(b + Reg.enable_status),
        .status = hw.read(b + Reg.status),
        .raw_intr_stat = hw.read(b + Reg.raw_intr_stat),
        .tx_abrt_source = hw.read(b + Reg.tx_abrt_source),
        .txflr = hw.read(b + Reg.txflr),
        .rxflr = hw.read(b + Reg.rxflr),
    };
}

/// Is anything at this address?
///
/// **Trustworthy only since `write` learned to wait** (2026-07-28). Before
/// that it reported the byte queued rather than acknowledged, so this returned
/// true for every address on the bus and a scan found 120 devices.
///
/// **Probes with a one-byte read, not a write, and that is a fix rather than a
/// preference.**
///
/// It used to send one byte, `0x00`. Two things were wrong with that:
///
///   1. **It writes to the device.** On a register-based part that sets the
///      register pointer; the pico-sdk's own `bus_scan` uses
///      `i2c_read_blocking` for exactly this reason. A scan should not change
///      what it scans.
///   2. **It did not work.** On 2026-08-26, with an RV-3028 at 0x52 and an
///      ICM-20948 at 0x68 on a Pimoroni Breakout Garden, the write form found
///      *nothing* while the SDK's read form found both. Same board, same pins,
///      same session — the reference was built, flashed and run to settle it.
///      The write path stalls where the read path does not.
///
/// **The `write` stall this once warned about is fixed** (2026-08-26, and
/// proven on an RV-3028 with a combined `write(stop=false)` + `read`). `write`
/// waited for *room* in the TX FIFO; it now waits for `TX_EMPTY`, which with
/// `TX_EMPTY_CTRL` set means the shift register has finished. The note is kept
/// in this shape because the two reasons above for probing with a read stand on
/// their own — a scan should not write to what it scans.
pub fn probe(inst: Instance, addr: u7, timeout_polls: u32) bool {
    var rx: [1]u8 = undefined;
    read(inst, addr, &rx, true, timeout_polls) catch return false;
    return true;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "timing: 100 kHz and 400 kHz from a 12 MHz clock" {
    const clk = 12_000_000;

    const std_t = Timing.compute(clk, 100_000);
    const std_actual = std_t.actualHz(clk);
    try testing.expect(std_actual > 95_000 and std_actual <= 100_000);

    const fast_t = Timing.compute(clk, 400_000);
    const fast_actual = fast_t.actualHz(clk);
    try testing.expect(fast_actual > 380_000 and fast_actual <= 400_000);
}

test "timing: SCL low period is never shorter than the high period" {
    // Required by the I2C standard, and a bus that violates it works with most
    // devices and not all — the worst kind of bug to ship.
    for ([_]u32{ 100_000, 400_000 }) |target| {
        const t = Timing.compute(12_000_000, target);
        try testing.expect(t.lcnt >= t.hcnt);
    }
}

test "timing: never overruns the requested rate" {
    // Running an I2C bus faster than requested can exceed a device's rating;
    // running it slower is always safe. Same asymmetry as the SPI divider.
    for ([_]u32{ 50_000, 100_000, 200_000, 400_000 }) |target| {
        const t = Timing.compute(12_000_000, target);
        try testing.expect(t.actualHz(12_000_000) <= target);
    }
}

test "command bits match the DW_apb_i2c layout" {
    try testing.expectEqual(@as(u32, 0x100), cmd_read);
    try testing.expectEqual(@as(u32, 0x200), cmd_stop);
    try testing.expectEqual(@as(u32, 0x400), cmd_restart);
}

test "cmdFlags: a combined register read" {
    // The ds3231/rv3028 shape: write(reg, stop=false) then read(out, stop=true).
    // The pointer write is one byte, no restart armed, no stop.
    try testing.expectEqual(@as(u32, 0), cmdFlags(0, 1, false, false));
    // The read that follows is the first command with restart armed, and the
    // last with a stop -- so it carries BOTH, which is the case that a
    // `first`/`last` pair written as if they were exclusive gets wrong.
    try testing.expectEqual(@as(u32, 0x600), cmdFlags(0, 1, true, true));
}

test "cmdFlags: multi-byte reads set each bit exactly once" {
    // 4 bytes, restart armed, stop at the end.
    try testing.expectEqual(@as(u32, 0x400), cmdFlags(0, 4, true, true));
    try testing.expectEqual(@as(u32, 0), cmdFlags(1, 4, true, true));
    try testing.expectEqual(@as(u32, 0), cmdFlags(2, 4, true, true));
    try testing.expectEqual(@as(u32, 0x200), cmdFlags(3, 4, true, true));
}

test "cmdFlags: nothing is set when nothing is asked for" {
    try testing.expectEqual(@as(u32, 0), cmdFlags(0, 1, false, false));
    try testing.expectEqual(@as(u32, 0), cmdFlags(0, 8, false, false));
    try testing.expectEqual(@as(u32, 0), cmdFlags(7, 8, false, false));
}

test "restart_on_next starts clear" {
    // A transfer must never open with a repeated START it did not earn: on a
    // freshly initialised block there is no held bus to continue.
    try testing.expect(!restart_on_next[0]);
    try testing.expect(!restart_on_next[1]);
}
