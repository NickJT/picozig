//! gSPI PIO transport for the Infineon CYW43439 on RP2350 (Pico 2 W).
//!
//! Drives the 3-pin half-duplex gSPI bus with a PIO state machine, ported from
//! the pico-sdk's `cyw43_bus_pio_spi.pio` / `.c`. Pins: GP23 `WL_REG_ON`, GP24
//! `WL_DATA` (both directions — this is the half of "half-duplex" that matters),
//! GP25 `WL_CS`, GP29 `WL_CLK`.
//!
//! **Why PIO and not a CPU loop.** The predecessor to this file bit-banged the
//! clock from GPIO register writes with no delay and no turnaround. That cannot
//! meet the CYW43's setup/hold at any RP2350 clock, and — worse — a bus that is
//! merely *marginal* fails intermittently rather than cleanly, which is
//! indistinguishable from a protocol bug and was diagnosed as one for a while.
//! The state machine makes the timing a property of the hardware instead of of
//! the compiler's instruction scheduling.
//!
//! ## Byte order, which is the part that bites
//!
//! The state machine shifts **MSB-first, 32 bits at a time**, so a word placed
//! in the TX FIFO appears on the wire high byte first. The pico-sdk instead
//! keeps native `uint32_t` buffers and byte-swaps in the DMA
//! (`channel_config_set_bswap`), which makes its wire order equal to its
//! *memory* order. Polling the FIFO from the CPU, we do that swap explicitly:
//!
//!   - a value `v` sent the way the SDK sends it  → FIFO word `@byteSwap(v)`
//!   - a byte buffer                              → FIFO word `readInt(.big)`
//!   - the boot-time 16-bit path (`SWAP32`/`rev16` in C, then the DMA swap)
//!     collapses to a plain **halfword swap** of `v`; see `swapHalfwords`.
//!
//! Those three lines are the whole of it, and every one of them was derived by
//! following the SDK's bytes rather than by reasoning about what "big endian
//! mode" ought to mean — the chip's own naming does not survive that reasoning.

const std = @import("std");
const rp2350 = @import("rp2350");

const pio = rp2350.pio;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const bounded = rp2350.bounded;

// ---------------------------------------------------------------------------
// Wire format
// ---------------------------------------------------------------------------

/// gSPI function targets, per the CYW43 datasheet.
pub const Function = enum(u2) {
    bus = 0,
    backplane = 1,
    /// Function 2: the WLAN SDPCM data engine, shared with the Bluetooth pipe.
    wlan_or_bt = 2,
    /// Alias kept for callers written before the BT pipe was named.
    pub const wlan: Function = .wlan_or_bt;
};

/// The 32-bit gSPI command header.
///
/// **This is the one packer.** There were two — `Header.pack()` in `cyw43.zig`
/// and `packCmd()` here — and they disagreed about where `inc` lives, so half
/// the driver emitted `inc` at bit 27 (colliding with the top bit of `addr`,
/// and leaving bit 30 clear) while the other half hardcoded it at bit 30. Both
/// were covered by tests that asserted each function's own output.
///
/// Canonical layout, from `make_cmd` in `cyw43_bus_pio_spi.c`:
/// `write(31) | inc(30) | fn(28..29) | addr(11..27) | len(0..10)`.
pub const Header = struct {
    write: bool,
    fn_num: Function,
    addr: u17,
    len: u11,
    /// Auto-increment the address across a multi-word transfer. The SDK passes
    /// true everywhere; a false here means a FIFO-style port.
    inc: bool = true,

    pub fn pack(self: Header) u32 {
        return (@as(u32, @intFromBool(self.write)) << 31) |
            (@as(u32, @intFromBool(self.inc)) << 30) |
            (@as(u32, @backingInt(self.fn_num)) << 28) |
            (@as(u32, self.addr) << 11) |
            @as(u32, self.len);
    }
};

/// Convenience wrapper around `Header.pack`, matching `make_cmd`'s argument
/// order so the two can be read side by side.
pub fn makeCmd(write: bool, inc: bool, fn_num: Function, addr: u17, len: u11) u32 {
    return (Header{ .write = write, .inc = inc, .fn_num = fn_num, .addr = addr, .len = len }).pack();
}

/// Swap the two halfwords of a word.
///
/// This is what the SDK's `SWAP32` (a `rev16` — bytes swapped *within* each
/// halfword) becomes once the DMA's own byte-swap is folded in, and it is only
/// ever right during the boot window before the 32-bit handshake: the chip
/// powers up in 16-bit little-endian mode, where a 32-bit quantity arrives as
/// two big-endian halfwords, low half first.
pub fn swapHalfwords(v: u32) u32 {
    return (v >> 16) | (v << 16);
}

// ---------------------------------------------------------------------------
// gSPI bus registers (Function 0)
// ---------------------------------------------------------------------------

pub const reg = struct {
    pub const bus_control: u17 = 0x0000;
    /// 16-bit interrupt status; bits are cleared by writing 1.
    pub const interrupt: u17 = 0x0004;
    /// 16-bit interrupt mask.
    pub const interrupt_enable: u17 = 0x0006;
    pub const status: u17 = 0x0008;
    /// Reads `test_pattern` on a live chip. **Not a chip-ID register** — the
    /// predecessor probed 0x0000 (which is `bus_control`) and compared against
    /// a chip ID that register never holds.
    pub const read_test: u17 = 0x0014;
    pub const resp_delay_f1: u17 = 0x001D;
};

/// `SPI_INTERRUPT_REGISTER` / `SPI_INTERRUPT_ENABLE_REGISTER` bits.
pub const intr = struct {
    pub const data_unavailable: u16 = 0x0001;
    pub const f2_f3_fifo_rd_underflow: u16 = 0x0002;
    pub const f2_f3_fifo_wr_overflow: u16 = 0x0004;
    pub const command_error: u16 = 0x0008;
    pub const data_error: u16 = 0x0010;
    pub const f2_packet_available: u16 = 0x0020;
    pub const f1_overflow: u16 = 0x0080;
    pub const f1_intr: u16 = 0x2000;
};

/// Error bits cleared once at bring-up, so a later read reports this run's
/// faults rather than the boot window's.
pub const intr_clear_at_init: u8 =
    intr.data_unavailable | intr.command_error | intr.data_error | intr.f1_overflow;

/// The interrupts worth having enabled for WLAN-only operation.
pub const intr_enable_wlan: u16 =
    intr.f2_f3_fifo_rd_underflow | intr.f2_f3_fifo_wr_overflow |
    intr.command_error | intr.data_error | intr.f2_packet_available | intr.f1_overflow;

pub const test_pattern: u32 = 0xFEED_BEAD;

/// `SPI_BUS_CONTROL` fields.
pub const bus_ctrl = struct {
    pub const word_length_32: u32 = 0x01;
    pub const endian_big: u32 = 0x02;
    /// **Bit 4 selects the edge the chip drives read data on**, which is half
    /// of a matched pair whose other half is the host's PIO program. Datasheet
    /// Table 6:
    ///
    ///   * `0` — normal mode: chip samples on the rising edge, **outputs on the
    ///     falling edge**. The host must then sample on the rising edge.
    ///   * `1` — high-speed mode: chip **samples and outputs on the rising
    ///     edge**. The host must then sample half a period later, on the
    ///     falling edge.
    ///
    /// §4.2 lists "a configurable active edge for shifting" as a feature of
    /// gSPI mode, and this is that bit. It had been carried here as an opaque
    /// part of the handshake word since rung 1, with the consequence that the
    /// two halves of the pair were chosen independently and nobody noticed:
    /// this driver sets high-speed mode (chip outputs on rising) and runs
    /// `gap0_sample1` (host samples on rising), which is a mismatched pair.
    /// At 3 MHz the half-period is 167 ns and both land deep inside the eye,
    /// so it works. See `Variant` for what happens when it does not.
    pub const high_speed_mode: u32 = 0x10;
    pub const interrupt_polarity_high: u32 = 0x20;
    pub const wake_up: u32 = 0x80;
    /// Byte 2 of the register: `INTR_WITH_STATUS`.
    pub const intr_with_status: u32 = 0x02;
};

/// The word written to `SPI_BUS_CONTROL` to leave 16-bit boot mode.
///
/// **Not `0x8B`.** `cyw43_ll.c:1433` composes three bytes: the mode bits, a
/// response-delay byte at 8..15, and the status-enable byte at 16..23. The
/// interrupt-polarity bit is present because the Pico 2 W wires `WL_HOST_WAKE`
/// (to GP24, the data pin).
pub fn busControlWord(high_speed: bool) u32 {
    return bus_ctrl.word_length_32 | bus_ctrl.endian_big |
        (if (high_speed) bus_ctrl.high_speed_mode else 0) |
        bus_ctrl.wake_up | bus_ctrl.interrupt_polarity_high |
        (0x04 << 8) |
        (bus_ctrl.intr_with_status << 16);
}

/// The handshake word as every rung through 27 has sent it: high-speed mode on.
pub const bus_control_word: u32 = busControlWord(true);

/// `SPI_STATUS_REGISTER` bit 5 — Function 2 will accept a packet.
pub const status_f2_rx_ready: u32 = 0x0000_0020;

/// `SPI_STATUS_REGISTER` bit 8 — Function 2 has a packet waiting. When set,
/// bits 9..19 hold its length in bytes.
pub const status_f2_pkt_available: u32 = 0x0000_0100;

/// Extract the waiting packet's length from `SPI_STATUS_REGISTER`.
/// The widest packet length the status word can express: the field is 11 bits.
///
/// **A receive buffer smaller than this can be wedged rather than merely
/// overrun**, because a packet the driver declines to read is never drained.
pub const max_packet_length: u32 = 0x7FF;

pub fn packetLength(status: u32) u32 {
    return (status >> 9) & max_packet_length;
}

/// Bytes of response delay clocked ahead of every Function 1 read.
///
/// **16, and prepended inside the same transaction** rather than sent
/// separately — `cyw43_bus_pio_spi.c` reserves pad words at the head of its
/// header array and points the command at an offset past them. The `0x04`
/// written into `resp_delay_f1` is a different quantity (a field value, not a
/// byte count) and conflating the two produces reads that are short by 12
/// bytes and look like corruption.
pub const backplane_read_pad_bytes: usize = 16;

/// The largest response delay the transport will allocate buffer space for.
pub const max_read_pad_bytes: usize = 16;

// ---------------------------------------------------------------------------
// The PIO program
// ---------------------------------------------------------------------------

/// `spi_gap01_sample0`, transliterated from `cyw43_bus_pio_spi.pio`.
///
/// Upstream's own selection at high CPU clock, and **not the one that works
/// here** — it reads 0/8 on this board. Kept because it is the SDK's default
/// and therefore the first thing to re-try if the bus ever stops answering;
/// see `Variant` for the measurements.
///
/// ```
/// lp:      out pins, 1     side 0     ; 0
///          jmp x-- lp      side 1     ; 1
/// lp1_end:                            ; 2
///          set pindirs, 0  side 0     ; 2   release the data pin
///          nop             side 1     ; 3   turnaround
/// lp2:     in pins, 1      side 0     ; 4
///          jmp y-- lp2     side 1     ; 5
/// end:                                ; 6
/// ```
const gap01_sample0 = [_]u16{
    pio.out(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.x_dec, 0, pio.sideSet(1, 1, 0)),
    pio.set(.pindirs, 0, pio.sideSet(1, 0, 0)),
    pio.nop(pio.sideSet(1, 1, 0)),
    pio.in(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.y_dec, 4, pio.sideSet(1, 1, 0)),
};

/// `spi_gap0_sample1` — one less turnaround cycle, and samples on the high
/// clock. Upstream marks it "for lower cpu speed".
const gap0_sample1 = [_]u16{
    pio.out(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.x_dec, 0, pio.sideSet(1, 1, 0)),
    pio.set(.pindirs, 0, pio.sideSet(1, 0, 0)),
    pio.in(.pins, 1, pio.sideSet(1, 1, 0)),
    pio.jmp(.y_dec, 3, pio.sideSet(1, 0, 0)),
};

/// `spi_gap010_sample1` — a full extra clock of turnaround, sampling high.
const gap010_sample1 = [_]u16{
    pio.out(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.x_dec, 0, pio.sideSet(1, 1, 0)),
    pio.set(.pindirs, 0, pio.sideSet(1, 0, 0)),
    pio.nop(pio.sideSet(1, 1, 0)),
    pio.nop(pio.sideSet(1, 0, 0)),
    pio.in(.pins, 1, pio.sideSet(1, 1, 0)),
    pio.jmp(.y_dec, 5, pio.sideSet(1, 0, 0)),
};

/// `gap0101_sample0` — **not one of the SDK's.** The fourth cell of a grid the
/// SDK's three leave incomplete; see `Variant`.
///
/// ```
/// lp1_end: set pindirs, 0  side 0   ; 2
///          nop             side 1   ; 3
///          nop             side 0   ; 4
///          nop             side 1   ; 5
/// lp2:     in pins, 1      side 0   ; 6
///          jmp y-- lp2     side 1   ; 7
/// ```
const gap0101_sample0 = [_]u16{
    pio.out(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.x_dec, 0, pio.sideSet(1, 1, 0)),
    pio.set(.pindirs, 0, pio.sideSet(1, 0, 0)),
    pio.nop(pio.sideSet(1, 1, 0)),
    pio.nop(pio.sideSet(1, 0, 0)),
    pio.nop(pio.sideSet(1, 1, 0)),
    pio.in(.pins, 1, pio.sideSet(1, 0, 0)),
    pio.jmp(.y_dec, 6, pio.sideSet(1, 1, 0)),
};

/// Which gSPI program to run: **a 2x2 grid, not a list of three.**
///
/// **Selectable because the right one is an empirical question about this
/// board at this bus rate, not a fact you can read off.** They differ only in
/// how many half-cycles of turnaround sit between releasing the data pin and
/// the first sample, and on which clock level that sample lands — which is
/// exactly the thing a wrong answer makes look like a protocol bug.
///
/// ## The grid
///
/// Counting half-cycles from the `set pindirs, 0` that releases the data pin,
/// two independent things vary, and reading the SDK's three as a flat list
/// hides that one of them is not a phase adjustment at all:
///
/// | | sample on rising (phase 0) | sample on falling (phase +1/2) |
/// |---|---|---|
/// | **first output bit** | `gap0_sample1` | `gap01_sample0` |
/// | **second output bit** | `gap010_sample1` | `gap0101_sample0` |
///
/// * **Phase** is the sub-bit adjustment. It compensates round-trip delay —
///   host clock out, chip clock-to-out, data back in — and is the axis that
///   must move as the bus rate rises, because that delay is fixed while the
///   bit period shrinks.
/// * **Bit offset** is a whole-bit shift, and is a different quantity
///   entirely: `gap010_sample1` delivers *two* rising edges before its first
///   sample where `gap0_sample1` delivers one, so it starts capturing at the
///   chip's second output bit. That is a response-delay question, not a
///   timing-margin one.
///
/// The SDK ships the first three and calls the choice "cpu speed"; the fourth
/// is ours, added so the grid can be swept rather than three of its corners.
/// (`spi_gap0_sample1_regular` in `cyw43_bus_pio_spi.pio` is **byte-identical**
/// to `spi_gap0_sample1` and is referenced nowhere in the SDK — it is a dead
/// duplicate, not a fourth program.)
///
/// ## The other half of the pair is not in this file
///
/// The host's sample edge is only meaningful against the edge **the chip
/// drives on**, and that is selected by `bus_ctrl.high_speed_mode` in the
/// handshake word — see its doc comment. The two must be chosen together.
/// This driver has always set high-speed mode (chip outputs on the rising
/// edge) while defaulting to `gap0_sample1` (host samples on the rising edge),
/// which is a mismatched pair that survives only because at 3 MHz the
/// half-period is 167 ns.
///
/// ## What this board actually does
///
/// Measured on a Pico 2 W with `example/cyw43-probe`, which sweeps the
/// variants and counts how many of eight test-register reads return
/// `0xFEEDBEAD`. **At a 3 MHz bus, not 150 MHz** — this said "at 150 MHz"
/// until 2026-08-24, and it was never true: `cyw43-probe` calls
/// `clocks.init(board.clock_plan)`, `clock_plan` is `null` for a Pico 2 W, and
/// it has never been anything else, so clk_sys was the bare 12 MHz crystal and
/// the bus was `clk_sys / 4`. Getting this wrong made a bus-rate result look
/// like a board result. Reproduced on hardware 2026-08-24: 0/8, 7/8, 0/8.
///
/// | Variant | Hits |
/// |---|---|
/// | `gap01_sample0` | 0/8 |
/// | **`gap0_sample1`** | **7/8** |
/// | `gap010_sample1` | 0/8 |
///
/// **`gap0_sample1` is the one that works here, and that contradicts the
/// SDK's own comment**, which marks it "for lower cpu speed" and selects
/// `gap01_sample0` "for high cpu speed". Do not read the table as disproving
/// the comment in general — upstream drives its FIFOs by DMA and we poll them,
/// and it does not enable input sync bypass in the same breath as sampling on
/// the opposite clock level. The honest statement is narrower: on this board,
/// in this driver, the sample must land on the high clock with one turnaround
/// cycle, and the sweep is kept in the probe example so the next person can
/// re-run it rather than trust this table.
///
/// ## And on an RM2 module it is the other one — measured 2026-08-16
///
/// The RM2 is the same CYW43439 on the same four pins, driven by this same
/// driver from a test image on a board with an RM2. **That board runs clk_sys at
/// 200 MHz and the image never calls `setSysHz`, so that run was at a
/// 50 MHz bus** — `clk_sys / 4` with the bare divisor of the day, and exactly
/// the part's 50 MHz maximum. That is the number to compare against the 3 MHz
/// above; it is not a like-for-like board comparison:
///
/// | Variant | Reached |
/// |---|---|
/// | **`gap01_sample0`** | **rung 16 — a UTC timestamp** |
/// | `gap0_sample1` | rung 1 — the test register, one bit out of phase |
/// | `gap010_sample1` | rung 11 — test register fine, CLM load times out |
///
/// **Inverted from the Pico 2 W's result — and the inversion is bus rate, not
/// board.** `example/gspi-sweep` settled it on 2026-08-24 by crossing rate with
/// program on a single Pico 2 W at a fixed 150 MHz clk_sys:
///
/// | bus rate | `gap0_sample1` | `gap01_sample0` |
/// |---|---|---|
/// | 3.000 MHz | **works** | one bit early |
/// | 9.375 MHz | **works** | one bit early |
/// | 12.50 MHz | **works** | one bit early |
/// | 18.75 MHz | **works** | one bit early |
/// | 25.00 MHz | works (high-speed mode only) | **works** |
/// | 37.50 MHz | one bit marginal | **works** |
///
/// One variable moving one way, with a crossover between 18.75 and 25 MHz —
/// exactly what the SDK's own "for lower cpu speed" / "for high cpu speed"
/// comments have said all along. **So the choice is a bus-rate fact, and the
/// two board results were never in conflict.** The failing values identify
/// the mechanism, because
/// `0xFDDA7D5B` (shifted a bit) and `0xFEED3EAD` (one bit marginal) are
/// different faults and the difference is the whole diagnosis.
///
/// Two things that run taught, both cheap to reuse:
///
/// * **A wrong variant produces legible data, not noise.** `0xFF765F56` came
///   back where `0xFEEDBEAD` was expected — which is the wire stream (BEAD then
///   FEED) shifted exactly one bit. Reading the wrong value rather than
///   discarding it named the variable immediately.
/// * **A working test register is not a working bus.** `gap010_sample1` read
///   the pattern correctly, loaded 224 KB of firmware and completed an IOCTL
///   before failing at the CLM blob. A sweep that stops at the first variant
///   which answers stops one variant early.
///
///   There is now a mechanism for that, rather than just a caution: the test
///   register is read **before the handshake**, in 16-bit boot mode, so it is
///   the only traffic in the whole driver that runs at the chip's power-on
///   edge rather than the one `bus_control` selects. A variant can therefore
///   pass the probe and fail everything after it, which is exactly what that
///   run did. Any sweep must read past the handshake — a 32-bit Function 0
///   read and a Function 1 backplane read — or it is measuring a phase almost
///   nothing else uses.
///
/// The missing eighth hit is the **first** read of the eight, every time
/// (bitmask `0b11111110`). That is the chip waking rather than a marginal
/// sample point, and it is why `probe` retries — as does the SDK, ten times at
/// 1 ms, for the same reason.
pub const Variant = enum {
    /// Bit 0, phase +1/2. The SDK's own choice at high CPU clock.
    gap01_sample0,
    /// Bit 0, phase 0. Measured correct on the Pico 2 W at 3 MHz.
    gap0_sample1,
    /// Bit 1, phase 0.
    gap010_sample1,
    /// Bit 1, phase +1/2. **Ours, not the SDK's** — the missing fourth cell.
    gap0101_sample0,

    /// Rising edges delivered before this program's first sample: 1 for the
    /// programs that capture the chip's first output bit, 2 for the ones that
    /// skip it. See the grid in this type's doc comment.
    pub fn bitOffset(self: Variant) u1 {
        return switch (self) {
            .gap01_sample0, .gap0_sample1 => 0,
            .gap010_sample1, .gap0101_sample0 => 1,
        };
    }

    /// Whether the sample lands on the rising edge (`false`) or half a clock
    /// later, on the falling edge (`true`).
    pub fn samplesLate(self: Variant) bool {
        return switch (self) {
            .gap0_sample1, .gap010_sample1 => false,
            .gap01_sample0, .gap0101_sample0 => true,
        };
    }

    /// The lowest bus rate at which the **late** phase is measured to work.
    ///
    /// Rung A bracketed the crossover to `(18.75, 25]` MHz and **rung A2
    /// narrowed it to `(20, 25]`** by running a 200 MHz core, where the same
    /// divisors reach rates a 150 MHz core cannot express. Measured on two
    /// boards:
    ///
    /// * phase 0 (`gap0_sample1`) works to **33.3 MHz**, fails at 37.5
    /// * phase +1/2 (`gap01_sample0`) fails at **20 MHz**, works from 25
    /// * so **25–33.3 MHz is a band where either program works**
    ///
    /// This threshold sits at the bottom of that band, on a measured point, so
    /// every choice `forBusHz` makes is one that was actually run rather than
    /// interpolated. A threshold of 20 MHz would select the late phase at a
    /// rate where it is measured **not** to answer.
    pub const late_phase_min_hz: u32 = 25_000_000;

    /// The program to run at a given bus rate.
    ///
    /// **The bus rate and the program are one decision.** Changing the divisor
    /// without changing this is what rung 21d did, and it presents as
    /// `ChipNotResponding` at bring-up rather than as anything to do with
    /// timing.
    pub fn forBusHz(bus_hz: u32) Variant {
        return if (bus_hz >= late_phase_min_hz) .gap01_sample0 else .gap0_sample1;
    }

    pub fn program(self: Variant) []const u16 {
        return switch (self) {
            .gap01_sample0 => &gap01_sample0,
            .gap0_sample1 => &gap0_sample1,
            .gap010_sample1 => &gap010_sample1,
            .gap0101_sample0 => &gap0101_sample0,
        };
    }

    /// Index just past the transmit loop — the wrap target for a write-only
    /// transfer, which must not fall through into the receive half.
    pub fn lp1End(_: Variant) u5 {
        return 2;
    }

    /// Index just past the whole program.
    pub fn end(self: Variant) u5 {
        return @intCast(self.program().len);
    }
};

/// State-machine clock divisor, from `CYW43_PIO_CLOCK_DIV_INT` = 2 (frac 0).
/// Two `clk_sys` per instruction, and the program spends two instructions per
/// bit, so the bus clock is `clk_sys / 4`.
///
/// **This is the divisor for a crystal-clocked board and nothing else.** It was
/// a bare constant, which silently welds the bus rate to clk_sys: a Pico 2 W
/// runs clk_sys off the 12 MHz crystal, so `clk_sys / 4` is 3 MHz, and every
/// rung of this package was proven at that. Raising clk_sys to 150 MHz for CPU
/// headroom took the bus to 37.5 MHz with it and the chip stopped answering at
/// rung 1 — `readTestRegister` never returned the pattern. See `setSysHz`.
///
/// **That failure was the sample phase, not the rate** (rung A, 2026-08-24):
/// 37.5 MHz works on this exact board with `.gap01_sample0`. So the pairing
/// that matters is divisor *and* variant, and `variant`'s default goes with
/// this constant — change one and you must change the other.
const default_clock_div: u32 = 2;

/// The bus rate every rung in this package has been proven **end to end** at,
/// in Hz — firmware download, CLM blob, join, data path, audio.
///
/// **Not a limit, and no longer an open question.** Rung A drove this driver's
/// bring-up and register traffic at 37.5 MHz on a Pico 2 W with
/// `.gap01_sample0`, and the datasheet caps the part at 50 MHz (Table 43, T1
/// min 20.8 ns). What has *not* been done at that rate is everything above
/// bring-up, which is why this constant still says 3 MHz and why `setSysHz`
/// still holds the bus here by default.
///
/// Holding the bus at a proven rate while clk_sys moves keeps the two variables
/// separate: a board can buy CPU headroom without re-opening a transport that
/// took twenty rungs to get right. Raise this only with a `bt-throughput` run
/// across the rate ladder and a cold-boot soak behind it — and raise `variant`
/// in the same commit.
pub const proven_bus_hz: u32 = 3_000_000;

pub const BoardPins = struct {
    wl_reg_on: rp2350.Pin = 23,
    wl_data: rp2350.Pin = 24,
    wl_cs: rp2350.Pin = 25,
    wl_clk: rp2350.Pin = 29,
};

pub const Error = error{
    NotPowered,
    /// A transfer was asked for that the state machine cannot express.
    BadTransferLength,
} || bounded.Error;

// ---------------------------------------------------------------------------
// Transaction trace
// ---------------------------------------------------------------------------

/// When set, every register transaction is printed on UART0 in **exactly the
/// format the canonical driver's `CYW43_VERBOSE_DEBUG` uses**, so the two logs
/// can be diffed line by line.
///
/// That format match is the whole point — a trace in our own house style would
/// have to be read and translated, and the errors this is meant to catch are
/// precisely the ones that survive being read. Capture the canonical log with
/// the pico-sdk `blink` example built with CYW43 tracing on, run this, and
/// diff.
pub var trace: bool = false;

fn fnName(f: Function) []const u8 {
    return switch (f) {
        .bus => "BUS_FUNCTION",
        .backplane => "BACKPLANE_FUNCTION",
        .wlan_or_bt => "WLAN_FUNCTION",
    };
}

fn traceReg(op: []const u8, f: Function, addr: u17, val: u32) void {
    if (!trace) return;
    rp2350.uart.print(.uart0, "cyw43_{s} {s} 0x{x}=0x{x}\n", .{ op, fnName(f), addr, val });
}

// ---------------------------------------------------------------------------
// The bus
// ---------------------------------------------------------------------------

pub const GspiBus = struct {
    pins: BoardPins,
    block: pio.Block = .pio0,
    sm: u32 = 0,
    offset: u5 = 0,
    powered: bool = false,
    cur_window: u32 = 0,

    /// Set by `onHostWakeIrq`, cleared by `takeHostWake`. The ISR's entire
    /// output — see the host-wake section below.
    host_wake_pending: bool = false,

    /// Whether host wake is wanted at all. Transfers mask the interrupt and
    /// restore it from this, rather than unconditionally re-enabling something
    /// the application never asked for.
    host_wake_enabled: bool = false,

    /// Which gSPI program the state machine runs.
    ///
    /// **The default goes with `default_clock_div`, not with the board.** A
    /// crystal-clocked board runs a 3 MHz bus, and 3 MHz wants phase 0. Raise
    /// the bus rate past ~20 MHz and this must become `gap01_sample0` — see
    /// `Variant` for the measured crossover. A caller that changes one without
    /// the other gets a chip that stops answering.
    variant: Variant = .gap0_sample1,

    /// Bytes of response delay the chip inserts before Function 1 read data.
    ///
    /// **Sixteen, as the SDK sends — and this comment used to say the opposite
    /// at length.** It claimed the part "does not accept 16", on the evidence
    /// that the register was written twice and still read back 4. It really did
    /// read 4, and the reading was worthless: `b4ebb0a` made that measurement,
    /// and `84a166f` — *the very next commit* — fixed the bug where **every
    /// ordinary write was silently dropped**. So the write never landed and the
    /// register was faithfully reporting its power-on default, which datasheet
    /// Table 6 gives as exactly 4 for `0x1D`. The measurement was of a chip
    /// that had never been configured.
    ///
    /// `busInit` now writes 16 and stores 16 without reading back, and the
    /// RM2 board reads 16 too. Four remains the default only because it is the
    /// chip's own value before anyone writes one.
    ///
    /// **The lesson is the ordering, not the number**: a measurement taken
    /// through a broken write path confirms whatever the hardware already held.
    resp_delay_bytes: usize = 4,

    /// The PIO divisor, defaulting to the crystal-clocked value. `setSysHz`
    /// and `setBusDiv` write it; nothing else does.
    clock_div: u32 = default_clock_div,

    /// Transfers in which the state machine had to wait for the CPU to empty
    /// the RX FIFO. **Not errors** — the wire was stretched, not corrupted —
    /// but a rising count means the bus is running faster than this core can
    /// poll it, and the effective rate is below nominal.
    rx_stalls: u32 = 0,

    /// Which edge the **chip** drives read data on, sent in the handshake word.
    /// True — the default and what every rung through 27 ran — is high-speed
    /// mode: the chip outputs on the rising edge. See `bus_ctrl.high_speed_mode`,
    /// and `Variant` for the host half of the pair.
    ///
    /// **Leave it true.** Rung A found it changed one cell in 48 and called it
    /// nearly inert; rung A2 ran the same grid to the part's 50 MHz maximum and
    /// found three differing cells, every one at a timing margin — including
    /// 50 MHz itself, which works in high-speed mode and fails in normal mode.
    /// The bit buys real margin. It is second-order to the sample phase, which
    /// is why a ladder stopping at 37.5 MHz could not tell "no effect" from
    /// "effect smaller than the margin".
    high_speed: bool = true,

    pub fn init(pins: BoardPins) GspiBus {
        return .{ .pins = pins };
    }

    /// Hold the bus at `proven_bus_hz` on a board whose clk_sys is not the bare
    /// crystal. **Call it before `powerOn`**; the divisor is written when the
    /// state machine is configured.
    ///
    /// A caller that raises clk_sys and forgets this gets a bus scaled up by
    /// the same factor, which fails at rung 1 with `ChipNotResponding` rather
    /// than misbehaving quietly — the one good property of that bug.
    pub fn setSysHz(self: *GspiBus, sys_hz: u32) void {
        // Two PIO instructions per bit, `clock_div` clk_sys cycles each.
        const div = sys_hz / (2 * proven_bus_hz);
        self.clock_div = @max(1, div);
    }

    /// What the bus will actually run at, given a clk_sys. Rounding in the
    /// integer divisor means this is not always `proven_bus_hz` exactly.
    pub fn busHz(self: GspiBus, sys_hz: u32) u32 {
        return sys_hz / (2 * self.clock_div);
    }

    /// Set the PIO divisor directly, for a caller that wants an exact ratio
    /// rather than a target rate that rounds.
    ///
    /// **This is the sweep rig's entry point, not an application's** — an
    /// application should say what rate it wants and let `setSysHz` hold it.
    /// A divisor is only the right unit when the point is to walk `clk_sys / 4`,
    /// `/6`, `/8` ... and see where the transport stops answering.
    ///
    /// The datasheet caps the bus at **50 MHz** (Table 43: clock period T1 min
    /// 20.8 ns), and the 45-55% duty requirement in the same table is why this
    /// takes an integer: a fractional PIO divisor jitters the duty cycle.
    /// `busHz` will tell you what you asked for; it will not stop you asking
    /// for something out of spec.
    pub fn setBusDiv(self: *GspiBus, div: u32) void {
        self.clock_div = @max(1, div);
    }

    /// Choose the edge the chip drives read data on. **Call it before
    /// `busInit`**; it is sent in the handshake word.
    pub fn setHighSpeed(self: *GspiBus, high_speed: bool) void {
        self.high_speed = high_speed;
    }

    /// Pick a different gSPI program. Takes effect at the next `powerOn`.
    pub fn setVariant(self: *GspiBus, v: Variant) void {
        self.variant = v;
    }

    // -- power and pin setup ------------------------------------------------

    /// `cyw43_spi_gpio_setup` followed by `cyw43_spi_reset`, then the PIO
    /// bring-up of `cyw43_spi_init`.
    ///
    /// **The reset is an off/on cycle with a 250 ms settle, not a bare
    /// assert.** The predecessor drove `WL_REG_ON` high and waited 20 ms. A
    /// chip that was already powered therefore never reset at all, and 20 ms is
    /// an order of magnitude short of what the part needs to bring its internal
    /// regulators and 32 kHz oscillator up — so a cold boot raced the first
    /// register read.
    pub fn powerOn(self: *GspiBus) void {
        gpio.initOutput(self.pins.wl_reg_on);
        gpio.initOutput(self.pins.wl_cs);
        gpio.set(self.pins.wl_cs);
        gpio.initOutput(self.pins.wl_data);
        gpio.clear(self.pins.wl_data);

        gpio.clear(self.pins.wl_reg_on);
        timer.delayMs(20);
        gpio.set(self.pins.wl_reg_on);
        timer.delayMs(250);

        self.initPio();
        self.powered = true;
        self.cur_window = 0;
    }

    /// Millisecond delay, exposed through the bus rather than reached for
    /// directly by the driver above.
    ///
    /// **This is a seam, not a convenience wrapper.** The driver's retry loops
    /// need to wait; reaching for `rp2350.timer` from `cyw43.zig` would make
    /// that file un-runnable on a host, and the usual fix for that — branching
    /// on `builtin.target.os.tag` inside the driver — is precisely the pattern
    /// that let the predecessor's transport return a hardcoded chip ID to its
    /// own tests. Routing the delay through the injected bus keeps one code
    /// path on both targets.
    pub fn delayMs(_: *GspiBus, ms: u32) void {
        timer.delayMs(ms);
    }

    // -- host wake ----------------------------------------------------------
    //
    // **The chip's interrupt arrives on the data pin.** `WL_D` carries three
    // signals weighted by resistance — the host's drive direct, the chip's
    // data through 470R, the chip's IRQ through 10K — so the interrupt can
    // only move the line when nothing else is driving it, which is to say
    // when CS is high. The resistors are the only arbitration there is.
    //
    // Two facts from the RP2350 datasheet make the rest follow:
    //
    //  * **§9.5: the pad input is always connected**, whatever FUNCSEL says.
    //    So GP24 raises IO_BANK0 interrupts while the PIO owns it — including
    //    on every data bit that happens to be high. **Masking around a
    //    transfer is therefore mandatory, not a refinement.**
    //  * **Level interrupts are not latched.** Masking cannot lose one: if the
    //    chip still wants attention the level is still high, and re-arming
    //    fires immediately. That is what makes mask-and-restore safe here and
    //    would not be true of an edge trigger.

    fn setHostWakeIrq(self: *GspiBus, on: bool) void {
        gpio.setIrqEnabled(self.pins.wl_data, .level_high, on);
    }

    /// Start delivering host-wake interrupts.
    ///
    /// **This does not touch the pin's function or the NVIC.** The pad belongs
    /// to the PIO and must stay that way; the caller enables
    /// `nvic.IrqNum.io_bank0` and provides the handler, because a library that
    /// installed a vector would be claiming an interrupt line the application
    /// may share with other pins.
    pub fn enableHostWake(self: *GspiBus) void {
        self.host_wake_enabled = true;
        @atomicStore(bool, &self.host_wake_pending, false, .release);
        self.setHostWakeIrq(true);
    }

    pub fn disableHostWake(self: *GspiBus) void {
        self.host_wake_enabled = false;
        self.setHostWakeIrq(false);
    }

    /// **Call this from the application's `io_bank0` handler and nothing
    /// else**, and note what it does *not* do: it does not touch the bus.
    ///
    /// A level-high interrupt whose pin is still high re-fires the instant the
    /// handler returns, and the only way to quiet it is to service the chip —
    /// which needs the bus, which is the one thing an ISR sharing a bus with
    /// the application must not take. So the handler masks and records, and
    /// the work happens where the bus is safe.
    pub fn onHostWakeIrq(self: *GspiBus) void {
        self.setHostWakeIrq(false);
        @atomicStore(bool, &self.host_wake_pending, true, .release);
    }

    /// Read and clear the flag. True means the chip asked for attention since
    /// the last call.
    pub fn takeHostWake(self: *GspiBus) bool {
        return @atomicRmw(bool, &self.host_wake_pending, .Xchg, false, .acq_rel);
    }

    /// Re-arm after servicing. Safe to call when nothing was pending.
    pub fn rearmHostWake(self: *GspiBus) void {
        if (self.host_wake_enabled) self.setHostWakeIrq(true);
    }

    /// The clock seam, and it is here for the same reason `delayMs` is: a
    /// stack above this bus must be able to ask what time it is without
    /// reaching for `rp2350.timer` and becoming un-runnable on a host.
    ///
    /// `now64` counts microseconds since boot and is 64-bit, so the
    /// millisecond value does not wrap in any run this hardware will see.
    pub fn nowMs(_: *GspiBus) u64 {
        return timer.now64() / 1000;
    }

    pub fn powerOff(self: *GspiBus) void {
        pio.setEnabled(self.block, self.sm, false);
        gpio.initOutput(self.pins.wl_cs);
        gpio.set(self.pins.wl_cs);
        gpio.initOutput(self.pins.wl_reg_on);
        gpio.clear(self.pins.wl_reg_on);
        self.powered = false;
    }

    fn initPio(self: *GspiBus) void {
        pio.init(self.block);
        pio.loadProgramRelocated(self.block, self.offset, self.variant.program());

        // The clock pad carries the only edges the chip times against, so it
        // gets both halves of the pad treatment: 12 mA so the edge is stiff
        // under load, fast slew so it is square. Drive strength alone leaves a
        // shallow edge, and a shallow edge on a fast bus is sampled late.
        gpio.initFunctionOutput(self.pins.wl_clk, self.block.funcsel());
        gpio.setDriveStrength(self.pins.wl_clk, .ma_12);
        gpio.setSlewFast(self.pins.wl_clk, true);

        // The data pin is read back on the same wire it was driven from, so it
        // needs its input buffer on and a pull-down to hold it defined during
        // the turnaround while neither end drives. Schmitt for the same reason.
        gpio.initFunctionInput(self.pins.wl_data, self.block.funcsel(), .down);

        // Two clk_sys cycles of synchroniser latency is a large fraction of a
        // bit period at this divisor, and the sample lands on the clock edge
        // rather than after it. This is the one setting with no visible symptom
        // other than "reads are wrong".
        pio.setInputSyncBypass(self.block, @as(u32, 1) << @truncate(self.pins.wl_data));

        pio.configure(self.block, self.sm, self.smConfig(self.variant.end()));
        pio.setConsecutivePindirs(self.block, self.sm, @truncate(self.pins.wl_clk), 1, true);
    }

    fn smConfig(self: GspiBus, wrap_at: u5) pio.SmConfig {
        return .{
            .wrap_bottom = self.offset,
            .wrap_top = self.offset + wrap_at - 1,

            .out_base = @truncate(self.pins.wl_data),
            .out_count = 1,
            .set_base = @truncate(self.pins.wl_data),
            .set_count = 1,
            .in_base = @truncate(self.pins.wl_data),
            .sideset_base = @truncate(self.pins.wl_clk),
            .sideset_count = 1,

            // MSB-first both ways: the wire is big-endian within each 32-bit
            // word, whatever the chip's "endian" bit is called.
            .out_shiftdir = .left,
            .in_shiftdir = .left,
            .autopull = true,
            .autopush = true,
            .pull_threshold = 32,
            .push_threshold = 32,

            .clkdiv = pio.clkdivInt(self.clock_div),
        };
    }

    // -- the transfer primitive ---------------------------------------------

    /// One gSPI transaction: shift out every word of `tx`, then shift in
    /// `rx.len` words.
    ///
    /// Words are FIFO words — already in wire order, high byte first. The
    /// conversions from native values and from byte buffers are the three lines
    /// in this file's header comment, and the helpers below are the only things
    /// that should be doing them.
    ///
    /// Mirrors `cyw43_spi_transfer`, including the two details that are easy to
    /// drop: the wrap is moved to `lp1_end` for a write-only transfer so the
    /// machine cannot fall into the receive half, and a write-only transfer
    /// waits on the **sticky** TX-stall flag before releasing the pin, because
    /// an empty FIFO does not mean the last word has finished shifting.
    pub fn transfer(self: *GspiBus, tx: []const u32, rx: []u32) Error!void {
        try self.startTransfer(tx, rx.len);

        if (rx.len > 0) {
            // Receive cannot begin until the whole transmit loop has drained,
            // so the four-deep RX FIFO cannot overflow behind us here.
            for (rx) |*w| w.* = try pio.read(self.block, self.sm);
        } else {
            self.finishWriteOnly();
        }

        self.endTransfer();
    }

    /// Receive straight into a byte buffer, big-endian per word, discarding
    /// `skip_words` of response padding first.
    ///
    /// **The byte swap happens inside the receive loop, and that is the whole
    /// point.** The loop is already stalled on `rxEmpty` waiting for the wire —
    /// at 37.5 MHz a word lands every 853 ns, which is 128 clk_sys cycles at
    /// 150 MHz — so anything the CPU does between reads is free until it
    /// exceeds that. Doing it afterwards instead is not free: it is a second
    /// serial pass over the data with the bus idle.
    ///
    /// That is exactly what `readBytes` used to do, and it cost **172,864 ns
    /// per kilobyte** — a term rung C measured, fitted, and could not make go
    /// away by raising the bus rate, because none of it was on the wire. Per
    /// word it was a 4-byte stack temporary, a `writeInt`, a runtime `@min` and
    /// a **runtime-length `@memcpy`**, which at this package's default
    /// `ReleaseSmall` is a real call. The partial-word tail is why that length
    /// was runtime, and handling the tail once outside the loop is what makes
    /// the fast path a plain aligned store.
    ///
    /// **`rx_stalls` counts, and does not fail.** The loop keeping up is a bet
    /// about cycle counts, so it is worth measuring — but losing that bet is
    /// not a fault. `in` with autopush stalls the state machine on a full FIFO
    /// rather than dropping data, side-set holds the clock through the stall,
    /// and a synchronous bus just sees a stretched cycle. See `pio.rxStalled`.
    ///
    /// This was briefly a fatal `ReceiveOverrun` error, on a misreading of the
    /// flag. It stopped `app/bt-speaker` at bring-up on a bus that was working
    /// perfectly, which is a more expensive failure than the one it was
    /// guarding against.
    pub fn transferInto(self: *GspiBus, tx: []const u32, skip_words: usize, buf: []u8) Error!void {
        const words = (buf.len + 3) / 4;
        try self.startTransfer(tx, skip_words + words);

        // **Resolve the FIFO addresses once.** `pio.read` recomputes them per
        // call, and at `ReleaseSmall` that is ~25 instructions a word rather
        // than the two APB reads the job needs — see `pio.RxPort`. This loop is
        // the one place in the driver where that matters, because it is the
        // only one racing the wire.
        const rx = pio.RxPort.of(self.block, self.sm);

        for (0..skip_words) |_| _ = try rx.read();

        // Whole words: a 32-bit byte-reversed store into an aligned slot.
        var i: usize = 0;
        while (i + 4 <= buf.len) : (i += 4) {
            std.mem.writeInt(u32, buf[i..][0..4], try rx.read(), .big);
        }

        // The tail, once. A block is only ever a partial word at its very end.
        if (i < buf.len) {
            var w: [4]u8 = undefined;
            std.mem.writeInt(u32, &w, try rx.read(), .big);
            @memcpy(buf[i..], w[0 .. buf.len - i]);
        }

        if (pio.rxStalled(self.block, self.sm)) self.rx_stalls +%= 1;
        self.endTransfer();
    }

    /// Configure, seed the loop counters, drop CS, start, and push `tx`.
    fn startTransfer(self: *GspiBus, tx: []const u32, rx_words: usize) Error!void {
        // Mandatory, not tidiness: the pad input stays connected under PIO, so
        // every high data bit would otherwise raise the level-high host-wake
        // interrupt. Restored in `endTransfer`.
        if (self.host_wake_enabled) self.setHostWakeIrq(false);
        if (!self.powered) return error.NotPowered;
        if (tx.len == 0) return error.BadTransferLength;

        const rx_only_words = rx_words;
        const wrap_at: u5 = if (rx_only_words > 0) self.variant.end() else self.variant.lp1End();

        pio.setEnabled(self.block, self.sm, false);
        pio.configure(self.block, self.sm, self.smConfig(wrap_at));
        pio.clearFifos(self.block, self.sm);
        pio.clearFifoDebug(self.block, self.sm);
        pio.setConsecutivePindirs(self.block, self.sm, @truncate(self.pins.wl_data), 1, true);
        pio.restart(self.block, self.sm);

        // Seed the two loop counters through the FIFO. `out x, 32` auto-pulls,
        // so the push must come first — an `out` on an empty FIFO stalls here
        // with no diagnostic at all.
        // FIFOs were just cleared and the machine is disabled, so these two
        // cannot block. The `tx` loop below can, and is bounded.
        pio.writeUnchecked(self.block, self.sm, @intCast(tx.len * 32 - 1));
        pio.exec(self.block, self.sm, pio.out(.x, 32, 0));
        pio.writeUnchecked(self.block, self.sm, if (rx_only_words > 0) @intCast(rx_only_words * 32 - 1) else 0);
        pio.exec(self.block, self.sm, pio.out(.y, 32, 0));

        pio.jumpTo(self.block, self.sm, self.offset);

        gpio.clear(self.pins.wl_cs);
        pio.setEnabled(self.block, self.sm, true);

        // **Bounded since 2026-08-25.** This is the wait the 2026-08-22 hang
        // was sitting in: a machine that stops consuming fills the FIFO and
        // this used to spin forever, leaving a powered board and no diagnostic.
        for (tx) |w| try pio.write(self.block, self.sm, w);
    }

    /// The write-only tail: wait for the last word to leave, then release the
    /// data pin.
    ///
    /// **Clear the stall flag here, after the last word is in the FIFO, and
    /// only then wait for it.** It is sticky. Clearing it at the top of the
    /// transfer and waiting here is the obvious arrangement and it is wrong:
    /// the state machine briefly outruns the CPU while the words are being
    /// pushed, which sets the flag, so the wait returns immediately and CS
    /// rises before the final word has been shifted out. The chip discards the
    /// truncated transaction.
    ///
    /// The effect was that **every ordinary write was silently dropped** while
    /// reads worked perfectly — a read's single command word is followed by the
    /// receive phase, which keeps the machine clocking, so it never depended on
    /// this. Measured directly: writing 0x10 to SPI_RESP_DELAY_F1 left it
    /// reading 0x04, and writing 0x55 to a scratch register left it reading
    /// 0x00.
    fn finishWriteOnly(self: *GspiBus) void {
        pio.clearFifoDebug(self.block, self.sm);
        while (!pio.txStalled(self.block, self.sm)) {}
        pio.setEnabled(self.block, self.sm, false);
        pio.setConsecutivePindirs(self.block, self.sm, @truncate(self.pins.wl_data), 1, false);
    }

    /// Park the output low, raise CS, and let the host-wake line settle.
    fn endTransfer(self: *GspiBus) void {
        // Leave the output register low so the next transfer starts from a
        // defined level the moment the pin turns around.
        pio.exec(self.block, self.sm, pio.mov(.pins, .none, .null_, 0));

        gpio.set(self.pins.wl_cs);
        // The data pin doubles as the host-wake interrupt, and a positive CS
        // edge can latch a spurious one. The SDK waits 100 ns here; a 1 µs
        // delay is the finest this timer offers and is comfortably enough.
        //
        // **That delay is now load-bearing rather than defensive**: the line
        // is being released to a 10K pull and has to settle before the level
        // is sampled, or the last data bit reads as an interrupt.
        timer.delayUs(1);

        // Re-arm, unless the ISR has already flagged something nobody has
        // collected — re-entering the handler for a wake already recorded is
        // just work.
        if (self.host_wake_enabled and !@atomicLoad(bool, &self.host_wake_pending, .acquire)) {
            self.setHostWakeIrq(true);
        }
    }

    // -- register access ----------------------------------------------------

    /// Read a 32-bit register in the chip's **boot-time 16-bit mode**.
    ///
    /// Only valid before the `bus_control` handshake, and only on Function 0 —
    /// the backplane needs response padding this path does not send.
    pub fn readReg32Swap(self: *GspiBus, fn_num: Function, addr: u17) Error!u32 {
        std.debug.assert(fn_num != .backplane);
        const tx = [_]u32{swapHalfwords(makeCmd(false, true, fn_num, addr, 4))};
        var rx = [_]u32{0};
        try self.transfer(&tx, &rx);
        return swapHalfwords(rx[0]);
    }

    /// Write a 32-bit register in the chip's boot-time 16-bit mode. This exists
    /// to send exactly one thing: the handshake that ends 16-bit mode.
    pub fn writeReg32Swap(self: *GspiBus, fn_num: Function, addr: u17, val: u32) Error!void {
        const tx = [_]u32{
            swapHalfwords(makeCmd(true, true, fn_num, addr, 4)),
            swapHalfwords(val),
        };
        try self.transfer(&tx, &.{});
    }

    /// Words of response delay clocked ahead of read data for `fn_num`.
    pub fn padWords(self: GspiBus, fn_num: Function) usize {
        return if (fn_num == .backplane) self.resp_delay_bytes / 4 else 0;
    }

    fn readRegSized(self: *GspiBus, fn_num: Function, addr: u17, size: u11) Error!u32 {
        const pad_words = self.padWords(fn_num);
        const tx = [_]u32{@byteSwap(makeCmd(false, true, fn_num, addr, size))};

        var rx_buf: [1 + max_read_pad_bytes / 4]u32 = undefined;
        const rx = rx_buf[0 .. pad_words + 1];
        try self.transfer(&tx, rx);

        // The padding is clocked first; the register value is the word after it.
        const v = @byteSwap(rx[pad_words]);
        traceReg(switch (size) {
            1 => "read_reg_u8",
            2 => "read_reg_u16",
            else => "read_reg_u32",
        }, fn_num, addr, v);
        return v;
    }

    pub fn readReg32(self: *GspiBus, fn_num: Function, addr: u17) Error!u32 {
        return self.readRegSized(fn_num, addr, 4);
    }

    pub fn readReg16(self: *GspiBus, fn_num: Function, addr: u17) Error!u16 {
        return @truncate(try self.readRegSized(fn_num, addr, 2));
    }

    pub fn readReg8(self: *GspiBus, fn_num: Function, addr: u17) Error!u8 {
        return @truncate(try self.readRegSized(fn_num, addr, 1));
    }

    fn writeRegSized(self: *GspiBus, fn_num: Function, addr: u17, val: u32, size: u11) Error!void {
        const tx = [_]u32{
            @byteSwap(makeCmd(true, true, fn_num, addr, size)),
            @byteSwap(val),
        };
        traceReg(switch (size) {
            1 => "write_reg_u8",
            2 => "write_reg_u16",
            else => "write_reg_u32",
        }, fn_num, addr, val);
        try self.transfer(&tx, &.{});
    }

    pub fn writeReg32(self: *GspiBus, fn_num: Function, addr: u17, val: u32) Error!void {
        return self.writeRegSized(fn_num, addr, val, 4);
    }

    pub fn writeReg16(self: *GspiBus, fn_num: Function, addr: u17, val: u16) Error!void {
        return self.writeRegSized(fn_num, addr, val, 2);
    }

    pub fn writeReg8(self: *GspiBus, fn_num: Function, addr: u17, val: u8) Error!void {
        return self.writeRegSized(fn_num, addr, val, 1);
    }

    // -- block transfers ----------------------------------------------------

    /// Longest payload one command header can describe: the `len` field is 11
    /// bits, counted in bytes.
    pub const max_block_bytes: usize = 0x7FF;

    /// Write a byte buffer to a function. `data` is padded up to a word
    /// boundary on the wire; the header still describes its true length.
    /// In RAM for the same reason as `readBytes`, and measured the same way:
    /// with only the read path moved, bt-speaker reached 18.75 MHz; with this
    /// one moved too, 25 MHz. See `readBytes`.
    pub fn writeBytes(self: *GspiBus, fn_num: Function, addr: u17, data: []const u8) linksection(".ramfunc") Error!void {
        if (data.len == 0 or data.len > max_block_bytes) return error.BadTransferLength;

        var tx_buf: [1 + (max_block_bytes + 3) / 4]u32 = undefined;
        const words = (data.len + 3) / 4;
        tx_buf[0] = @byteSwap(makeCmd(true, true, fn_num, addr, @intCast(data.len)));

        var i: usize = 0;
        while (i < words) : (i += 1) {
            var w: [4]u8 = .{ 0, 0, 0, 0 };
            const remaining = data.len - i * 4;
            const n = @min(@as(usize, 4), remaining);
            @memcpy(w[0..n], data[i * 4 ..][0..n]);
            // Wire order is memory order; a big-endian read is what puts the
            // first byte in the FIFO word's high byte, which goes out first.
            tx_buf[1 + i] = std.mem.readInt(u32, &w, .big);
        }

        try self.transfer(tx_buf[0 .. 1 + words], &.{});
    }

    /// Read a byte buffer from a function, inserting the Function 1 response
    /// padding when one is required.
    /// **In RAM, and that is not a micro-optimisation — it is the difference
    /// between this working and not.**
    ///
    /// Measured 2026-08-24. `bt-throughput` ran a 37.5 MHz bus clean over 20
    /// boots; `app/bt-speaker`, on the same driver code and the same board,
    /// corrupted its backplane self-test at anything above 12.5 MHz. Two
    /// binaries, one bus, opposite outcomes — and the bring-up sequences are
    /// identical line for line. What differs is that bt-speaker is roughly 1.5x
    /// the size, so this loop is evicted from the XIP cache between transfers
    /// and refetched from flash while the wire is running.
    ///
    /// Moving it here took bt-speaker from **12.5 MHz to 25 MHz**, and moved
    /// its 37.5 MHz failure from bring-up (rung 5) to the HCI stage (rung 21).
    /// Nothing else changed. `.text` placement was eliminated first by proving
    /// the optimize mode innocent — `ReleaseFast`, which is what bt-speaker
    /// builds this package at, passes in the rig at 37.5 MHz — and interrupts
    /// by there being none in either application.
    ///
    /// **A `linksection` on `transferInto` does nothing**: it is inlined here,
    /// and the first attempt at this test was a silent no-op that read as
    /// disproof. Check the symbol table, not the source.
    ///
    /// Costs about 1 KB of SRAM alongside `writeBytes`.
    pub fn readBytes(self: *GspiBus, fn_num: Function, addr: u17, buf: []u8) linksection(".ramfunc") Error!void {
        if (buf.len == 0 or buf.len > max_block_bytes) return error.BadTransferLength;

        const pad_words = self.padWords(fn_num);
        const tx = [_]u32{@byteSwap(makeCmd(false, true, fn_num, addr, @intCast(buf.len)))};
        try self.transferInto(&tx, pad_words, buf);
    }

    // -- backplane windowing ------------------------------------------------

    /// Point the backplane's 32 KB window at `addr`.
    pub fn setBackplaneWindow(self: *GspiBus, addr: u32) Error!void {
        const window = addr & ~@as(u32, 0x7FFF);
        if (window == self.cur_window) return;

        // **High, then mid, then low — the order is load-bearing.** The window
        // is committed when the low byte is written, so writing low first
        // latches a window whose upper bytes are still the previous value, and
        // every subsequent access lands wherever the chip was last pointed.
        // The symptom is unmistakable once you know it: reads return a
        // plausible constant (whatever lives at the stale window) for every
        // address you ask for. The SDK writes them in this order and only
        // writes the bytes that changed.
        try self.writeReg8(.backplane, 0x1000C, @truncate(window >> 24));
        try self.writeReg8(.backplane, 0x1000B, @truncate(window >> 16));
        try self.writeReg8(.backplane, 0x1000A, @truncate(window >> 8));

        self.cur_window = window;
    }

    /// Restore the window to chipcommon, as the SDK does after every
    /// register-level backplane access.
    ///
    /// **This is not tidiness — the read does not work without it.** Measured:
    /// thirty-two consecutive reads of chipcommon's chip-ID register through a
    /// cached window returned `DATA_UNAVAILABLE` every time, and the very next
    /// read, preceded by a window write, returned the correct value. A windowed
    /// access appears to need the window registers freshly written; restoring
    /// to chipcommon afterwards guarantees the next access to anything else
    /// rewrites them.
    ///
    /// `cyw43_read_backplane` and `cyw43_write_backplane` both end with
    /// `cyw43_set_backplane_window(self, CHIPCOMMON_BASE_ADDRESS)`.
    /// `cyw43_download_resource` deliberately does **not** — it drives the
    /// window itself per block — which is why the chunked paths below do not
    /// call this.
    const chipcommon_window: u32 = 0x1800_0000;

    fn restoreWindow(self: *GspiBus) Error!void {
        try self.setBackplaneWindow(chipcommon_window);
    }

    /// Read one register at a full 32-bit backplane address.
    ///
    /// The backplane is reached through a 32 KB window, so an address must be
    /// split: the window register takes the high bits, and the command carries
    /// the low 15 with `0x8000` set to select 32-bit access. Callers that pass
    /// a raw backplane address straight to `readReg32` address the wrong thing
    /// entirely — silently, because the bus still answers.
    pub fn readBackplaneReg32(self: *GspiBus, addr: u32) Error!u32 {
        try self.setBackplaneWindow(addr);
        const v = try self.readReg32(.backplane, windowed(addr));
        try self.restoreWindow();
        return v;
    }

    pub fn readBackplaneReg8(self: *GspiBus, addr: u32) Error!u8 {
        try self.setBackplaneWindow(addr);
        const v = try self.readReg8(.backplane, windowed(addr));
        try self.restoreWindow();
        return v;
    }

    pub fn writeBackplaneReg32(self: *GspiBus, addr: u32, val: u32) Error!void {
        try self.setBackplaneWindow(addr);
        try self.writeReg32(.backplane, windowed(addr), val);
        try self.restoreWindow();
    }

    pub fn writeBackplaneReg8(self: *GspiBus, addr: u32, val: u8) Error!void {
        try self.setBackplaneWindow(addr);
        try self.writeReg8(.backplane, windowed(addr), val);
        try self.restoreWindow();
    }

    fn windowed(addr: u32) u17 {
        return @intCast((addr & 0x7FFF) | 0x8000);
    }

    /// Write to a backplane address, re-windowing and chunking as needed.
    ///
    /// Lengths are padded to a **4-byte** boundary, which `writeBytes` already
    /// does on the wire. `CYW43_WRITE_BYTES_PAD` is `ALIGN(len, 4)` on SPI —
    /// the 64-byte figure belongs to SDIO, and briefly lived here by mistake.
    ///
    /// The 0x8000 flag on the windowed address selects 32-bit backplane access.
    pub fn writeBackplaneChunked(self: *GspiBus, addr: u32, data: []const u8) Error!void {
        const chunk_max: usize = 64;
        var offset: usize = 0;
        while (offset < data.len) {
            const target = addr + @as(u32, @intCast(offset));
            try self.setBackplaneWindow(target);

            // Never span the window boundary inside one command.
            const to_window_end = 0x8000 - (target & 0x7FFF);
            const n = @min(@min(chunk_max, data.len - offset), to_window_end);

            try self.writeBytes(.backplane, windowed(target), data[offset .. offset + n]);
            offset += n;
        }
    }

    /// Read from a backplane address, re-windowing and chunking as needed.
    ///
    /// **Block reads do not carry the 4-byte access flag, and block writes do.**
    /// That asymmetry is not a guess: `cyw43_download_resource` sets
    /// `SBSDIO_SB_ACCESS_2_4B_FLAG` on the address it writes and then, three
    /// lines later, reads the same block back with a bare
    /// `dest_addr & BACKPLANE_ADDR_MASK`. Setting it on a read returns
    /// convincing garbage rather than an error.
    pub fn readBackplaneChunked(self: *GspiBus, addr: u32, buf: []u8) Error!void {
        const chunk_max: usize = 64;
        var offset: usize = 0;
        while (offset < buf.len) {
            const target = addr + @as(u32, @intCast(offset));
            try self.setBackplaneWindow(target);

            const to_window_end = 0x8000 - (target & 0x7FFF);
            const n = @min(@min(chunk_max, buf.len - offset), to_window_end);

            try self.readBytes(.backplane, @intCast(target & 0x7FFF), buf[offset .. offset + n]);
            offset += n;
        }
    }
};

// ---------------------------------------------------------------------------
// Host unit tests
//
// These assert bytes on the wire, not that a function returns what it returns.
// The predecessor's tests checked `packCmd` against a constant computed the
// same way `packCmd` computed it, and passed for as long as it was wrong.
// ---------------------------------------------------------------------------

const testing = std.testing;

test "Header.pack matches make_cmd's bit layout" {
    // write=1, inc=1, fn=1, addr=0x1000, len=4
    //   1<<31 | 1<<30 | 1<<28 | 0x1000<<11 | 4
    try testing.expectEqual(
        @as(u32, 0xD080_0004),
        makeCmd(true, true, .backplane, 0x1000, 4),
    );

    // The read that proves the chip is alive: fn=0, addr=0x0014, len=4.
    try testing.expectEqual(
        @as(u32, 0x4000_A004),
        makeCmd(false, true, .bus, reg.read_test, 4),
    );
}

test "inc is bit 30 and does not collide with addr" {
    const with = makeCmd(false, true, .bus, 0x1FFFF, 0);
    const without = makeCmd(false, false, .bus, 0x1FFFF, 0);

    // The only difference is bit 30 — the whole point, given the predecessor
    // put `inc` at bit 27 where the top of a 17-bit address already lives.
    try testing.expectEqual(@as(u32, 1) << 30, with ^ without);
    // A full-width address occupies 11..27 and nothing above it.
    try testing.expectEqual(@as(u32, 0x0FFF_F800), without);
}

test "function number occupies bits 28..29" {
    try testing.expectEqual(@as(u32, 0) << 28, makeCmd(false, false, .bus, 0, 0));
    try testing.expectEqual(@as(u32, 1) << 28, makeCmd(false, false, .backplane, 0, 0));
    try testing.expectEqual(@as(u32, 2) << 28, makeCmd(false, false, .wlan_or_bt, 0, 0));
}

test "swapHalfwords is SWAP32 with the DMA byte-swap folded in" {
    // rev16 then byte-swap, spelled out the long way, must equal one halfword
    // swap. This is the identity the boot-mode path rests on.
    const v: u32 = 0xAABB_CCDD;
    const rev16 = ((v & 0xFF00_0000) >> 8) | ((v & 0x00FF_0000) << 8) |
        ((v & 0x0000_FF00) >> 8) | ((v & 0x0000_00FF) << 8);
    try testing.expectEqual(@byteSwap(rev16), swapHalfwords(v));
    try testing.expectEqual(@as(u32, 0xCCDD_AABB), swapHalfwords(v));
}

test "boot-mode test-register read puts A0 04 40 00 on the wire" {
    // Ground truth from the SDK: buf[0] = SWAP32(make_cmd(false, true, BUS,
    // 0x0014, 4)) = 0x004004A0, whose memory bytes — and therefore whose wire
    // bytes — are A0 04 40 00.
    const fifo_word = swapHalfwords(makeCmd(false, true, .bus, reg.read_test, 4));
    var wire: [4]u8 = undefined;
    std.mem.writeInt(u32, &wire, fifo_word, .big);
    try testing.expectEqualSlices(u8, &.{ 0xA0, 0x04, 0x40, 0x00 }, &wire);
}

test "the chip's boot-mode reply for FEEDBEAD decodes back to FEEDBEAD" {
    // The chip sends the halves low-first, each big-endian: BE AD FE ED.
    const received_wire = [_]u8{ 0xBE, 0xAD, 0xFE, 0xED };
    const fifo_word = std.mem.readInt(u32, &received_wire, .big);
    try testing.expectEqual(test_pattern, swapHalfwords(fifo_word));
}

test "bus control word is 0x000204B3, not 0x8B" {
    try testing.expectEqual(@as(u32, 0x0002_04B3), bus_control_word);

    // Spelled out field by field, so a changed constant fails here rather than
    // silently on hardware.
    try testing.expectEqual(@as(u32, 0xB3), bus_control_word & 0xFF);
    try testing.expectEqual(@as(u32, 0x04), (bus_control_word >> 8) & 0xFF);
    try testing.expectEqual(@as(u32, 0x02), (bus_control_word >> 16) & 0xFF);
}

test "the handshake transfer is two words, both halfword-swapped" {
    const cmd = swapHalfwords(makeCmd(true, true, .bus, reg.bus_control, 4));
    const val = swapHalfwords(bus_control_word);

    var wire: [8]u8 = undefined;
    std.mem.writeInt(u32, wire[0..4], cmd, .big);
    std.mem.writeInt(u32, wire[4..8], val, .big);

    // make_cmd = 0xC0000004 -> rev16 0x00C00400 -> memory/wire 00 04 C0 00
    // value    = 0x000204B3 -> rev16 0x0200B304 -> memory/wire 04 B3 00 02
    try testing.expectEqualSlices(u8, &.{
        0x00, 0x04, 0xC0, 0x00,
        0x04, 0xB3, 0x00, 0x02,
    }, &wire);
}

test "PIO program encodes to the pioasm output for spi_gap01_sample0" {
    // Hand-checked against the RP2350 instruction encoding, §11.5.
    // Side-set is 1 bit with no delay, so bits 12:8 are 0b0S000.
    try testing.expectEqual(@as(u16, 0x6001), gap01_sample0[0]); // out pins, 1   side 0
    try testing.expectEqual(@as(u16, 0x1040), gap01_sample0[1]); // jmp x-- 0     side 1
    try testing.expectEqual(@as(u16, 0xE080), gap01_sample0[2]); // set pindirs,0 side 0
    try testing.expectEqual(@as(u16, 0xB042), gap01_sample0[3]); // nop           side 1
    try testing.expectEqual(@as(u16, 0x4001), gap01_sample0[4]); // in pins, 1    side 0
    try testing.expectEqual(@as(u16, 0x1084), gap01_sample0[5]); // jmp y-- 4     side 1

    try testing.expectEqual(@as(u5, 6), Variant.gap01_sample0.end());
}

test "write-only transfers wrap before the receive half" {
    // If these two are ever equal, a write-only transfer falls through into
    // `in pins` and clocks 32 bits nobody asked for, hanging the read that
    // follows it.
    for ([_]Variant{ .gap01_sample0, .gap0_sample1, .gap010_sample1 }) |v| {
        try testing.expect(v.lp1End() < v.end());
        // Every variant releases the data pin with the same instruction at the
        // same index; only what follows it differs.
        try testing.expectEqual(pio.set(.pindirs, 0, pio.sideSet(1, 0, 0)), v.program()[v.lp1End()]);
    }
}

test "backplane reads pad by 16 bytes, other functions by none" {
    // The value written into resp_delay_f1 is 4; the padding clocked ahead of
    // the data is 16. Conflating them is a 12-byte read offset.
    try testing.expectEqual(@as(usize, 16), backplane_read_pad_bytes);
    try testing.expectEqual(@as(u32, 0x04), (bus_control_word >> 8) & 0xFF);
}

test "the crystal-clocked divisor is unchanged, so proven rungs stay proven" {
    var bus = GspiBus.init(.{});
    // The default, before anyone says otherwise: the SDK's constant.
    try testing.expectEqual(@as(u32, 2), bus.clock_div);

    // And a board explicitly saying "12 MHz crystal" must land on exactly the
    // same number. If this ever differs, every rung measured at 3 MHz was
    // measured on a bus this code no longer produces.
    bus.setSysHz(12_000_000);
    try testing.expectEqual(@as(u32, 2), bus.clock_div);
    try testing.expectEqual(@as(u32, 3_000_000), bus.busHz(12_000_000));
}

test "raising clk_sys holds the bus at its proven rate" {
    var bus = GspiBus.init(.{});
    bus.setSysHz(150_000_000);
    try testing.expectEqual(@as(u32, 25), bus.clock_div);
    try testing.expectEqual(proven_bus_hz, bus.busHz(150_000_000));

    // **The assertion that fails against the old code**: a bare `clock_div = 2`
    // at 150 MHz is a 37.5 MHz bus, which is what stopped the chip answering.
    try testing.expect(bus.busHz(150_000_000) != 150_000_000 / 4);
}

test "the divisor never reaches zero, however slow the clock" {
    var bus = GspiBus.init(.{});
    bus.setSysHz(1_000_000);
    try testing.expect(bus.clock_div >= 1);
}

test "the handshake word is unchanged when nobody asks for anything else" {
    // Every rung through 27 sent 0x000204B3. `busControlWord` must reproduce it
    // exactly for the default, or the constant this replaced was not a constant.
    try testing.expectEqual(@as(u32, 0x0002_04B3), bus_control_word);
    try testing.expectEqual(bus_control_word, busControlWord(true));

    // Normal mode differs in exactly one bit, and it is bit 4.
    try testing.expectEqual(bus_control_word & ~bus_ctrl.high_speed_mode, busControlWord(false));
    try testing.expectEqual(@as(u32, 0x10), bus_control_word ^ busControlWord(false));

    // A fresh bus asks for high-speed mode, as it always has.
    try testing.expect(GspiBus.init(.{}).high_speed);
}

test "the four programs are a 2x2 grid of bit offset against sample phase" {
    // The grid is the whole point of having four: two axes that mean different
    // things, not four points on one. Asserting the corners keeps a later
    // edit from quietly making it a list again.
    try testing.expectEqual(@as(u1, 0), Variant.gap0_sample1.bitOffset());
    try testing.expectEqual(@as(u1, 0), Variant.gap01_sample0.bitOffset());
    try testing.expectEqual(@as(u1, 1), Variant.gap010_sample1.bitOffset());
    try testing.expectEqual(@as(u1, 1), Variant.gap0101_sample0.bitOffset());

    try testing.expect(!Variant.gap0_sample1.samplesLate());
    try testing.expect(Variant.gap01_sample0.samplesLate());
    try testing.expect(!Variant.gap010_sample1.samplesLate());
    try testing.expect(Variant.gap0101_sample0.samplesLate());

    // All four corners are distinct.
    const all = [_]Variant{ .gap0_sample1, .gap01_sample0, .gap010_sample1, .gap0101_sample0 };
    for (all, 0..) |a, i| for (all[i + 1 ..]) |b| {
        try testing.expect(a.bitOffset() != b.bitOffset() or a.samplesLate() != b.samplesLate());
    };
}

test "every program releases the data pin at the same index and ends where it says" {
    // `lp1End` is hardcoded to 2 for all variants, which is only true while
    // every program keeps its transmit loop to instructions 0 and 1. A new
    // variant that broke that would wrap a write-only transfer into the
    // receive half — silently, and only for writes.
    inline for ([_]Variant{ .gap01_sample0, .gap0_sample1, .gap010_sample1, .gap0101_sample0 }) |v| {
        try testing.expectEqual(@as(u5, 2), v.lp1End());
        try testing.expectEqual(@as(u5, @intCast(v.program().len)), v.end());
        try testing.expect(v.end() > v.lp1End());
    }
    try testing.expectEqual(@as(u5, 8), Variant.gap0101_sample0.end());
}

test "the sample lands on the clock level the variant's name claims" {
    // Decode the side-set bit of the `in pins` instruction straight out of the
    // encoded program, rather than trusting the table above it. Side-set data
    // is bit 12 for a 1-bit side-set with no delay cycles.
    const in_opcode: u16 = 0x4000;
    inline for ([_]Variant{ .gap01_sample0, .gap0_sample1, .gap010_sample1, .gap0101_sample0 }) |v| {
        var found = false;
        for (v.program()) |instr| {
            if (instr & 0xE000 != in_opcode) continue;
            const side_high = (instr >> 12) & 1 == 1;
            // Sampling on the high clock is phase 0; on the low clock is +1/2.
            try testing.expectEqual(v.samplesLate(), !side_high);
            found = true;
        }
        try testing.expect(found);
    }
}

test "setBusDiv is exact where setSysHz rounds" {
    var bus = GspiBus.init(.{});

    // clk_sys / 4 — what the SDK runs, and what this work is trying to reach.
    bus.setBusDiv(2);
    try testing.expectEqual(@as(u32, 2), bus.clock_div);
    try testing.expectEqual(@as(u32, 37_500_000), bus.busHz(150_000_000));

    // The whole ladder the sweep walks, at 150 MHz.
    const ladder = [_]struct { div: u32, hz: u32 }{
        .{ .div = 2, .hz = 37_500_000 },
        .{ .div = 3, .hz = 25_000_000 },
        .{ .div = 4, .hz = 18_750_000 },
        .{ .div = 6, .hz = 12_500_000 },
        .{ .div = 8, .hz = 9_375_000 },
        .{ .div = 25, .hz = 3_000_000 },
    };
    for (ladder) |step| {
        bus.setBusDiv(step.div);
        try testing.expectEqual(step.hz, bus.busHz(150_000_000));
    }

    // Every rung of that ladder is inside the part's 50 MHz maximum.
    for (ladder) |step| {
        bus.setBusDiv(step.div);
        try testing.expect(bus.busHz(150_000_000) <= 50_000_000);
    }

    // Zero is not a divisor; it must not become a divide-by-zero downstream.
    bus.setBusDiv(0);
    try testing.expectEqual(@as(u32, 1), bus.clock_div);
}

test "the sweep's slowest rung reproduces the rate every rung was proven at" {
    // div 25 at 150 MHz and div 2 at 12 MHz are the same 3 MHz bus. That is the
    // sweep's internal control: if its slowest cell disagrees with the twenty-one
    // rungs measured on the crystal, the rig is wrong before any cell is read.
    var fast = GspiBus.init(.{});
    fast.setBusDiv(25);
    var slow = GspiBus.init(.{});
    slow.setSysHz(12_000_000);
    try testing.expectEqual(slow.busHz(12_000_000), fast.busHz(150_000_000));
    try testing.expectEqual(proven_bus_hz, fast.busHz(150_000_000));
}

test "the program follows the bus rate, and only where it was measured" {
    // Every rate below is one rung A actually ran. The point of asserting them
    // one by one is that an interpolated threshold would still pass a test
    // written as "low picks early, high picks late".
    try testing.expectEqual(Variant.gap0_sample1, Variant.forBusHz(3_000_000));
    try testing.expectEqual(Variant.gap0_sample1, Variant.forBusHz(9_375_000));
    try testing.expectEqual(Variant.gap0_sample1, Variant.forBusHz(12_500_000));
    try testing.expectEqual(Variant.gap0_sample1, Variant.forBusHz(18_750_000));
    try testing.expectEqual(Variant.gap01_sample0, Variant.forBusHz(25_000_000));
    try testing.expectEqual(Variant.gap01_sample0, Variant.forBusHz(37_500_000));

    // The default divisor and the default variant must agree, or a board that
    // takes both defaults gets a bus that cannot answer.
    const crystal_bus_hz = 12_000_000 / (2 * default_clock_div);
    try testing.expectEqual(GspiBus.init(.{}).variant, Variant.forBusHz(crystal_bus_hz));

    // And the rate this package is proven at end to end picks the same one.
    try testing.expectEqual(GspiBus.init(.{}).variant, Variant.forBusHz(proven_bus_hz));
}

test "receiving into bytes matches the pass it replaced, and consumes the same words" {
    // `transferInto` splits what used to be one per-word loop into a
    // whole-words fast path plus a single tail. Two things must hold for every
    // block length, and the second is the one with teeth: the state machine's
    // `y` counter is seeded from `(len + 3) / 4`, so a decomposition that read
    // one word fewer would leave a stale word in the RX FIFO — and it would
    // surface on the *next* transfer, as corruption with no local cause.
    for (1..65) |len| {
        var wire: [17]u32 = undefined;
        for (&wire, 0..) |*x, k| x.* = 0x1122_3344 +% @as(u32, @intCast(k)) *% 0x0101_0101;

        // The loop this replaced.
        var expect: [64]u8 = undefined;
        const words = (len + 3) / 4;
        var i: usize = 0;
        while (i < words) : (i += 1) {
            var tmp: [4]u8 = undefined;
            std.mem.writeInt(u32, &tmp, wire[i], .big);
            const n = @min(@as(usize, 4), len - i * 4);
            @memcpy(expect[i * 4 ..][0..n], tmp[0..n]);
        }

        // The loop that replaced it.
        var actual: [64]u8 = undefined;
        var taken: usize = 0;
        var j: usize = 0;
        while (j + 4 <= len) : (j += 4) {
            std.mem.writeInt(u32, actual[j..][0..4], wire[taken], .big);
            taken += 1;
        }
        if (j < len) {
            var tmp: [4]u8 = undefined;
            std.mem.writeInt(u32, &tmp, wire[taken], .big);
            @memcpy(actual[j..len], tmp[0 .. len - j]);
            taken += 1;
        }

        try testing.expectEqualSlices(u8, expect[0..len], actual[0..len]);
        try testing.expectEqual(words, taken);
    }
}

test "a block read asks for exactly the words the wire will deliver" {
    // The padding is discarded by reading it, not by skipping it, so the y
    // counter must cover padding *and* payload.
    var bus = GspiBus.init(.{});
    bus.resp_delay_bytes = backplane_read_pad_bytes;
    try testing.expectEqual(@as(usize, 4), bus.padWords(.backplane));
    try testing.expectEqual(@as(usize, 0), bus.padWords(.bus));
    try testing.expectEqual(@as(usize, 0), bus.padWords(.wlan_or_bt));

    // A 64-byte Function 1 block — the largest the chip allows, per datasheet
    // Table 6: "F1 maximum packet size 12'h40". This is why `chunk_max` is 64
    // and cannot be tuned upwards, and why a kilobyte costs 21 words of wire
    // for every 16 of payload: 76% is the structural ceiling, not a transport
    // inefficiency to be optimised away.
    const payload_words = 64 / 4;
    const wire_words = 1 + bus.padWords(.backplane) + payload_words;
    try testing.expectEqual(@as(usize, 21), wire_words);
    try testing.expectEqual(@as(usize, 76), payload_words * 100 / wire_words);
}

test "the crossover threshold sits inside the band where both programs work" {
    // Rung A2 measured phase 0 working to 33.3 MHz and phase +1/2 working from
    // 25, so 25 to 33.3 MHz is a band where either answers. The threshold must
    // land inside it: below the band it would pick a phase measured to fail,
    // above it the same in the other direction.
    const phase0_max_hz: u32 = 33_333_000;
    const phase_half_min_hz: u32 = 25_000_000;
    try testing.expect(Variant.late_phase_min_hz >= phase_half_min_hz);
    try testing.expect(Variant.late_phase_min_hz <= phase0_max_hz);

    // The two rates that bracket it were both run, on two boards.
    try testing.expectEqual(Variant.gap0_sample1, Variant.forBusHz(20_000_000));
    try testing.expectEqual(Variant.gap01_sample0, Variant.forBusHz(33_333_000));

    // And the part's documented maximum picks the phase that was measured to
    // carry it — 50 MHz works on gap01_sample0 and returns 0xFF765F56, the
    // stream shifted one bit, on gap0_sample1.
    try testing.expectEqual(Variant.gap01_sample0, Variant.forBusHz(50_000_000));
}
