//! POWMAN — the always-on power manager, and the timer that can wake the chip.
//!
//! **The block that decides whether a battery device is viable.**
//! A battery firmware on this platform measured 3.48 mA attributable to the clocked chip
//! against a 200-400 uA target, and POWMAN's low-power states are where the
//! rest of that goes. It also owns the **always-on timer**, which is the only
//! thing on this chip that keeps counting while the switched core is powered
//! down, and therefore the only thing that can bring it back unaided.
//!
//! This module deliberately provides the timer and the alarm **before** the
//! power-down request, because in that order each piece can be proven while the
//! chip is still awake and reachable. See `example/powman-check`.
//!
//! ## The password, and why a write can vanish
//!
//! **Every register up to and including offset 0xac is password protected**
//! (§6.4): "Password-protected POWMAN registers require a password (0x5AFE) to
//! be written to the top 16 bits to enable the write operation... Writes to
//! protected registers that don't include the password are ignored, setting a
//! flag in the BADPASSWD register."
//!
//! So a plain `hw.write` to POWMAN does nothing at all, silently — no fault, no
//! error, and a read-back that shows the old value. That is the single easiest
//! way to lose an afternoon here, which is why **nothing in this module writes
//! POWMAN without going through `write`, `setBits` or `clrBits` below**, and why
//! `badPasswd()` is exposed: a non-zero value there is proof that something
//! wrote without the password.
//!
//! Reads deliberately do not return the password, so a naive
//! read-modify-write drops it. `setBits`/`clrBits` handle that.
//!
//! ## The always-on timer
//!
//! A 64-bit millisecond counter fed from one of four sources — LPOSC (the
//! always-on ~32 kHz oscillator), XOSC, or a 1 kHz / 1 Hz GPIO input. LPOSC is
//! the one that survives a power-down, and is therefore the one a wake alarm
//! must run from.
//!
//! **LPOSC is not accurate.** It is an RC oscillator with a trim field, not a
//! crystal, and the datasheet's own frequency fields (`LPOSC_FREQ_KHZ_INT` and
//! `_FRAC`) exist because software is expected to *measure* it rather than
//! assume it. Treat an alarm as "roughly this long", not as timekeeping — that
//! is what `lib/rv3028` and a DS3231 are for.

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// The write password, in the top 16 bits of every protected register.
///
/// **Not optional and not checked by the hardware in any visible way.** A write
/// without it is discarded and counted in `BADPASSWD`; the register keeps its
/// old value and nothing reports an error.
pub const password: u32 = 0x5afe << 16;

/// Registers at or below this offset are password protected (§6.4).
pub const protected_limit: u32 = 0xac;

pub const Reg = struct {
    pub const badpasswd: u32 = hw.powman_base + 0x00;
    pub const lposc: u32 = hw.powman_base + 0x28;
    pub const chip_reset: u32 = hw.powman_base + 0x2c;
    pub const wdsel: u32 = hw.powman_base + 0x30;
    pub const seq_cfg: u32 = hw.powman_base + 0x34;
    pub const state: u32 = hw.powman_base + 0x38;
    pub const set_time_63to48: u32 = hw.powman_base + 0x60;
    pub const set_time_47to32: u32 = hw.powman_base + 0x64;
    pub const set_time_31to16: u32 = hw.powman_base + 0x68;
    pub const set_time_15to0: u32 = hw.powman_base + 0x6c;
    pub const read_time_upper: u32 = hw.powman_base + 0x70;
    pub const read_time_lower: u32 = hw.powman_base + 0x74;
    pub const alarm_time_63to48: u32 = hw.powman_base + 0x78;
    pub const alarm_time_47to32: u32 = hw.powman_base + 0x7c;
    pub const alarm_time_31to16: u32 = hw.powman_base + 0x80;
    pub const alarm_time_15to0: u32 = hw.powman_base + 0x84;
    pub const timer: u32 = hw.powman_base + 0x88;
    pub const pwrup0: u32 = hw.powman_base + 0x8c;
};

/// `POWMAN_TIMER` fields.
pub const Timer = struct {
    pub const nonsec_write: u32 = 1 << 0;
    pub const run: u32 = 1 << 1;
    pub const clear: u32 = 1 << 2;
    pub const alarm_enab: u32 = 1 << 4;
    /// **The bit that makes a power-down recoverable.** "Alarm wakes the chip
    /// from low power mode." Without it an alarm fires into a chip that is not
    /// listening, and only a power cycle brings it back.
    pub const pwrup_on_alarm: u32 = 1 << 5;
    /// W1C. "Alarm has fired. Write to 1 to clear the alarm."
    pub const alarm: u32 = 1 << 6;

    pub const use_lposc: u32 = 1 << 8;
    pub const use_xosc: u32 = 1 << 9;
    pub const use_gpio_1khz: u32 = 1 << 10;
    pub const use_gpio_1hz: u32 = 1 << 13;

    // Read-only "what it is actually running from" bits. **Worth checking
    // rather than assuming**: `use_*` is a request, `using_*` is the answer,
    // and a source that never started leaves the timer stopped with every
    // request bit set exactly as written.
    pub const using_xosc: u32 = 1 << 16;
    pub const using_lposc: u32 = 1 << 17;
    pub const using_gpio_1khz: u32 = 1 << 18;
    pub const using_gpio_1hz: u32 = 1 << 19;
};

/// `POWMAN_STATE` fields. Bits 0-3 are `CURRENT` (read-only), 4-7 are `REQ`.
///
/// **0 means powered up and 1 means powered down**, which is the opposite of
/// what the names suggest on first reading.
pub const State = struct {
    pub const current_mask: u32 = 0x0000_000f;
    pub const req_shift: u5 = 4;
    pub const req_mask: u32 = 0x0000_00f0;

    // Status flags. **Every one of these was taken from the SDK's `powman.h`
    // and pinned by the test below**, after a first draft put `bad_req` at
    // bit 9 from memory — which is `PWRUP_WHILE_WAITING`, a completely
    // different condition. A wrong bit here reads a healthy block as faulty or,
    // worse, a rejected power-down as accepted.
    /// A request was registered but not acted on.
    pub const req_ignored: u32 = 1 << 8;
    /// A power-up arrived while a power-down was still pending.
    pub const pwrup_while_waiting: u32 = 1 << 9;
    /// **A software request was rejected** — the one to check after asking for
    /// a state change.
    pub const bad_sw_req: u32 = 1 << 10;
    /// A hardware-initiated request was rejected.
    pub const bad_hw_req: u32 = 1 << 11;
    /// The request is registered and POWMAN is waiting for the processors to
    /// halt. Stays set until they do, and the request can be rewritten
    /// or cancelled during that window.
    pub const waiting: u32 = 1 << 12;
    pub const changing: u32 = 1 << 13;

    pub const sram1: u32 = 1 << 0;
    pub const sram0: u32 = 1 << 1;
    pub const xip: u32 = 1 << 2;
    pub const swcore: u32 = 1 << 3;
};

// ---------------------------------------------------------------------------
// Writing, with the password
// ---------------------------------------------------------------------------

/// Write a protected register. The password is added here so no caller can
/// forget it.
pub inline fn write(reg: u32, value: u32) void {
    hw.write(reg, password | (value & 0xffff));
}

/// Read-modify-write, re-adding the password the read did not return.
pub inline fn setBits(reg: u32, bits: u32) void {
    hw.write(reg, password | ((hw.read(reg) | bits) & 0xffff));
}

pub inline fn clrBits(reg: u32, bits: u32) void {
    hw.write(reg, password | ((hw.read(reg) & ~bits) & 0xffff));
}

/// Writes that were discarded for want of a password.
///
/// **Read this before believing anything else in this module.** A non-zero
/// value means some write did not happen and the register it targeted still
/// holds its old contents, with no other symptom.
pub fn badPasswd() u32 {
    return hw.read(Reg.badpasswd);
}

// ---------------------------------------------------------------------------
// The always-on timer
// ---------------------------------------------------------------------------

/// Which oscillator drives the millisecond tick.
pub const Source = enum {
    /// The always-on RC oscillator. **The only source that survives a
    /// power-down**, and therefore the only one a wake alarm can use.
    /// Approximate — see the module header.
    lposc,
    /// The crystal. Accurate, and stops when the crystal does.
    xosc,
};

/// Start the always-on timer on `src`, from zero.
///
/// Returns false if the timer did not take the source — the `using_*` bit is
/// the hardware's own answer, and a request that was ignored leaves the timer
/// stopped while every requested bit reads back exactly as written.
pub fn timerStart(src: Source) bool {
    // Pause while reconfiguring, then clear, then select, then run — the shape
    // the SDK's `powman_timer_set_1khz_tick_source_*` uses.
    clrBits(Reg.timer, Timer.run);
    setBits(Reg.timer, Timer.clear);

    switch (src) {
        .lposc => setBits(Reg.timer, Timer.use_lposc),
        .xosc => setBits(Reg.timer, Timer.use_xosc),
    }

    setBits(Reg.timer, Timer.run);

    // **Wait for the source to actually take, and do not read it immediately.**
    // `USE_*` is a request that self-clears once the switch completes, and
    // `USING_*` sets only after it has crossed into the always-on domain — so
    // there is a window in which *neither* is set and the timer reads as
    // running from nothing. Measured on hardware 2026-08-25: a check taken
    // straight after the write saw exactly that (`TIMER` = 0x02) and reported
    // failure on a timer that was counting perfectly well. The SDK spins here
    // too; it just does not say why.
    const want: u32 = switch (src) {
        .lposc => Timer.using_lposc,
        .xosc => Timer.using_xosc,
    };
    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(Reg.timer) & want == 0) {
        s.tick() catch return false;
    }
    return hw.read(Reg.timer) & Timer.run != 0;
}

pub fn timerRunning() bool {
    return hw.read(Reg.timer) & Timer.run != 0;
}

/// The 64-bit millisecond count.
///
/// Reads high, low, high again and retries on a carry — the pair is not
/// latched, so the naive high-then-low can straddle a rollover and produce a
/// value that was never on the clock. Same shape as the SDK's own reader.
pub fn timeMs() u64 {
    var hi = hw.read(Reg.read_time_upper);
    while (true) {
        const lo = hw.read(Reg.read_time_lower);
        const next_hi = hw.read(Reg.read_time_upper);
        if (hi == next_hi) return (@as(u64, hi) << 32) | lo;
        hi = next_hi;
    }
}

/// Arm the alarm at an absolute millisecond value.
///
/// `wake_chip` sets `PWRUP_ON_ALARM`, which is what makes a subsequent
/// power-down recoverable without a power cycle. **Pass true before any
/// power-down and prove the alarm fires first** — see `example/powman-check`.
///
/// The alarm is disabled while its time is written: the datasheet requires it
/// ("This field must only be written when POWMAN_ALARM_ENAB=0"), and a 64-bit
/// value written in four 16-bit pieces to a live comparator could otherwise
/// match halfway through.
pub fn setAlarmMs(at_ms: u64, wake_chip: bool) void {
    clrBits(Reg.timer, Timer.alarm_enab);

    write(Reg.alarm_time_15to0, @truncate(at_ms));
    write(Reg.alarm_time_31to16, @truncate(at_ms >> 16));
    write(Reg.alarm_time_47to32, @truncate(at_ms >> 32));
    write(Reg.alarm_time_63to48, @truncate(at_ms >> 48));

    // Clear a stale fire before enabling, or an alarm from a previous arming
    // reads as this one going off immediately.
    setBits(Reg.timer, Timer.alarm);

    if (wake_chip) {
        setBits(Reg.timer, Timer.pwrup_on_alarm);
    } else {
        clrBits(Reg.timer, Timer.pwrup_on_alarm);
    }
    setBits(Reg.timer, Timer.alarm_enab);
}

pub fn alarmFired() bool {
    return hw.read(Reg.timer) & Timer.alarm != 0;
}

/// Clear a fired alarm. Write-1-to-clear.
pub fn clearAlarm() void {
    setBits(Reg.timer, Timer.alarm);
}

// ---------------------------------------------------------------------------
// Power state
// ---------------------------------------------------------------------------

/// What `STATE.CURRENT` reports, decoded. **A set bit means powered DOWN.**
pub fn currentState() u32 {
    return hw.read(Reg.state) & State.current_mask;
}

/// True if the last request was rejected. Sticky until the next request.
///
/// The hardware refuses two shapes outright (§POWMAN_STATE): "any combination
/// of power up and power down requests" in one write, and "any request that
/// results in swcore being powered and xip unpowered". A rejected request is
/// **not** ignored — the power-up half is still performed, "To do nothing would
/// risk entering an unrecoverable lock-up state."
pub fn badRequest() bool {
    return hw.read(Reg.state) & (State.bad_sw_req | State.bad_hw_req) != 0;
}

/// Ask POWMAN to change which domains are powered. **A set bit powers a domain
/// DOWN.**
///
/// Writes only the `REQ` field: `CURRENT` is read-only, and the status flags
/// are write-1-to-clear, so writing zeroes there leaves them alone.
///
/// **This does not return in the usual sense when `swcore` is in `req`.** The
/// core stops, and when something powers it back up execution resumes at the
/// reset vector — not at the next statement. Anything that must survive belongs
/// in a domain that stays powered, or in the always-on registers.
///
/// Two request shapes are rejected outright (§POWMAN_STATE): a mixture of
/// power-up and power-down in one write, and anything that would leave `swcore`
/// powered while `xip` is not. A rejected request still performs the power-up
/// half — "To do nothing would risk entering an unrecoverable lock-up state" —
/// and sets `BAD_SW_REQ`. Check `badRequest()` if the core is still running
/// afterwards, because that is itself the symptom.
pub fn requestState(req: u32) void {
    write(Reg.state, (req & 0x0f) << State.req_shift);
}

// ---------------------------------------------------------------------------
// Tests — the arithmetic only; everything else needs the block.
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the password sits in the top 16 bits and leaves the value alone" {
    try testing.expectEqual(@as(u32, 0x5afe_0000), password);
    // A value must not bleed into the password field, or the write is silently
    // discarded — the exact failure this module exists to prevent.
    try testing.expectEqual(@as(u32, 0), password & 0xffff);
}

test "protected range covers every register this module writes" {
    // §6.4: everything up to and including 0xac needs the password. If a future
    // register is added above that, this test is where the assumption is
    // recorded rather than silently carried.
    const written = [_]u32{
        Reg.lposc,          Reg.wdsel,            Reg.state,
        Reg.timer,          Reg.alarm_time_15to0, Reg.alarm_time_63to48,
        Reg.set_time_15to0,
    };
    for (written) |r| {
        try testing.expect(r - hw.powman_base <= protected_limit);
    }
}

test "state bits: set means powered down, and the fields do not overlap" {
    try testing.expectEqual(@as(u32, 0x0f), State.current_mask);
    try testing.expectEqual(@as(u32, 0xf0), State.req_mask);
    try testing.expectEqual(@as(u32, 0), State.current_mask & State.req_mask);
    // swcore is the top of the four, which is why powering it down is the
    // request that cannot be undone from the core itself.
    try testing.expectEqual(@as(u32, 1) << 3, State.swcore);
}

test "state status flags match the datasheet, bit for bit" {
    // **Pinned because a first draft had `bad_req` at bit 9 from memory.** That
    // is `PWRUP_WHILE_WAITING`; the rejection flags are at 10 and 11. Reading
    // the wrong bit would report a rejected power-down as accepted, which is
    // the one mistake here with no second chance.
    try testing.expectEqual(@as(u32, 0x0100), State.req_ignored);
    try testing.expectEqual(@as(u32, 0x0200), State.pwrup_while_waiting);
    try testing.expectEqual(@as(u32, 0x0400), State.bad_sw_req);
    try testing.expectEqual(@as(u32, 0x0800), State.bad_hw_req);
    try testing.expectEqual(@as(u32, 0x1000), State.waiting);
    try testing.expectEqual(@as(u32, 0x2000), State.changing);

    // And none of them collides with the two four-bit fields below.
    const flags = State.req_ignored | State.pwrup_while_waiting |
        State.bad_sw_req | State.bad_hw_req | State.waiting | State.changing;
    try testing.expectEqual(@as(u32, 0), flags & (State.current_mask | State.req_mask));
}

test "alarm time splits into four 16-bit writes without losing a bit" {
    const t: u64 = 0x1234_5678_9abc_def0;
    const pieces = [_]u16{
        @truncate(t),
        @truncate(t >> 16),
        @truncate(t >> 32),
        @truncate(t >> 48),
    };
    var rebuilt: u64 = 0;
    var i: usize = pieces.len;
    while (i > 0) {
        i -= 1;
        rebuilt = (rebuilt << 16) | pieces[i];
    }
    try testing.expectEqual(t, rebuilt);
}
