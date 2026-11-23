//! Watchdog.
//!
//! For a device nobody is watching: one that has wedged must recover on its
//! own, because nobody is going to notice a dark LED and power-cycle it. A
//! wedged indicator is worse than an absent one, since it reads as "all clear".
//!
//! **Also the answer to "how did we get here?"** — `resetReason` distinguishes a
//! watchdog reboot from a power-on, and the scratch registers survive the reset,
//! so firmware can leave a breadcrumb before it dies and read it back afterwards.
//! Without that, a device found running normally gives no hint that it rebooted
//! six times overnight.

const std = @import("std");
const hw = @import("hw.zig");

/// The counter is clocked by the TICKS block's watchdog generator, which
/// `clocks.init()` must have started. **A watchdog whose tick generator is not
/// running never fires** — the same trap TIMER0 has, and just as silent.
///
/// Historical note worth knowing: on RP2040 this counter decremented twice per
/// microsecond, so every delay had to be doubled. RP2350 fixed that — the value
/// here is plain microseconds.
pub const max_delay_us: u32 = hw.Watchdog.ctrl_time_mask;

pub const ResetReason = enum {
    /// Power-on, brownout, or the RUN pin — i.e. not the watchdog.
    power_on,
    /// The countdown expired: something wedged.
    timer,
    /// Software asked for a reset via `trigger`.
    forced,
};

/// Why the chip last started. Read this **before** `start`, and ideally report
/// it — a silent watchdog reboot loop is exactly the failure this device must
/// not hide.
pub fn resetReason() ResetReason {
    const r = hw.read(hw.Watchdog.reason);
    if (r & hw.Watchdog.reason_force != 0) return .forced;
    if (r & hw.Watchdog.reason_timer != 0) return .timer;
    return .power_on;
}

/// Start the watchdog with a `delay_us` timeout.
///
/// `pause_on_debug` keeps the countdown frozen while a debugger has the core
/// halted. Leave it true during development or every breakpoint reboots the
/// board — which looks exactly like a crash, and wastes an afternoon.
pub fn start(delay_us: u32, pause_on_debug: bool) void {
    const t = @min(delay_us, max_delay_us);

    // Disable while reprogramming, so a short remaining count cannot fire
    // between the LOAD write and the enable.
    hw.write(hw.Watchdog.ctrl, 0);

    // **Select what a timeout actually resets. Without this the watchdog does
    // nothing at all.**
    //
    // `PSM.WDSEL` decides which stages of the power-on state machine a watchdog
    // reset restarts, and it is zero out of reset — so the counter expires,
    // `REASON.TIMER` is set, and the chip carries straight on. Datasheet §7.4.3
    // and §12.9; the SDK does the same thing in `watchdog_enable`.
    //
    // **Found 2026-08-25 by `example/watchdog-check`**, which armed the
    // watchdog, stopped feeding it, and then sat there counting for twelve
    // seconds. This module's own header says a wedged device "must recover on
    // its own" — it could not have, and nothing had ever asked it to.
    //
    // ROSC and XOSC are excluded: resetting the oscillators would stop the
    // clock the reset sequence itself runs on.
    hw.setBits(hw.Psm.wdsel, hw.Psm.wdsel_reboot);
    hw.write(hw.Watchdog.load, t);

    var ctrl: u32 = hw.Watchdog.ctrl_enable;
    if (pause_on_debug) {
        ctrl |= hw.Watchdog.ctrl_pause_dbg0 |
            hw.Watchdog.ctrl_pause_dbg1 |
            hw.Watchdog.ctrl_pause_jtag;
    }
    hw.write(hw.Watchdog.ctrl, ctrl);
    feed(t);
}

/// Restart the countdown. Must be called more often than the timeout.
pub fn feed(delay_us: u32) void {
    hw.write(hw.Watchdog.load, @min(delay_us, max_delay_us));
}

/// Microseconds left before the watchdog bites, read from `CTRL[23:0]`.
///
/// **The one direct way to see the watchdog counting.** Two reads a known
/// interval apart that show no change mean the counter is not running — either
/// the TICKS watchdog generator was never started (`clocks.init` does it) or
/// the whole block is paused, and both look identical from anywhere else.
pub fn remainingUs() u32 {
    return hw.read(hw.Watchdog.ctrl) & hw.Watchdog.ctrl_time_mask;
}

pub fn disable() void {
    hw.write(hw.Watchdog.ctrl, 0);
}

/// Reboot immediately.
///
/// **Selects what the reset covers first, and that is not optional.**
/// `PSM.WDSEL` is zero out of reset, and with it zero a watchdog trigger
/// restarts *no* stage of the power-on state machine — the chip sets
/// `REASON.FORCE` and carries on. `start` has set it since 2026-08-25, when
/// `example/watchdog-check` found a watchdog that counted for twelve seconds
/// and rebooted nothing; **this function was left with the same defect**,
/// where it is worse: a caller asking for a reboot gets a permanent `wfi`
/// spin instead. Found 2026-08-27, wanting a deliberate reboot from
/// `app/bt-speaker`.
///
/// ROSC and XOSC are excluded for the reason `start` gives: resetting the
/// oscillators would stop the clock the reset sequence runs on.
pub fn trigger() noreturn {
    hw.setBits(hw.Psm.wdsel, hw.Psm.wdsel_reboot);
    hw.write(hw.Watchdog.ctrl, hw.Watchdog.ctrl_trigger);
    while (true) asm volatile ("wfi");
}

/// Scratch registers survive a watchdog reset.
///
/// **Only indices 4-7 are safely available**: the bootrom uses the lower ones
/// to pass boot information, and clobbering those can change how the chip
/// starts. Restricted here rather than documented, because "documented" is how
/// the first four get used by accident.
pub const Scratch = enum(u3) { s4 = 4, s5 = 5, s6 = 6, s7 = 7 };

pub fn scratchWrite(which: Scratch, value: u32) void {
    hw.write(hw.Watchdog.scratch0 + @as(u32, @backingInt(which)) * 4, value);
}

pub fn scratchRead(which: Scratch) u32 {
    return hw.read(hw.Watchdog.scratch0 + @as(u32, @backingInt(which)) * 4);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "timeout is clamped to the 24-bit counter rather than wrapping" {
    // A wrapped value would produce a very SHORT timeout from a request for a
    // very long one — a reboot loop from an innocent-looking constant.
    try testing.expectEqual(@as(u32, 0x00ff_ffff), max_delay_us);
    try testing.expectEqual(max_delay_us, @min(std.math.maxInt(u32), max_delay_us));
    // ~16.7 s is the practical ceiling.
    try testing.expect(max_delay_us > 16_000_000 and max_delay_us < 17_000_000);
}

test "scratch registers are restricted to the four the bootrom does not use" {
    // The enum makes indices 0-3 unrepresentable; assert the addresses of the
    // ones that remain.
    try testing.expectEqual(@as(u32, 4), @backingInt(Scratch.s4));
    try testing.expectEqual(@as(u32, 7), @backingInt(Scratch.s7));
    try testing.expectEqual(
        hw.Watchdog.scratch0 + 16,
        hw.Watchdog.scratch0 + @as(u32, @backingInt(Scratch.s4)) * 4,
    );
}

test "control bits match the datasheet" {
    try testing.expectEqual(@as(u32, 1) << 30, hw.Watchdog.ctrl_enable);
    try testing.expectEqual(@as(u32, 1) << 31, hw.Watchdog.ctrl_trigger);
    try testing.expectEqual(@as(u32, 0x00ff_ffff), hw.Watchdog.ctrl_time_mask);

    // **The three that were rotated.** Datasheet §12.9's CTRL table: 24
    // PAUSE_JTAG, 25 PAUSE_DBG0, 26 PAUSE_DBG1. Pinned here because the only
    // caller sets all three at once, so no behaviour would ever reveal a swap.
    try testing.expectEqual(@as(u32, 1) << 24, hw.Watchdog.ctrl_pause_jtag);
    try testing.expectEqual(@as(u32, 1) << 25, hw.Watchdog.ctrl_pause_dbg0);
    try testing.expectEqual(@as(u32, 1) << 26, hw.Watchdog.ctrl_pause_dbg1);
}

test "a watchdog reset restarts everything except the oscillators" {
    // Resetting ROSC or XOSC would stop the clock the reset sequence runs on.
    try testing.expectEqual(@as(u32, 0), hw.Psm.wdsel_reboot & hw.Psm.wdsel_rosc);
    try testing.expectEqual(@as(u32, 0), hw.Psm.wdsel_reboot & hw.Psm.wdsel_xosc);

    // And everything else is selected. A zero here is the bug this test exists
    // for: it is the reset value, and it makes the watchdog inert.
    try testing.expectEqual(@as(u32, 0x01ff_fff3), hw.Psm.wdsel_reboot);
    try testing.expect(hw.Psm.wdsel_reboot != 0);
}
