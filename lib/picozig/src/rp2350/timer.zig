//! Microsecond timing from TIMER0.
//!
//! Requires `clocks.init()` first — TIMER0 counts the TICKS block's 1 us tick,
//! which is derived from clk_ref and is **disabled at reset**. Calling anything
//! here before that hangs.

const hw = @import("hw.zig");

/// Microseconds since the timer started, low 32 bits. Wraps every ~71 minutes,
/// which every function here handles with wrapping arithmetic, so it is the
/// right width for delays and for the self-test. Use `now64` for a clock that
/// does not wrap.
pub inline fn now() u32 {
    return hw.read(hw.Timer0.timerawl);
}

/// Full 64-bit microsecond count.
///
/// Uses the latching pair rather than the raw one: reading TIMELR latches
/// TIMEHR, so low-then-high is atomic by construction. The RAW registers have no
/// such side effect and would need a read-high/read-low/re-read-high retry loop
/// to survive a carry landing between the two reads.
pub fn now64() u64 {
    const lo = hw.read(hw.Timer0.timelr);
    const hi = hw.read(hw.Timer0.timehr);
    return (@as(u64, hi) << 32) | lo;
}

/// Busy-wait for at least `us` microseconds.
///
/// Wrapping subtraction, so a rollover mid-delay is a non-event: the difference
/// stays correct across the wrap. Written the naive way (`target = start + us;
/// while (now() < target)`) this would hang for up to 71 minutes whenever the
/// counter happened to wrap during the wait — a bug that shows up roughly once
/// an hour and is thoroughly unpleasant to reproduce.
pub fn delayUs(us: u32) void {
    const start = now();
    while (now() -% start < us) {}
}

// ---------------------------------------------------------------------------
// Alarms, and sleeping instead of spinning
// ---------------------------------------------------------------------------

/// Which of TIMER0's four alarms. Alarm 0 is IRQ `timer0_0`, and so on.
pub const Alarm = u2;

/// Set by the alarm ISR, read by `sleepUs`. One per alarm so two call sites
/// cannot steal each other's wake-up.
var fired: [4]bool = @splat(false);

/// **Call this from the application's alarm ISR** and nothing else:
///
/// ```zig
/// pub const interrupts = struct {
///     pub fn timer0_0() callconv(.c) void { timer.onAlarm(0); }
/// };
/// ```
///
/// The vector table is built from the application's `interrupts` namespace at
/// comptime (see `boot.zig`), so a library cannot install this itself — which is
/// why the wiring is one line in the app and the logic is all here.
pub fn onAlarm(alarm: Alarm) void {
    hw.write(hw.Timer0.intr, @as(u32, 1) << alarm); // write-1-to-clear
    @atomicStore(bool, &fired[alarm], true, .release);
}

/// Sleep for `us` microseconds with the core clock-gated, instead of spinning.
///
/// **Why this exists**: `delayUs` busy-waits, so a firmware whose main loop is
/// `delayMs(10)` runs the M33 flat out forever. Every microamp of a battery
/// budget goes into a `while` loop reading a register.
///
/// Requires two things of the caller, both checked by nothing:
///   * the alarm's IRQ enabled in the NVIC (`nvic.enable(nvic.IrqNum.timer0_0)`)
///   * an ISR for it that calls `onAlarm`
///
/// Without them this returns immediately rather than hanging — see the guard —
/// so a missing ISR degrades to the old spin-free-but-busy behaviour rather than
/// to a dead device.
///
/// **The equality-compare trap is handled here.** The alarm fires only when the
/// counter *equals* the written value, so a target that slips into the past
/// between computing and arming would not fire for another 71.6 minutes. After
/// arming, the target is re-checked against the counter and abandoned if it has
/// already gone by.
pub fn sleepUs(alarm: Alarm, us: u32) void {
    const mask = @as(u32, 1) << alarm;
    const target = now() +% us;

    @atomicStore(bool, &fired[alarm], false, .release);
    hw.write(hw.Timer0.intr, mask); // discard anything stale
    hw.setBits(hw.Timer0.inte, mask);
    hw.write(hw.Timer0.alarm(alarm), target); // arming is a side effect of the write

    while (!@atomicLoad(bool, &fired[alarm], .acquire)) {
        // Modular "has target already passed?": true once `now` is at or beyond
        // it, false while it is still ahead. Also the escape hatch if no ISR was
        // wired up at all.
        if (now() -% target < 0x8000_0000) break;
        asm volatile ("wfi");
    }

    hw.write(hw.Timer0.armed, mask); // disarm if we left early
    hw.clrBits(hw.Timer0.inte, mask);
    hw.write(hw.Timer0.intr, mask);
}

/// Millisecond form of `sleepUs`. Caps at the alarm's 32-bit microsecond range.
pub fn sleepMs(alarm: Alarm, ms: u32) void {
    sleepUs(alarm, ms *| 1000);
}

pub fn delayMs(ms: u32) void {
    // Split into 1 ms steps so the u32 microsecond arithmetic cannot overflow
    // on a long delay, and so a wrap is only ever crossed inside one short step.
    var left = ms;
    while (left > 0) : (left -= 1) delayUs(1000);
}
