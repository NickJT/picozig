//! powman-check — prove the always-on timer and its wake alarm, while awake.
//!
//! **The prerequisite for dormant sleep, and deliberately not dormant sleep
//! itself.** Powering down the switched core is the thing that takes a battery
//! device from 3.48 mA to something a battery can live with — and it is also the thing
//! that makes the board unreachable until something wakes it. The only thing
//! that can wake it unaided is POWMAN's always-on timer.
//!
//! So this proves the wake mechanism *first*, with the chip fully awake and the
//! debugger attached the whole time:
//!
//!   1. The always-on timer starts on LPOSC, and its 64-bit millisecond count
//!      actually advances.
//!   2. An alarm armed a few seconds out **fires**, with `PWRUP_ON_ALARM` set —
//!      the exact configuration a power-down would rely on.
//!   3. `BADPASSWD` is still zero, i.e. every POWMAN write landed.
//!
//! If all three hold, the wake path is sound and the power-down below is safe
//! to arm. If any fails — most likely (2) — then arming it would produce a
//! board that never comes back, and the only remedy is a power cycle.
//!
//! ```bash
//! zig build swd-powman-check -Dboard=pico2w
//! zig build watch-powman-check      # after ~8 s
//! ```
//!
//! ## Why LPOSC and not XOSC
//!
//! XOSC is accurate and stops when the switched core does. LPOSC is the
//! always-on RC oscillator: it keeps counting through a power-down, which is
//! the entire point. It is **not** a timekeeper — the datasheet ships
//! `LPOSC_FREQ_KHZ_INT`/`_FRAC` precisely because software is expected to
//! measure it rather than assume 32 kHz. `elapsed_ms` below is therefore
//! "roughly", and comparing it against TIMER0's crystal-derived microseconds is
//! how far off it is on this part.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const powman = rp2350.powman;

/// How far ahead to arm the alarm. Long enough to be unambiguous, short enough
/// that a `watch` run a few seconds after flashing sees the result.
const alarm_delay_ms: u64 = 3000;

/// Give up waiting for the alarm after this long. **Bounded on purpose**: an
/// alarm that never fires is precisely the condition that must not be allowed
/// to look like a slow one, because the next step depends on it.
const alarm_timeout_ms: u32 = 8000;

/// How long to stay powered down. Short, because someone is watching the bench.
const dormant_ms: u64 = 10_000;

const Watch = extern struct {
    magic: u32,
    words: u32,

    /// 1 if the timer reported running on the source that was asked for.
    /// `use_*` is a request; `using_*` is the answer.
    timer_started: u32,
    /// `POWMAN_TIMER` read back, so every bit above is checkable.
    timer_reg: u32,

    /// The millisecond count, low word, sampled twice a second apart.
    /// **Two equal readings mean the timer is not counting**, which a single
    /// reading cannot show.
    t0_lo: u32,
    t1_lo: u32,

    /// 1 if the alarm fired within `alarm_timeout_ms`.
    ///
    /// **This is the field that licenses the power-down.** A 0 here means a
    /// dormant request would produce a board that never wakes.
    alarm_fired: u32,
    /// How long the alarm actually took, by LPOSC's own count.
    alarm_took_ms: u32,
    /// The same interval measured by TIMER0 off the crystal, in milliseconds.
    /// **The ratio of these two is LPOSC's error**, which is worth knowing
    /// before anyone uses it to time anything.
    alarm_took_xosc_ms: u32,

    /// `POWMAN_BADPASSWD`. **Non-zero means a POWMAN write was silently
    /// discarded** and some register above still holds its old value.
    bad_passwd: u32,
    /// `POWMAN_STATE`: bits 0-3 current (set = powered down), 8 REQ_IGNORED,
    /// 10 BAD_SW_REQ, 11 BAD_HW_REQ, 12 WAITING.
    ///
    /// **`0x100` on a board that never made a request is normal** — REQ_IGNORED
    /// is sticky and something before this firmware set it. What matters after
    /// a real request is bits 10 and 11.
    state: u32,
    /// 1 once every check above has been made and recorded.
    complete: u32,
    /// Boots since this block was first initialised. **A power-down that woke
    /// correctly shows 2 here**; a fresh flash shows 1.
    boots: u32,
    /// Set just before the power-down request, so the next boot can tell a wake
    /// from a reset. Lives in SRAM, which is why SRAM stays powered.
    went_dormant: u32,
    /// The always-on millisecond count read *after* waking, minus the value
    /// recorded before powering down. **This is the proof it really slept**:
    /// the AON timer is the only thing still counting while swcore is off.
    slept_ms: u32,
    /// `timeMs()` immediately before the request, kept across the power-down.
    dormant_at_ms: u32,
};

pub const watch_magic: u32 = 0x70_77_6d_6e; // "pwmn"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    // **A wake from dormant arrives here, at the reset vector**, with SRAM
    // still holding what the previous run left. So the block is only
    // initialised when it is not ours — otherwise the wake would erase the
    // evidence that it happened.
    const waking = watch.magic == watch_magic and watch.went_dormant == 1;

    if (!waking) watch = .{
        .magic = 0,
        .words = @sizeOf(Watch) / 4,
        .timer_started = 0,
        .timer_reg = 0,
        .t0_lo = 0,
        .t1_lo = 0,
        .alarm_fired = 0,
        .alarm_took_ms = 0,
        .alarm_took_xosc_ms = 0,
        .bad_passwd = 0xFFFF_FFFF,
        .state = 0,
        .complete = 0,
        .boots = 0,
        .went_dormant = 0,
        .slept_ms = 0,
        .dormant_at_ms = 0,
    };

    clocks.init(board.clock_plan);
    gpio.init();
    watch.magic = watch_magic;
    watch.boots +%= 1;

    if (waking) {
        // Woke with the dormant marker set. **`slept_ms` is what decides
        // whether that was a real wake or just a reset**: a plain `reset run`
        // while the marker was set lands here too, and during this run it
        // reported 116,839 ms against a 10,000 ms request — i.e. the elapsed
        // time since the marker was written, not a sleep. Compare it against
        // `dormant_ms` before believing it.
        watch.slept_ms = @truncate(powman.timeMs() -% watch.dormant_at_ms);
        watch.went_dormant = 0;
        powman.clearAlarm();
        powman.clrBits(powman.Reg.timer, powman.Timer.alarm_enab);
        watch.state = rp2350.hw.read(powman.Reg.state);
        watch.bad_passwd = powman.badPasswd();
        watch.complete = 2;
        while (true) timer.delayMs(500);
    }

    // --- 1. does the always-on timer run? ----------------------------------
    watch.timer_started = @intFromBool(powman.timerStart(.lposc));
    watch.timer_reg = rp2350.hw.read(powman.Reg.timer);

    watch.t0_lo = @truncate(powman.timeMs());
    timer.delayMs(1000);
    watch.t1_lo = @truncate(powman.timeMs());

    // --- 2. does an alarm fire, configured to wake the chip? ---------------
    //
    // `wake_chip = true` is not incidental: this arms exactly the configuration
    // a power-down would depend on, so what is proven here is the thing that
    // would be relied on there — not an approximation of it.
    const armed_at = powman.timeMs();
    powman.setAlarmMs(armed_at + alarm_delay_ms, true);

    const xosc_start = timer.now();
    var waited: u32 = 0;
    while (!powman.alarmFired() and waited < alarm_timeout_ms) : (waited += 10) {
        timer.delayMs(10);
    }
    const xosc_elapsed_us = timer.now() -% xosc_start;

    watch.alarm_fired = @intFromBool(powman.alarmFired());
    watch.alarm_took_ms = @truncate(powman.timeMs() -% armed_at);
    watch.alarm_took_xosc_ms = xosc_elapsed_us / 1000;
    powman.clearAlarm();

    // --- 3. did every POWMAN write actually land? --------------------------
    watch.bad_passwd = powman.badPasswd();
    watch.state = rp2350.hw.read(powman.Reg.state);
    watch.complete = 1;

    // ----------------------------------------------------------------------
    // THE POWER-DOWN IS COMMENTED OUT ON PURPOSE.
    // ----------------------------------------------------------------------
    //
    // **Standing rule: an example must not put the board into a state it cannot
    // get itself out of.** Read `alarm_fired` first — if it is 0, the wake
    // cannot work and uncommenting this produces a board that stays off.
    // Uncommenting is the interlock.
    //
    // ## It was armed and run on 2026-08-25, and the board never went dark
    //
    // **A power-down request is ignored while a Debug Probe is attached.** Run
    // twice, cleanly, with SRAM kept powered so the evidence survived:
    //
    //   * `BADPASSWD` 0 — the write was well formed and accepted.
    //   * `STATE` 0x00000100 — `REQ` reads **0**, and `REQ_IGNORED` is set.
    //   * the core carried on running, reachable over SWD throughout.
    //
    // `REQ_IGNORED` is documented as "a software state change request was
    // ignored because it clashed with an ongoing hardware or **debugger**
    // request". The distinction from an *invalid* request matters: the
    // datasheet says an invalid one still registers in `REQ` and sets
    // `BAD_SW_REQ`. Ours does neither — it is refused before it is registered.
    //
    // Writing 1 to `REQ_IGNORED` to clear it (with the password, `BADPASSWD`
    // stayed 0) did not clear it, which is what an *ongoing* clash looks like:
    // the condition re-asserts as fast as the flag is cleared.
    //
    // **So dormant sleep cannot be exercised with the probe connected**, and
    // that is not a limitation of this code. It also means testing it and
    // measuring the current it saves are necessarily the same session, because
    // a current reading already requires the probe unhooked.
    // With no SWD, the evidence has to be something visible — a pin, an LED, or
    // the meter itself — not this block.
    //
    // ## Recovery, written out so nobody has to re-derive it
    //
    //   1. **Wait.** With `PWRUP_ON_ALARM` set and the alarm armed, the chip
    //      powers itself back up when the alarm matches. `alarm_fired` above is
    //      the evidence that path works.
    //   2. **Power cycle.** Unplug USB and re-plug.
    //   3. **BOOTSEL + a UF2**, if it comes back into the same image and goes
    //      straight down again:
    //
    //          zig build flash-blink-hello -Dboard=pico2w
    //
    // ## What arming it looks like
    //
    // **Keep SRAM powered.** Only `swcore` and `xip` go down — that leaves
    // `.selftest` alive across the power-down, which is the only way the next
    // boot can show it slept rather than merely reset. It is also how a battery
    // device would use this: state in SRAM, core off between events.
    //
    // if (watch.alarm_fired == 1) {
    //     powman.setAlarmMs(powman.timeMs() + dormant_ms, true);
    //     watch.dormant_at_ms = @truncate(powman.timeMs());
    //     watch.went_dormant = 1;
    //
    //     // Set = powered down. swcore may not be left powered while xip is
    //     // not, so the two go together.
    //     powman.requestState(powman.State.swcore | powman.State.xip);
    //
    //     // The core stops in here. Execution resumes at the reset vector when
    //     // the alarm powers the domain back up -- not on the next line.
    //     while (true) asm volatile ("wfi");
    // }

    // Alive, reachable, and holding the results. Nothing above powered
    // anything down.
    while (true) {
        timer.delayMs(500);
        watch.words = @sizeOf(Watch) / 4;
    }
}
