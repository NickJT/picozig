//! watchdog-check — arm it, feed it, let it bite, and prove which happened.
//!
//! The C-SDK's `hello_watchdog` enables the watchdog, feeds it five times, then
//! stops and lets the board reboot, printing at each step. That needs a terminal
//! to mean anything, and a watchdog is exactly the thing you want to test on a
//! board whose terminal is the suspect.
//!
//! So this walks the same ground and reports through the **scratch registers**,
//! which survive the reset, plus a `.selftest` block read over SWD. It runs
//! three phases across three boots, each triggered by the previous one:
//!
//!   1. **Clean boot** — arm with a 500 ms timeout, then feed for two seconds.
//!      Surviving that is the proof `feed` works, and it is the half the C-SDK
//!      example does not really demonstrate: five updates in a row take
//!      microseconds and would survive a watchdog that was never armed.
//!      Then stop feeding and wait to be bitten.
//!   2. **Timer reboot** — `resetReason` reads `.timer`. Check the scratch
//!      registers carried the generation across, then call `trigger()`.
//!   3. **Forced reboot** — `resetReason` reads `.forced`. Disable the
//!      watchdog and rest.
//!
//! **It terminates.** A watchdog example that leaves a board rebooting for ever
//! is a nuisance on a bench and fights the debugger for the SWD port; phase 3
//! disables the watchdog and sits still. There is also a hard generation cap, so
//! a bug in the phase logic stops rather than cycling.
//!
//! ## What the scratch registers are actually for
//!
//! Not this demo. **`resetReason` tells you the watchdog fired; the scratch
//! registers tell you what the firmware was doing when it did.** A device found
//! running normally gives no hint that it rebooted six times overnight, and
//! `s4`-`s7` are the four words that can say so. Only those four: the bootrom
//! uses the lower ones to pass boot information.
//!
//! ```bash
//! zig build swd-watchdog-check -Dboard=pico2w
//! zig build watch-watchdog-check       # wait ~4 s after flashing for phase 3
//! ```

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const watchdog = rp2350.watchdog;

/// Short enough that the whole three-boot cycle finishes while you wait, long
/// enough that the feeding phase is unambiguous.
const timeout_us: u32 = 500_000;

/// How long phase 1 feeds before giving up. Four timeouts' worth: surviving
/// this cannot be luck.
const feed_ms: u32 = 2000;
const feed_interval_ms: u32 = 100;

/// Marks the scratch registers as ours rather than whatever the last firmware
/// left. **Without this the first boot after a different image reads that
/// image's data as a generation count**, which is the `.selftest` trap in a
/// different set of registers.
const scratch_magic: u32 = 0x5744_4f47; // "WDOG"

/// A bug in the phase logic must stop, not cycle.
const max_generations: u32 = 4;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// How many boots this cycle has seen, from the scratch registers.
    generation: u32,
    /// `resetReason` on each of the three boots: 0 power_on, 1 timer,
    /// 2 forced, 0xF not reached.
    reason_1: u32,
    reason_2: u32,
    reason_3: u32,
    /// 1 if the board survived the whole of phase 1's feeding.
    ///
    /// **This is the field that proves `feed` does something.** A watchdog that
    /// was never armed also survives being fed, so the pass condition is this
    /// *and* `reason_2 == 1` — survived while fed, bitten when not.
    fed_ok: u32,
    /// 1 if `scratch_magic` was still in s4 after the timer reboot. The scratch
    /// registers surviving a watchdog reset is the whole reason to use them.
    scratch_survived: u32,
    /// The value phase 1 left in s6, read back in phase 2. Arbitrary; the point
    /// is that it arrives unchanged.
    scratch_payload: u32,
    /// 1 once phase 3 has disabled the watchdog and settled. **Until this is 1
    /// the board may still reboot under you**, which matters when the next
    /// thing you do is flash it.
    complete: u32,
};

pub const watch_magic: u32 = 0x77_64_6f_67; // "wdog"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

const payload: u32 = 0xC0FF_EE00;

fn reasonCode(r: watchdog.ResetReason) u32 {
    return switch (r) {
        .power_on => 0,
        .timer => 1,
        .forced => 2,
    };
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();

    // **Read the reason before anything else touches the watchdog.** `start`
    // does not clear it, but a `trigger` later in this same function would, and
    // then the next boot's reason would describe this boot's last act rather
    // than its cause.
    const reason = watchdog.resetReason();

    // `.selftest` is NOLOAD and survives resets by design — which is exactly
    // what this example needs, and also why it must not be re-initialised on
    // the second and third boots. Only a clean boot starts the record.
    const continuing = watch.magic == watch_magic and
        watchdog.scratchRead(.s4) == scratch_magic and
        reason != .power_on;

    if (!continuing) {
        watch = .{
            .magic = watch_magic,
            .ticks = 0,
            .generation = 0,
            .reason_1 = reasonCode(reason),
            .reason_2 = 0xF,
            .reason_3 = 0xF,
            .fed_ok = 0,
            .scratch_survived = 0,
            .scratch_payload = 0,
            .complete = 0,
        };
        watchdog.scratchWrite(.s4, scratch_magic);
        watchdog.scratchWrite(.s5, 0);
        watchdog.scratchWrite(.s6, payload);
    }

    const generation = watchdog.scratchRead(.s5);
    watch.generation = generation;

    if (generation >= max_generations) {
        // Something is wrong with the phase logic. Stop rather than cycle.
        watchdog.disable();
        watch.complete = 2;
        rest();
    }

    switch (reason) {
        .power_on => phase1(),
        .timer => phase2(generation),
        .forced => phase3(),
    }
}

/// Arm, feed for long enough that survival is not luck, then stop.
fn phase1() noreturn {
    watchdog.scratchWrite(.s5, 1);

    // `pause_on_debug` true: without it every breakpoint reboots the board,
    // which looks exactly like the fault under investigation. The `watch` step
    // never halts, so this does not weaken the test.
    watchdog.start(timeout_us, true);

    var elapsed: u32 = 0;
    while (elapsed < feed_ms) : (elapsed += feed_interval_ms) {
        timer.delayMs(feed_interval_ms);
        watchdog.feed(timeout_us);
    }

    // Survived four timeouts' worth while being fed.
    watch.fed_ok = 1;

    // Now stop feeding. The next thing that happens is a reset, and the only
    // way to observe it is the next boot.
    rest();
}

/// The watchdog bit. Confirm what carried over, then reboot on purpose.
fn phase2(generation: u32) noreturn {
    watch.reason_2 = reasonCode(.timer);
    watch.generation = generation;
    watch.scratch_survived = @intFromBool(watchdog.scratchRead(.s4) == scratch_magic);
    watch.scratch_payload = watchdog.scratchRead(.s6);

    watchdog.scratchWrite(.s5, generation + 1);

    // **Disable before triggering.** `trigger` reboots immediately, but leaving
    // an armed watchdog across the reset would have phase 3 racing a timeout it
    // did not ask for — and a reboot loop is the one outcome this example must
    // not produce.
    watchdog.disable();
    timer.delayMs(50); // let the SWD reader see phase 2 if it is watching
    watchdog.trigger();
}

/// A forced reboot, which is the last thing this example does.
fn phase3() noreturn {
    watch.reason_3 = reasonCode(.forced);
    watchdog.disable();
    watch.complete = 1;
    rest();
}

/// Sit still, counting, so the SWD reader has a liveness signal.
fn rest() noreturn {
    while (true) {
        timer.delayMs(250);
        watch.ticks +%= 1;
    }
}
