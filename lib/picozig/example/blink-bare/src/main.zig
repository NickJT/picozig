//! blink-bare — the least a board can do, and why its clock might not start.
//!
//! **The bring-up example for a board that flashes cleanly and then says
//! nothing.** Every other example here begins with `clocks.init`, and that call
//! contains an unbounded wait on the crystal becoming stable. A board whose
//! XOSC never starts therefore spins there forever: SWD works, the image
//! verifies, the UART is silent, and nothing distinguishes it from a board that
//! is not executing at all.
//!
//! So this one calls **no clocks code whatsoever**. It runs on the ROSC, which
//! the bootrom leaves running, toggles a pin, and counts. What it proves is
//! narrow and that is the point:
//!
//!   * **the LED blinks** — the chip fetches, executes and drives a pad, so any
//!     fault is above this line;
//!   * **the LED does not blink but `ticks` advances over SWD** — the core runs
//!     and the pin or its wiring does not;
//!   * **neither** — the image is not running: check the IMAGE_DEF, the reset,
//!     or whether the flash write actually landed.
//!
//! It then answers the next question without ever risking a hang: it starts the
//! crystal exactly as `clocks.init` does and waits for `STABLE` with a **bound**,
//! recording whether it arrived and how long it took. `clocks.init`'s own wait
//! is deliberately still unbounded (see `bounded.zig`, and the roadmap's note on
//! what was left out of that pass), so this is currently the only way to ask
//! "did the crystal start?" without betting the session on the answer being yes.
//!
//! ## No delay function, on purpose
//!
//! `timer.delayUs` counts TIMER0's microsecond tick, and that generator is
//! started by `clocks.init` — the very thing being avoided. A delay built on it
//! would hang for the same reason. So the delay here is a plain instruction
//! loop with a `volatile` counter, and the blink rate is whatever the ROSC
//! happens to be doing. **That is a feature**: the ROSC is uncalibrated and
//! varies part to part and with temperature, so a visibly wrong rate is not a
//! fault, and a rate that changes between runs is not either.
//!
//! ```bash
//! zig build swd-blink-bare -Dboard=pico2w
//! zig build watch-blink-bare        # run twice: `ticks` advancing is liveness
//! ```

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const hw = rp2350.hw;
const gpio = rp2350.gpio;
const bounded = rp2350.bounded;

/// Where the LED is. GP2 by default — a pin that is nothing else on either
/// board, so it needs no claim and cannot collide with the CYW43's reserved
/// four (23, 24, 25, 29) or with the debug UART on GP0/GP1.
const led_pin: rp2350.Pin = 2;

/// Roughly a quarter second at whatever the ROSC is running at. Not calibrated
/// and cannot be — see the header.
const loop_count: u32 = 400_000;

/// Read over SWD by `zig build watch-blink-bare`, and pinned at the base of
/// SRAM so it needs no symbols.
const Watch = extern struct {
    magic: u32,
    /// Blink half-cycles completed. **The liveness proof**: read it twice, and
    /// if it has moved the core is running whatever the LED is doing.
    ticks: u32,
    /// 1 if XOSC reported `STABLE` within the bound, 0 if it did not.
    ///
    /// **A 0 here is the answer to "why is every other example silent?"** —
    /// `clocks.init` waits on this bit with no bound at all, so a crystal that
    /// never starts hangs the firmware before its first `uart.print`.
    xosc_stable: u32,
    /// Polls taken to reach `STABLE`, or the limit if it never did. A value
    /// close to the limit is a crystal that is starting slowly, which is a
    /// different fault from one that is not starting.
    xosc_polls: u32,
    /// `XOSC.STATUS` as read after the attempt. Bit 31 is STABLE, bit 24 is
    /// BADWRITE, bit 12 is ENABLED.
    ///
    /// **ENABLED clear means the write was rejected.** CTRL fields are 12-bit
    /// magic codes and an invalid one is *silently ignored*, leaving the
    /// previous value — so a wrong code does not fault, it just quietly leaves
    /// the chip on the ROSC.
    xosc_status: u32,
    /// `XOSC.COUNT` after the attempt, for a crystal that is oscillating but
    /// not yet trusted.
    xosc_count: u32,
    /// The chip's package, as `silicon.package()` decodes it: proves SYSINFO is
    /// out of reset and readable, which is a second, independent sign of life
    /// that does not involve a pad.
    package: u32,
    /// Silicon stepping, same purpose.
    stepping: u32,
};

pub const watch_magic: u32 = 0x62_61_72_65; // "bare"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    // File scope, not inside `main` — see clock-check for what happens
    // otherwise.
    _ = &boot;
}

/// Busy-wait by counting, with the counter `volatile` so the optimiser cannot
/// delete the loop at `ReleaseSmall`. **A blink example whose delay was
/// optimised out looks like a board running far too fast**, which is a
/// convincing and entirely wrong diagnosis.
fn spin(n: u32) void {
    var i: u32 = 0;
    while (i < n) : (i += 1) {
        asm volatile ("" ::: .{ .memory = true });
    }
}

pub fn main() noreturn {
    // `.selftest` is NOLOAD and outside `.bss`, so `_reset` does not zero it
    // and a fresh image inherits the previous one's values field for field.
    // Initialise all of it before anything else runs, or the first reading is
    // the *last* example's data wearing this struct's field names.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .xosc_stable = 0,
        .xosc_polls = 0,
        .xosc_status = 0,
        .xosc_count = 0,
        .package = 0xFFFF_FFFF,
        .stepping = 0xFFFF_FFFF,
    };

    // Releases IO_BANK0 and PADS_BANK0. A pad register written while the block
    // is still in reset accepts the write and reads back zero, with no error.
    gpio.init();
    gpio.initOutput(led_pin);

    // Sign of life before anything that could possibly wait: three fast
    // flashes, so a board that gets this far and then stops still says so.
    var n: u32 = 0;
    while (n < 6) : (n += 1) {
        gpio.toggle(led_pin);
        spin(loop_count / 8);
    }
    gpio.clear(led_pin);

    watch.magic = watch_magic;

    // SYSINFO out of reset and answering: a second sign of life that involves
    // no pad and no clock.
    watch.package = @backingInt(rp2350.silicon.package());
    watch.stepping = @backingInt(rp2350.silicon.stepping());

    // --- the crystal, asked rather than assumed --------------------------
    //
    // Exactly `clocks.startXosc`'s sequence, with the one difference that
    // matters here: the wait is bounded, so a dead crystal is a reading rather
    // than a hang.
    hw.write(hw.Xosc.status, hw.Xosc.status_badwrite); // W1C; clears a stale flag
    hw.write(hw.Xosc.startup, rp2350.clocks.startup_delay);
    hw.write(hw.Xosc.ctrl, hw.Xosc.enable | hw.Xosc.freq_range_1_15mhz);

    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(hw.Xosc.status) & hw.Xosc.status_stable == 0) {
        s.tick() catch break;
    }
    watch.xosc_polls = s.count();
    watch.xosc_status = hw.read(hw.Xosc.status);
    watch.xosc_count = hw.read(hw.Xosc.count);
    watch.xosc_stable = @intFromBool(watch.xosc_status & hw.Xosc.status_stable != 0);

    // --- blink forever ----------------------------------------------------
    //
    // **Still on the ROSC**, whatever the crystal did. Switching clk_sys onto a
    // crystal that did not start is how a board stops dead, and this example
    // exists precisely to survive that case and report it.
    while (true) {
        gpio.toggle(led_pin);
        spin(loop_count);
        watch.ticks +%= 1;
    }
}
