//! pio-tick — five rungs between "a state machine runs" and "a scan-out runs".
//!
//! ```
//! zig build swd-pio-tick               # flash it
//! zig build swd-pio-tick -Dtick-pll    # ...at 200 MHz instead of the crystal
//! zig build watch-pio-tick             # read the block (run it twice)
//! ```
//!
//! **The stages themselves live in `picozig`'s `pio_selftest` module**, not
//! here. This file is the shell: a clock plan, a `.selftest` block, and a
//! heartbeat. That split exists because the first version of this example was
//! all in `main`, passed on a Pico LiPo 2, and then could not be built for the
//! board it was written to bisect — a board package that brings its own
//! `board`, which this file imports. That package carries the other shell
//! over the same module.
//!
//! See `../../../src/pio_selftest.zig` for what the five stages are and why.
//!
//! **Never `-c halt` on this target.** It stops cm0 only, the following `resume`
//! fails with `[rp2350.cm1] not halted`, and the chip is left stopped. Memory
//! reads work on a running core.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const pio_selftest = @import("pio_selftest");
const build_options = @import("build_options");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const pio = rp2350.pio;
const timer = rp2350.timer;

/// The clock this ladder runs at. **`-Dtick-pll` is what makes this image a
/// control rather than a demonstration.**
///
/// The panel it exists to bisect against runs at 200 MHz off the PLL; both Pico
/// boards run at 12 MHz on the crystal. A ladder that only ever passed at
/// 12 MHz would leave the clock as an untested variable — and "the control did
/// not cover the variable that actually differed" is a mistake a display
/// scan-out investigation made more than once.
///
/// `clocks.Plan` is chip-level and knows nothing about boards, so asking any
/// RP2350 for 200 MHz is legitimate even where the board's own plan is the
/// crystal. `sys_khz` in the block is FC0-*measured*, so the result says which
/// of the two actually ran rather than which was requested.
///
/// It must be resolved at file scope because `clocks.init` takes its plan
/// `comptime` — a deliberate choice there, since every divisor downstream is
/// computed from it.
const clock_plan: ?clocks.Plan = if (build_options.use_pll)
    clocks.Plan.require(200_000_000)
else
    board.clock_plan;

/// PIO1, because that is the block a display scan-out used. Nothing here
/// depends on which, and running on the same block as the thing this is a rung
/// beneath removes one difference for free.
const block: pio.Block = .pio1;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// clk_sys as measured, not as requested. Stage E's expectation divides by
    /// it, so a wrong value here would make a correct machine look broken.
    sys_khz: u32,
    result: pio_selftest.Result,
};

pub const watch_magic: u32 = 0x00b1_7104;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    // `.selftest` is NOLOAD and `_reset` does not zero it, so a fresh image
    // inherits the previous one's values field for field. Zeroed in full rather
    // than partially, because a stale word here reads as a result.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .sys_khz = 0,
        .result = @import("std").mem.zeroes(pio_selftest.Result),
    };

    clocks.init(clock_plan);
    const sys_khz = clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_sys) orelse 0;
    watch.sys_khz = sys_khz;

    watch.result = pio_selftest.run(.{
        .block = block,
        .sys_hz = sys_khz * 1000,
    });
    watch.magic = watch_magic;

    const sign_of_life: ?rp2350.Pin = board.led;
    if (sign_of_life) |p| gpio.initOutput(p);

    var on = false;
    while (true) {
        timer.delayMs(500);
        on = !on;
        if (sign_of_life) |p| {
            if (on) gpio.set(p) else gpio.clear(p);
        }
        watch.ticks +%= 1;
    }
}
