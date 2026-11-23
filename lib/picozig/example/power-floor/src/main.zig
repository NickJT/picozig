//! power-floor — what the board draws when the firmware does nothing at all.
//!
//! **A control, not a program.** It exists to answer one question with a meter:
//! of a board's idle current, how much is the firmware's doing and how much is
//! the board's? Everything else is measured by difference against it.
//!
//! The question came from a real measurement. A battery firmware idled at
//! 7.27 mA on a Pico LiPo 2; replacing its busy-wait main loop with `wfi`
//! brought that to 6.82 mA — a saving of 0.455 mA, about 6%. Core *execution*
//! turned out to be a twentieth of the budget, so the remaining 6.4 mA is
//! something else: the board's power LED, the PSRAM, the QSPI flash, the
//! AP7366 LDO's quiescent draw, the RP2350's always-on domains, or peripherals
//! left enabled. Sleep-mode work cannot tell those apart. This can.
//!
//! **What it deliberately does not do**: no GPIO, no ADC, no SPI, no PWM, and
//! **no UART** — a console is a peripheral, and one clocked peripheral is
//! exactly the kind of thing being measured. That is why the boot banner every
//! other example prints is absent here, and why the word count that
//! `build.zig`'s `watch_words` has to match is published in the block itself
//! rather than announced over a wire.
//!
//! It does initialise clocks, so that clk_sys is the board's 12 MHz crystal
//! rather than the ROSC. Without that the comparison would be against a
//! different clock source as well as a different program, and two variables
//! move at once.
//!
//! ## How to use it
//!
//!   1. `zig build swd-power-floor`
//!   2. `zig build watch-power-floor` — confirm `magic` and `reached_sleep`.
//!      **This step is not optional.** A board that faulted before the loop
//!      draws very little and reads as a triumph; the block is the only thing
//!      that distinguishes sleeping from dead, because nothing here blinks.
//!   3. Unhook the probe, disconnect USB, meter in series with the battery.
//!
//! Read against that firmware's number under identical conditions. If the
//! two are close, almost nothing is ours to fix in software and the answer is a
//! hardware inventory. If this one is far lower, the gap is what the firmware
//! leaves running, which is a much cheaper problem.

const rp2350 = @import("rp2350");
const board = @import("board");
// Imported for its side effects only: `boot` exports the vector table and
// `_reset`, which nothing references by name.
const boot = @import("boot");

comptime {
    _ = &boot;
}

const clocks = rp2350.clocks;

/// Published at the base of SRAM so `mdw 0x20000000 4` reads it with no symbol
/// table. Four words — keep `build.zig`'s `watch_words` in step.
const Watch = extern struct {
    magic: u32,
    /// `@sizeOf(Watch) / 4`, so a stale `watch_words` in `build.zig` is visible
    /// in the dump itself. There is no way to check that from the build.
    words: u32,
    /// What the board asked for, so a dump proves which clock the measurement
    /// was taken at.
    sys_hz: u32,
    /// 1 once the sleep loop has been entered. The whole point of the block.
    reached_sleep: u32,
};

pub const watch_magic: u32 = 0x0f10_0140;

export var watch: Watch linksection(".selftest") = undefined;

pub fn main() noreturn {
    // Zeroed first, so a warm reset cannot leave a stale pass behind.
    watch.magic = 0;

    clocks.init(board.clock_plan);

    watch = .{
        .magic = watch_magic,
        .words = @sizeOf(Watch) / 4,
        .sys_hz = board.sys_hz,
        .reached_sleep = 1,
    };

    // No interrupt is enabled anywhere, so nothing will ever wake this. That is
    // the intent: the deepest state reachable without touching POWMAN, held
    // indefinitely. `wfi` in a loop rather than once, because an exception —
    // a debugger attaching, say — would otherwise fall through into whatever
    // follows.
    while (true) asm volatile ("wfi");
}
