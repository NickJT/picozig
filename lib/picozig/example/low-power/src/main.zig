//! low-power — a ladder of increasingly severe idle states, one per meter reading.
//!
//! **A measurement fixture, not a demonstration.** `power-floor` answers "what
//! does the board draw when the firmware does nothing?"; this answers "and how
//! much of what is left can firmware actually take away?" Each rung is
//! cumulative and selected at build time, so exactly one variable moves between
//! two readings:
//!
//! ```bash
//! zig build swd-low-power -Dpower-level=0     # baseline: equals power-floor
//! zig build swd-low-power -Dpower-level=1     # + core 1 held off
//! zig build swd-low-power -Dpower-level=2     # + unused clocks gated
//! zig build swd-low-power -Dpower-level=3     # + unused blocks held in reset
//! zig build watch-low-power                   # confirm it got where it claims
//! ```
//!
//! ## Why these three, and why before POWMAN
//!
//! A battery firmware on this platform measured **3.48 mA** attributable to
//! the clocked chip against a 200-400 uA target, and the roadmap's own note on it says three
//! cheaper things come before writing a POWMAN dormant path. This is those
//! three, each independently measurable:
//!
//!   1. **Core 1 is never parked.** The bootrom leaves it wherever it stopped,
//!      clocked, and a single-core firmware never says otherwise. One register.
//!   2. **Every peripheral clock is ungated at boot**, including the ones for
//!      blocks this firmware will never touch — USB, ADC, HSTX.
//!   3. **Blocks left out of reset draw current even when idle.**
//!
//! If those take the chip's share well below 1 mA, POWMAN dormant becomes an
//! optimisation rather than a necessity — which is a decision worth having
//! before writing it, not after.
//!
//! ## Reading it honestly
//!
//! **Take every reading the same way**:
//! battery only, USB disconnected (a charger swamps everything), probe
//! unhooked, and the `.selftest` block read over SWD *before* unhooking. That
//! last step is not optional here — **a board that faulted on the way to its
//! sleep loop draws very little and reads as a triumph.** `reached_sleep` is
//! the only thing that tells a sleeping board from a dead one, because nothing
//! here blinks and there is no UART.
//!
//! ## The trap, and why rung 1 ships disarmed
//!
//! **Rung 1 is commented out in the source.** It makes the board unflashable
//! over SWD, and an example must not put a board into a state it cannot get
//! itself out of — so `-Dpower-level=1` currently measures the same thing as
//! level 0 until someone reads the block comment at rung 1 and takes the
//! markers off. The recovery command is written out there.
//!
//! Why it is disarmed: OpenOCD's `rp2350.cfg`
//! examines and halts *both* cores; with `PROC1` forced off there is no core 1
//! to halt, and `program ... reset` fails with "timed out while waiting for
//! target halted / Unable to reset target". The image is fine, the chip is
//! fine, and every subsequent `zig build swd-*` fails until something clears
//! the bit.
//!
//! So every rung above 0 waits `-Dpower-delay` seconds first (default 15),
//! exactly as `flash-wedge` does: **there is always a window in which a plain
//! `zig build swd-low-power -Dpower-level=0` gets the board back.** Pass 0 to
//! remove the window, and mean it.
//!
//! If a board is already stuck, this clears it with no power cycle and no
//! BOOTSEL — the bit is memory-mapped and the debugger can reach it while the
//! core is halted:
//!
//! ```bash
//! openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
//!   -c "adapter speed 1000" -c "init" -c "mww 0x40018004 0" -c "exit"
//! ```
//!
//! A power cycle also clears it. Proven both ways on 2026-08-25.
//!
//! ## What it deliberately does not touch
//!
//! `clk_sys`, `clk_ref`, and the TICKS generators. Those carry the core and
//! TIMER0, and gating them would stop the very loop being measured. The
//! deepest rung here is still a running chip in `wfi` — turning the chip *off*
//! is POWMAN's job and is the next rung, not one of these.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const build_options = @import("build_options");

const hw = rp2350.hw;
const clocks = rp2350.clocks;
const timer = rp2350.timer;

/// How far up the ladder this build goes. Cumulative: 3 includes 1 and 2.
const level: u32 = build_options.level;

/// Seconds of ordinary, reflashable board before the rungs are applied.
/// See the header: rung 1 costs the SWD reset path.
const delay_s: u32 = build_options.delay_s;

const Watch = extern struct {
    magic: u32,
    /// Published so a reader never has to trust `build.zig`'s `watch_words`.
    words: u32,
    /// Which rung this image was built for.
    level: u32,
    /// **Set immediately before the `wfi` loop, and nothing else sets it.**
    /// A very low meter reading with this at 0 is a crashed board, not a
    /// sleeping one, and the two are indistinguishable on a meter.
    reached_sleep: u32,
    /// What was actually applied, one bit per rung: 1 core-1 off, 2 clocks
    /// gated, 4 blocks reset. **Separate from `level`** so a rung that silently
    /// did nothing is visible rather than assumed.
    applied: u32,
    /// `PSM.FRCE_OFF` read back. Bit 24 is PROC1.
    frce_off: u32,
    /// `CLK_PERI_CTRL` read back — bit 11 is ENABLE. The one gate that would
    /// break a UART if this example had one.
    clk_peri_ctrl: u32,
    /// `RESETS.RESET` read back, so the third rung is checkable against the
    /// bits it claims to have set.
    resets: u32,
    /// Counts up once a second through the safety window, then stops.
    /// **A reader that sees this still moving has time to reflash**; once it
    /// stops at `delay_s`, rung 1 has been applied and SWD reset is gone.
    window_s: u32,
};

pub const watch_magic: u32 = 0x6c_6f_70_77; // "lopw"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

/// Clocks this firmware will never use. **Not `clk_sys`, `clk_ref` or the
/// TICKS generators** — those carry the core and TIMER0.
const spare_clock_ctrls = [_]u32{
    hw.Clocks.clk_usb_ctrl,
    hw.Clocks.clk_adc_ctrl,
    hw.Clocks.clk_hstx_ctrl,
    // clk_peri last: it is the one a UART would be on, and putting it at the
    // end keeps the ordering obvious to anyone adding a console for debugging
    // and wondering where it went.
    hw.Clocks.clk_peri_ctrl,
};

/// Blocks held in reset at the deepest rung.
///
/// **Everything the firmware does not use, and nothing it does.** `io_bank0`,
/// `pads_bank0`, `timer0` and `syscfg` are absent on purpose: pads left in
/// reset float, which on A2 is erratum E9's leakage case and would *cost*
/// current rather than save it.
const spare_resets: u32 = hw.Resets.adc |
    hw.Resets.hstx |
    hw.Resets.i2c0 |
    hw.Resets.i2c1 |
    hw.Resets.pio0 |
    hw.Resets.pio1 |
    hw.Resets.pio2 |
    hw.Resets.pwm |
    hw.Resets.sha256 |
    hw.Resets.spi0 |
    hw.Resets.spi1 |
    hw.Resets.trng |
    hw.Resets.uart0 |
    hw.Resets.uart1;

pub fn main() noreturn {
    // `.selftest` is NOLOAD and survives a reset by design, so a fresh image
    // inherits the previous one's values field for field — and on this example
    // that would mean reading the *last* rung's `reached_sleep` as this one's.
    watch = .{
        .magic = 0,
        .words = @sizeOf(Watch) / 4,
        .level = level,
        .reached_sleep = 0,
        .applied = 0,
        .frce_off = 0,
        .clk_peri_ctrl = 0,
        .resets = 0,
        .window_s = 0,
    };

    // Crystal, not ROSC — otherwise every reading compares a different clock
    // source as well as a different program, and two variables move at once.
    // Same reasoning as `power-floor`, and the two must stay comparable.
    clocks.init(board.clock_plan);

    watch.magic = watch_magic;

    // **The safety window**, kept for whoever re-arms rung 1 above. With rung 1
    // commented out nothing here costs the SWD reset path, so this is currently
    // a pause and not a rescue — but it must stay, because the moment someone
    // uncomments that block it becomes the only thing standing between a
    // measurement run and a board that needs an external command to reflash.
    if (level >= 1) {
        var s: u32 = 0;
        while (s < delay_s) : (s += 1) {
            timer.delayMs(1000);
            watch.window_s = s + 1;
        }
    }

    // --- rung 1: park core 1 -- COMMENTED OUT ON PURPOSE ---------------------
    //
    // **Disabled 2026-08-25 on a standing rule: an example must not put the
    // board into a state it cannot get itself out of.** Forcing `PROC1` off
    // costs the SWD reset path — OpenOCD's `rp2350.cfg` halts *both* cores and
    // there is no core 1 to halt, so `program … reset` fails with "timed out
    // while waiting for target halted / Unable to reset target". The safety
    // window below helps, but once it closes the board needs an external
    // command or a power cycle, and that is exactly what this rule forbids.
    //
    // The measurement is still worth taking — that 3.48 mA has core 1
    // in it, unparked, doing nothing. So the line is kept, disarmed, and
    // uncommenting it is the interlock: anyone who reads this understands what
    // they are turning on and how to undo it.
    //
    // ## Recovery, written out so nobody has to re-derive it
    //
    // The bit is memory-mapped and the debugger can reach it while the core is
    // halted, so **this needs no power cycle and no BOOTSEL** — proven on
    // hardware 2026-08-25:
    //
    //     openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
    //       -c "adapter speed 1000" -c "init" -c "mww 0x40018004 0" -c "exit"
    //
    // `0x40018004` is `PSM.FRCE_OFF`; writing 0 releases core 1. Then reflash
    // normally. `zig build swd-rescue` and a power cycle both also clear it.
    //
    // if (level >= 1) {
    //     hw.setBits(hw.Psm.frce_off, hw.Psm.frce_off_proc1);
    //     watch.applied |= 1;
    // }

    // --- rung 2: gate the clocks nothing here uses --------------------------
    if (level >= 2) {
        for (spare_clock_ctrls) |ctrl| {
            hw.clrBits(ctrl, hw.Clocks.peri_enable);
        }
        watch.applied |= 2;
    }

    // --- rung 3: hold unused blocks in reset --------------------------------
    if (level >= 3) {
        // **No `unreset` afterwards, and no wait for RESET_DONE.** That
        // register reports blocks coming *out* of reset; going in has nothing
        // to wait for.
        hw.setBits(hw.Resets.reset, spare_resets);
        watch.applied |= 4;
    }

    // Read back what the hardware actually holds, not what was written. A
    // peripheral still in reset accepts writes and reads zero, and this
    // example's whole output is register state.
    watch.frce_off = hw.read(hw.Psm.frce_off);
    watch.clk_peri_ctrl = hw.read(hw.Clocks.clk_peri_ctrl);
    watch.resets = hw.read(hw.Resets.reset);

    watch.reached_sleep = 1;

    // `wfi` rather than a spin: a busy loop is the difference between measuring
    // an idle board and measuring the core running flat out, which is the first
    // thing `power-floor` established.
    while (true) asm volatile ("wfi");
}
