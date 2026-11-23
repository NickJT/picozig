//! clock-check — what is clk_sys *actually* running at?
//!
//! ```
//! zig build swd-clock-check      # flash it
//! zig build watch-clock-check                      # read the block, twice
//! ```
//!
//! ## Why this exists
//!
//! Because a wrong system clock is the most quietly destructive thing that can
//! happen on this chip, and nothing reports it.
//!
//! Every divisor in the system is computed from `board.sys_hz`: UART baud, the
//! SPI prescaler, a PWM period, a pixel clock. If clk_sys is not what
//! `board.sys_hz` claims, none of those fail — they all come out wrong by the
//! same ratio, and present as a UART talking gibberish, a radio that will not
//! ACK, and a panel that displays noise. Three unrelated-looking faults with
//! one cause, and no diagnostic pointing at the clock.
//!
//! Worse, the obvious check is circular. Reading `PLL_SYS.FBDIV` back tells you
//! what *we wrote*, and `CS.LOCK` tells you the loop closed — neither says the
//! loop closed on the frequency intended. So this example uses the **FC0
//! frequency counter**, which counts the clock under test against clk_ref. That
//! is a measurement, and it can disagree with the plan.
//!
//! ## The three numbers that matter
//!
//! `sys_claimed` is what `board.sys_hz` says, computed at compile time from the
//! board's `clock_plan`. `sys_measured_khz` is what FC0 counted. **If those two
//! disagree, everything downstream is wrong** — and this example exists to make
//! that a number you can read rather than a symptom you have to infer.
//!
//! `ref_measured_khz` is the control. It should read 12000 on every board here,
//! and if it does not, the reference the other measurement was made against is
//! itself wrong and no reading means anything.
//!
//! ## Why it needs no board hardware
//!
//! It drives nothing. `blink-hello` needs an LED and a UART; `adc-battery`
//! needs a battery divider. This needs a crystal, which is the one peripheral a
//! board cannot omit — which makes it the right first thing to run on a board
//! nobody has run anything on yet, before it is known what else works.
//!
//! Everything it reports goes to the SWD-readable block rather than the UART,
//! for the same reason: a board being brought up for the first time may not
//! have a working console, and the question "what is my clock doing" is most
//! urgent exactly when nothing else is answering.
//!
//! **Never `-c halt` on this target.** Memory reads work on a running core, and
//! halting takes only cm0, leaves cm1 unhalted, and strands the chip looking
//! exactly like the fault you are chasing.

const rp2350 = @import("rp2350");
const board = @import("board");
const build_options = @import("build_options");
// Side effects only: `boot` exports the vector table and `_reset`, which
// nothing references by name.
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const hw = rp2350.hw;
const timer = rp2350.timer;

/// How long between heartbeats.
const period_ms: u32 = 1000;

/// The plan this image actually runs, from `-Dclock-pll-khz`. Zero — the
/// default — means the board's own choice, which on both Pico boards is the
/// crystal and no PLL.
///
/// **This exists to cold-boot-test an overclock**, because every prior "200 MHz
/// is fine on a Pico board" result came from `swd-*` runs that reset a warm
/// chip with settled rails. See `build.zig`.
///
/// Resolved at file scope because `clocks.init` takes its plan `comptime`.
const clock_plan: ?clocks.Plan = if (build_options.pll_khz != 0)
    clocks.Plan.require(build_options.pll_khz * 1000)
else
    board.clock_plan;

/// What this image believes clk_sys will be — **not** `board.sys_hz`, which is
/// the board's own plan and is wrong whenever `-Dclock-pll-khz` overrides it.
const claimed_khz: u32 = if (clock_plan) |p| p.sys_hz / 1000 else clocks.crystal_sys_hz / 1000;

/// All-`u32`, so every field is one `mdw` word.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// What `board.sys_hz` claims, in kHz — so it compares directly against
    /// the measurement below without a unit conversion in your head.
    sys_claimed_khz: u32,
    /// What FC0 counted for clk_sys. **This is the one that can surprise you.**
    sys_measured_khz: u32,
    /// The control: clk_ref, which should be the 12 MHz crystal.
    ref_measured_khz: u32,
    /// The PLL's own output, measured ahead of the clk_sys mux. Distinguishes
    /// "the PLL is wrong" from "the PLL is right but clk_sys is not on it".
    pll_measured_khz: u32,
    /// `PLL_SYS.CS`, for LOCK and REFDIV.
    pll_cs: u32,
    /// `CLK_SYS_SELECTED`, one-hot. 0x1 means clk_ref, 0x2 means aux (the PLL).
    clk_sys_selected: u32,
};

pub const watch_magic: u32 = 0x00c1_0c4b;

/// `.selftest` is `NOLOAD` and outside `.bss`, so `_reset`'s zeroing never
/// touches it and the block survives a warm reset.
export var watch: Watch linksection(".selftest") = undefined;

// ---------------------------------------------------------------------------

comptime {
    // `boot` is imported for the linker's benefit, not this file's, and the
    // reference has to be at **file scope**: inside `main` it is a runtime
    // no-op that does not force the module to be analysed. Get that wrong and
    // the link succeeds, warns "cannot find entry symbol _reset", and emits an
    // ELF with no loadable segments — which then fails in `elf2uf2` rather than
    // in the compiler, one step away from the actual mistake.
    _ = &boot;
}

pub fn main() noreturn {
    // **Initialise every field before anything else runs.** `.selftest` is
    // NOLOAD, so a freshly flashed image inherits the previous image's values
    // field for field, and they read as current data. That is not a
    // hypothetical: `cc1101-listen` once reported `packets = 0x14` before it
    // had received anything, because 0x14 was `cc1101-bringup`'s version byte
    // still sitting at that offset. Zero the magic first, so a chip that dies
    // during bring-up is distinguishable from one that never started.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .sys_claimed_khz = claimed_khz,
        .sys_measured_khz = 0,
        .ref_measured_khz = 0,
        .pll_measured_khz = 0,
        .pll_cs = 0,
        .clk_sys_selected = 0,
    };

    // The whole point of the exercise. On a board with no `clock_plan` this is
    // the crystal-direct sequence unchanged; on one that declares a plan — or
    // when `-Dclock-pll-khz` supplies one — it brings up the PLL and moves
    // clk_sys onto it.
    //
    // **If this image is going to wedge the board, it wedges here**, and the
    // wedge is a stopped clk_sys: `magic` stays 0, and SWD gets `Examination
    // failed` on both cores. That is the signature being hunted, not a fault.
    clocks.init(clock_plan);

    // Measure before touching anything else, so that a clock so wrong the rest
    // of bring-up would hang is still recorded. Ordered cheapest-to-trust
    // first: if `ref` is not 12000 the other two mean nothing, because clk_ref
    // is what they were counted against.
    watch.ref_measured_khz = clocks.measureKhz(hw.Clocks.fc0_src_clk_ref) orelse 0;
    watch.sys_measured_khz = clocks.measureKhz(hw.Clocks.fc0_src_clk_sys) orelse 0;
    watch.pll_measured_khz = clocks.measureKhz(hw.Clocks.fc0_src_pll_sys) orelse 0;
    watch.pll_cs = hw.read(hw.Pll.sys_cs);
    watch.clk_sys_selected = hw.read(hw.Clocks.clk_sys_selected);

    watch.magic = watch_magic;

    const sign_of_life: ?rp2350.Pin = board.led;
    if (sign_of_life) |p| gpio.initOutput(p);

    var on = false;
    while (true) {
        timer.delayMs(period_ms);
        on = !on;
        if (sign_of_life) |p| {
            if (on) gpio.set(p) else gpio.clear(p);
        }
        // Last, so that a tick advancing means the whole loop body ran.
        watch.ticks +%= 1;
    }
}
