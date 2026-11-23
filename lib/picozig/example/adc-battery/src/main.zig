//! adc-battery — one peripheral beyond `blink-hello`, and a debugger you
//! already have.
//!
//! Once a second it reads the battery voltage and the on-die temperature,
//! prints both over the UART, and writes them into a small block of RAM that a
//! Debug Probe can read **without halting the chip**:
//!
//! ```
//! zig build swd-adc-battery      # flash it
//! zig build watch-adc-battery    # read the block — run it twice
//! ```
//!
//! ## Why the RAM block, when the UART already prints the numbers
//!
//! Because the UART is a peripheral, and a peripheral is a thing that can be
//! broken. When output stops, the interesting question is whether the program
//! is stuck or merely silent — and that question cannot be answered from the
//! channel that has gone quiet. `watch` answers it: `ticks` advances between
//! two runs, or it does not.
//!
//! This is the debugging idiom to learn on this chip. It needs OpenOCD and nothing
//! else — no GDB, no toolchain beyond what flashing already required.
//!
//! **It reads a running core, and that is deliberate.** Never `-c halt` on
//! RP2350: it takes only cm0, the following `resume` fails with
//! `[rp2350.cm1] not halted`, and the chip is left stopped — which then looks
//! exactly like the hang you were trying to diagnose. Cortex-M memory reads go through the
//! AHB access port and work perfectly well on a running target, so halting buys
//! nothing here anyway.
//!
//! ## Why the battery, and not just any ADC channel
//!
//! Because it is the shortest honest demonstration of what `board.zig` is for.
//! Three facts are needed to turn a 12-bit count into a battery voltage, and
//! not one of them is guessable from the chip datasheet:
//!
//!   * VBAT sense is **GP43**, which is **ADC channel 3** (GP40-47 are ADC0-7).
//!   * The divider is **÷3** — published on the schematic and nowhere else.
//!   * It is RC-filtered and needs **250 ms** to settle from cold. Skip that
//!     wait and a full 4.2 V cell reads about 192 mV, i.e. "flat". That is not
//!     hypothetical; it is what this board actually did.
//!
//! The board file states all three, so this example asks for none of them.
//!
//! ## Things to change
//!
//! * `period_ms` — the sample interval.
//! * the averaging count in `adc.readAveraged(ch, 8)`. Drop it to 1 and watch
//!   the raw count get noisier; the first conversion after a channel change is
//!   discarded regardless, because the sample-and-hold is still carrying charge
//!   from the previous channel.
//! * read a different channel. Any pin you sample must be claimed `.analog` in
//!   `board.claims` first — the pin map is validated at compile time.

const rp2350 = @import("rp2350");
const board = @import("board");
// Side effects only: `boot` exports the vector table and `_reset`, which
// nothing references by name. Without the import the linker has no reason to
// include the startup code and the image has no entry point.
const boot = @import("boot");

const adc = rp2350.adc;
const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

/// How long between samples.
const period_ms: u32 = 1000;

// ---------------------------------------------------------------------------
// The watch block
// ---------------------------------------------------------------------------

/// Pinned by the linker to the base of SRAM (0x20000000), so `mdw` can read it
/// with no symbol table at all:
///
///   +0x00  magic         0xadcba7 once main() has finished bring-up
///   +0x04  ticks         increments once per sample — **the liveness proof**
///   +0x08  vbat_raw      last raw 12-bit count, before any conversion
///   +0x0c  vbat_mv       battery millivolts, i.e. raw scaled by the ÷3 divider
///   +0x10  temp_centi_c  on-die temperature, hundredths of a degree, signed
///
/// `extern` for a guaranteed C layout: field order and offsets are the whole
/// interface here, and Zig's default layout is explicitly allowed to reorder.
/// All-`u32` so every field is one `mdw` word — `temp_centi_c` is genuinely
/// signed and is bit-cast on the way in, which is why the watch step's output
/// needs reading as two's complement below freezing.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    vbat_raw: u32,
    vbat_mv: u32,
    temp_centi_c: u32,
};

pub const watch_magic: u32 = 0x00ad_cba7;

/// `.selftest` is `NOLOAD` and sits outside `.bss`, so `_reset`'s zeroing never
/// touches it and the block survives a warm reset — you can reset the target
/// and still read what it last held. The section is picozig's; see
/// `rp2350.ld`.
export var watch: Watch linksection(".selftest") = undefined;

// ---------------------------------------------------------------------------

pub fn main() noreturn {
    // Zero the magic first, so a chip that dies during bring-up is
    // distinguishable from one that never started. A stale magic from a
    // previous image in NOLOAD memory would be the most misleading thing this
    // block could possibly report.
    watch.magic = 0;

    // Crystal, then pads, then console — the same three calls as blink-hello,
    // and in the same order for the same reasons. `clocks.init()` also enables
    // the TICKS generator that TIMER0 counts on; without it every delay below
    // would hang forever.
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== adc-battery ===\n", .{});
    uart.print(.uart0, "board      {s}\n", .{board.name});

    // Brings the ADC out of reset and points clk_adc at the crystal. A
    // peripheral still held in reset accepts every register write and reads
    // back zero, with no error at all — hence `unreset` inside, spinning on
    // RESET_DONE.
    // Bounded since 2026-08-25: an ADC whose clock never arrived never reports
    // ready, and this example's whole job is to say what the hardware is doing.
    adc.init() catch {
        uart.print(.uart0, "\nADC never became ready — check clk_adc and RESET_DONE.\n", .{});
        while (true) asm volatile ("wfi");
    };

    const temp_ch = adc.tempChannel();
    uart.print(.uart0, "temp ch    {d} (package-dependent)\n", .{temp_ch});

    if (board.vbat_sense) |pin| {
        // Analogue input: digital input buffer OFF and no pulls. Clearing IE is
        // not tidiness — a pad left digitally enabled at a mid-rail analogue
        // voltage is erratum E9's ~120 uA leakage case exactly, and it would
        // also load the 500 kOhm divider being measured.
        adc.initPin(pin);
        uart.print(.uart0, "vbat       GP{d}, ADC{d}, divider /{d}\n", .{
            pin,
            board.vbat_adc_channel.?,
            board.vbat_divider,
        });

        // The RC filter on the divider. Cold boot only — the divider is
        // permanently connected, so the cap stays charged while the battery is.
        uart.print(.uart0, "settling   {d} ms\n", .{board.vbat_settle_ms});
        timer.delayMs(board.vbat_settle_ms);
    } else {
        // No battery sense on the Pico 2 W. The temperature half still works,
        // so this is a note rather than a build error.
        uart.print(.uart0, "vbat       not present on this board — temperature only\n", .{});
    }

    uart.print(.uart0, "watch      mdw 0x20000000 {d}\n\n", .{@sizeOf(Watch) / 4});
    watch.magic = watch_magic;

    while (true) {
        // The temperature sensor costs bias current, so it is enabled only
        // around the reading rather than left on.
        adc.setTempSensor(true);
        const temp_raw = adc.readAveraged(temp_ch, 8) catch {
            adc.setTempSensor(false);
            uart.print(.uart0, "conversion timed out\n", .{});
            timer.delayMs(1000);
            continue;
        };
        const temp_centi = adc.rawToTempCentiC(temp_raw);
        adc.setTempSensor(false);

        var raw: u12 = 0;
        var mv: u32 = 0;
        if (board.vbat_adc_channel) |ch| {
            // Eight conversions averaged, with the first discarded to let the
            // mux and sample-and-hold settle after the channel change.
            raw = adc.readAveraged(ch, 8) catch {
                uart.print(.uart0, "vbat conversion timed out\n", .{});
                timer.delayMs(1000);
                continue;
            };
            mv = adc.rawToMvThrough(raw, board.vbat_divider);
        }

        // Two's complement below zero, and `{d}` on the fraction would print
        // "-3.-4" without the abs. Integer maths throughout: this target has no
        // FPU enabled, and the build subtracts `fpregs` precisely so a stray
        // float cannot be emitted and hard-fault on first use.
        const whole = @divTrunc(temp_centi, 100);
        const frac: u32 = @abs(@rem(temp_centi, 100));
        const sign = if (temp_centi < 0 and whole == 0) "-" else "";

        uart.print(.uart0, "vbat {d:>4} mV (raw {d:>4})   temp {s}{d}.{d:0>2} C\n", .{
            mv,
            raw,
            sign,
            whole,
            frac,
        });

        watch.ticks +%= 1;
        watch.vbat_raw = raw;
        watch.vbat_mv = mv;
        watch.temp_centi_c = @bitCast(temp_centi);

        timer.delayMs(period_ms);
    }
}

comptime {
    // `boot` is imported for the linker's benefit, not this file's.
    // Referencing it stops anyone "tidying" the import away, which would
    // produce an image with no entry point and no diagnostic beyond a board
    // that does nothing.
    _ = &boot;
}
