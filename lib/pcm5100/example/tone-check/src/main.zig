//! tone-check — bring up the PCM5100 I2S engine and report what can be measured.
//!
//! **This is a measurement, not a demonstration.** It does not print that a
//! tone was produced, because the board cannot observe its own headphone jack.
//! What it prints is what it can actually read back:
//!
//!   - the clock divisor it computed, and the sample rate that follows from it
//!   - whether the state machine is enabled and its program counter is moving
//!   - the DMA read pointer, and the buffer index it lands in
//!   - the TX FIFO's sticky underrun flag
//!   - **late fills**: times the DMA reached the buffer while it was being
//!     written, which is the failure this board can actually have
//!
//! `late_fills` is the number that matters. clk_sys here is 12 MHz — the Pico
//! 2 W runs straight off the crystal — leaving 272 CPU cycles per sample, and
//! a fill that misses that deadline does **not** show up as an underrun: the
//! DMA always has a buffer, so it plays half-stale data and the FIFO never
//! stalls. Hence the wavetable generator below rather than `SineGenerator`.
//!
//! A pass is: `sm_running = 1`, `swaps` climbing at roughly 172 per second
//! (44100 / 256), `underruns = 0` and `late_fills = 0`. **A pass is not
//! evidence of sound** — that needs headphones, and until someone has
//! listened, nothing should claim it.
//!
//! Wiring: Pimoroni Pico Audio Pack on a Pico 2 W. DIN GP9, BCK GP10,
//! LRCK GP11, amp enable GP22.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const pcm5100 = @import("pcm5100");
const tone_config = @import("tone_config");

const plan: ?rp2350.clocks.Plan = if (tone_config.sys_mhz == 0)
    board.clock_plan
else
    rp2350.clocks.Plan.require(tone_config.sys_mhz * 1_000_000);

/// The clock actually running, which every divisor below must follow.
const sys_hz: u32 = if (plan) |p| p.sys_hz else board.sys_hz;

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const pio = rp2350.pio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    err_code: u32,
    /// Sample rate the clock plan actually delivers, in Hz.
    actual_rate: u32,
    /// `SMx_CLKDIV` as written.
    clkdiv: u32,
    /// Buffer swaps counted since start.
    swaps: u32,
    /// Times the TX FIFO ran dry. Catches a stopped DMA, **not** a late fill.
    underruns: u32,
    /// Times the DMA overtook the fill. Non-zero means audible distortion.
    late_fills: u32,
    /// 1 once the state machine has been enabled.
    sm_running: u32,
};

pub const watch_magic: u32 = 0x5100_0003;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

/// File scope, not a local: two DMA channels hold pointers into this for as
/// long as it plays. See `PicoAudio`'s doc comment.
/// One cycle of sine at half scale, baked into flash.
const tone_table = pcm5100.sineTable(
    @as(f32, @floatFromInt(tone_config.volume_pct)) / 100.0,
);

/// **Two octaves apart, so nobody has to judge a small difference.** 440 Hz is
/// an A; 1760 Hz is the A two octaves above. Which ear has which is not a
/// subtle call at that spacing, and the whole point is an answer nobody has to
/// hedge.
const split_left_hz: u32 = 440;
const split_right_hz: u32 = 1760;

var audio = pcm5100.PicoAudio.init(
    pcm5100.default_pins,
    pcm5100.default_sample_rate,
    board.sys_hz,
);

pub fn main() noreturn {
    clocks.init(plan);
    gpio.init();
    // Not `board.uartConfig()`: that derives the baud from the board's
    // *declared* clock, which `-Dsys-mhz` overrides.
    uart.init(.uart0, .{ .inst = .uart0, .tx = board.uart_tx, .rx = board.uart_rx, .clk_hz = sys_hz });

    uart.print(.uart0, "\n=== PCM5100 tone-check ===\nboard {s}, clk_sys {d} Hz\n", .{
        board.name,
        sys_hz,
    });

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .err_code = 0,
        .actual_rate = audio.plan.sample_rate_hz,
        .clkdiv = audio.plan.clkdiv,
        .swaps = 0,
        .underruns = 0,
        .late_fills = 0,
        .sm_running = 0,
    };

    uart.print(.uart0, "clock plan: clkdiv {x:0>8} (int {d} frac {d}), sm {d} Hz, frames {d} Hz, err {d} ppm\n", .{
        audio.plan.clkdiv,
        audio.plan.clkdiv >> 16,
        (audio.plan.clkdiv >> 8) & 0xff,
        audio.plan.pio_clock_hz,
        audio.plan.sample_rate_hz,
        audio.plan.errorPpm(pcm5100.default_sample_rate),
    });

    // Fill both buffers before anything is clocked out, so the first frames the
    // DAC sees are the tone rather than the zeroes `init` left behind.
    //
    // The table is built at comptime; at 12 MHz there is no room to compute a
    // sine per sample. `sample_rate_hz` and not the requested rate, so the
    // pitch follows the divisor that was actually programmed.
    var tone = pcm5100.WavetableGenerator.init(&tone_table, 440, audio.plan.sample_rate_hz);
    var tone_l = pcm5100.WavetableGenerator.init(&tone_table, split_left_hz, audio.plan.sample_rate_hz);
    var tone_r = pcm5100.WavetableGenerator.init(&tone_table, split_right_hz, audio.plan.sample_rate_hz);

    if (tone_config.split) {
        uart.print(.uart0,
            \\
            \\=== channel order check (roadmap 2.1) ===
            \\
            \\  LEFT  should be {d} Hz -- the LOW tone
            \\  RIGHT should be {d} Hz -- the HIGH tone, two octaves up
            \\
            \\Put the headphones on the right way round. If the high tone is in
            \\your left ear, `packStereoFrame` has the halves swapped and every
            \\stereo thing this driver has ever played was mirrored.
            \\
            \\Volume is {d}% of full scale, set by -Dvolume-pct.
            \\
        , .{ split_left_hz, split_right_hz, tone_config.volume_pct });
        pcm5100.fillStereoSplit(&audio.buffers[0], &tone_l, &tone_r);
        pcm5100.fillStereoSplit(&audio.buffers[1], &tone_l, &tone_r);
    } else {
        tone.fillStereoBuffer(&audio.buffers[0]);
        tone.fillStereoBuffer(&audio.buffers[1]);
    }

    audio.start() catch |err| {
        watch.err_code = 1;
        uart.print(.uart0, "start failed: {s}\n", .{@errorName(err)});
        while (true) timer.delayMs(1000);
    };

    watch.sm_running = 1;
    uart.print(.uart0, "state machine enabled on PIO{d} sm{d}, DMA {d}->{d}\n", .{
        audio.block.index(),
        audio.sm,
        audio.dma_data,
        audio.dma_restart,
    });

    // The flag is sticky and survives a reset, so anything latched before now
    // belongs to whatever ran last.
    audio.clearUnderrun();
    audio.enableAmplifier();
    uart.print(.uart0, "amplifier enabled on GP{d}\n", .{pcm5100.default_pins.amp_en});

    var swaps: u32 = 0;
    var underruns: u32 = 0;
    var late_fills: u32 = 0;
    var ticks: u32 = 0;

    while (true) {
        // Fill the buffer the DMA is not on. Naming it rather than re-asking
        // afterwards is the whole point: if the DMA has reached `target` by the
        // time the fill returns, the fill lost the race and some of what is
        // about to play is stale.
        const target = 1 - audio.playingIndex();
        if (tone_config.split) {
            pcm5100.fillStereoSplit(&audio.buffers[target], &tone_l, &tone_r);
        } else {
            tone.fillStereoBuffer(&audio.buffers[target]);
        }
        if (audio.playingIndex() == target) late_fills += 1;

        // Wait until the fresh buffer is the one playing.
        while (audio.playingIndex() != target) {}
        swaps += 1;

        if (audio.underrunSeen()) {
            underruns += 1;
            audio.clearUnderrun();
        }

        watch.swaps = swaps;
        watch.underruns = underruns;
        watch.late_fills = late_fills;

        // 44100 / 256 is ~172 swaps per second, so this is roughly once a
        // second without needing a timer alongside the audio loop.
        if (swaps % 172 == 0) {
            ticks += 1;
            watch.ticks = ticks;
            uart.print(.uart0, "[{d}s] swaps {d}, underruns {d}, late fills {d}\n", .{
                ticks,
                swaps,
                underruns,
                late_fills,
            });
        }
    }
}
