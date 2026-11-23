//! Texas Instruments PCM5100 / PCM5100A I2S audio DAC driver for RP2350.
//!
//! Freestanding. Drives the Pimoroni Pico Audio Pack: a PIO state machine
//! generates BCLK, LRCK and the serial data, and a self-restarting pair of DMA
//! channels feeds it from a double buffer with no CPU involvement between
//! refills.
//!
//! ## Status
//!
//! **Heard on hardware**: `app/bt-speaker` plays music through it, and the
//! channel order was confirmed by ear (below). `zig build test` covers the pure
//! parts — the program encoding, the clock arithmetic, the frame packer, the
//! sine generator. Nothing in this file prints a success it has not observed: a reported success that
//! nothing checked has already been mistaken for a working driver once.
//!
//! ## The wire format, and where the bit delay lives
//!
//! Philips I2S: LRCK low is the left channel, data is MSB-first, and **LRCK
//! changes one bit period before the MSB of the word it selects**. That one
//! period is the whole of the "I2S vs left-justified" difference, and getting
//! it wrong does not silence the DAC — it shifts every sample by a bit and
//! sounds like distortion rather than like a bug.
//!
//! The program below puts the LRCK transition on the same falling BCLK edge
//! that starts the *last* bit period of the previous channel, which is what
//! produces that delay. See `program` for the instruction-by-instruction
//! derivation.
//!
//! ## Clocking
//!
//! `SCK` (pin 12) is tied to GND on the Audio Pack, so the PCM5100A runs its
//! internal PLL off BCLK/LRCK and there is no MCLK to generate. 32 BCLK
//! periods per frame at two PIO instructions per period is 64 PIO cycles per
//! frame — see `ClockPlan`.

const std = @import("std");
const rp2350 = @import("rp2350");

const hw = rp2350.hw;
const gpio = rp2350.gpio;
const pio = rp2350.pio;
const dma = rp2350.dma;

pub const synth = @import("synth.zig");

/// Pimoroni Pico Audio Pack default pinout.
pub const default_pins = BoardPins{};

/// Configurable board pin layout.
///
/// **`lrck` must be `bclk + 1`.** The state machine drives both from one
/// two-bit side-set field, and side-set pins are consecutive by construction —
/// `start` checks this rather than letting it fail as a dead channel.
pub const BoardPins = struct {
    data: rp2350.Pin = 9,
    bclk: rp2350.Pin = 10,
    lrck: rp2350.Pin = 11,
    amp_en: rp2350.Pin = 22,
};

/// Ways a pin layout can be unusable. Both are compile-time-shaped mistakes
/// that the hardware reports as silence, which is why they are checked.
pub const Error = error{
    /// `lrck` is not `bclk + 1`; the side-set field cannot reach both.
    NonConsecutiveClockPins,
    /// A signal pin is at GP32 or above. **PIO's pin-base fields are five
    /// bits**, so a state machine addresses a 32-pin window; reaching the upper
    /// GPIOs of an RP2350B needs `pio.setGpioBase`, which this driver does not
    /// set. Truncating instead would drive some other pin 32 places down, which
    /// presents as a dead DAC and an unrelated pin toggling.
    PinOutOfPioRange,
} || rp2350.bounded.Error;

/// Check a layout against what the state machine can actually address.
///
/// Pure, so the rules are host-tested rather than only discoverable by running
/// `start` on a board. `amp_en` is exempt: it is a plain GPIO, not a PIO pin.
pub fn validatePins(pins: BoardPins) Error!void {
    if (@as(u32, pins.lrck) != @as(u32, pins.bclk) + 1) return Error.NonConsecutiveClockPins;
    if (pins.data >= 32 or pins.lrck >= 32) return Error.PinOutOfPioRange;
}

/// Default audio sample rate in Hz (44.1 kHz CD quality).
pub const default_sample_rate: u32 = 44100;

/// Stereo frames per buffer. Two of these alternate, so the CPU has one
/// buffer's worth of time — 5.8 ms at 44.1 kHz — to fill the idle one.
pub const buffer_sample_count: usize = 256;

/// One buffer of packed stereo frames.
pub const AudioBuffer = [buffer_sample_count]u32;

/// How long one buffer lasts at `default_sample_rate`, in microseconds.
///
/// **The deadline, and it is worth having a name.** A consumer refilling on
/// buffer swaps has this long per buffer and no longer; miss it and the DMA
/// replays what is already there, with `underrunSeen` still false. `bt-speaker`
/// missed it about once a second and the only visible trace was its fill count
/// running slightly under `elapsed / this`.
pub const buffer_period_us: u64 = (buffer_sample_count * 1_000_000) / default_sample_rate;

// ---------------------------------------------------------------------------
// Frame packing
// ---------------------------------------------------------------------------

/// Pack 16-bit left and right samples into the 32-bit word the state machine
/// shifts out.
///
/// Left occupies the high half because the program shifts **left**, MSB first,
/// and LRCK is low — the left channel — for the first 16 bits of the frame.
pub fn packStereoFrame(left: i16, right: i16) u32 {
    const u_l: u16 = @bitCast(left);
    const u_r: u16 = @bitCast(right);
    return (@as(u32, u_l) << 16) | @as(u32, u_r);
}

/// Inverse of `packStereoFrame`.
pub fn unpackStereoFrame(frame: u32) struct { left: i16, right: i16 } {
    const u_l: u16 = @truncate(frame >> 16);
    const u_r: u16 = @truncate(frame & 0xFFFF);
    return .{
        .left = @bitCast(u_l),
        .right = @bitCast(u_r),
    };
}

// ---------------------------------------------------------------------------
// The PIO program
// ---------------------------------------------------------------------------

/// Side-set width: bit 0 is BCLK, bit 1 is LRCK. `sideset_base` is therefore
/// the BCLK pin, and LRCK is the pin above it.
pub const sideset_bits: u3 = 2;

/// Side-set field for a given LRCK/BCLK pair, with no delay. Every instruction
/// in the program is one cycle, so a delay here would stretch one half of one
/// bit period and nothing wants that.
fn side(comptime lrck: u1, comptime bclk: u1) u5 {
    return pio.sideSet(sideset_bits, (@as(u32, lrck) << 1) | bclk, 0);
}

/// The I2S transmit program. Eight instructions, 64 cycles per stereo frame.
///
/// Each bit period is two instructions: the first drives BCLK low (and, being
/// the falling edge, is where data changes and where LRCK may transition), the
/// second drives it high (the edge on which the DAC samples).
///
/// ```text
/// addr                                 side  LRCK BCLK
///   0 left:  out pins, 1               0b00   0    0    15 bits, MSB first
///   1        jmp x--, left             0b01   0    1
///   2        out pins, 1               0b10   1    0    16th bit; LRCK -> right
///   3        set x, 14                 0b11   1    1
///   4 right: out pins, 1               0b10   1    0    15 bits
///   5        jmp x--, right            0b11   1    1
///   6        out pins, 1               0b00   0    0    16th bit; LRCK -> left
///   7 entry: set x, 14                 0b01   0    1    wraps to 0
/// ```
///
/// **`jmp x--` with x = 14 runs the body 15 times, not 14** — it jumps while x
/// is non-zero and decrements afterwards, so the pass that finds x = 0 still
/// executes. 15 in the loop plus the peeled one below it is the 16 bits of a
/// channel, and the peeled instruction exists precisely so LRCK can change on
/// the last bit rather than after it.
///
/// The two `set x, 14` instructions are not overhead: each is the BCLK-high
/// half of that 16th bit period, which is why the frame is exactly 64 cycles.
pub const program = [_]u16{
    pio.out(.pins, 1, side(0, 0)),
    pio.jmp(.x_dec, 0, side(0, 1)),
    pio.out(.pins, 1, side(1, 0)),
    pio.set(.x, 14, side(1, 1)),
    pio.out(.pins, 1, side(1, 0)),
    pio.jmp(.x_dec, 4, side(1, 1)),
    pio.out(.pins, 1, side(0, 0)),
    pio.set(.x, 14, side(0, 1)),
};

/// Where the program is entered: the last instruction, whose wrap starts the
/// first left-channel bit at address 0 on the next cycle.
pub const entry_offset: u5 = 7;

/// PIO cycles per stereo frame: 32 BCLK periods, two instructions each.
pub const cycles_per_frame: u32 = 64;

// ---------------------------------------------------------------------------
// Clocking
// ---------------------------------------------------------------------------

/// The state machine divisor for a wanted sample rate, and what that divisor
/// actually delivers.
///
/// Pure arithmetic, host-tested below — the divisor is the one number here
/// that is wrong silently. A state machine clocked wrongly still produces a
/// clean I2S stream; the DAC's PLL locks to it and plays at the wrong pitch.
pub const ClockPlan = struct {
    /// `SMx_CLKDIV`: integer part in 31:16, fractional 256ths in 15:8.
    clkdiv: u32,
    /// The state machine clock this divisor produces.
    pio_clock_hz: u32,
    /// The frame rate that follows from it — what the DAC will really run at.
    sample_rate_hz: u32,

    pub fn compute(sys_hz: u32, sample_rate: u32) ClockPlan {
        // 16.8 fixed point, so scale by 256 before dividing.
        const wanted: u64 = @as(u64, cycles_per_frame) * @as(u64, sample_rate);
        const scaled: u64 = @as(u64, sys_hz) * 256;
        var fixed: u64 = if (wanted == 0) 256 else scaled / wanted;
        // **A CLKDIV integer part of 0 means 65536, not 1.** Clamping to 1.0
        // keeps an over-fast request from wrapping into the slowest possible
        // clock, which is the worst way for this to be wrong.
        if (fixed < 256) fixed = 256;
        if (fixed > 0xff_ffff) fixed = 0xff_ffff;
        const pio_hz: u32 = @intCast(scaled / fixed);
        return .{
            .clkdiv = @intCast(fixed << 8),
            .pio_clock_hz = pio_hz,
            .sample_rate_hz = pio_hz / cycles_per_frame,
        };
    }

    /// Signed parts-per-million error against the requested rate. Audio
    /// tolerates a good deal of this — the DAC follows BCLK — but it is worth
    /// being able to read rather than assume.
    pub fn errorPpm(self: ClockPlan, wanted: u32) i32 {
        if (wanted == 0) return 0;
        const got: i64 = self.sample_rate_hz;
        const want: i64 = wanted;
        return @intCast(@divTrunc((got - want) * 1_000_000, want));
    }
};

// ---------------------------------------------------------------------------
// Tone generation
// ---------------------------------------------------------------------------

/// Entries in a `SineTable`. 256 makes the table index the top byte of the
/// phase accumulator, so stepping it costs an add and a shift.
pub const wavetable_len: usize = 256;

pub const SineTable = [wavetable_len]i16;

/// One cycle of sine, scaled by `volume`. Intended to be called at comptime —
/// the point of the table is that no float work happens while audio is playing.
pub fn sineTable(volume: f32) SineTable {
    // 256 iterations of float work is more than the default comptime branch
    // budget, and the failure is a compile error at the *call* site rather than
    // here, which reads as the caller's fault.
    @setEvalBranchQuota(wavetable_len * 20);
    var t: SineTable = undefined;
    for (&t, 0..) |*v, i| {
        const phase = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(wavetable_len));
        const wave = std.math.sin(phase * 2.0 * std.math.pi);
        const scaled = std.math.clamp(wave * volume * 32767.0, -32768.0, 32767.0);
        v.* = @intFromFloat(scaled);
    }
    return t;
}

/// Integer table-lookup tone generator.
///
/// **This exists because `SineGenerator` is too slow to feed this DAC on the
/// board it is written for.** A Pico 2 W runs clk_sys straight off the crystal
/// at 12 MHz — `lib/picozig`'s board file makes that choice deliberately — which
/// leaves **272 CPU cycles per sample** at 44.1 kHz. `std.math.sin` plus the
/// float multiplies around it does not fit in that, and the way it fails is
/// nasty: the DMA never starves, it just plays a half-written buffer, so the
/// symptom is distortion and `underrunSeen` stays false throughout. See
/// `PicoAudio.underrunSeen`.
///
/// This costs an add, a shift and a load per sample.
pub const WavetableGenerator = struct {
    table: *const SineTable,
    /// Phase as a 0.32 fraction of a cycle. The top 8 bits index the table, and
    /// the accumulator wraps on overflow, which is exactly the phase wrap.
    phase: u32 = 0,
    /// Phase increment per sample, same fixed-point scale.
    step: u32,

    pub fn init(table: *const SineTable, frequency_hz: u32, sample_rate: u32) WavetableGenerator {
        return .{ .table = table, .step = phaseStep(frequency_hz, sample_rate) };
    }

    /// `frequency / sample_rate`, as a 0.32 fixed-point fraction. Pure, and
    /// host-tested — a wrong step is a wrong pitch and nothing else complains.
    pub fn phaseStep(frequency_hz: u32, sample_rate: u32) u32 {
        if (sample_rate == 0) return 0;
        const num: u64 = @as(u64, frequency_hz) << 32;
        return @truncate(num / @as(u64, sample_rate));
    }

    pub fn getNextSample(self: *WavetableGenerator) i16 {
        const index: usize = self.phase >> 24;
        self.phase +%= self.step;
        return self.table[index];
    }

    pub fn fillStereoBuffer(self: *WavetableGenerator, out: []u32) void {
        for (out) |*word| {
            const sample = self.getNextSample();
            word.* = packStereoFrame(sample, sample);
        }
    }
};

/// Fill a buffer from **two independent generators**, one per channel.
///
/// **The channel order is settled: left really is the high half.** Confirmed by
/// ear on 2026-09-15 against powered speakers, by a channel check: 400 Hz on the right for 5 s, 1 kHz on the left for 2 s,
/// silence either side of both.
///
/// It stayed open so long because `fillStereoBuffer` writes the same sample to
/// both halves, so its output is mono however the halves are wired — and mono
/// sounds perfect the wrong way round. A host test had asserted the order since
/// the beginning and nothing this driver could play would have contradicted it.
///
/// **Two pitches, not one, and that was the difference.** A first attempt used
/// 1 kHz on both channels and was reported as genuinely hard to call: two
/// speakers a short distance apart playing the same tone do not localise,
/// because the room puts the image where it likes. Asking *which note* rather
/// than *which side* takes the listener's judgement out of the measurement.
///
/// Level is the `volume` the generators' table was built with — see
/// `sineTable`. There is deliberately no second attenuation knob here: one
/// place to set the level is enough, and **this gets played into headphones on
/// someone's head**, so the caller should be looking at that number rather than
/// at a shift buried in a fill loop.
pub fn fillStereoSplit(out: []u32, left: *WavetableGenerator, right: *WavetableGenerator) void {
    for (out) |*word| {
        word.* = packStereoFrame(left.getNextSample(), right.getNextSample());
    }
}

/// Floating-point sine phase accumulator.
///
/// **Too slow to fill a buffer in real time on a 12 MHz Pico 2 W** — see
/// `WavetableGenerator`, which is what a tone on this board should use. Kept
/// because it is the readable reference the table is checked against, and
/// because a board running a PLL has cycles to spare.
pub const SineGenerator = struct {
    frequency: f32 = 440.0,
    phase: f32 = 0.0,
    volume: f32 = 0.5,

    pub fn getNextSample(self: *SineGenerator, sample_rate: f32) i16 {
        if (sample_rate <= 0.0) return 0;
        const phase_inc = self.frequency / sample_rate;
        self.phase += phase_inc;
        if (self.phase >= 1.0) {
            self.phase -= @floor(self.phase);
        }

        const wave = std.math.sin(self.phase * 2.0 * std.math.pi);
        const out_f = wave * self.volume * 32767.0;
        const clamped = std.math.clamp(out_f, -32768.0, 32767.0);
        return @intFromFloat(clamped);
    }

    pub fn fillStereoBuffer(self: *SineGenerator, out: []u32, sample_rate: f32) void {
        for (out) |*word| {
            const s = self.getNextSample(sample_rate);
            word.* = packStereoFrame(s, s);
        }
    }
};

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// A running I2S output.
///
/// **This must live at a stable address for as long as it plays.** Two DMA
/// channels hold pointers into `buffers` and `addr_ring`; a `PicoAudio` that
/// is moved or that goes out of scope leaves the hardware reading whatever
/// took its place. A file-scope `var` in the application is the intended home.
///
/// Default PIO block is **PIO1**, because `lib/cyw43`'s gSPI transport occupies
/// PIO0 and an A2DP sink needs both at once. DMA channels default to 4 and 5,
/// clear of the low channels a display scan-out typically takes.
pub const PicoAudio = struct {
    pins: BoardPins,
    sample_rate: u32,
    plan: ClockPlan,

    block: pio.Block = .pio1,
    sm: u32 = 0,
    /// Where in the block's shared instruction memory the program is loaded.
    program_offset: u5 = 0,

    /// Feeds the TX FIFO from one buffer.
    dma_data: u32 = 4,
    /// Re-points and retriggers `dma_data` at the other buffer.
    dma_restart: u32 = 5,

    buffers: [2]AudioBuffer = @splat(@as([buffer_sample_count]u32, @splat(0))),

    /// The two buffer addresses, walked circularly by `dma_restart`.
    /// `align(8)` is load-bearing: the ring wraps by masking address bits, so a
    /// misaligned ring reads its neighbours instead of wrapping.
    addr_ring: [2]u32 align(8) = .{ 0, 0 },

    comptime {
        if (@sizeOf([2]u32) != 8) @compileError("addr_ring must be 8 bytes to match ring_bits = 3");
    }

    pub fn init(pins: BoardPins, sample_rate: u32, sys_hz: u32) PicoAudio {
        return .{
            .pins = pins,
            .sample_rate = sample_rate,
            .plan = ClockPlan.compute(sys_hz, sample_rate),
        };
    }

    // -- lifecycle ---------------------------------------------------------

    /// Bring up the state machine and start the DMA pair. Returns with clocks
    /// running and silence (all-zero buffers) streaming.
    ///
    /// **The amplifier is deliberately not enabled here.** Enabling it before
    /// BCLK is stable puts a click through the headphones; call
    /// `enableAmplifier` once this has returned.
    pub fn start(self: *PicoAudio) Error!void {
        try validatePins(self.pins);
        const data_base: u5 = @intCast(self.pins.data);
        const bclk_base: u5 = @intCast(self.pins.bclk);

        pio.init(self.block);
        dma.init();

        // PIO owns the output enable of any pin muxed to it, so the directions
        // have to be set through the state machine — see `setConsecutivePindirs`.
        gpio.initFunctionOutput(self.pins.data, self.block.funcsel());
        gpio.initFunctionOutput(self.pins.bclk, self.block.funcsel());
        gpio.initFunctionOutput(self.pins.lrck, self.block.funcsel());

        pio.loadProgramRelocated(self.block, self.program_offset, &program);
        pio.configure(self.block, self.sm, self.smConfig());
        pio.setConsecutivePindirs(self.block, self.sm, data_base, 1, true);
        pio.setConsecutivePindirs(self.block, self.sm, bclk_base, 2, true);
        pio.prepare(self.block, self.sm, self.program_offset + entry_offset);

        // DMA before enable: `prepare` clears the FIFO, and starting the
        // machine on an empty FIFO stalls it immediately.
        self.startDma();
        pio.setEnabled(self.block, self.sm, true);
    }

    /// Stop the stream and mute. Safe to call on a device that never started.
    ///
    /// **Can now fail, and that failure is the interesting one.** The abort
    /// below is bounded; `error.Timeout` here means a channel would not come to
    /// rest, which is the documented shape of the poisoning that survives a
    /// warm reset and a reflash. Before this it was an unbounded spin, so the
    /// same condition presented as a board that had simply stopped.
    pub fn stop(self: *PicoAudio) rp2350.bounded.Error!void {
        self.disableAmplifier();
        pio.setEnabled(self.block, self.sm, false);
        // **Both channels named.** Erratum RP2350-E5: aborting one member of a
        // chain can fire the other's trigger, so an abort naming only the data
        // channel restarts the pair it was meant to stop.
        try dma.abort((@as(u16, 1) << @truncate(self.dma_data)) |
            (@as(u16, 1) << @truncate(self.dma_restart)));
    }

    /// **Only valid after `validatePins`.** The casts below narrow a `Pin` to
    /// the 5 bits PIO's base fields hold, and `start` is the only caller.
    fn smConfig(self: *const PicoAudio) pio.SmConfig {
        return .{
            .wrap_bottom = self.program_offset,
            .wrap_top = self.program_offset + entry_offset,
            .out_base = @intCast(self.pins.data),
            .out_count = 1,
            .sideset_base = @intCast(self.pins.bclk),
            .sideset_count = sideset_bits,
            // **MSB first**, which is not the hardware default and is what I2S
            // requires. `packStereoFrame` puts the left sample in the high half
            // to match.
            .out_shiftdir = .left,
            .autopull = true,
            .pull_threshold = 32,
            // The RX FIFO is dead weight on a transmit-only machine, and
            // donating it doubles the depth to 8 words — 181 us of slack at
            // 44.1 kHz before an underrun becomes audible.
            .fifo_join = .tx,
            .clkdiv = self.plan.clkdiv,
        };
    }

    // -- DMA ---------------------------------------------------------------

    fn dataConfig(self: *const PicoAudio) dma.Config {
        return .{
            .size = .word,
            .incr_read = true,
            .incr_write = false,
            .treq = pio.txDreq(self.block, self.sm),
            .chain_to = self.dma_restart,
        };
    }

    /// Arm the pair and start playing buffer 0.
    ///
    /// The restart channel copies one word — the *other* buffer's address —
    /// into the data channel's `AL3_READ_ADDR_TRIG`, which both re-points and
    /// retriggers it. Its own read pointer walks `addr_ring` with `ring_bits`
    /// set, so the two buffers alternate forever with no interrupt and no CPU
    /// write: a display scan-out's mechanism with a two-entry ring.
    fn startDma(self: *PicoAudio) void {
        self.addr_ring[0] = @intFromPtr(&self.buffers[0]);
        self.addr_ring[1] = @intFromPtr(&self.buffers[1]);

        // **Entry 1, not 0.** The data channel is handed buffer 0 by hand
        // below, so the first restart owes the caller buffer 1; starting the
        // ring at 0 replays buffer 0 and leaves the pair permanently one
        // buffer behind. A real, measured bug in a display scan-out first.
        hw.write(hw.Dma.readAddr(self.dma_restart), @intFromPtr(&self.addr_ring[1]));
        hw.write(hw.Dma.writeAddr(self.dma_restart), hw.Dma.al3ReadAddrTrig(self.dma_data));
        hw.write(hw.Dma.transCount(self.dma_restart), 1);
        // **`al1Ctrl`, the non-triggering alias.** Writing CTRL_TRIG here would
        // fire one transfer immediately, consuming a ring entry before the
        // chain has run once.
        hw.write(hw.Dma.al1Ctrl(self.dma_restart), (dma.Config{
            .size = .word,
            .incr_read = true,
            .incr_write = false,
            // log2(2 entries * 4 bytes). Must track `addr_ring`'s size and
            // alignment together; see the comptime check on the field.
            .ring_bits = 3,
        }).word(self.dma_restart));

        dma.start(
            self.dma_data,
            self.dataConfig(),
            self.addr_ring[0],
            self.block.base() + hw.Pio.txf(self.sm),
            buffer_sample_count,
        );
    }

    // -- buffers -----------------------------------------------------------

    /// Which buffer the DMA is reading right now, from its live read pointer.
    /// Transfers remaining in the buffer the DMA is reading.
    ///
    /// **This is the reliable handoff signal, and the address is not.** The
    /// count walks down through the buffer and is *reloaded* when the restart
    /// channel re-triggers the data channel, so a reading larger than the last
    /// one means exactly one thing: a buffer was handed over. There are no
    /// ranges to straddle and no moment where the answer is ambiguous.
    ///
    /// **Corrected 2026-08-23, by measurement rather than reasoning.** This
    /// said both address-based readings below reported handoffs that had not
    /// happened, `playingIndex` at roughly twice the true rate. It does not:
    /// `app/bt-speaker` measured **172.3 fills a second against a predicted
    /// 172.27** over half a minute, with 2 to 6 late fills across whole runs.
    /// The 830 us detections that looked like spurious edges were a start-up
    /// artefact. `playingBuffer`'s null-between-buffers ambiguity is real, but
    /// it only bites a caller that treats null as a third state instead of
    /// skipping it.
    ///
    /// **The failure that actually costs audio is a fill that never happened**,
    /// and no reading here can see it: two buffers is a parity, so a gap longer
    /// than `buffer_period_us` can contain two flips and leave the index
    /// unchanged. Time the gap between fills instead.
    pub fn transfersRemaining(self: *const PicoAudio) u32 {
        return hw.read(hw.Dma.transCount(self.dma_data));
    }

    /// The data channel's current read address, raw. For diagnostics that want
    /// to see what the DMA is doing rather than be told.
    pub fn readAddress(self: *const PicoAudio) u32 {
        return hw.read(hw.Dma.readAddr(self.dma_data));
    }

    /// Which buffer the DMA is reading, or **null while it is between them**.
    ///
    /// `playingIndex` below cannot express "neither": it asks whether the read
    /// address lies inside buffer 1 and answers 0 for everything else. The
    /// moment the data channel finishes buffer 1 its address sits one past the
    /// end — outside both — and reads as buffer 0. **A caller watching for
    /// changes therefore sees a flip that did not happen**, refills the buffer
    /// the DMA is about to read, and hears silence where the audio was.
    ///
    /// Measured in `app/bt-speaker`: detections 830 us apart, where a
    /// 256-frame buffer at 44.1 kHz turns over every 5,805 us. Prefer this;
    /// `playingIndex` is kept because `tone-check` reads it and a tone survives
    /// the ambiguity.
    pub fn playingBuffer(self: *const PicoAudio) ?usize {
        const addr = hw.read(hw.Dma.readAddr(self.dma_data));
        const size = @sizeOf(AudioBuffer);
        const base0 = @intFromPtr(&self.buffers[0]);
        if (addr >= base0 and addr < base0 + size) return 0;
        const base1 = @intFromPtr(&self.buffers[1]);
        if (addr >= base1 and addr < base1 + size) return 1;
        return null;
    }

    pub fn playingIndex(self: *const PicoAudio) usize {
        const addr = hw.read(hw.Dma.readAddr(self.dma_data));
        const base1 = @intFromPtr(&self.buffers[1]);
        const end1 = base1 + @sizeOf(AudioBuffer);
        return if (addr >= base1 and addr < end1) 1 else 0;
    }

    /// The buffer the DMA is not reading — the one that is safe to fill.
    pub fn idleBuffer(self: *PicoAudio) *AudioBuffer {
        return &self.buffers[1 - self.playingIndex()];
    }

    /// Spin until the DMA moves to the other buffer — bounded, and it reports.
    ///
    /// The intended loop is `fill(idleBuffer()); try awaitSwap();` — fill the
    /// idle one, then wait for the hardware to start playing it, at which point
    /// the buffer that just finished becomes idle. Filling without waiting
    /// races the DMA into the buffer being written.
    ///
    /// **This is the wait that made the I2S poisoning look like a hang.**
    /// Whichever of `tone-check` and `app/bt-speaker` runs second gets no
    /// audio: the DMA is left with `BUSY` clear and `TRANS_COUNT` frozen, so
    /// the read address never moves, so this predicate is never false. An
    /// unbounded spin turns that into a silent stop with no diagnostic — which
    /// is most of why the condition cost an afternoon to characterise.
    ///
    /// `error.Timeout` here means **the engine has stopped, not that audio is
    /// late**: the limit is a whole transfer's worth of polls against a buffer
    /// that turns over every 5.8 ms. A caller that gets it should report the
    /// diagnostic block rather than retry — see `underflowed` and
    /// `transfersRemaining`, which say *how* it stopped.
    pub fn awaitSwap(self: *PicoAudio) rp2350.bounded.Error!void {
        var s = rp2350.bounded.Spin.init(rp2350.bounded.transfer_polls);
        const start_index = self.playingIndex();
        while (self.playingIndex() == start_index) try s.tick();
    }

    // -- diagnostics -------------------------------------------------------

    /// True if the TX FIFO has run dry since the flag was last cleared.
    ///
    /// **This does not detect a late refill, and that is the failure you should
    /// expect.** The DMA pair always has a buffer to read, so the FIFO is fed
    /// whether or not the CPU finished writing it; missing the deadline plays
    /// half-stale data and leaves this flag false. What it does catch is the
    /// DMA itself stopping — a bus error, an aborted channel, a chain that
    /// never re-armed.
    ///
    /// To catch a late fill, compare `playingIndex` before and after writing:
    /// if the DMA reached the buffer you were filling, you missed it.
    ///
    /// Sticky, and it survives a block reset, so clear it before timing a run
    /// or you are reading some earlier program's stalls.
    pub fn underrunSeen(self: *const PicoAudio) bool {
        return pio.txStalled(self.block, self.sm);
    }

    pub fn clearUnderrun(self: *const PicoAudio) void {
        pio.clearFifoDebug(self.block, self.sm);
    }

    // -- amplifier ---------------------------------------------------------

    /// Enable the PAM8908 headphone amplifier (GP22 high).
    ///
    /// Call this *after* `start`, so the DAC is already being clocked.
    pub fn enableAmplifier(self: *const PicoAudio) void {
        gpio.initOutput(self.pins.amp_en);
        gpio.set(self.pins.amp_en);
    }

    /// Mute the PAM8908 (GP22 low).
    pub fn disableAmplifier(self: *const PicoAudio) void {
        gpio.initOutput(self.pins.amp_en);
        gpio.clear(self.pins.amp_en);
    }
};

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "pin validation accepts the Audio Pack layout" {
    try validatePins(default_pins);
}

test "pin validation rejects clock pins the side-set cannot reach" {
    // Side-set pins are consecutive by construction, so LRCK has to sit
    // directly above BCLK. Swapping them is the plausible version of this
    // mistake and it must not reach the hardware.
    try testing.expectError(Error.NonConsecutiveClockPins, validatePins(.{
        .data = 9,
        .bclk = 11,
        .lrck = 10,
    }));
    try testing.expectError(Error.NonConsecutiveClockPins, validatePins(.{
        .data = 9,
        .bclk = 10,
        .lrck = 12,
    }));
}

test "pin validation rejects pins outside the state machine's 32-pin window" {
    try testing.expectError(Error.PinOutOfPioRange, validatePins(.{
        .data = 40,
        .bclk = 10,
        .lrck = 11,
    }));
    try testing.expectError(Error.PinOutOfPioRange, validatePins(.{
        .data = 9,
        .bclk = 32,
        .lrck = 33,
    }));
    // GP22 for the amplifier is a plain GPIO and is not subject to this.
    try validatePins(.{ .data = 9, .bclk = 10, .lrck = 11, .amp_en = 47 });
}

test "wavetable phase step is frequency over sample rate" {
    // 440 Hz at 44117 Hz should advance the accumulator by 440/44117 of a full
    // turn each sample. Checked as a ratio so the fixed-point scale is the
    // thing under test, not a magic constant.
    const step = WavetableGenerator.phaseStep(440, 44_117);
    const turns_per_sample = @as(f64, @floatFromInt(step)) / 4294967296.0;
    try testing.expectApproxEqAbs(440.0 / 44117.0, turns_per_sample, 1e-9);

    // A full turn per sample is the Nyquist-and-beyond edge; it must not wrap
    // to something small.
    try testing.expectEqual(@as(u32, 0), WavetableGenerator.phaseStep(440, 0));
}

test "wavetable completes one cycle in the expected number of samples" {
    const table = comptime sineTable(1.0);
    var gen = WavetableGenerator.init(&table, 441, 44_100);

    // 441 Hz at 44.1 kHz is 100 samples per cycle. The step is truncated rather
    // than rounded, so the accumulator lands just *short* of the wrap by at
    // most one count per sample — 100 out of 2^32, or 0.4 ppm of pitch. This is
    // a bound and not an equality for that reason, and the bound is the number
    // worth knowing: it is where the tuning error of this generator comes from.
    for (0..100) |_| _ = gen.getNextSample();
    const short_of_wrap = 0 -% gen.phase;
    try testing.expect(short_of_wrap <= 100);
    try testing.expect(short_of_wrap > 0);
}

test "wavetable tracks the float sine it replaces" {
    // The table is the fast path; `SineGenerator` is the readable one. If they
    // disagree by much, the table is wrong.
    const table = comptime sineTable(1.0);
    var quarter: usize = 0;
    while (quarter < wavetable_len) : (quarter += 1) {
        const phase = @as(f32, @floatFromInt(quarter)) / @as(f32, @floatFromInt(wavetable_len));
        const want = std.math.sin(phase * 2.0 * std.math.pi) * 32767.0;
        const got: f32 = @floatFromInt(table[quarter]);
        try testing.expect(@abs(want - got) <= 1.0);
    }
}

test "a split fill puts the left generator in the high half" {
    // **The claim hardware is being asked to confirm**, stated here so the two
    // agree on what "left" means before anyone puts headphones on. If the ear
    // disagrees with this, the fault is in the PIO program or the wiring --
    // not here.
    const table = sineTable(0.5);
    var lo = WavetableGenerator.init(&table, 440, 44_100);
    var hi = WavetableGenerator.init(&table, 1760, 44_100);

    var buf: [64]u32 = @splat(0);
    fillStereoSplit(&buf, &lo, &hi);

    // Reference generators, run independently to the same point.
    var ref_lo = WavetableGenerator.init(&table, 440, 44_100);
    var ref_hi = WavetableGenerator.init(&table, 1760, 44_100);
    for (buf) |word| {
        const f = unpackStereoFrame(word);
        try testing.expectEqual(ref_lo.getNextSample(), f.left);
        try testing.expectEqual(ref_hi.getNextSample(), f.right);
    }

    // And the two halves must actually differ, or the test proves nothing --
    // which is exactly the trap `fillStereoBuffer` fell into.
    var differs = false;
    for (buf) |word| {
        const f = unpackStereoFrame(word);
        if (f.left != f.right) differs = true;
    }
    try testing.expect(differs);
}

test "the table's volume is the only level control, and it bounds both channels" {
    // **Because this ends up in headphones.** A quiet table must produce quiet
    // samples on both halves, not just the one anyone happened to check.
    const quiet = sineTable(0.1);
    var lo = WavetableGenerator.init(&quiet, 440, 44_100);
    var hi = WavetableGenerator.init(&quiet, 1760, 44_100);
    var buf: [256]u32 = @splat(0);
    fillStereoSplit(&buf, &lo, &hi);

    const ceiling: i16 = @intFromFloat(0.1 * 32767.0 + 1);
    for (buf) |word| {
        const f = unpackStereoFrame(word);
        try testing.expect(f.left <= ceiling and f.left >= -ceiling);
        try testing.expect(f.right <= ceiling and f.right >= -ceiling);
    }
}

test "wavetable fills a stereo buffer with both channels equal" {
    const table = comptime sineTable(0.5);
    var gen = WavetableGenerator.init(&table, 440, 44_100);

    var buf: [64]u32 = undefined;
    gen.fillStereoBuffer(&buf);

    var non_zero: usize = 0;
    for (buf) |word| {
        const s = unpackStereoFrame(word);
        try testing.expectEqual(s.left, s.right);
        if (word != 0) non_zero += 1;
    }
    try testing.expect(non_zero > 0);
}

test "pack and unpack stereo frame" {
    const left: i16 = 12345;
    const right: i16 = -23456;

    const frame = packStereoFrame(left, right);
    const unpacked = unpackStereoFrame(frame);

    try testing.expectEqual(left, unpacked.left);
    try testing.expectEqual(right, unpacked.right);
}

test "left sample occupies the half that shifts out first" {
    // The program shifts left, MSB first, and the first 16 bits of a frame are
    // sent while LRCK is low — the left channel. So the left sample must be the
    // high half, or the channels swap.
    try testing.expectEqual(@as(u32, 0x1234_0000), packStereoFrame(0x1234, 0));
    try testing.expectEqual(@as(u32, 0x0000_5678), packStereoFrame(0, 0x5678));
}

test "i2s program encodes the frame this driver claims" {
    // Eight instructions, and the frame length follows from them: 15 loop
    // passes plus one peeled bit per channel, two cycles each.
    try testing.expectEqual(@as(usize, 8), program.len);

    const bits_per_channel = 15 + 1;
    try testing.expectEqual(@as(u32, 64), 2 * 2 * bits_per_channel);
    try testing.expectEqual(cycles_per_frame, 2 * 2 * bits_per_channel);

    // Both loop counters are set to 14, which runs their bodies 15 times.
    try testing.expectEqual(pio.set(.x, 14, side(1, 1)), program[3]);
    try testing.expectEqual(pio.set(.x, 14, side(0, 1)), program[7]);

    // The jumps target the first instruction of their own loop.
    try testing.expectEqual(pio.jmp(.x_dec, 0, side(0, 1)), program[1]);
    try testing.expectEqual(pio.jmp(.x_dec, 4, side(1, 1)), program[5]);
}

test "i2s program puts the LRCK edge one bit before the MSB" {
    // The two instructions that carry an LRCK change are the peeled `out`s at
    // 2 and 6 — the 16th bit of a channel, not the first bit of the next one.
    // That one-period offset is the I2S bit delay; without it this is
    // left-justified format and every sample is shifted.
    try testing.expectEqual(pio.out(.pins, 1, side(1, 0)), program[2]);
    try testing.expectEqual(pio.out(.pins, 1, side(0, 0)), program[6]);

    // And the bit that follows each of them is the new channel's MSB, driven
    // with LRCK already at its new level.
    try testing.expectEqual(pio.out(.pins, 1, side(1, 0)), program[4]);
    try testing.expectEqual(pio.out(.pins, 1, side(0, 0)), program[0]);
}

test "every instruction drives BCLK on alternate cycles" {
    // Bit 0 of the side-set field is BCLK. Reading it back out of the encoded
    // instructions catches a program whose clock has a gap in it — which would
    // still look like plausible I2S on a scope for 63 out of 64 cycles.
    const delay_bits: u3 = 5 - sideset_bits;
    for (program, 0..) |instr, i| {
        const ds: u5 = @truncate((instr >> 8) & 0x1f);
        const bclk: u32 = (ds >> delay_bits) & 1;
        try testing.expectEqual(@as(u32, @intCast(i % 2)), bclk);
    }
}

test "clock plan hits 44.1 kHz from 150 MHz within a few parts per million" {
    const plan = ClockPlan.compute(150_000_000, 44_100);

    // 150e6 * 256 / (64 * 44100) = 13605.4 -> 53 + 37/256.
    try testing.expectEqual(@as(u32, 53), plan.clkdiv >> 16);
    try testing.expectEqual(@as(u32, 37), (plan.clkdiv >> 8) & 0xff);

    try testing.expect(plan.sample_rate_hz >= 44_090 and plan.sample_rate_hz <= 44_110);
    const ppm = plan.errorPpm(44_100);
    try testing.expect(ppm > -200 and ppm < 200);
}

test "clock plan refuses to wrap into a divisor of zero" {
    // A sample rate the system clock cannot reach would divide to less than
    // 1.0, and an integer part of 0 means 65536 — the slowest clock available,
    // not the fastest. Clamped instead.
    const plan = ClockPlan.compute(1_000_000, 44_100);
    try testing.expectEqual(@as(u32, 1), plan.clkdiv >> 16);
    try testing.expectEqual(@as(u32, 0), (plan.clkdiv >> 8) & 0xff);
    try testing.expectEqual(@as(u32, 1_000_000 / 64), plan.sample_rate_hz);
}

test "clock plan tolerates a zero sample rate" {
    const plan = ClockPlan.compute(150_000_000, 0);
    try testing.expect(plan.clkdiv >> 16 >= 1);
    try testing.expectEqual(@as(i32, 0), plan.errorPpm(0));
}

test "sine generator bounds and frequency" {
    var gen = SineGenerator{
        .frequency = 440.0,
        .volume = 0.8,
    };

    var min_s: i16 = 32767;
    var max_s: i16 = -32768;

    for (0..100) |_| {
        const s = gen.getNextSample(44100.0);
        if (s < min_s) min_s = s;
        if (s > max_s) max_s = s;
    }

    try testing.expect(min_s <= -25000);
    try testing.expect(max_s >= 25000);
}

test "synth engine channel ADSR envelope" {
    var ch = synth.AudioChannel{
        .attack_ms = 10,
        .decay_ms = 50,
        .sustain = 32767,
        .release_ms = 50,
        .volume = 32767,
        .frequency = 440.0,
    };

    // Initially idle
    try testing.expectEqual(synth.EnvelopeState.idle, ch.env_state);
    try testing.expectEqual(@as(i16, 0), ch.getNextSample(44100.0));

    // Trigger attack
    ch.triggerAttack();
    try testing.expectEqual(synth.EnvelopeState.attack, ch.env_state);

    // Advance 5ms -> mid attack
    for (0..220) |_| {
        _ = ch.getNextSample(44100.0);
    }
    try testing.expect(ch.env_level > 0.3 and ch.env_level < 0.8);
}

test "synth engine 5-channel buffer generation" {
    var synth_engine = synth.Synth.init();
    var buf: [64]u32 = @as([64]u32, @splat(0));

    synth_engine.updatePlayback(0);
    synth_engine.fillBuffer(&buf, 44100.0);

    var non_zero: usize = 0;
    for (buf) |word| {
        if (word != 0) non_zero += 1;
    }

    try testing.expect(non_zero > 0);
}
