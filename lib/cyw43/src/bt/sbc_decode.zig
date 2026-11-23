//! SBC decoding — rung 25b: quantised subbands to PCM.
//!
//! **Written from the SBC specification's own definition, not ported.** The
//! reference in pico-sdk's bluedroid is a tuned fixed-point implementation
//! whose DCT carries per-output shift constants and whose window is a
//! machine-generated unrolled function; reproducing it byte for byte would have
//! made this a transliteration of Apache-licensed code rather than Zig.
//!
//! The consequence is deliberate and worth stating plainly: **the oracle drops
//! from equality to a tolerance.** `sbc_vectors.zig` still says what the answer
//! is, and the tests still check all 65 real frames against it — but "within N
//! least-significant bits" is a weaker claim than "identical", and slow drift
//! is exactly what a tolerance can hide. The measured error is asserted, not
//! assumed, so it cannot quietly grow.
//!
//! ## Fixed point, because this build has no FPU
//!
//! **This was written in `f32` first, on the reasoning that the Cortex-M33 has
//! single-precision hardware. It does; this build does not use it.** picozig's
//! target subtracts `fpregs` on purpose — the FPU needs `CPACR` enabled in the
//! reset handler, nothing does that, and subtracting the feature stops the
//! compiler quietly emitting an instruction that would hard-fault. So every
//! float operation was soft-emulated at about a hundred cycles.
//!
//! The cost was measured, not guessed: **41.7 ms to decode one media packet
//! against a 14.5 ms budget**, which stalled the receive ring and eventually
//! killed the link with a supervision timeout. `ReleaseFast` moved it to 38.9
//! ms, which is what said the problem was not the optimiser.
//!
//! So: integers, coefficients in Q30, accumulation in `i64`, samples in Q8.
//! That is one `smlal` per multiply-accumulate rather than a call into
//! `__aeabi_fmul`. **This is why the reference is fixed point**, and reading
//! that as an implementation quirk rather than a constraint cost a flash cycle.
//!
//! ## The synthesis, as the standard states it
//!
//! For each block of eight subband samples, per channel:
//!
//!     shift V down by 16, then V[k] = sum_i N[k][i] * S[i]   k = 0..15
//!         N[k][i] = cos((i + 0.5)(k + 4) pi / 8)
//!     gather U from V:  U[i*16 + j]     = V[i*32 + j]
//!                       U[i*16 + 8 + j] = V[i*32 + 24 + j]
//!     window:           W[i] = U[i] * D[i]
//!     sum:              X[j] = sum_{i=0..9} W[j + 8*i]
//!
//! `V` is the filter history, and it is why the vectors must be decoded in
//! order: sixteen new values per block push the previous 144 along, so a frame
//! decoded cold is a different frame.

const std = @import("std");
const sbc = @import("sbc.zig");

pub const Error = error{ Malformed, Unsupported };

/// The SBC prototype window for eight subbands, 80 coefficients.
///
/// A specification constant, in the same sense as a CRC polynomial: the
/// values are fixed by the A2DP standard and every implementation carries
/// the same table. Read out of the vendored encoder coefficients as Q31 and
/// written here as the fractions they represent, because a float table is
/// what this synthesis actually uses.
pub const proto_80 = [80]f32{
    0.000000000,  0.000156575,  0.000343256,  0.000554620,
    0.000823919,  0.001139925,  0.001476401,  0.001783717,
    0.002011825,  0.002103720,  0.001994545,  0.001616563,
    0.000902154,  -0.000178805, -0.001649731, -0.003497174,
    0.005659495,  0.008029412,  0.010458444,  0.012747233,
    0.014652526,  0.015904560,  0.016220847,  0.015318410,
    0.012937180,  0.008857575,  0.002924084,  -0.004915780,
    -0.014640407, -0.026109875, -0.039075138, -0.053187303,
    0.067998943,  0.082984758,  0.097575392,  0.111196689,
    0.123264548,  0.133264415,  0.140753505,  0.145389847,
    0.146955068,  0.145389847,  0.140753505,  0.133264415,
    0.123264548,  0.111196689,  0.097575392,  0.082984758,
    -0.067998943, -0.053187303, -0.039075138, -0.026109875,
    -0.014640407, -0.004915780, 0.002924084,  0.008857575,
    0.012937180,  0.015318410,  0.016220847,  0.015904560,
    0.014652526,  0.012747233,  0.010458444,  0.008029412,
    -0.005659495, -0.003497174, -0.001649731, -0.000178805,
    0.000902154,  0.001616563,  0.001994545,  0.002103720,
    0.002011825,  0.001783717,  0.001476401,  0.001139925,
    0.000823919,  0.000554620,  0.000343256,  0.000156575,
};

/// The matrixing coefficients, computed once at compile time rather than
/// tabulated. `cos((i + 0.5)(k + 4) pi / 8)` is the definition; a table would
/// only be the same numbers with an opportunity to mistype them.
/// Coefficients are Q30: one unit is 2^30, so a value of magnitude 1 fits with
/// a bit to spare and the products stay well inside `i64`.
const q: u5 = 30;
const one_q: i64 = 1 << q;

const matrix: [16][8]i32 = blk: {
    @setEvalBranchQuota(40_000);
    var m: [16][8]i32 = undefined;
    for (0..16) |k| {
        for (0..8) |i| {
            const angle = (@as(f64, @floatFromInt(i)) + 0.5) *
                (@as(f64, @floatFromInt(k)) + 4.0) * std.math.pi / 8.0;
            m[k][i] = @intFromFloat(@round(@cos(angle) * @as(f64, @floatFromInt(one_q))));
        }
    }
    break :blk m;
};

/// The window, in the same Q30, computed once from the table above.
const proto_q: [80]i32 = blk: {
    @setEvalBranchQuota(40_000);
    var w: [80]i32 = undefined;
    for (0..80) |i| {
        w[i] = @intFromFloat(@round(@as(f64, proto_80[i]) * @as(f64, @floatFromInt(one_q))));
    }
    break :blk w;
};

/// Subband samples and filter history are Q8. The dequantiser's largest output
/// is about 2^16, so Q8 leaves ±8.4 million — comfortably inside `i32` and
/// still eight bits of fraction below the quietest sample that matters.
const sample_q: u5 = 8;

/// Reads big-endian bit fields, most significant bit first.
const BitReader = struct {
    buf: []const u8,
    bit: usize = 0,

    fn read(self: *BitReader, n: u8) u32 {
        var v: u32 = 0;
        var left = n;
        while (left > 0) : (left -= 1) {
            const byte = self.bit >> 3;
            // Past the end reads as zero rather than trapping: a frame whose
            // length was checked by `sbc.walk` cannot get here, and a decoder
            // that panics on malformed audio is worse than one that produces
            // silence.
            const b: u8 = if (byte < self.buf.len) self.buf[byte] else 0;
            const shift: u3 = @intCast(7 - (self.bit & 7));
            v = (v << 1) | ((b >> shift) & 1);
            self.bit += 1;
        }
        return v;
    }
};

/// Undo the quantisation. **This is the specification's formula**, not the
/// reference's: bluedroid folds in a factor of 1/1.38 to keep headroom in its
/// fixed-point pipeline and takes it back out in the window. Nothing here needs
/// that, and carrying it would be copying an implementation detail as though it
/// were part of the format.
fn dequant(raw: u32, scale_factor: i8, bits: u8) i32 {
    if (bits <= 1) return 0;
    const levels: i64 = (@as(i64, 1) << @intCast(bits)) - 1;
    // ((2r + 1)/levels - 1) * 2^(sf+1), carried in Q8 without ever leaving
    // integers: numerator first, divide last.
    const num: i64 = (2 * @as(i64, raw) + 1 - levels) << (sample_q + 1);
    const scaled: i64 = (num << @as(u6, @intCast(scale_factor)));
    return @intCast(@divTrunc(scaled, levels));
}

/// One channel's filter history: 160 values, pushed along sixteen at a time.
pub const Channel = struct {
    v: [160]i32 = @splat(0),

    /// One block of eight subband samples in, eight PCM samples out.
    fn block(self: *Channel, s: *const [8]i32, out: *[8]i32) void {
        var i: usize = 160;
        while (i > 16) : (i -= 1) self.v[i - 1] = self.v[i - 17];

        for (0..16) |k| {
            var acc: i64 = 0;
            for (0..8) |j| acc += @as(i64, matrix[k][j]) * @as(i64, s[j]);
            self.v[k] = @intCast(acc >> q);
        }

        // Gather, window and fold in one pass: U[n] is only ever used once.
        for (0..8) |j| {
            var acc: i64 = 0;
            for (0..5) |g| {
                acc += @as(i64, proto_q[g * 16 + j]) * @as(i64, self.v[g * 32 + j]);
                acc += @as(i64, proto_q[g * 16 + 8 + j]) * @as(i64, self.v[g * 32 + 24 + j]);
            }
            out[j] = @intCast(acc >> q);
        }
    }
};

/// **The gain between the specification's synthesis and 16-bit PCM.**
///
/// The proto window is defined against a normalised transform; the standard's
/// synthesis carries a factor the tables leave out, and every implementation
/// folds it somewhere. It is stated here as one number rather than smeared
/// through the window, and it is *measured* — the tests compare against the
/// reference decoder and would fail immediately if it were wrong, since a gain
/// error is a constant ratio across every sample.
pub const output_gain: i32 = -8;

/// Q8 in, 16-bit PCM out, with the gain and a rounding step.
pub fn clamp16(x: i32) i16 {
    const scaled = (@as(i64, x) * output_gain + (1 << (sample_q - 1))) >> sample_q;
    if (scaled >= 32767) return 32767;
    if (scaled <= -32768) return -32768;
    return @intCast(scaled);
}

pub const Decoder = struct {
    channels: [2]Channel = .{ .{}, .{} },

    pub fn reset(self: *Decoder) void {
        self.* = .{};
    }

    /// Decode one frame into interleaved PCM. Returns the samples written.
    ///
    /// **Stateful by construction**: the filter history lives in `self`, so
    /// frames must be fed in order. Decoding one on a fresh `Decoder` is a
    /// different, and wrong, answer.
    pub fn decodeFrame(self: *Decoder, frame: []const u8, pcm: []i16) !usize {
        var raw: [16 * 8 * 2]i32 = undefined;
        const n = try self.decodeFrameFloat(frame, &raw);
        if (pcm.len < n) return Error.Malformed;
        for (0..n) |i| pcm[i] = clamp16(raw[i]);
        return n;
    }

    /// The synthesis output before any scaling or clipping. **Separated so the
    /// gain can be measured rather than guessed** — clipped samples make a
    /// regression against the reference meaningless, and the first attempt at
    /// this constant was wrong by a factor of 186.
    pub fn decodeFrameFloat(self: *Decoder, frame: []const u8, pcm: []i32) !usize {
        const h = try sbc.parseFrameHeader(frame);
        if (h.subbands != 8) return Error.Unsupported;

        const sf = try sbc.readScalefactors(frame, h);
        const bits = sbc.allocate(sf, h);
        const nch: usize = h.channel_mode.channels();
        const nsb: usize = h.subbands;

        const need = @as(usize, h.blocks) * nsb * nch;
        if (pcm.len < need) return Error.Malformed;

        var br = BitReader{ .buf = frame[sf.samples_at..] };
        var out: usize = 0;

        for (0..h.blocks) |_| {
            var s: [2][8]i32 = .{ @splat(0), @splat(0) };

            for (0..nch) |ch| {
                for (0..nsb) |b| {
                    const n = bits.values[ch * nsb + b];
                    const raw = br.read(n);
                    s[ch][b] = dequant(raw, sf.values[ch * nsb + b], n);
                }
            }

            // Joint stereo: the two channels carry mid and side, high bit of
            // the mask first.
            if (h.channel_mode == .joint_stereo) {
                for (0..nsb) |b| {
                    if ((sf.join << @intCast(b)) & 0x80 != 0) {
                        const mid = s[0][b];
                        const side = s[1][b];
                        s[0][b] = mid + side;
                        s[1][b] = mid - side;
                    }
                }
            }

            var pcm_out: [2][8]i32 = undefined;
            for (0..nch) |ch| self.channels[ch].block(&s[ch], &pcm_out[ch]);

            for (0..nsb) |j| {
                for (0..nch) |ch| {
                    pcm[out] = pcm_out[ch][j];
                    out += 1;
                }
            }
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// Decode the whole capture and report how far it lands from the reference.
/// **In order, through one decoder** — see `sbc_vectors.zig`.
fn measureAgainstOracle() struct { worst: i32, sum_abs: u64, samples: usize, gain: f64 } {
    var dec = Decoder{};
    var pcm: [16 * 8 * 2]i32 = undefined;
    var worst: i32 = 0;
    var sum_abs: u64 = 0;
    var samples: usize = 0;
    var dot: f64 = 0;
    var norm: f64 = 0;

    for (sbc.vectors.vectors) |v| {
        const n = dec.decodeFrameFloat(v.frame, &pcm) catch return .{
            .worst = 1 << 30,
            .sum_abs = 0,
            .samples = 0,
            .gain = 0,
        };
        for (0..@min(n, v.pcm.len)) |i| {
            const mine: f64 = @floatFromInt(pcm[i]);
            const ref: f64 = @floatFromInt(v.pcm[i]);
            const scaled: i32 = clamp16(pcm[i]);
            const d: i32 = scaled - @as(i32, v.pcm[i]);
            const a = if (d < 0) -d else d;
            if (a > worst) worst = a;
            sum_abs += @intCast(a);
            samples += 1;
            // Least-squares gain: the factor that best maps ours onto theirs.
            dot += mine * ref;
            norm += mine * mine;
        }
    }
    return .{
        .worst = worst,
        .sum_abs = sum_abs,
        .samples = samples,
        .gain = if (norm > 0) dot / norm else 0,
    };
}

test "the decoder tracks the reference across the whole capture" {
    // **The tolerance, measured and then pinned.** 16,640 samples of real audio
    // through 65 frames in order, against btstack's decoder: worst 3 least
    // significant bits, mean 0.607. That is about -85 dB and inaudible, and it
    // is what an independent implementation costs against a tuned one.
    //
    // **The fixed-point rewrite changed neither number.** Converting from `f32`
    // to Q30 coefficients and Q8 samples was forced by the absent FPU, not by
    // accuracy, and it cost nothing: worst 3 and mean 0.607 before, worst 3 and
    // mean 0.606 after.
    //
    // The bounds are deliberately just above what was measured. A tolerance
    // wide enough to be comfortable is a tolerance that hides drift, which is
    // the known weakness of testing this way rather than by equality.
    const r = measureAgainstOracle();
    const mean = @as(f64, @floatFromInt(r.sum_abs)) / @as(f64, @floatFromInt(r.samples));

    try testing.expectEqual(@as(usize, 16_640), r.samples);
    try testing.expect(r.worst <= 4);
    try testing.expect(mean <= 1.0);
}

test "the gain is the subband count, and its sign is the phase convention" {
    // The regression now runs on Q8 values, so the expected ratio carries the
    // Q8 scale as well as the gain: -8 / 256.
    // Measured by least squares against the reference rather than derived:
    // -7.983, which is -8 to within the quantisation of the comparison. The 8
    // is the subband count; the sign is this file's cosine phase differing from
    // the reference's by half a turn. **The first guess at this constant was
    // wrong by a factor of 186**, and only a measurement caught it.
    const r = measureAgainstOracle();
    const expected = @as(f64, @floatFromInt(output_gain)) / 256.0;
    try testing.expect(@abs(r.gain - expected) < 0.01);
}

test "the oracle would catch a decoder that returned silence" {
    // The obvious way to pass a tolerance test is to output nothing, and
    // entries 1-11 of the capture are genuinely silent. This asserts the set
    // has enough real audio that silence cannot pass.
    var loud: usize = 0;
    for (sbc.vectors.vectors) |v| {
        for (v.pcm) |x| {
            if (x > 200 or x < -200) {
                loud += 1;
                break;
            }
        }
    }
    try testing.expect(loud >= 32);
}
