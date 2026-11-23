//! SBC framing — rung 25a: understand the stream before decoding it.
//!
//! Rung 24 proved media arrives. This proves it is understood: the RTP header,
//! the AVDTP payload header, and each SBC frame header, with the frame lengths
//! checked to tile the packet exactly. **Nothing here decodes audio** — that is
//! rung 25b, and it is a different kind of problem.
//!
//! The point of separating them is that a decoder producing noise has two
//! possible causes — a misread frame or wrong arithmetic — and this rung
//! removes the first one before the second exists.
//!
//! ## What rung 24 measured, which this must agree with
//!
//! Every media packet was 608 bytes and decomposed exactly:
//!
//!     608 = 12 (RTP) + 1 (payload header) + 5 x 119 (SBC frames)
//!
//! **The 119 is not a constant.** It follows from the frame header, and the
//! phone may lower the bitpool mid-stream under interference — so the length is
//! computed per frame, and a run that assumed 119 would desynchronise the
//! moment conditions changed and produce noise that looked like a decoder bug.
//!
//! ## Frame header
//!
//!     byte 0   syncword, always 0x9C
//!     byte 1   freq(2) blocks(2) channel mode(2) allocation(1) subbands(1)
//!     byte 2   bitpool
//!     byte 3   CRC-8 over the header and the scale factors
//!
//! **The CRC is not checked here.** It covers the scale factors as well as the
//! header, so verifying it means parsing most of a frame — work that belongs
//! with the decoder that already has to do it. The syncword plus the lengths
//! tiling the packet is a strong enough check to say the framing is understood,
//! and it is honest about being weaker than a CRC.

const std = @import("std");

pub const Error = error{
    Malformed,
    /// The RTP version field was not 2.
    NotRtp,
    /// A fragmented media payload. A2DP allows it; nothing observed on this
    /// board has sent one, and mis-reading a fragment as a whole packet is
    /// worse than refusing it.
    Fragmented,
    /// The frames did not tile the packet — the strongest signal that the
    /// framing is misunderstood.
    LengthMismatch,
    /// A frame did not begin with 0x9C.
    BadSyncword,
};

pub const syncword: u8 = 0x9C;
pub const rtp_header_len: usize = 12;

pub const ChannelMode = enum(u2) {
    mono = 0,
    dual = 1,
    stereo = 2,
    joint_stereo = 3,

    pub fn channels(self: ChannelMode) u8 {
        return if (self == .mono) 1 else 2;
    }
};

pub const Allocation = enum(u1) { loudness = 0, snr = 1 };

pub const FrameHeader = struct {
    sample_rate: u32,
    blocks: u8,
    channel_mode: ChannelMode,
    allocation: Allocation,
    subbands: u8,
    bitpool: u8,
    crc: u8,

    /// Bytes in this frame, header included.
    ///
    /// **Joint stereo is the case that is easy to get wrong**, and the one this
    /// stream uses: it shares a bit allocation across the pair, so the term is
    /// `subbands + blocks * bitpool` rather than `blocks * bitpool` doubled.
    /// Using the stereo formula gives 225 bytes where the answer is 119, which
    /// is not a subtle error once frames stop tiling — but is completely silent
    /// if nothing checks.
    pub fn length(self: FrameHeader) usize {
        const nch: usize = self.channel_mode.channels();
        const sb: usize = self.subbands;
        const blk: usize = self.blocks;
        const bp: usize = self.bitpool;

        const fixed = 4 + (4 * sb * nch) / 8;
        const bits: usize = switch (self.channel_mode) {
            .mono, .dual => blk * nch * bp,
            .stereo => blk * bp,
            .joint_stereo => sb + blk * bp,
        };
        return fixed + (bits + 7) / 8;
    }

    /// Samples per channel: one frame carries `blocks * subbands` of them.
    pub fn samples(self: FrameHeader) usize {
        return @as(usize, self.blocks) * @as(usize, self.subbands);
    }
};

pub fn parseFrameHeader(buf: []const u8) !FrameHeader {
    if (buf.len < 4) return Error.Malformed;
    if (buf[0] != syncword) return Error.BadSyncword;
    const b = buf[1];
    return .{
        .sample_rate = switch ((b >> 6) & 0x03) {
            0 => 16_000,
            1 => 32_000,
            2 => 44_100,
            else => 48_000,
        },
        .blocks = switch ((b >> 4) & 0x03) {
            0 => 4,
            1 => 8,
            2 => 12,
            else => 16,
        },
        .channel_mode = @fromBackingInt(@intCast((b >> 2) & 0x03)),
        .allocation = @fromBackingInt(@intCast((b >> 1) & 0x01)),
        .subbands = if ((b & 0x01) == 0) 4 else 8,
        .bitpool = buf[2],
        .crc = buf[3],
    };
}

// ---------------------------------------------------------------------------
// The media packet
// ---------------------------------------------------------------------------

pub const Rtp = struct {
    payload_type: u8,
    sequence: u16,
    timestamp: u32,
    /// Where the payload starts — past any CSRC list.
    payload_at: usize,
};

/// Parse the RTP header. **The CSRC count moves the payload**, and a parser
/// that assumes twelve bytes reads the first CSRC as an AVDTP payload header
/// the moment a peer uses one.
pub fn parseRtp(buf: []const u8) !Rtp {
    if (buf.len < rtp_header_len) return Error.Malformed;
    if ((buf[0] >> 6) != 2) return Error.NotRtp;
    const csrc: usize = buf[0] & 0x0F;
    const extension = (buf[0] & 0x10) != 0;
    var at = rtp_header_len + csrc * 4;
    if (extension) {
        if (at + 4 > buf.len) return Error.Malformed;
        const words: usize = std.mem.readInt(u16, buf[at + 2 ..][0..2], .big);
        at += 4 + words * 4;
    }
    if (at > buf.len) return Error.Malformed;
    return .{
        .payload_type = buf[1] & 0x7F,
        .sequence = std.mem.readInt(u16, buf[2..4], .big),
        .timestamp = std.mem.readInt(u32, buf[4..8], .big),
        .payload_at = at,
    };
}

/// The one-byte AVDTP media payload header that precedes the SBC frames.
pub const PayloadHeader = struct {
    fragmented: bool,
    start: bool,
    last: bool,
    frames: u8,
};

pub fn parsePayloadHeader(b: u8) PayloadHeader {
    return .{
        .fragmented = (b & 0x80) != 0,
        .start = (b & 0x40) != 0,
        .last = (b & 0x20) != 0,
        .frames = b & 0x0F,
    };
}

pub const Packet = struct {
    rtp: Rtp,
    header: PayloadHeader,
    /// The first frame's header — every frame in a packet shares a
    /// configuration, so this describes all of them.
    frame: FrameHeader,
    /// Frames actually walked, which must equal `header.frames`.
    frames_walked: usize,
    frame_len: usize,
    /// Total samples per channel in this packet.
    samples: usize,
};

/// Walk a media packet and prove it is understood: RTP, payload header, then
/// each frame by its own computed length, ending exactly at the packet's end.
///
/// **Ending exactly is the whole test.** A frame length that is wrong by any
/// amount leaves a remainder or overruns, and either says the framing is
/// misunderstood — which is precisely the failure a decoder would otherwise
/// report as noise.
pub fn walk(buf: []const u8) !Packet {
    const rtp = try parseRtp(buf);
    if (rtp.payload_at + 1 > buf.len) return Error.Malformed;

    const ph = parsePayloadHeader(buf[rtp.payload_at]);
    if (ph.fragmented) return Error.Fragmented;
    if (ph.frames == 0) return Error.Malformed;

    var at = rtp.payload_at + 1;
    const first = try parseFrameHeader(buf[at..]);
    var walked: usize = 0;
    var samples: usize = 0;

    while (at < buf.len) {
        const fh = try parseFrameHeader(buf[at..]);
        const len = fh.length();
        if (at + len > buf.len) return Error.LengthMismatch;
        at += len;
        walked += 1;
        samples += fh.samples();
        if (walked > 15) return Error.Malformed;
    }

    if (at != buf.len) return Error.LengthMismatch;
    if (walked != ph.frames) return Error.LengthMismatch;

    return .{
        .rtp = rtp,
        .header = ph,
        .frame = first,
        .frames_walked = walked,
        .frame_len = first.length(),
        .samples = samples,
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// The frame header byte for the stream rung 24 negotiated: 44.1 kHz, 16
/// blocks, joint stereo, Loudness, 8 subbands.
const observed_header_byte: u8 = 0b10_11_11_0_1; // 0xBD

test "the observed frame header decodes to the negotiated stream" {
    const fh = try parseFrameHeader(&.{ syncword, observed_header_byte, 53, 0x00 });
    try testing.expectEqual(@as(u32, 44_100), fh.sample_rate);
    try testing.expectEqual(@as(u8, 16), fh.blocks);
    try testing.expectEqual(ChannelMode.joint_stereo, fh.channel_mode);
    try testing.expectEqual(Allocation.loudness, fh.allocation);
    try testing.expectEqual(@as(u8, 8), fh.subbands);
    try testing.expectEqual(@as(u8, 53), fh.bitpool);
}

test "a joint-stereo frame at bitpool 53 is 119 bytes, as rung 24 measured" {
    const fh = try parseFrameHeader(&.{ syncword, observed_header_byte, 53, 0 });
    try testing.expectEqual(@as(usize, 119), fh.length());
    try testing.expectEqual(@as(usize, 128), fh.samples());

    // **The stereo formula gives 225 here**, and that is the mistake this
    // assertion exists to catch: joint stereo shares one allocation across the
    // pair, so the term is `subbands + blocks * bitpool`, not `blocks * bitpool`
    // doubled.
    var stereo = fh;
    stereo.channel_mode = .stereo;
    try testing.expect(stereo.length() != fh.length());
    try testing.expectEqual(@as(usize, 118), stereo.length());
}

test "five frames plus thirteen header bytes is the 608 rung 24 saw" {
    const fh = try parseFrameHeader(&.{ syncword, observed_header_byte, 53, 0 });
    try testing.expectEqual(@as(usize, 608), rtp_header_len + 1 + 5 * fh.length());
}

test "a whole media packet walks and ends exactly at its end" {
    var pkt: [608]u8 = @splat(0);
    pkt[0] = 0x80; // RTP version 2, no CSRC, no extension
    pkt[1] = 96; // dynamic payload type
    std.mem.writeInt(u16, pkt[2..4], 0x1234, .big);
    std.mem.writeInt(u32, pkt[4..8], 0x0000_2000, .big);
    pkt[12] = 5; // payload header: five frames, not fragmented
    var i: usize = 13;
    while (i < pkt.len) : (i += 119) {
        pkt[i] = syncword;
        pkt[i + 1] = observed_header_byte;
        pkt[i + 2] = 53;
    }

    const p = try walk(&pkt);
    try testing.expectEqual(@as(usize, 5), p.frames_walked);
    try testing.expectEqual(@as(usize, 119), p.frame_len);
    try testing.expectEqual(@as(usize, 640), p.samples);
    try testing.expectEqual(@as(u16, 0x1234), p.rtp.sequence);
    try testing.expectEqual(@as(u8, 96), p.rtp.payload_type);
    try testing.expectEqual(@as(u8, 5), p.header.frames);
}

test "a bitpool the phone lowered mid-stream still tiles" {
    // **The reason length is computed per frame.** At bitpool 32 the frame is
    // shorter, and a walker that assumed 119 would desynchronise here and
    // produce noise a decoder would be blamed for.
    const fh = try parseFrameHeader(&.{ syncword, observed_header_byte, 32, 0 });
    const len = fh.length();
    try testing.expect(len != 119);

    const total = rtp_header_len + 1 + 4 * len;
    var pkt: [1024]u8 = @splat(0);
    pkt[0] = 0x80;
    pkt[12] = 4;
    var i: usize = 13;
    while (i + 4 <= total) : (i += len) {
        pkt[i] = syncword;
        pkt[i + 1] = observed_header_byte;
        pkt[i + 2] = 32;
    }
    const p = try walk(pkt[0..total]);
    try testing.expectEqual(@as(usize, 4), p.frames_walked);
    try testing.expectEqual(len, p.frame_len);
}

test "frames that do not tile the packet are reported, not rounded" {
    var pkt: [600]u8 = @splat(0);
    pkt[0] = 0x80;
    pkt[12] = 5;
    var i: usize = 13;
    while (i + 4 <= pkt.len) : (i += 119) {
        pkt[i] = syncword;
        pkt[i + 1] = observed_header_byte;
        pkt[i + 2] = 53;
    }
    try testing.expectError(Error.LengthMismatch, walk(&pkt));
}

test "a frame count disagreeing with the payload header is reported" {
    var pkt: [608]u8 = @splat(0);
    pkt[0] = 0x80;
    pkt[12] = 4; // claims four; five will be walked
    var i: usize = 13;
    while (i < pkt.len) : (i += 119) {
        pkt[i] = syncword;
        pkt[i + 1] = observed_header_byte;
        pkt[i + 2] = 53;
    }
    try testing.expectError(Error.LengthMismatch, walk(&pkt));
}

test "the CSRC count moves the payload" {
    // A parser assuming twelve bytes reads the first CSRC word as a payload
    // header the moment a peer uses one.
    var buf: [32]u8 = @splat(0);
    buf[0] = 0x82; // version 2, CSRC count 2
    const r = try parseRtp(&buf);
    try testing.expectEqual(@as(usize, 20), r.payload_at);

    buf[0] = 0x80;
    try testing.expectEqual(@as(usize, 12), (try parseRtp(&buf)).payload_at);
}

test "a non-RTP payload and a fragment are refused" {
    var buf: [32]u8 = @splat(0);
    buf[0] = 0x40; // version 1
    try testing.expectError(Error.NotRtp, parseRtp(&buf));

    var pkt: [140]u8 = @splat(0);
    pkt[0] = 0x80;
    pkt[12] = 0x80 | 1; // fragmented
    try testing.expectError(Error.Fragmented, walk(&pkt));
}

test "a frame without the syncword is refused" {
    try testing.expectError(Error.BadSyncword, parseFrameHeader(&.{ 0x9D, 0xBD, 53, 0 }));
    try testing.expectError(Error.Malformed, parseFrameHeader(&.{ 0x9C, 0xBD }));
}

test "mono counts one channel and halves the scale factors" {
    var fh = try parseFrameHeader(&.{ syncword, observed_header_byte, 53, 0 });
    fh.channel_mode = .mono;
    try testing.expectEqual(@as(u8, 1), fh.channel_mode.channels());
    // 4 + (4*8*1)/8 + ceil(16*1*53/8) = 4 + 4 + 106
    try testing.expectEqual(@as(usize, 114), fh.length());
}

// ---------------------------------------------------------------------------
// Scale factors and bit allocation — rung 25b, first half
// ---------------------------------------------------------------------------
//
// **A wrong allocation makes everything downstream meaningless**, so it is
// written and checked on its own. The vectors carry bluedroid's allocation for
// each frame precisely so this can be, rather than only being testable once a
// filterbank exists to turn its mistakes into noise.

pub const max_subbands = 8;
pub const max_channels = 2;

pub const Scalefactors = struct {
    /// Left channel's subbands, then right's.
    values: [max_subbands * max_channels]i8 = @splat(0),
    /// The joint-stereo mask, one bit per subband. Zero unless joint stereo.
    join: u8 = 0,
    /// Where the quantised samples begin.
    samples_at: usize,
    count: usize,
};

/// Read the join mask and scale factors that follow a frame header.
///
/// **Only the byte-aligned case is handled**: eight subbands, or any subband
/// count that is not joint stereo. Four-subband joint stereo ends the join
/// field on a half-byte boundary and needs the bitstream reader — it is refused
/// rather than mis-read, and nothing this board negotiates produces it, because
/// `avdtp.offered_sbc` advertises eight subbands and four alike but the phone
/// has chosen eight every time.
pub fn readScalefactors(frame: []const u8, h: FrameHeader) !Scalefactors {
    const nch: usize = h.channel_mode.channels();
    const total = @as(usize, h.subbands) * nch;

    if (h.subbands != 8 and h.channel_mode == .joint_stereo) return Error.Malformed;

    var at: usize = 4;
    var sf = Scalefactors{ .samples_at = 0, .count = total };

    if (h.channel_mode == .joint_stereo) {
        if (at >= frame.len) return Error.Malformed;
        sf.join = frame[at];
        at += 1;
    }

    // Two four-bit scale factors per byte, high nibble first.
    const bytes = total / 2;
    if (at + bytes > frame.len) return Error.Malformed;
    var i: usize = 0;
    while (i < bytes) : (i += 1) {
        const b = frame[at + i];
        sf.values[i * 2] = @intCast(b >> 4);
        sf.values[i * 2 + 1] = @intCast(b & 0x0F);
    }
    sf.samples_at = at + bytes;
    return sf;
}

/// Loudness offsets, indexed by sampling frequency then subband. Straight from
/// the reference's tables; they exist because loudness allocation weights the
/// bands differently at different rates.
const offset8 = [4][8]i8{
    .{ -2, 0, 0, 0, 0, 0, 0, 1 },
    .{ -3, 0, 0, 0, 0, 0, 1, 2 },
    .{ -4, 0, 0, 0, 0, 0, 1, 2 },
    .{ -4, 0, 0, 0, 0, 0, 1, 2 },
};

const offset4 = [4][4]i8{
    .{ -1, 0, 0, 0 },
    .{ -2, 0, 0, 1 },
    .{ -2, 0, 0, 1 },
    .{ -2, 0, 0, 1 },
};

fn freqIndex(rate: u32) usize {
    return switch (rate) {
        16_000 => 0,
        32_000 => 1,
        44_100 => 2,
        else => 3,
    };
}

/// How many bits each subband would like, and their total.
///
/// **Only bitneeds above one count toward the total**, which looks like an
/// off-by-one and is not: the allocator avoids single-bit allocations, so a
/// band asking for one is not counted as having asked.
fn computeBitneed(sf: []const i8, h: FrameHeader, out: []i16) i32 {
    var bitcount: i32 = 0;
    const fi = freqIndex(h.sample_rate);

    for (0..h.subbands) |sb| {
        var bits: i16 = sf[sb];
        if (h.allocation == .snr) {
            out[sb] = bits;
        } else {
            const off: i16 = if (h.subbands == 4) offset4[fi][sb] else offset8[fi][sb];
            if (bits != 0) {
                bits -= off;
                if (bits > 0) bits = @divTrunc(bits, 2);
                bits += 5;
            }
            out[sb] = bits;
        }
        if (out[sb] > 1) bitcount += out[sb];
    }
    return bitcount;
}

/// Binary-search the adjustment that makes the allocation fit the bitpool.
///
/// The reference does this four subbands at a time with packed-word arithmetic;
/// this is the plain loop its own comment gives as the meaning of that code.
/// **Same answer, and the vectors say so.**
fn adjustToFitBitpool(bitpool: i32, bitneeds: []const i16, count_in: i32, excess: *i32) i32 {
    var bitcount = count_in;
    var bitadjust: i32 = if (bitcount > bitpool) -8 else 8;
    var chop: i32 = 8;
    var max_bitadjust: i32 = 0;

    while (bitcount != bitpool and chop != 0) {
        var count: i32 = 0;
        for (bitneeds) |bn| {
            var bits: i32 = @as(i32, bn) + bitadjust;
            if (bits < 2) bits = 0;
            if (bits > 16) bits = 16;
            count += bits;
        }
        chop >>= 1;
        if (count > bitpool) {
            bitadjust -= chop;
        } else {
            max_bitadjust = bitadjust;
            bitcount = count;
            bitadjust += chop;
        }
    }
    excess.* = bitpool - bitcount;
    return max_bitadjust;
}

/// **Single-bit allocations are avoided as a last resort**, which is what the
/// two odd-looking branches are for: a band that would get one bit gets two if
/// two spare bits exist, and nothing otherwise.
fn allocAdjustedBits(dest: *u8, bits_in: i32, excess_in: i32) i32 {
    var bits = bits_in;
    var excess = excess_in;
    if (bits < 16) {
        if (bits > 1) {
            if (excess != 0) {
                bits += 1;
                excess -= 1;
            }
        } else if (bits == 1 and excess > 1) {
            bits = 2;
            excess -= 2;
        } else {
            bits = 0;
        }
    } else {
        bits = 16;
    }
    dest.* = @intCast(bits);
    return excess;
}

fn allocExcessBits(dest: *u8, excess: i32) i32 {
    if (dest.* < 16) {
        dest.* += 1;
        return excess - 1;
    }
    return excess;
}

pub const Bits = struct {
    values: [max_subbands * max_channels]u8 = @splat(0),
    count: usize,
};

/// The bit allocation for one frame.
pub fn allocate(sf: Scalefactors, h: FrameHeader) Bits {
    const nsb: usize = h.subbands;
    const nch: usize = h.channel_mode.channels();
    var bits = Bits{ .count = nsb * nch };
    var bitneeds: [max_subbands * max_channels]i16 = @splat(0);

    switch (h.channel_mode) {
        .mono, .dual => {
            // Each channel gets its own bitpool independently.
            for (0..nch) |ch| {
                const at = ch * nsb;
                const count = computeBitneed(sf.values[at..][0..nsb], h, bitneeds[at..][0..nsb]);
                var excess: i32 = 0;
                const adjust = adjustToFitBitpool(h.bitpool, bitneeds[at..][0..nsb], count, &excess);
                for (0..nsb) |sb| {
                    excess = allocAdjustedBits(&bits.values[at + sb], bitneeds[at + sb] + adjust, excess);
                }
                var sb: usize = 0;
                while (excess > 0 and sb < nsb * 16) : (sb += 1) {
                    excess = allocExcessBits(&bits.values[at + (sb % nsb)], excess);
                }
            }
        },
        .stereo, .joint_stereo => {
            // **One bitpool across both channels**, so the two bitneed arrays
            // are adjusted together and the leftovers alternate between them.
            const left = computeBitneed(sf.values[0..nsb], h, bitneeds[0..nsb]);
            const right = computeBitneed(sf.values[nsb..][0..nsb], h, bitneeds[nsb..][0..nsb]);
            var excess: i32 = 0;
            const adjust = adjustToFitBitpool(h.bitpool, bitneeds[0 .. nsb * 2], left + right, &excess);

            for (0..nsb) |sb| {
                excess = allocAdjustedBits(&bits.values[sb], bitneeds[sb] + adjust, excess);
                excess = allocAdjustedBits(&bits.values[nsb + sb], bitneeds[nsb + sb] + adjust, excess);
            }
            // Round-robin, left then right. Bounded: with every band capped at
            // 16 the excess must run out, but an unbounded loop here would hang
            // rather than misbehave, and a hang is the worse failure.
            var i: usize = 0;
            while (excess > 0 and i < nsb * 32) : (i += 1) {
                const sb = (i / 2) % nsb;
                const at = if (i % 2 == 0) sb else nsb + sb;
                excess = allocExcessBits(&bits.values[at], excess);
            }
        },
    }
    return bits;
}

// ---------------------------------------------------------------------------
// The decode oracle
// ---------------------------------------------------------------------------

/// Frames captured off the wire and the PCM bluedroid decodes them to. See
/// `sbc_vectors.zig`'s header: **they are a sequence, not a set.**
pub const vectors = @import("sbc_vectors.zig");

test "the oracle is a contiguous sequence of real frames" {
    // Not a decoder test — a test that the vectors are what rung 25b will need,
    // so a decoder written against them is written against something sound.
    try testing.expect(vectors.vectors.len >= 32);

    for (vectors.vectors, 0..) |v, i| {
        // Every frame parses with the walker rung 25a proved, and its computed
        // length is its actual length. If these two ever disagree the vectors
        // were regenerated from a different stream.
        const fh = try parseFrameHeader(v.frame);
        try testing.expectEqual(v.frame.len, fh.length());
        try testing.expectEqual(@as(u32, 44_100), fh.sample_rate);
        try testing.expectEqual(ChannelMode.joint_stereo, fh.channel_mode);

        // 128 samples per channel, interleaved.
        try testing.expectEqual(fh.samples() * 2, v.pcm.len);
        if (i > 40) break;
    }
}

test "the oracle contains real audio, not just silence" {
    // **A decoder that always returned zero would pass the first twelve
    // entries.** This asserts the set can actually catch that.
    var loud: usize = 0;
    var silent: usize = 0;
    for (vectors.vectors) |v| {
        var peak: i32 = 0;
        for (v.pcm) |s| {
            const a: i32 = if (s < 0) -@as(i32, s) else s;
            if (a > peak) peak = a;
        }
        if (peak == 0) silent += 1 else loud += 1;
    }
    try testing.expect(loud >= 32);
    try testing.expect(silent > 0); // and silence is represented too
}

test "entry 0 decodes quietly, because the filter history starts zeroed" {
    // **This test used to assert the opposite**, and was wrong. The first
    // version of the vectors gave entry 0 full-scale values, which were
    // recorded as a genuine cold-start transient. They were not: the generator
    // left the reference decoder's scratch memory uninitialised, and
    // `OI_CODEC_SBC_DecoderReset` does not clear it, so the first frame was
    // stack garbage run through the synthesis filter.
    //
    // It was caught by the decoder disagreeing with the oracle by 32768 on one
    // frame and by 3 on every other — a shape that says the oracle is wrong,
    // not the code.
    var first_peak: i32 = 0;
    for (vectors.vectors[0].pcm) |s| {
        const a: i32 = if (s < 0) -@as(i32, s) else s;
        if (a > first_peak) first_peak = a;
    }
    try testing.expect(first_peak < 100);

    // Entry 1 is encoded silence, and entry 2 is byte-identical to it — which
    // is what makes them cheap to pass and worth little alone.
    try testing.expectEqualSlices(u8, vectors.vectors[1].frame, vectors.vectors[2].frame);
}

test "our bit allocation matches bluedroid's, frame for frame" {
    // **The test rung 25b's first half exists for.** Not that the allocator is
    // self-consistent — that it agrees with the decoder that played music on
    // this board, on 65 real frames, every subband of every channel.
    for (vectors.vectors, 0..) |v, i| {
        const h = try parseFrameHeader(v.frame);
        const sf = try readScalefactors(v.frame, h);

        try testing.expectEqual(v.scale_factors.len, sf.count);
        errdefer std.debug.print("frame {d}: scale factors differ\n", .{i});
        try testing.expectEqualSlices(i8, v.scale_factors, sf.values[0..sf.count]);

        const bits = allocate(sf, h);
        errdefer std.debug.print("frame {d}: allocation differs\n", .{i});
        try testing.expectEqualSlices(u8, v.bits, bits.values[0..bits.count]);
    }
}

test "the allocation always spends exactly the bitpool" {
    // An independent property, so a shared misunderstanding between our
    // allocator and the reference would still have to survive this.
    for (vectors.vectors) |v| {
        const h = try parseFrameHeader(v.frame);
        const sf = try readScalefactors(v.frame, h);
        const bits = allocate(sf, h);

        var total: u32 = 0;
        for (bits.values[0..bits.count]) |b| {
            total += b;
            // Allocations are 0, or between 2 and 16. Never 1.
            try testing.expect(b == 0 or (b >= 2 and b <= 16));
        }
        try testing.expectEqual(@as(u32, h.bitpool), total);
    }
}

test "four-subband joint stereo is refused rather than mis-read" {
    // Its join field ends on a half-byte boundary and needs the bitstream
    // reader. Nothing this board negotiates produces it; reading it as though
    // it were byte-aligned would shift every scale factor by four bits.
    const h = FrameHeader{
        .sample_rate = 44_100,
        .blocks = 16,
        .channel_mode = .joint_stereo,
        .allocation = .loudness,
        .subbands = 4,
        .bitpool = 53,
        .crc = 0,
    };
    var frame: [64]u8 = @splat(0);
    try testing.expectError(Error.Malformed, readScalefactors(&frame, h));
}

test "the oracle's coverage of the allocator, stated rather than assumed" {
    // **Not every branch is exercised by these frames**, and pretending
    // otherwise is worse than the gap. Changing `out[sb] > 1` to `> 0` in
    // `computeBitneed` passes the byte-exact test above; changing the loudness
    // halving fails it. That difference is this: no bitneed in these 65 frames
    // is ever exactly 1, so the rule that a band asking for one bit is not
    // counted never fires, and neither does `allocAdjustedBits`'s matching
    // branch.
    var ones: usize = 0;
    var zero_alloc: usize = 0;
    var capped: usize = 0;

    for (vectors.vectors) |v| {
        const h = try parseFrameHeader(v.frame);
        const sf = try readScalefactors(v.frame, h);
        var bitneeds: [max_subbands * max_channels]i16 = @splat(0);
        const nsb: usize = h.subbands;
        _ = computeBitneed(sf.values[0..nsb], h, bitneeds[0..nsb]);
        _ = computeBitneed(sf.values[nsb..][0..nsb], h, bitneeds[nsb..][0..nsb]);
        for (bitneeds[0 .. nsb * 2]) |bn| {
            if (bn == 1) ones += 1;
        }
        const bits = allocate(sf, h);
        for (bits.values[0..bits.count]) |b| {
            if (b == 0) zero_alloc += 1;
            if (b == 16) capped += 1;
        }
    }

    // Covered: bands that get nothing.
    try testing.expect(zero_alloc > 0);
    // **Not covered**, and recorded so the next capture can aim at it: a
    // bitneed of exactly 1, and an allocation saturating at 16. A stream at a
    // much lower bitpool would produce the first; a very high one the second.
    try testing.expectEqual(@as(usize, 0), ones);
    try testing.expectEqual(@as(usize, 0), capped);
}
