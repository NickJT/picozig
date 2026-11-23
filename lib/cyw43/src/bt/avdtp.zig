//! AVDTP — rung 24, the negotiation that turns a discovered sink into a stream.
//!
//! Rung 23b ended with the phone opening a channel to PSM `0x0019` and finding
//! nobody home. This is what answers it. Every byte below is checked against
//! a capture of btstack's own sink on this board, taken during a run that
//! ended in music.
//!
//! ```
//! DISCOVER              -> one endpoint: SEID 1, audio, sink
//! GET_ALL_CAPABILITIES  -> media transport, a media codec, delay reporting
//! SET_CONFIGURATION     -> the phone chooses; accept
//! OPEN                  -> accept, then a second channel on the same PSM
//! START                 -> accept, and media begins
//! ```
//!
//! ## The header is two bytes and three of its fields matter
//!
//!     byte 0   transaction << 4 | packet_type << 2 | message_type
//!     byte 1   signal identifier
//!
//! **The transaction label must be echoed.** A response carrying a different
//! one is ignored by the peer, which presents as a request that was never
//! answered — the same failure shape as L2CAP's signalling identifier.
//!
//! **`packet_type` is the fragmentation field**, and this file only handles
//! single packets. That is honest for the exchange above, where the longest
//! message is thirteen bytes against an MTU of 672, and `parseHeader` says so
//! rather than silently mis-reading a fragment.
//!
//! ## What we advertise, and why it is not what the reference advertises
//!
//! btstack offers **every** sampling frequency, because its decoder handles
//! them all. This offers **44.1 kHz only**.
//!
//! That is not timidity: `lib/pcm5100`'s I2S engine is clocked for 44.1 kHz, and
//! a sink that advertises 48 kHz and then plays it 8.8% slow is worse than one
//! that never offered. Everything the decoder genuinely treats as a parameter —
//! channel mode, block length, subband count, allocation method — is offered in
//! full. **Advertise what the hardware can do, not what the codec can express.**

const std = @import("std");

pub const Error = error{
    Malformed,
    BufferTooSmall,
    /// A fragmented signalling message. Not handled, and refused rather than
    /// mis-read: nothing in an A2DP setup exchange comes close to an MTU.
    Fragmented,
};

pub const message = struct {
    pub const command: u8 = 0;
    pub const general_reject: u8 = 1;
    pub const response_accept: u8 = 2;
    pub const response_reject: u8 = 3;
};

pub const packet = struct {
    pub const single: u8 = 0;
    pub const start: u8 = 1;
    pub const cont: u8 = 2;
    pub const end: u8 = 3;
};

pub const sig = struct {
    pub const discover: u8 = 0x01;
    pub const get_capabilities: u8 = 0x02;
    pub const set_configuration: u8 = 0x03;
    pub const get_configuration: u8 = 0x04;
    pub const reconfigure: u8 = 0x05;
    pub const open: u8 = 0x06;
    pub const start: u8 = 0x07;
    pub const close: u8 = 0x08;
    pub const suspend_: u8 = 0x09;
    pub const abort: u8 = 0x0A;
    pub const security_control: u8 = 0x0B;
    pub const get_all_capabilities: u8 = 0x0C;
    pub const delay_report: u8 = 0x0D;
};

/// Service categories, as they appear in a capabilities or configuration block.
pub const category = struct {
    pub const media_transport: u8 = 1;
    pub const reporting: u8 = 2;
    pub const recovery: u8 = 3;
    pub const content_protection: u8 = 4;
    pub const header_compression: u8 = 5;
    pub const multiplexing: u8 = 6;
    pub const media_codec: u8 = 7;
    pub const delay_reporting: u8 = 8;
};

pub const media_type_audio: u8 = 0x00;
pub const codec_sbc: u8 = 0x00;

/// The stream endpoint this device offers. **One**, because it has one DAC.
pub const local_seid: u8 = 1;

pub const Header = struct {
    transaction: u8,
    message_type: u8,
    signal: u8,
    payload: []const u8,
};

pub fn parseHeader(buf: []const u8) !Header {
    if (buf.len < 2) return Error.Malformed;
    const packet_type = (buf[0] >> 2) & 0x03;
    if (packet_type != packet.single) return Error.Fragmented;
    return .{
        .transaction = buf[0] >> 4,
        .message_type = buf[0] & 0x03,
        .signal = buf[1] & 0x3F,
        .payload = buf[2..],
    };
}

/// Build a response. **The transaction label is the caller's to supply from the
/// command it answers** — inventing one here would be the bug this signature
/// exists to prevent.
pub fn buildResponse(
    out: []u8,
    transaction: u8,
    signal: u8,
    accept: bool,
    payload: []const u8,
) ![]u8 {
    if (out.len < 2 + payload.len) return Error.BufferTooSmall;
    out[0] = (transaction << 4) |
        (packet.single << 2) |
        (if (accept) message.response_accept else message.response_reject);
    out[1] = signal;
    @memcpy(out[2..][0..payload.len], payload);
    return out[0 .. 2 + payload.len];
}

// ---------------------------------------------------------------------------
// DISCOVER
// ---------------------------------------------------------------------------

/// The two bytes describing one stream endpoint.
///
///     byte 0   seid << 2 | in_use << 1
///     byte 1   media_type << 4 | tsep << 3
///
/// **`tsep` is 1 for a sink.** Clearing it advertises a source, and a phone
/// looking for somewhere to send audio would pass over this device entirely.
pub fn endpointInfo(seid: u8, in_use: bool, is_sink: bool) [2]u8 {
    return .{
        (seid << 2) | (@as(u8, @intFromBool(in_use)) << 1),
        (media_type_audio << 4) | (@as(u8, @intFromBool(is_sink)) << 3),
    };
}

// ---------------------------------------------------------------------------
// SBC capabilities and configuration
// ---------------------------------------------------------------------------

pub const sbc_freq = struct {
    pub const hz_16000: u8 = 0x80;
    pub const hz_32000: u8 = 0x40;
    pub const hz_44100: u8 = 0x20;
    pub const hz_48000: u8 = 0x10;
};

pub const sbc_channel = struct {
    pub const mono: u8 = 0x08;
    pub const dual: u8 = 0x04;
    pub const stereo: u8 = 0x02;
    pub const joint_stereo: u8 = 0x01;
};

pub const sbc_blocks = struct {
    pub const b4: u8 = 0x80;
    pub const b8: u8 = 0x40;
    pub const b12: u8 = 0x20;
    pub const b16: u8 = 0x10;
};

pub const sbc_subbands = struct {
    pub const s4: u8 = 0x08;
    pub const s8: u8 = 0x04;
};

pub const sbc_allocation = struct {
    pub const snr: u8 = 0x02;
    pub const loudness: u8 = 0x01;
};

/// What this device offers, as the four SBC-specific capability bytes.
///
/// **44.1 kHz only** — see the header. Everything else is offered in full,
/// because those are parameters a decoder reads rather than clocks it has to
/// produce.
pub const offered_sbc = [4]u8{
    sbc_freq.hz_44100 |
        sbc_channel.mono | sbc_channel.dual | sbc_channel.stereo | sbc_channel.joint_stereo,
    sbc_blocks.b4 | sbc_blocks.b8 | sbc_blocks.b12 | sbc_blocks.b16 |
        sbc_subbands.s4 | sbc_subbands.s8 |
        sbc_allocation.snr | sbc_allocation.loudness,
    2, // minimum bitpool
    53, // maximum bitpool, as the reference offers
};

/// The capabilities block for GET_ALL_CAPABILITIES: media transport, the codec,
/// and delay reporting — the same three the reference advertises.
pub fn writeCapabilities(out: []u8) ![]u8 {
    const total = 2 + 2 + 6 + 2;
    if (out.len < total) return Error.BufferTooSmall;
    var i: usize = 0;

    out[i] = category.media_transport;
    out[i + 1] = 0;
    i += 2;

    out[i] = category.media_codec;
    out[i + 1] = 6;
    out[i + 2] = media_type_audio << 4;
    out[i + 3] = codec_sbc;
    @memcpy(out[i + 4 ..][0..4], &offered_sbc);
    i += 8;

    out[i] = category.delay_reporting;
    out[i + 1] = 0;
    i += 2;

    return out[0..i];
}

pub const SbcConfig = struct {
    sample_rate: u32,
    channel_mode: u8,
    blocks: u8,
    subbands: u8,
    allocation: u8,
    min_bitpool: u8,
    max_bitpool: u8,

    pub fn channels(self: SbcConfig) u8 {
        return if (self.channel_mode == sbc_channel.mono) 1 else 2;
    }
};

/// Decode the four SBC bytes a SET_CONFIGURATION carries.
///
/// **Exactly one bit may be set in each field.** A configuration is a choice,
/// not a capability, and a peer that leaves two bits set has said something
/// meaningless — refused here rather than resolved by picking one, because
/// picking one silently is how a stream ends up decoded at the wrong rate.
pub fn parseSbcConfig(b: []const u8) !SbcConfig {
    if (b.len < 4) return Error.Malformed;

    const freq_bits = b[0] & 0xF0;
    const chan_bits = b[0] & 0x0F;
    const block_bits = b[1] & 0xF0;
    const sub_bits = b[1] & 0x0C;
    const alloc_bits = b[1] & 0x03;

    if (@popCount(freq_bits) != 1 or @popCount(chan_bits) != 1) return Error.Malformed;
    if (@popCount(block_bits) != 1 or @popCount(sub_bits) != 1) return Error.Malformed;
    if (@popCount(alloc_bits) != 1) return Error.Malformed;

    return .{
        .sample_rate = switch (freq_bits) {
            sbc_freq.hz_16000 => 16_000,
            sbc_freq.hz_32000 => 32_000,
            sbc_freq.hz_44100 => 44_100,
            sbc_freq.hz_48000 => 48_000,
            else => return Error.Malformed,
        },
        .channel_mode = chan_bits,
        .blocks = switch (block_bits) {
            sbc_blocks.b4 => 4,
            sbc_blocks.b8 => 8,
            sbc_blocks.b12 => 12,
            sbc_blocks.b16 => 16,
            else => return Error.Malformed,
        },
        .subbands = if (sub_bits == sbc_subbands.s4) 4 else 8,
        .allocation = alloc_bits,
        .min_bitpool = b[2],
        .max_bitpool = b[3],
    };
}

/// Walk a configuration block and return the SBC configuration inside it.
///
/// The block is a run of `[category][length][value...]`, and the media codec
/// entry can sit anywhere in it — the reference puts media transport first, but
/// nothing requires that.
pub fn findSbcConfig(block: []const u8) !SbcConfig {
    var i: usize = 0;
    while (i + 2 <= block.len) {
        const cat = block[i];
        const len: usize = block[i + 1];
        if (i + 2 + len > block.len) return Error.Malformed;
        if (cat == category.media_codec and len >= 6) {
            const v = block[i + 2 ..][0..len];
            if ((v[0] >> 4) != media_type_audio or v[1] != codec_sbc) return Error.Malformed;
            return parseSbcConfig(v[2..]);
        }
        i += 2 + len;
    }
    return Error.Malformed;
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "a command header round-trips, and the transaction label survives" {
    // Transaction 5, single packet, command, DISCOVER.
    const cmd = [_]u8{ (5 << 4) | (0 << 2) | 0, sig.discover };
    const h = try parseHeader(&cmd);
    try testing.expectEqual(@as(u8, 5), h.transaction);
    try testing.expectEqual(message.command, h.message_type);
    try testing.expectEqual(sig.discover, h.signal);

    var buf: [16]u8 = undefined;
    const rsp = try buildResponse(&buf, h.transaction, h.signal, true, &.{ 0x04, 0x08 });
    // **The label is echoed** — a response with a different one is ignored, and
    // presents as a request nobody answered.
    try testing.expectEqual(@as(u8, 5), rsp[0] >> 4);
    try testing.expectEqual(message.response_accept, rsp[0] & 0x03);
    try testing.expectEqual(sig.discover, rsp[1]);
    try testing.expectEqualSlices(u8, &.{ 0x04, 0x08 }, rsp[2..]);
}

test "a reject is the same shape with a different message type" {
    var buf: [8]u8 = undefined;
    const rsp = try buildResponse(&buf, 3, sig.set_configuration, false, &.{});
    try testing.expectEqual(message.response_reject, rsp[0] & 0x03);
    try testing.expectEqual(@as(u8, 3), rsp[0] >> 4);
}

test "a fragmented message is refused rather than mis-read" {
    // packet_type 1 = start of a fragmented message.
    const frag = [_]u8{ (5 << 4) | (1 << 2) | 0, sig.discover };
    try testing.expectError(Error.Fragmented, parseHeader(&frag));
    try testing.expectError(Error.Malformed, parseHeader(&.{0x00}));
}

test "the endpoint we advertise is the one the reference advertised" {
    // From the capture of btstack's sink: DISCOVER accept carried `04 08`.
    const info = endpointInfo(local_seid, false, true);
    try testing.expectEqualSlices(u8, &.{ 0x04, 0x08 }, &info);

    // In use sets bit 1; a source clears bit 3, and a phone looking for
    // somewhere to send audio would skip this device entirely.
    try testing.expectEqualSlices(u8, &.{ 0x06, 0x08 }, &endpointInfo(1, true, true));
    try testing.expectEqualSlices(u8, &.{ 0x04, 0x00 }, &endpointInfo(1, false, false));
}

test "our capabilities differ from the reference in exactly one field" {
    var buf: [32]u8 = undefined;
    const caps = try writeCapabilities(&buf);

    // The reference sent `0100 0706 0000ffff0235 0800`.
    var expected: [32]u8 = undefined;
    const ref = try std.fmt.hexToBytes(&expected, "010007060000ffff02350800");

    try testing.expectEqual(ref.len, caps.len);
    // Same structure: media transport, codec, delay reporting.
    try testing.expectEqual(ref[0], caps[0]); // category 1
    try testing.expectEqual(ref[2], caps[2]); // category 7
    try testing.expectEqual(ref[10], caps[10]); // category 8

    // **The one difference: frequency.** The reference offers all four; this
    // offers 44.1 kHz only, because the I2S engine is clocked for it.
    try testing.expectEqual(@as(u8, 0xFF), ref[6]);
    try testing.expectEqual(@as(u8, 0x2F), caps[6]);
    try testing.expectEqual(sbc_freq.hz_44100, caps[6] & 0xF0);
    // Everything a decoder merely reads is still offered in full.
    try testing.expectEqual(ref[7], caps[7]);
    try testing.expectEqual(ref[8], caps[8]); // min bitpool 2
    try testing.expectEqual(ref[9], caps[9]); // max bitpool 53
}

test "the configuration the phone chose decodes to the stream we expect" {
    // From the trace: SET_CONFIGURATION carried `0404 0100 0706 0000 21150235 0800`.
    var buf: [32]u8 = undefined;
    const block = try std.fmt.hexToBytes(&buf, "0100070600002115023508" ++ "00");
    const cfg = try findSbcConfig(block);

    try testing.expectEqual(@as(u32, 44_100), cfg.sample_rate);
    try testing.expectEqual(sbc_channel.joint_stereo, cfg.channel_mode);
    try testing.expectEqual(@as(u8, 16), cfg.blocks);
    try testing.expectEqual(@as(u8, 8), cfg.subbands);
    try testing.expectEqual(sbc_allocation.loudness, cfg.allocation);
    try testing.expectEqual(@as(u8, 2), cfg.min_bitpool);
    try testing.expectEqual(@as(u8, 53), cfg.max_bitpool);
    try testing.expectEqual(@as(u8, 2), cfg.channels());
}

test "a configuration with two bits set in a field is refused" {
    // 0x30 is 44.1 *and* 48 kHz — a capability, not a choice. Picking one
    // silently is how a stream ends up decoded at the wrong rate.
    try testing.expectError(Error.Malformed, parseSbcConfig(&.{ 0x31, 0x15, 2, 53 }));
    try testing.expectError(Error.Malformed, parseSbcConfig(&.{ 0x21, 0x35, 2, 53 })); // two block sizes
    try testing.expectError(Error.Malformed, parseSbcConfig(&.{ 0x23, 0x15, 2, 53 })); // two channel modes
    try testing.expectError(Error.Malformed, parseSbcConfig(&.{ 0x21, 0x17, 2, 53 })); // two allocations
    try testing.expectError(Error.Malformed, parseSbcConfig(&.{ 0x21, 0x15 }));
}

test "the codec entry is found wherever it sits in the block" {
    // Media transport first, as the reference sends it.
    var a: [32]u8 = undefined;
    const first = try std.fmt.hexToBytes(&a, "0100070600002115023508" ++ "00");
    _ = try findSbcConfig(first);

    // And with delay reporting ahead of it, which nothing forbids.
    var b: [32]u8 = undefined;
    const later = try std.fmt.hexToBytes(&b, "08000100" ++ "070600002115" ++ "0235");
    const cfg = try findSbcConfig(later);
    try testing.expectEqual(@as(u32, 44_100), cfg.sample_rate);
}

test "a block with no codec entry, or a truncated one, is refused" {
    try testing.expectError(Error.Malformed, findSbcConfig(&.{ 0x01, 0x00, 0x08, 0x00 }));
    try testing.expectError(Error.Malformed, findSbcConfig(&.{ 0x07, 0x06, 0x00 }));
    try testing.expectError(Error.Malformed, findSbcConfig(&.{}));
}

test "a non-SBC codec is refused rather than decoded as SBC" {
    // Codec type 2 is MPEG-2/4 AAC. Reading its parameters as SBC's would
    // produce a plausible configuration for a stream nothing here can decode.
    var buf: [16]u8 = undefined;
    const aac = try std.fmt.hexToBytes(&buf, "070600022115023 5"[0..0] ++ "0706000221150235");
    try testing.expectError(Error.Malformed, findSbcConfig(aac));
}
