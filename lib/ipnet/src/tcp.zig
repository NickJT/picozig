//! TCP segments on the wire — RFC 9293 §3.1. Rung N5.
//!
//! **Pure, like `net.zig` and `dhcp.zig`**: functions over caller-supplied
//! bytes, no device, no state, no allocator. There is no connection here — that
//! is N6 — and keeping the two apart is what lets every field of a segment be
//! asserted against a real capture before any state machine exists to argue
//! with.
//!
//! ## What it is checked against
//!
//! A TCP session captured 2026-08-31 between a Pico W running the pico-sdk's
//! `tcp_client` and a Mac. **The fixtures below are literal bytes from that
//! capture**, not segments this module built and then read back — which
//! would only prove it agrees with itself. Where a test asserts a checksum, it
//! asserts the *receiver's* test (a message including its own checksum sums to
//! zero), because that is what a peer actually does.
//!
//! ## What the capture settled
//!
//! **lwIP's SYN offers `[mss 1460]` and nothing else** — no window scale, no
//! SACK-permitted, no timestamps — and macOS's SYN-ACK mirrors it. So the
//! options this parser must handle are MSS, End-of-Option-List and No-Operation,
//! and the ones it must *skip without understanding* are everything else. A
//! parser that rejected an unknown option would work against this peer and fail
//! against the next one.

const std = @import("std");
const net = @import("net.zig");

pub const header_len: usize = 20;

/// Control bits, in the order RFC 9293's Figure 1 lays them out: bit 7 is CWR
/// and bit 0 is FIN.
pub const Flag = struct {
    pub const fin: u8 = 1 << 0;
    pub const syn: u8 = 1 << 1;
    pub const rst: u8 = 1 << 2;
    pub const psh: u8 = 1 << 3;
    pub const ack: u8 = 1 << 4;
    pub const urg: u8 = 1 << 5;
    pub const ece: u8 = 1 << 6;
    pub const cwr: u8 = 1 << 7;
};

pub const Option = struct {
    pub const end: u8 = 0;
    pub const nop: u8 = 1;
    pub const mss: u8 = 2;
};

/// **The MSS this stack advertises, and the one it assumes if a peer sends
/// none.** 536 is RFC 9293's default for a peer that stays quiet; 1460 is what
/// fits an Ethernet MTU under the IPv4 and TCP headers, and is what both ends
/// of the capture offered.
pub const default_mss: u16 = 536;
pub const ethernet_mss: u16 = 1460;

pub const Segment = struct {
    src_ip: [4]u8,
    dest_ip: [4]u8,
    src_mac: [6]u8,
    src_port: u16,
    dest_port: u16,
    seq: u32,
    ack: u32,
    flags: u8,
    window: u16,
    /// From the peer's MSS option, or null when it sent none — **null and 536
    /// are different facts**, and collapsing them here would hide which peer
    /// said what.
    mss: ?u16,
    /// Points into the caller's frame; never copied.
    payload: []const u8,

    pub fn has(self: Segment, f: u8) bool {
        return (self.flags & f) != 0;
    }
};

/// Parse a received frame as IPv4/TCP addressed to `our_ip`.
///
/// **Bounded by the IP header's total-length field, not the frame length.**
/// Ethernet pads short frames to 60 bytes, so a bare ACK — 40 bytes of headers —
/// arrives with 6 bytes belonging to nobody. Measuring the payload by the frame
/// would hand those to the connection layer as data, and a stack that
/// acknowledged 6 bytes of padding would corrupt the stream in a way the peer
/// could never diagnose.
pub fn parseSegment(frame: []const u8, our_ip: [4]u8) !Segment {
    if (frame.len < net.eth_header_len + net.ipv4_header_len + header_len) return error.PacketTooShort;
    if (std.mem.readInt(u16, frame[12..14], .big) != net.EtherType.ipv4) return error.NotIpv4;

    const ip = frame[net.eth_header_len..];
    const ihl: usize = @as(usize, ip[0] & 0x0F) * 4;
    if ((ip[0] >> 4) != 4 or ihl < net.ipv4_header_len) return error.NotIpv4;
    if (ip[9] != net.IpProtocol.tcp) return error.NotTcp;

    // Fragments are refused rather than misread — a fragmented segment carries
    // part of a stream, and treating the part as the whole is worse than a loss.
    const frag = std.mem.readInt(u16, ip[6..8], .big);
    if ((frag & 0x2000) != 0 or (frag & 0x1FFF) != 0) return error.Fragmented;

    const total_len = std.mem.readInt(u16, ip[2..4], .big);
    if (total_len < ihl + header_len) return error.PacketTooShort;
    if (net.eth_header_len + total_len > frame.len) return error.PacketTooShort;

    var dest_ip: [4]u8 = undefined;
    @memcpy(&dest_ip, ip[16..20]);
    if (!std.mem.eql(u8, &dest_ip, &our_ip)) return error.NotForUs;

    const seg = frame[net.eth_header_len + ihl ..][0 .. total_len - ihl];
    const data_offset: usize = @as(usize, seg[12] >> 4) * 4;
    if (data_offset < header_len or data_offset > seg.len) return error.BadDataOffset;

    var src_ip: [4]u8 = undefined;
    @memcpy(&src_ip, ip[12..16]);

    // **The receiver's checksum test.** A segment carrying its own checksum
    // sums to zero when that checksum is right, so this needs no special case
    // for the field itself.
    const ph = net.pseudoHeader(src_ip, dest_ip, net.IpProtocol.tcp, @intCast(seg.len));
    if (net.checksumParts(&.{ &ph, seg }) != 0) return error.BadChecksum;

    var src_mac: [6]u8 = undefined;
    @memcpy(&src_mac, frame[6..12]);

    return .{
        .src_ip = src_ip,
        .dest_ip = dest_ip,
        .src_mac = src_mac,
        .src_port = std.mem.readInt(u16, seg[0..2], .big),
        .dest_port = std.mem.readInt(u16, seg[2..4], .big),
        .seq = std.mem.readInt(u32, seg[4..8], .big),
        .ack = std.mem.readInt(u32, seg[8..12], .big),
        .flags = seg[13],
        .window = std.mem.readInt(u16, seg[14..16], .big),
        .mss = parseMss(seg[header_len..data_offset]),
        .payload = seg[data_offset..],
    };
}

/// Walk the option list for an MSS. Everything else is skipped by its length
/// byte.
///
/// **An unknown option must be skipped, not rejected.** This capture's peers
/// offer only MSS, so a parser that refused anything else would pass every test
/// written against it and fail against the first peer that sends a timestamp.
///
/// **And a zero-length option would spin.** `NOP` and `END` are single bytes by
/// definition; everything else carries a length that includes the kind and the
/// length byte, so a length below 2 is malformed — and treating it as anything
/// but the end of the list is an infinite loop reachable from the network.
fn parseMss(opts: []const u8) ?u16 {
    var i: usize = 0;
    while (i < opts.len) {
        const kind = opts[i];
        if (kind == Option.end) return null;
        if (kind == Option.nop) {
            i += 1;
            continue;
        }
        if (i + 1 >= opts.len) return null;
        const len = opts[i + 1];
        if (len < 2 or i + len > opts.len) return null;
        if (kind == Option.mss and len == 4) {
            return std.mem.readInt(u16, opts[i + 2 ..][0..2], .big);
        }
        i += len;
    }
    return null;
}

pub const Build = struct {
    dest_mac: [6]u8,
    src_mac: [6]u8,
    src_ip: [4]u8,
    dest_ip: [4]u8,
    src_port: u16,
    dest_port: u16,
    seq: u32,
    ack: u32,
    flags: u8,
    window: u16,
    /// Emitted as an MSS option when set. **Only legal on a segment with SYN**
    /// — RFC 9293 §3.1: the MSS option "is only sent in the initial connection
    /// request".
    mss: ?u16 = null,
    payload: []const u8 = &.{},
};

pub fn buildSegment(b: Build, out_buf: []u8) ![]u8 {
    std.debug.assert(b.mss == null or (b.flags & Flag.syn) != 0);

    const opt_len: usize = if (b.mss != null) 4 else 0;
    const data_offset = header_len + opt_len;
    const total = net.eth_header_len + net.ipv4_header_len + data_offset + b.payload.len;
    if (out_buf.len < total) return error.BufferTooSmall;

    @memcpy(out_buf[0..6], &b.dest_mac);
    @memcpy(out_buf[6..12], &b.src_mac);
    std.mem.writeInt(u16, out_buf[12..14], net.EtherType.ipv4, .big);

    const ip = out_buf[net.eth_header_len..][0..net.ipv4_header_len];
    ip[0] = 0x45;
    ip[1] = 0x00;
    std.mem.writeInt(u16, ip[2..4], @intCast(net.ipv4_header_len + data_offset + b.payload.len), .big);
    std.mem.writeInt(u16, ip[4..6], 0, .big);
    std.mem.writeInt(u16, ip[6..8], 0x4000, .big); // don't fragment
    ip[8] = 64;
    ip[9] = net.IpProtocol.tcp;
    std.mem.writeInt(u16, ip[10..12], 0, .big);
    @memcpy(ip[12..16], &b.src_ip);
    @memcpy(ip[16..20], &b.dest_ip);
    std.mem.writeInt(u16, ip[10..12], net.computeChecksum(ip), .big);

    const seg = out_buf[net.eth_header_len + net.ipv4_header_len ..][0 .. data_offset + b.payload.len];
    std.mem.writeInt(u16, seg[0..2], b.src_port, .big);
    std.mem.writeInt(u16, seg[2..4], b.dest_port, .big);
    std.mem.writeInt(u32, seg[4..8], b.seq, .big);
    std.mem.writeInt(u32, seg[8..12], b.ack, .big);
    seg[12] = @intCast((data_offset / 4) << 4);
    seg[13] = b.flags;
    std.mem.writeInt(u16, seg[14..16], b.window, .big);
    std.mem.writeInt(u16, seg[16..18], 0, .big); // checksum, below
    std.mem.writeInt(u16, seg[18..20], 0, .big); // urgent pointer

    if (b.mss) |m| {
        seg[20] = Option.mss;
        seg[21] = 4;
        std.mem.writeInt(u16, seg[22..24], m, .big);
    }
    @memcpy(seg[data_offset..], b.payload);

    // Same pseudo-header arithmetic as UDP, and the same reason for
    // `checksumParts` rather than three calls: the sum pads once, at the end.
    const ph = net.pseudoHeader(b.src_ip, b.dest_ip, net.IpProtocol.tcp, @intCast(seg.len));
    // **Zero is not special here.** UDP treats a zero checksum as "none
    // computed" and sends 0xFFFF instead; TCP has no such exemption, and a
    // computed zero goes out as zero.
    std.mem.writeInt(u16, seg[16..18], net.checksumParts(&.{ &ph, seg }), .big);

    return out_buf[0..total];
}

// ---------------------------------------------------------------------------
// Rung N5 — asserted against the captured session
//
// **These are literal bytes off the wire**, captured 2026-08-31 between a Pico W
// running the pico-sdk's `tcp_client` (192.168.86.209) and a Mac
// (192.168.86.182). Nothing here was produced by `buildSegment` and read back:
// a module checked only against its own output proves that it is
// self-consistent, which is exactly what the `0a85b69` NTP client was.
// ---------------------------------------------------------------------------

const testing = std.testing;

const syn = [_]u8{
    0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB, 0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48,
    0x08, 0x00, 0x45, 0x00, 0x00, 0x2C, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x06,
    0x8C, 0xF1, 0xC0, 0xA8, 0x56, 0xD1, 0xC0, 0xA8, 0x56, 0xB6, 0xE5, 0xF0,
    0x10, 0x92, 0x00, 0x00, 0x19, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x60, 0x02,
    0x2D, 0xA0, 0x2B, 0xBE, 0x00, 0x00, 0x02, 0x04, 0x05, 0xB4,
};
const syn_ack = [_]u8{
    0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48, 0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB,
    0x08, 0x00, 0x45, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x40, 0x00, 0x40, 0x06,
    0x0B, 0xF4, 0xC0, 0xA8, 0x56, 0xB6, 0xC0, 0xA8, 0x56, 0xD1, 0x10, 0x92,
    0xE5, 0xF0, 0x58, 0xD2, 0x97, 0xB4, 0x00, 0x00, 0x19, 0x6E, 0x60, 0x12,
    0xFF, 0xFF, 0x68, 0xC6, 0x00, 0x00, 0x02, 0x04, 0x05, 0xB4,
};
const ack = [_]u8{
    0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB, 0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48,
    0x08, 0x00, 0x45, 0x00, 0x00, 0x28, 0x00, 0x03, 0x00, 0x00, 0xFF, 0x06,
    0x8C, 0xF4, 0xC0, 0xA8, 0x56, 0xD1, 0xC0, 0xA8, 0x56, 0xB6, 0xE5, 0xF0,
    0x10, 0x92, 0x00, 0x00, 0x19, 0x6E, 0x58, 0xD2, 0x97, 0xB5, 0x50, 0x10,
    0x2D, 0xA0, 0x52, 0xE3, 0x00, 0x00,
};
const rst = [_]u8{
    0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB, 0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48,
    0x08, 0x00, 0x45, 0x00, 0x00, 0x28, 0x00, 0x1B, 0x00, 0x00, 0xFF, 0x06,
    0x8C, 0xDC, 0xC0, 0xA8, 0x56, 0xD1, 0xC0, 0xA8, 0x56, 0xB6, 0xE5, 0xF0,
    0x10, 0x92, 0x00, 0x00, 0x69, 0x6F, 0x58, 0xD2, 0xE7, 0xB6, 0x50, 0x14,
    0xA0, 0x2D, 0x40, 0x4F, 0x00, 0x00,
};

const pico_ip = [4]u8{ 192, 168, 86, 209 };
const mac_ip = [4]u8{ 192, 168, 86, 182 };

test "the captured SYN parses, and its checksum is the peer's own" {
    // Parsed as the Mac would: addressed to 192.168.86.182.
    const s = try parseSegment(&syn, mac_ip);

    try testing.expectEqual(@as(u16, 58864), s.src_port);
    try testing.expectEqual(@as(u16, 4242), s.dest_port);
    try testing.expectEqual(@as(u32, 6509), s.seq);
    try testing.expectEqual(@as(u32, 0), s.ack);
    try testing.expect(s.has(Flag.syn));
    try testing.expect(!s.has(Flag.ack));
    try testing.expectEqual(@as(u16, 11680), s.window);
    try testing.expectEqual(@as(usize, 0), s.payload.len);

    // **lwIP offers MSS and nothing else** — no window scale, no SACK, no
    // timestamps — which is what makes this stack's decision to omit them a
    // match for a real peer rather than a shortcut.
    try testing.expectEqual(@as(?u16, 1460), s.mss);

    // `parseSegment` already rejected a bad checksum, so reaching here proves
    // the receiver's test passes against bytes this module did not create.
}

test "the captured SYN-ACK parses from the other direction" {
    const s = try parseSegment(&syn_ack, pico_ip);

    try testing.expectEqual(@as(u16, 4242), s.src_port);
    try testing.expectEqual(@as(u16, 58864), s.dest_port);
    try testing.expectEqual(@as(u32, 1490196404), s.seq);
    // **The ACK is the SYN's sequence plus one.** A SYN consumes a sequence
    // number without carrying a byte, which is the arithmetic N6 gets wrong
    // first if it is going to.
    try testing.expectEqual(@as(u32, 6510), s.ack);
    try testing.expect(s.has(Flag.syn) and s.has(Flag.ack));
    try testing.expectEqual(@as(u16, 65535), s.window);
    try testing.expectEqual(@as(?u16, 1460), s.mss);
}

test "the captured bare ACK carries no payload" {
    const s = try parseSegment(&ack, mac_ip);

    try testing.expect(s.has(Flag.ack) and !s.has(Flag.syn));
    try testing.expectEqual(@as(u32, 1490196405), s.ack);
    try testing.expectEqual(@as(?u16, null), s.mss);
    try testing.expectEqual(@as(usize, 0), s.payload.len);

    // **54 bytes is exactly the headers** — 14 + 20 + 20 — with nothing after
    // them. An earlier version of this test said the tail was Ethernet padding
    // and used that as the reason the payload must be zero, which was wrong:
    // `tcpdump` captured this above the layer that pads, so there is nothing
    // here to measure wrongly. The padded case is the test below, and the
    // mutation that measures the payload by the frame passes this one.
    try testing.expectEqual(@as(usize, 54), ack.len);
}

test "a padded ACK still carries no payload — the case the driver actually sees" {
    // **Ethernet pads a short frame to 60 bytes, and the CYW43 hands us the
    // padded version.** The capture does not show it because `tcpdump` sits
    // above the layer that adds it, so this is the wire as the *driver* sees it
    // rather than as the host does.
    var padded: [60]u8 = @splat(0);
    @memcpy(padded[0..ack.len], &ack);

    const s = try parseSegment(&padded, mac_ip);

    // **Six bytes of padding, and none of it is data.** A parser measuring the
    // payload by the frame length reports 6 bytes here — and a connection layer
    // that acknowledged six bytes nobody sent would advance the sequence number
    // past the peer's, corrupting the stream in a way the peer cannot diagnose
    // and cannot recover from.
    try testing.expectEqual(@as(usize, 0), s.payload.len);
    try testing.expectEqual(@as(u32, 1490196405), s.ack);
}

test "the captured RST is recognised as one" {
    const s = try parseSegment(&rst, mac_ip);
    try testing.expect(s.has(Flag.rst));
    try testing.expect(s.has(Flag.ack));
    // lwIP resets after a completed FIN exchange rather than lingering. N6 must
    // not read this as a failure.
}

test "a segment this stack builds parses as the peer would read it" {
    var buf: [128]u8 = undefined;
    const seg = try buildSegment(.{
        .dest_mac = .{ 0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB },
        .src_mac = .{ 0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48 },
        .src_ip = pico_ip,
        .dest_ip = mac_ip,
        .src_port = 50000,
        .dest_port = 4242,
        .seq = 1000,
        .ack = 2000,
        .flags = Flag.ack | Flag.psh,
        .window = 1460,
        .payload = "hello",
    }, &buf);

    // Round-tripped through the *parser*, which checks the checksum the way a
    // peer does — so this asserts more than that the builder agrees with itself.
    const s = try parseSegment(seg, mac_ip);
    try testing.expectEqual(@as(u32, 1000), s.seq);
    try testing.expectEqual(@as(u32, 2000), s.ack);
    try testing.expect(s.has(Flag.ack) and s.has(Flag.psh));
    try testing.expectEqualSlices(u8, "hello", s.payload);
    try testing.expectEqual(@as(?u16, null), s.mss);
}

test "a SYN this stack builds carries MSS the way the captured one does" {
    var buf: [128]u8 = undefined;
    const seg = try buildSegment(.{
        .dest_mac = .{ 0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB },
        .src_mac = .{ 0x28, 0xCD, 0xC1, 0x19, 0x4C, 0x48 },
        .src_ip = pico_ip,
        .dest_ip = mac_ip,
        .src_port = 50000,
        .dest_port = 4242,
        .seq = 1,
        .ack = 0,
        .flags = Flag.syn,
        .window = 1460,
        .mss = ethernet_mss,
    }, &buf);

    // **Byte-identical option encoding to the capture's**, which is the whole
    // reason to have the capture: `02 04 05 B4` is kind 2, length 4, 1460.
    const opts = seg[net.eth_header_len + net.ipv4_header_len + header_len ..];
    try testing.expectEqualSlices(u8, &[_]u8{ 0x02, 0x04, 0x05, 0xB4 }, opts[0..4]);
    try testing.expectEqualSlices(u8, &[_]u8{ 0x02, 0x04, 0x05, 0xB4 }, syn[syn.len - 4 ..]);

    const s = try parseSegment(seg, mac_ip);
    try testing.expectEqual(@as(?u16, 1460), s.mss);
    try testing.expectEqual(@as(usize, 0), s.payload.len);
}

test "segments this stack must refuse" {
    var buf: [128]u8 = undefined;

    // Addressed elsewhere.
    try testing.expectError(error.NotForUs, parseSegment(&syn, pico_ip));

    // Corrupted. A stack that acted on a segment failing its checksum would be
    // acting on noise, and TCP's whole contract is that it does not.
    @memcpy(buf[0..syn.len], &syn);
    buf[50] ^= 0xFF;
    try testing.expectError(error.BadChecksum, parseSegment(buf[0..syn.len], mac_ip));

    // A data offset claiming a header longer than the segment. **Unchecked,
    // this is a slice past the end of the frame** — reachable from the network
    // by anyone who cares to send it.
    @memcpy(buf[0..ack.len], &ack);
    buf[net.eth_header_len + net.ipv4_header_len + 12] = 0xF0; // 60-byte header
    try testing.expectError(error.BadDataOffset, parseSegment(buf[0..ack.len], mac_ip));

    // A fragment carries part of a stream; treating the part as the whole is
    // worse than losing it.
    @memcpy(buf[0..syn.len], &syn);
    std.mem.writeInt(u16, buf[net.eth_header_len + 6 ..][0..2], 0x2000, .big);
    try testing.expectError(error.Fragmented, parseSegment(buf[0..syn.len], mac_ip));
}

test "an unknown option is skipped, and a malformed one cannot spin" {
    // NOP, NOP, then a timestamp (kind 8, len 10) this stack does not
    // understand, then MSS. **It has to walk past the timestamp**, not refuse
    // it — the capture's peers send none, so a parser that rejected unknown
    // options would pass every test above and fail on the first peer that does.
    const opts = [_]u8{ 1, 1, 8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 2, 4, 0x05, 0xB4 };
    try testing.expectEqual(@as(?u16, 1460), parseMss(&opts));

    // **A length below 2 is malformed and must end the walk.** An option
    // claiming length 0 advances the cursor by nothing, and a `while` over it
    // is an infinite loop a stranger can trigger.
    try testing.expectEqual(@as(?u16, null), parseMss(&[_]u8{ 8, 0, 2, 4, 0x05, 0xB4 }));
    // A length running past the end is the same class of defect.
    try testing.expectEqual(@as(?u16, null), parseMss(&[_]u8{ 8, 40, 2, 4 }));
    // End-of-list stops the walk even with bytes after it.
    try testing.expectEqual(@as(?u16, null), parseMss(&[_]u8{ 0, 2, 4, 0x05, 0xB4 }));
}

// ---------------------------------------------------------------------------
// The connection — rung N6
//
// **A pure state machine.** It is fed parsed segments and a clock, and it
// returns what it wants sent; it never touches a device. That is the same split
// `dhcp.zig` has, and it is what lets a handshake, a refusal and a timeout all
// be tested without a network — the three things hardest to arrange on demand.
//
// The shipped contract, decided before the first line and not to be widened
// quietly:
//
//   * one connection at a time
//   * stop-and-wait: one segment in flight
//   * fixed MSS, no window scaling, no SACK
//   * out-of-order segments dropped, not queued
//   * no fragment reassembly
//
// N6 is the connection only. Data is N7, and `Established` is where it stops.
// ---------------------------------------------------------------------------

/// RFC 9293 §3.3.2.
///
/// **`listen` and `syn_received` arrived at rung N8**, and were deliberately
/// absent before it: a state present but unreachable is a claim this package
/// does not make.
pub const State = enum {
    closed,
    /// Waiting for somebody to connect to us. No remote address yet.
    listen,
    syn_sent,
    /// Their SYN answered; waiting for the ack that completes the handshake.
    syn_received,
    established,
    /// We sent FIN, waiting for it to be acknowledged.
    fin_wait_1,
    /// Our FIN is acknowledged; waiting for theirs.
    fin_wait_2,
    /// They sent FIN first; the application has not closed yet.
    close_wait,
    /// We answered their FIN with ours, waiting for the acknowledgement.
    last_ack,
    /// Both sides closed. Lingering, so a late retransmission finds a socket
    /// that knows what it was.
    time_wait,
};

pub const Error = error{
    /// The peer answered with RST — a port with nothing listening.
    ConnectionRefused,
    /// Nothing answered at all, and the retries ran out.
    ConnectTimeout,
    /// The peer reset an established connection.
    ConnectionReset,
};

/// What the machine wants put on the wire. Null means "nothing to send".
pub const Action = struct {
    flags: u8,
    seq: u32,
    ack: u32,
    /// Only on a SYN.
    mss: ?u16 = null,
    /// **Borrowed from the caller, never copied.** A `Connection` has no
    /// buffer and no allocator, so the bytes handed to `send` stay the
    /// caller's — and must stay *valid* until the peer acknowledges them,
    /// because a retransmission returns this same slice. Freeing or reusing it
    /// early retransmits whatever replaced it.
    data: []const u8 = &.{},
};

/// **RFC 6298 (2.1): one second until a measurement exists.**
pub const initial_rto_ms: u64 = 1000;

/// **(2.4): "if it is less than 1 second, then the RTO SHOULD be rounded up to
/// 1 second."**
///
/// Kept, and it is worth knowing what it costs here. Rung N4 measured this
/// LAN's round trip at **9 ms**, so this floor is roughly a hundred times the
/// real one and a single lost segment costs a full second. The RFC's reasoning
/// in §2.4 is that a large minimum keeps TCP conservative and avoids spurious
/// retransmissions, and that argument does not stop being true on a fast link —
/// a spurious retransmission is worse than a slow one. Lowering it is a
/// deliberate deviation from a MUST-adjacent SHOULD, not a tuning knob, and
/// nothing here has yet earned it.
pub const min_rto_ms: u64 = 1000;

/// A ceiling on the backoff. RFC 6298 permits a maximum "provided it is at
/// least 60 seconds".
pub const max_rto_ms: u64 = 60_000;

/// **Two minutes is RFC 9293's MSL, and TIME-WAIT is twice it.** That is four
/// minutes of holding a port on a device with one connection, which would make
/// a board unable to reconnect for four minutes after every close.
///
/// This ships **one second**, and the trade is worth stating rather than
/// hiding: the linger exists so a delayed duplicate from the old connection
/// cannot be accepted by a new one on the same four-tuple. On a LAN, where the
/// round trip measured at rung N4 is **9 ms**, a segment surviving a second is
/// already a thousand round trips old. On a path where it is not — anything
/// crossing the internet — this is too short, and a consumer that goes there
/// must raise it.
pub const time_wait_ms: u64 = 1000;

/// Retransmission of the SYN. **`tcp-blackhole.pcap` has lwIP sending two,
/// 2.74 s apart, then giving up**; `tcp-refused.pcap` has it sending one and
/// stopping instantly because a RST is an answer. Both behaviours are required
/// and they are different code paths.
pub const syn_rto_ms: u64 = 1000;
pub const syn_attempts: u32 = 4;

pub const Connection = struct {
    state: State = .closed,

    local_port: u16 = 0,
    remote_port: u16 = 0,
    remote_ip: [4]u8 = @splat(0),

    /// Send sequence: the next byte we will send.
    snd_nxt: u32 = 0,
    /// The oldest byte we have sent and not had acknowledged.
    snd_una: u32 = 0,
    /// Receive sequence: the next byte we expect from them.
    rcv_nxt: u32 = 0,
    /// What we advertise. **One MSS, because this is stop-and-wait** — telling
    /// a peer it may send more than we can hold is how the tail of every window
    /// gets dropped.
    rcv_wnd: u16 = ethernet_mss,
    /// Their advertised MSS, clamped. Null until a SYN carries one.
    snd_mss: u16 = default_mss,

    /// When the current retransmission is due, on the caller's clock.
    deadline_ms: u64 = 0,
    attempts: u32 = 0,

    /// Set when the peer's FIN has been seen and acknowledged.
    peer_closed: bool = false,

    // -- the send side: one segment in flight ------------------------------

    /// The unacknowledged payload, borrowed from the caller. Empty when
    /// nothing is outstanding — which, being stop-and-wait, is the only time
    /// `send` will accept more.
    pending: []const u8 = &.{},
    pending_seq: u32 = 0,
    /// **Karn's algorithm (§3): a retransmitted segment yields no RTT
    /// sample.** The ack is ambiguous — it may be answering the original or
    /// the copy — and feeding that ambiguity into the estimator is how an RTO
    /// collapses towards the wrong value and then retransmits constantly.
    pending_retransmitted: bool = false,
    sent_at_ms: u64 = 0,

    // -- the RTT estimator, in integers ------------------------------------
    //
    // **Scaled, because there is no FPU.** picozig subtracts `fpregs` from the
    // target on purpose, so every float operation is soft-emulated at about a
    // hundred cycles — `lib/cyw43`'s SBC decoder learned that the expensive
    // way. RFC 6298's alpha = 1/8 and beta = 1/4 are exact in binary, so the
    // usual BSD trick works: hold SRTT scaled by 8 and RTTVAR by 4, and the
    // shifts fall out.
    //
    // The pleasing consequence is that `rttvar4` **is** K*RTTVAR with K = 4,
    // so `RTO = SRTT + max(G, K*RTTVAR)` is `(srtt8 >> 3) + rttvar4` with no
    // multiply anywhere.

    srtt8: u32 = 0,
    rttvar4: u32 = 0,
    have_rtt: bool = false,
    rto_ms: u64 = initial_rto_ms,

    /// Payload accepted by the last `onSegment`, **borrowed from the frame it
    /// arrived in**. Valid until the next call; the caller copies what it
    /// wants to keep.
    received: []const u8 = &.{},

    /// Begin an active open. Returns the SYN to send.
    ///
    /// **The initial sequence number is the caller's to choose**, and it must
    /// not be a constant: RFC 9293 §3.4.1 wants it unpredictable, and a board
    /// that restarts with the same ISN can have an old connection's segments
    /// accepted by a new one on the same four-tuple.
    pub fn connect(
        self: *Connection,
        local_port: u16,
        remote_ip: [4]u8,
        remote_port: u16,
        isn: u32,
        now_ms: u64,
    ) Action {
        self.* = .{
            .state = .syn_sent,
            .local_port = local_port,
            .remote_port = remote_port,
            .remote_ip = remote_ip,
            .snd_nxt = isn +% 1, // the SYN consumes one
            .snd_una = isn,
            .deadline_ms = now_ms + syn_rto_ms,
            .attempts = 1,
        };
        return .{ .flags = Flag.syn, .seq = isn, .ack = 0, .mss = ethernet_mss };
    }

    /// RFC 6298 (2.2) and (2.3), in integers. `r` is the measured round trip.
    fn sampleRtt(self: *Connection, r_ms: u64) void {
        const r: u32 = @intCast(@min(r_ms, std.math.maxInt(u32)));
        if (!self.have_rtt) {
            // (2.2): SRTT <- R, RTTVAR <- R/2.
            self.srtt8 = r *| 8;
            self.rttvar4 = r *| 2; // (R/2) scaled by 4 is R*2
            self.have_rtt = true;
        } else {
            // (2.3), **and the order is mandated**: RTTVAR uses SRTT's value
            // from before SRTT is updated, so the delta is taken first.
            const srtt = self.srtt8 >> 3;
            const delta = if (r > srtt) r - srtt else srtt - r;
            // RTTVAR <- (1-1/4)*RTTVAR + (1/4)*|delta|, scaled by 4.
            self.rttvar4 = self.rttvar4 -| (self.rttvar4 >> 2) +| delta;
            // SRTT <- (1-1/8)*SRTT + (1/8)*R, scaled by 8.
            self.srtt8 = self.srtt8 -| (self.srtt8 >> 3) +| r;
        }
        // RTO <- SRTT + max(G, K*RTTVAR). `rttvar4` is already K*RTTVAR.
        const rto = @as(u64, self.srtt8 >> 3) + @as(u64, self.rttvar4);
        self.rto_ms = @min(@max(rto, min_rto_ms), max_rto_ms);
    }

    /// Hand data to the connection.
    ///
    /// **Returns null when something is already in flight** — this is
    /// stop-and-wait, and that is the whole of the congestion control. A
    /// caller offering more before the last segment is acknowledged is told
    /// no rather than having its data silently queued somewhere with no
    /// buffer to queue it in.
    ///
    /// The payload is **borrowed**: see `Action.data`.
    pub fn send(self: *Connection, payload: []const u8, now_ms: u64) ?Action {
        if (self.state != .established and self.state != .close_wait) return null;
        if (self.pending.len != 0 or payload.len == 0) return null;
        if (payload.len > self.snd_mss) return null;

        self.pending = payload;
        self.pending_seq = self.snd_nxt;
        self.pending_retransmitted = false;
        self.sent_at_ms = now_ms;
        self.snd_nxt +%= @intCast(payload.len);
        self.deadline_ms = now_ms + self.rto_ms;
        self.attempts = 1;

        return .{
            .flags = Flag.ack | Flag.psh,
            .seq = self.pending_seq,
            .ack = self.rcv_nxt,
            .data = payload,
        };
    }

    /// True when the caller may offer more data.
    pub fn writable(self: *const Connection) bool {
        return (self.state == .established or self.state == .close_wait) and self.pending.len == 0;
    }

    /// Wait for somebody to connect to `local_port`.
    ///
    /// **The ISN is still the caller's**, and for the same reason as
    /// `connect`: RFC 9293 §3.4.1 wants it unpredictable, and a board that
    /// restarts with the same one can have an old connection's segments
    /// accepted by a new one on the same four-tuple.
    pub fn listen(self: *Connection, local_port: u16, isn: u32) void {
        self.* = .{ .state = .listen, .local_port = local_port, .snd_una = isn };
    }

    /// Whether a segment belongs to this connection.
    ///
    /// **A listener has no remote address yet**, so it matches on the local
    /// port alone — but only a bare SYN. Accepting anything else would hand a
    /// listening socket a stray segment from a connection it has never heard
    /// of, and the four-tuple is not established until the SYN names it.
    pub fn matches(self: *const Connection, s: Segment) bool {
        if (self.state == .closed) return false;
        if (self.state == .listen) {
            return s.dest_port == self.local_port and s.has(Flag.syn) and !s.has(Flag.ack);
        }
        return s.src_port == self.remote_port and
            s.dest_port == self.local_port and
            std.mem.eql(u8, &s.src_ip, &self.remote_ip);
    }

    /// Feed a segment in. Returns what to send, or null.
    ///
    /// **A RST is checked before anything else**, because RFC 9293 §3.10.7
    /// says so and because the alternative is acting on a segment from a peer
    /// that has just told you it has no connection.
    pub fn onSegment(self: *Connection, s: Segment, now_ms: u64) Error!?Action {
        if (s.has(Flag.rst)) {
            const was = self.state;
            // **A reset in SYN-RECEIVED returns a passive open to LISTEN**
            // (§3.10.7.4) rather than closing it: the listener was never the
            // one that asked, so there is nothing to report and it should go
            // back to waiting. A server that closed here would stop listening
            // because one client changed its mind.
            if (was == .syn_received) {
                self.listen(self.local_port, self.snd_una);
                return null;
            }
            self.state = .closed;
            // **A RST after a completed FIN exchange is not a failure.** lwIP
            // frees its pcb and resets rather than lingering — the captured
            // session ends exactly that way — so a stack that
            // reported an error here would report one on every successful
            // close against that peer.
            if (was == .syn_sent) return Error.ConnectionRefused;
            if (was == .time_wait or was == .last_ack or was == .fin_wait_2) return null;
            return Error.ConnectionReset;
        }

        switch (self.state) {
            .listen => {
                // The `matches` filter has already refused anything that is
                // not a bare SYN, but this is reachable directly.
                if (!s.has(Flag.syn) or s.has(Flag.ack)) return null;

                self.remote_ip = s.src_ip;
                self.remote_port = s.src_port;
                self.rcv_nxt = s.seq +% 1; // their SYN consumes one
                self.snd_nxt = self.snd_una +% 1; // and so will ours
                if (s.mss) |m| self.snd_mss = @min(m, ethernet_mss);
                self.state = .syn_received;
                self.deadline_ms = now_ms + syn_rto_ms;
                self.attempts = 1;

                return .{
                    .flags = Flag.syn | Flag.ack,
                    .seq = self.snd_una,
                    .ack = self.rcv_nxt,
                    .mss = ethernet_mss,
                };
            },

            .syn_received => {
                if (s.has(Flag.syn)) {
                    // Their SYN again — theirs was duplicated or our SYN-ACK
                    // was lost. Answer it rather than ignoring it, but only if
                    // it is the same SYN.
                    if (s.seq +% 1 != self.rcv_nxt) return null;
                    return .{
                        .flags = Flag.syn | Flag.ack,
                        .seq = self.snd_una,
                        .ack = self.rcv_nxt,
                        .mss = ethernet_mss,
                    };
                }
                if (!s.has(Flag.ack)) return null;
                // **§3.10.7.4: SND.UNA < SEG.ACK =< SND.NXT.** Our SYN
                // consumed one sequence number, so the only acceptable ack is
                // `snd_nxt`. Taking any ack here lets a stale segment complete
                // a handshake that never happened.
                if (s.ack != self.snd_nxt) return null;
                self.snd_una = s.ack;
                self.state = .established;
                self.deadline_ms = 0;
                self.attempts = 0;
                return null;
            },

            .syn_sent => {
                if (!s.has(Flag.syn)) return null;
                if (s.has(Flag.ack)) {
                    // **Their ack must be for our SYN, which consumed one
                    // sequence number.** Accepting any ack here is how a stale
                    // segment from a previous connection completes this one.
                    if (s.ack != self.snd_nxt) return null;
                    self.snd_una = s.ack;
                    self.rcv_nxt = s.seq +% 1; // their SYN consumes one too
                    if (s.mss) |m| self.snd_mss = @min(m, ethernet_mss);
                    self.state = .established;
                    self.deadline_ms = 0;
                    return .{ .flags = Flag.ack, .seq = self.snd_nxt, .ack = self.rcv_nxt };
                }
                return null;
            },

            .established, .close_wait => {
                var must_ack = false;

                // **Their acknowledgement of our outstanding segment.**
                if (s.has(Flag.ack) and self.pending.len != 0) {
                    const covers = s.ack == self.pending_seq +% @as(u32, @intCast(self.pending.len));
                    if (covers) {
                        // Karn: only an un-retransmitted segment yields a
                        // sample, because only then is the ack unambiguous.
                        if (!self.pending_retransmitted) self.sampleRtt(now_ms -| self.sent_at_ms);
                        self.snd_una = s.ack;
                        self.pending = &.{};
                        self.deadline_ms = 0;
                        self.attempts = 0;
                    }
                }

                // **Their data, in order only.** Out of order is dropped, as
                // the shipped contract says — but it is still acknowledged
                // with `rcv_nxt`, which tells the peer exactly which byte we
                // are missing and makes it retransmit. Silence would leave it
                // waiting for its own timer.
                if (self.state == .established and s.payload.len != 0) {
                    if (s.seq == self.rcv_nxt) {
                        self.received = s.payload;
                        self.rcv_nxt +%= @intCast(s.payload.len);
                    }
                    must_ack = true;
                }

                if (s.has(Flag.fin) and self.state == .established and s.seq == self.rcv_nxt) {
                    self.rcv_nxt = s.seq +% 1;
                    self.peer_closed = true;
                    self.state = .close_wait;
                    return .{ .flags = Flag.ack, .seq = self.snd_nxt, .ack = self.rcv_nxt };
                }

                if (must_ack) return .{ .flags = Flag.ack, .seq = self.snd_nxt, .ack = self.rcv_nxt };
                return null;
            },

            .fin_wait_1 => {
                const acked = s.has(Flag.ack) and s.ack == self.snd_nxt;
                if (s.has(Flag.fin)) {
                    self.rcv_nxt = s.seq +% 1;
                    self.peer_closed = true;
                    // **Simultaneous close**: their FIN arrived before our own
                    // was acknowledged, so both are done at once.
                    self.state = if (acked) .time_wait else .last_ack;
                    self.deadline_ms = now_ms + time_wait_ms;
                    return .{ .flags = Flag.ack, .seq = self.snd_nxt, .ack = self.rcv_nxt };
                }
                if (acked) {
                    self.snd_una = s.ack;
                    self.state = .fin_wait_2;
                }
                return null;
            },

            .fin_wait_2 => {
                if (!s.has(Flag.fin)) return null;
                self.rcv_nxt = s.seq +% 1;
                self.peer_closed = true;
                self.state = .time_wait;
                self.deadline_ms = now_ms + time_wait_ms;
                return .{ .flags = Flag.ack, .seq = self.snd_nxt, .ack = self.rcv_nxt };
            },

            .last_ack => {
                if (s.has(Flag.ack) and s.ack == self.snd_nxt) {
                    self.state = .time_wait;
                    self.deadline_ms = now_ms + time_wait_ms;
                }
                return null;
            },

            .time_wait, .closed => return null,
        }
    }

    /// Close from this end. Returns the FIN to send, or null if there is
    /// nothing to close.
    pub fn close(self: *Connection, now_ms: u64) ?Action {
        switch (self.state) {
            .established => {
                self.state = .fin_wait_1;
                const seq = self.snd_nxt;
                self.snd_nxt +%= 1; // the FIN consumes one
                self.deadline_ms = now_ms + syn_rto_ms;
                return .{ .flags = Flag.fin | Flag.ack, .seq = seq, .ack = self.rcv_nxt };
            },
            .close_wait => {
                self.state = .last_ack;
                const seq = self.snd_nxt;
                self.snd_nxt +%= 1;
                self.deadline_ms = now_ms + syn_rto_ms;
                return .{ .flags = Flag.fin | Flag.ack, .seq = seq, .ack = self.rcv_nxt };
            },
            else => return null,
        }
    }

    /// Give the machine time. Returns a retransmission, or null.
    ///
    /// **Every wait here is bounded and reports.** A SYN that goes unanswered
    /// gives up after `syn_attempts` — `tcp-blackhole.pcap` is what that looks
    /// like on the wire — rather than leaving an application waiting on a host
    /// that is not there.
    pub fn tick(self: *Connection, now_ms: u64) Error!?Action {
        if (self.deadline_ms == 0 or now_ms < self.deadline_ms) return null;

        switch (self.state) {
            .syn_sent => {
                if (self.attempts >= syn_attempts) {
                    self.state = .closed;
                    return Error.ConnectTimeout;
                }
                self.attempts += 1;
                self.deadline_ms = now_ms + syn_rto_ms;
                // **Byte-identical to the first**, same sequence number: this
                // is a retransmission, not a new attempt. `tcp-blackhole.pcap`
                // shows lwIP doing exactly that, same seq and same checksum.
                return .{ .flags = Flag.syn, .seq = self.snd_una, .ack = 0, .mss = ethernet_mss };
            },
            .time_wait => {
                self.state = .closed;
                self.deadline_ms = 0;
                return null;
            },
            .syn_received => {
                if (self.attempts >= syn_attempts) {
                    // The client vanished mid-handshake. **Back to listening,
                    // not closed** — the socket was never theirs to end.
                    self.listen(self.local_port, self.snd_una);
                    return null;
                }
                self.attempts += 1;
                self.deadline_ms = now_ms + syn_rto_ms;
                return .{
                    .flags = Flag.syn | Flag.ack,
                    .seq = self.snd_una,
                    .ack = self.rcv_nxt,
                    .mss = ethernet_mss,
                };
            },

            .established, .close_wait => {
                if (self.pending.len == 0) {
                    self.deadline_ms = 0;
                    return null;
                }
                if (self.attempts >= syn_attempts) {
                    self.state = .closed;
                    self.deadline_ms = 0;
                    return Error.ConnectTimeout;
                }
                self.attempts += 1;
                self.pending_retransmitted = true;
                // **(5.5): back off by doubling.** Without this a lost segment
                // on a congested path is retried at a fixed rate, which is the
                // behaviour that makes congestion worse rather than better.
                self.rto_ms = @min(self.rto_ms * 2, max_rto_ms);
                self.deadline_ms = now_ms + self.rto_ms;
                return .{
                    .flags = Flag.ack | Flag.psh,
                    .seq = self.pending_seq,
                    .ack = self.rcv_nxt,
                    .data = self.pending,
                };
            },

            .fin_wait_1, .last_ack => {
                if (self.attempts >= syn_attempts) {
                    // **A peer that never acknowledges our FIN must not hold
                    // the port forever.** Giving up here is a lost close, which
                    // is untidy; hanging is worse.
                    self.state = .closed;
                    self.deadline_ms = 0;
                    return null;
                }
                self.attempts += 1;
                self.deadline_ms = now_ms + syn_rto_ms;
                return .{ .flags = Flag.fin | Flag.ack, .seq = self.snd_nxt -% 1, .ack = self.rcv_nxt };
            },
            else => {
                self.deadline_ms = 0;
                return null;
            },
        }
    }
};

// ---------------------------------------------------------------------------
// Rung N6 — the connection, with no network in sight
// ---------------------------------------------------------------------------

/// A segment as the peer would send it. Built here rather than through
/// `buildSegment` because these tests are about the *machine*, not the wire —
/// the wire is N5's business and is asserted against the capture.
fn peerSeg(flags: u8, seq: u32, ack_no: u32, mss: ?u16) Segment {
    return .{
        .src_ip = mac_ip,
        .dest_ip = pico_ip,
        .src_mac = .{ 0x3A, 0xB6, 0x81, 0xD2, 0x4E, 0xEB },
        .src_port = 4242,
        .dest_port = 50000,
        .seq = seq,
        .ack = ack_no,
        .flags = flags,
        .window = 65535,
        .mss = mss,
        .payload = &.{},
    };
}

test "the handshake, in the order the capture shows it" {
    var c = Connection{};
    const syn_out = c.connect(50000, mac_ip, 4242, 6509, 0);

    // The SYN this stack sends matches the captured one's shape: sequence as
    // given, MSS offered, nothing acknowledged.
    try testing.expectEqual(Flag.syn, syn_out.flags);
    try testing.expectEqual(@as(u32, 6509), syn_out.seq);
    try testing.expectEqual(@as(?u16, 1460), syn_out.mss);
    try testing.expectEqual(State.syn_sent, c.state);

    // **The SYN-ACK's ack is our sequence plus one**, which is the arithmetic
    // the capture pinned: a SYN consumes a sequence number without carrying a
    // byte.
    const reply = (try c.onSegment(peerSeg(Flag.syn | Flag.ack, 1490196404, 6510, 1460), 5)).?;
    try testing.expectEqual(State.established, c.state);
    try testing.expectEqual(Flag.ack, reply.flags);
    try testing.expectEqual(@as(u32, 6510), reply.seq);
    try testing.expectEqual(@as(u32, 1490196405), reply.ack);
    try testing.expectEqual(@as(u16, 1460), c.snd_mss);
}

test "a SYN-ACK acknowledging the wrong sequence is not our handshake" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 6509, 0);

    // **Off by one in the direction that matters**: acking the SYN's own
    // sequence rather than the next. Accepting any ack here is how a stale
    // segment from an earlier connection on the same four-tuple completes this
    // one, and it is indistinguishable from success afterwards.
    try testing.expectEqual(@as(?Action, null), try c.onSegment(peerSeg(Flag.syn | Flag.ack, 1, 6509, 1460), 5));
    try testing.expectEqual(State.syn_sent, c.state);
}

test "a refused port is reported at once, not waited out" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 6509, 0);

    // `tcp-refused.pcap`: the RST comes back 84 microseconds after the SYN and
    // lwIP stops immediately — **one SYN, no retransmission**, because a RST is
    // an answer. A stack that waited out its timer here would take four seconds
    // to report what the peer said instantly.
    try testing.expectError(Error.ConnectionRefused, c.onSegment(peerSeg(Flag.rst | Flag.ack, 0, 6510, null), 1));
    try testing.expectEqual(State.closed, c.state);
}

test "a black hole is given up on, bounded, and reported" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 6509, 0);

    // `tcp-blackhole.pcap`: nothing answers. Each retransmission carries **the
    // same sequence number** — it is a retransmission, not a new attempt, and
    // the capture shows lwIP's second SYN byte-identical to its first.
    var t: u64 = 0;
    var sent: u32 = 1;
    while (sent < syn_attempts) : (sent += 1) {
        t += syn_rto_ms;
        const again = (try c.tick(t)).?;
        try testing.expectEqual(Flag.syn, again.flags);
        try testing.expectEqual(@as(u32, 6509), again.seq);
    }

    // And then it stops, rather than leaving an application waiting on a host
    // that is not there.
    t += syn_rto_ms;
    try testing.expectError(Error.ConnectTimeout, c.tick(t));
    try testing.expectEqual(State.closed, c.state);
}

test "an active close walks FIN-WAIT-1, FIN-WAIT-2, TIME-WAIT" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);
    _ = try c.onSegment(peerSeg(Flag.syn | Flag.ack, 500, 101, 1460), 1);

    const fin = c.close(10).?;
    try testing.expectEqual(Flag.fin | Flag.ack, fin.flags);
    try testing.expectEqual(@as(u32, 101), fin.seq);
    try testing.expectEqual(State.fin_wait_1, c.state);

    // Their ack of our FIN — **our FIN consumed a sequence number**, so it
    // acknowledges 102.
    try testing.expectEqual(@as(?Action, null), try c.onSegment(peerSeg(Flag.ack, 501, 102, null), 11));
    try testing.expectEqual(State.fin_wait_2, c.state);

    // Then theirs, which we acknowledge.
    const last = (try c.onSegment(peerSeg(Flag.fin | Flag.ack, 501, 102, null), 12)).?;
    try testing.expectEqual(Flag.ack, last.flags);
    try testing.expectEqual(@as(u32, 502), last.ack);
    try testing.expectEqual(State.time_wait, c.state);

    // **TIME-WAIT is a linger, not a state to be stuck in.** It must expire on
    // its own, or a board with one connection could never reconnect.
    try testing.expectEqual(@as(?Action, null), try c.tick(12 + time_wait_ms));
    try testing.expectEqual(State.closed, c.state);
}

test "a RST after a completed close is not an error" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);
    _ = try c.onSegment(peerSeg(Flag.syn | Flag.ack, 500, 101, 1460), 1);
    _ = c.close(10);
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 102, null), 11);
    _ = try c.onSegment(peerSeg(Flag.fin | Flag.ack, 501, 102, null), 12);
    try testing.expectEqual(State.time_wait, c.state);

    // **The captured session ends exactly this way**: lwIP frees its pcb
    // and resets rather than lingering. A stack that reported an error here
    // would report one on every successful close against that peer — which is
    // to say, on every successful HTTP request at N10.
    try testing.expectEqual(@as(?Action, null), try c.onSegment(peerSeg(Flag.rst | Flag.ack, 502, 102, null), 13));
    try testing.expectEqual(State.closed, c.state);
}

test "a RST on an established connection is an error" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);
    _ = try c.onSegment(peerSeg(Flag.syn | Flag.ack, 500, 101, 1460), 1);

    // The other side of the same coin: here it means the peer threw the
    // connection away, and telling the application nothing would leave it
    // writing into a socket that no longer exists.
    try testing.expectError(Error.ConnectionReset, c.onSegment(peerSeg(Flag.rst, 501, 101, null), 5));
    try testing.expectEqual(State.closed, c.state);
}

test "a passive close: their FIN first, then ours" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);
    _ = try c.onSegment(peerSeg(Flag.syn | Flag.ack, 500, 101, 1460), 1);

    // They close first. **We acknowledge and stay open** — CLOSE-WAIT exists
    // because their having nothing more to say does not mean we have nothing.
    const a = (try c.onSegment(peerSeg(Flag.fin | Flag.ack, 501, 101, null), 5)).?;
    try testing.expectEqual(Flag.ack, a.flags);
    try testing.expectEqual(@as(u32, 502), a.ack);
    try testing.expectEqual(State.close_wait, c.state);
    try testing.expect(c.peer_closed);

    const fin = c.close(6).?;
    try testing.expectEqual(Flag.fin | Flag.ack, fin.flags);
    try testing.expectEqual(State.last_ack, c.state);

    _ = try c.onSegment(peerSeg(Flag.ack, 502, 102, null), 7);
    try testing.expectEqual(State.time_wait, c.state);
}

test "segments for somebody else's connection are not ours" {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);

    try testing.expect(c.matches(peerSeg(Flag.ack, 1, 101, null)));

    var wrong_port = peerSeg(Flag.ack, 1, 101, null);
    wrong_port.src_port = 4243;
    try testing.expect(!c.matches(wrong_port));

    var wrong_host = peerSeg(Flag.ack, 1, 101, null);
    wrong_host.src_ip = .{ 192, 168, 86, 99 };
    try testing.expect(!c.matches(wrong_host));

    // **A closed connection matches nothing.** Without this a late segment
    // from a finished connection is fed to a machine that has forgotten it.
    c.state = .closed;
    try testing.expect(!c.matches(peerSeg(Flag.ack, 1, 101, null)));
}

// ---------------------------------------------------------------------------
// Rung N7 — data, and the retransmission timer
// ---------------------------------------------------------------------------

/// A segment with a payload, as the peer would send it.
fn peerData(seq: u32, ack_no: u32, payload: []const u8) Segment {
    var s = peerSeg(Flag.ack | Flag.psh, seq, ack_no, null);
    s.payload = payload;
    return s;
}

/// An established connection: ISN 100, peer ISN 500.
fn established() Connection {
    var c = Connection{};
    _ = c.connect(50000, mac_ip, 4242, 100, 0);
    _ = c.onSegment(peerSeg(Flag.syn | Flag.ack, 500, 101, 1460), 1) catch unreachable;
    return c;
}

test "one segment in flight, and the second is refused rather than queued" {
    var c = established();
    try testing.expect(c.writable());

    const a = c.send("hello", 10).?;
    try testing.expectEqual(@as(u32, 101), a.seq);
    try testing.expectEqualSlices(u8, "hello", a.data);

    // **Stop-and-wait is the whole congestion control**, and it says no rather
    // than queueing into a buffer this type does not have. A caller told
    // "null" can hold its data; one whose data was silently accepted and
    // dropped cannot.
    try testing.expect(!c.writable());
    try testing.expectEqual(@as(?Action, null), c.send("world", 11));

    // Acknowledged, and it is writable again.
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 106, null), 20);
    try testing.expect(c.writable());
    try testing.expectEqual(@as(usize, 0), c.pending.len);
}

test "an ack that does not cover the segment does not release it" {
    var c = established();
    _ = c.send("hello", 10);

    // Five bytes were sent from 101, so only 106 covers them. **A partial ack
    // must not clear the pending segment** — releasing it would leave bytes
    // the peer never confirmed with nothing left to retransmit them.
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 103, null), 20);
    try testing.expect(!c.writable());
    try testing.expectEqual(@as(usize, 5), c.pending.len);
}

test "the RTT estimator follows RFC 6298, in integers" {
    var c = established();

    // (2.2), first measurement R = 40: SRTT = 40, RTTVAR = 20,
    // RTO = 40 + 4*20 = 120 — then floored at 1 s by (2.4).
    _ = c.send("hello", 100);
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 106, null), 140);
    try testing.expect(c.have_rtt);
    try testing.expectEqual(@as(u32, 40), c.srtt8 >> 3);
    try testing.expectEqual(@as(u32, 80), c.rttvar4); // K*RTTVAR = 4*20
    try testing.expectEqual(min_rto_ms, c.rto_ms);

    // (2.3), and **the second sample differs from the first on purpose**. An
    // earlier version of this test used 40 twice, which cannot fail: with
    // R' equal to SRTT the delta is zero whichever order the two updates are
    // done in, so it passed against a version that updated SRTT first —
    // exactly the ordering the RFC calls out as MUST.
    //
    // R' = 80 against SRTT = 40 separates them: RTTVAR must use SRTT's value
    // *before* SRTT is updated, giving delta 40 and K*RTTVAR 100. Updating
    // SRTT first gives delta 35 and 95.
    _ = c.send("again", 200);
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 111, null), 280);
    try testing.expectEqual(@as(u32, 45), c.srtt8 >> 3);
    try testing.expectEqual(@as(u32, 100), c.rttvar4);
}

test "a retransmitted segment yields no RTT sample — Karn's algorithm" {
    var c = established();
    _ = c.send("hello", 0);

    // Nothing answers; the timer fires and the segment goes again, identically.
    const again = (try c.tick(initial_rto_ms)).?;
    try testing.expectEqual(@as(u32, 101), again.seq);
    try testing.expectEqualSlices(u8, "hello", again.data);
    try testing.expect(c.pending_retransmitted);

    // **The ack that follows is ambiguous** — it may be answering the original
    // or the copy — so no sample is taken from it. Feeding that ambiguity into
    // the estimator is how an RTO collapses towards a wrong value and then
    // retransmits constantly.
    _ = try c.onSegment(peerSeg(Flag.ack, 501, 106, null), initial_rto_ms + 5);
    try testing.expect(!c.have_rtt);
    try testing.expectEqual(@as(u32, 0), c.srtt8);
}

test "the retransmission timer backs off, and then gives up" {
    var c = established();
    _ = c.send("hello", 0);
    try testing.expectEqual(initial_rto_ms, c.rto_ms);

    // (5.5): doubling. A fixed retry rate on a congested path is the behaviour
    // that makes congestion worse rather than better.
    var t: u64 = 0;
    var seen: u32 = 0;
    while (seen < syn_attempts - 1) : (seen += 1) {
        t += c.rto_ms;
        const a = (try c.tick(t)).?;
        // Every retransmission is the same bytes at the same sequence.
        try testing.expectEqual(@as(u32, 101), a.seq);
        try testing.expectEqualSlices(u8, "hello", a.data);
    }
    try testing.expectEqual(@as(u64, initial_rto_ms * 8), c.rto_ms);

    // And it is bounded: a peer that has gone away must not hold a caller
    // forever.
    t += c.rto_ms;
    try testing.expectError(Error.ConnectTimeout, c.tick(t));
    try testing.expectEqual(State.closed, c.state);
}

test "data in order is delivered and acknowledged" {
    var c = established();

    const a = (try c.onSegment(peerData(501, 101, "abcde"), 10)).?;
    try testing.expectEqualSlices(u8, "abcde", c.received);
    try testing.expectEqual(Flag.ack, a.flags);
    // **506, not 501**: the acknowledgement names the next byte expected, and
    // an off-by-one here makes the peer resend everything forever.
    try testing.expectEqual(@as(u32, 506), a.ack);
}

test "data out of order is dropped, and the ack says which byte is wanted" {
    var c = established();
    _ = try c.onSegment(peerData(501, 101, "abcde"), 10);

    // A gap: 511 arrives while 506 is expected. The shipped contract drops it
    // rather than queueing — but **it is still acknowledged with `rcv_nxt`**,
    // which tells the peer exactly which byte is missing and makes it
    // retransmit. Silence would leave it waiting on its own timer instead.
    c.received = &.{};
    const a = (try c.onSegment(peerData(511, 101, "fghij"), 20)).?;
    try testing.expectEqual(@as(usize, 0), c.received.len);
    try testing.expectEqual(@as(u32, 506), a.ack);
    try testing.expectEqual(@as(u32, 506), c.rcv_nxt);

    // The missing bytes, when they come, are taken.
    const b = (try c.onSegment(peerData(506, 101, "fghij"), 30)).?;
    try testing.expectEqualSlices(u8, "fghij", c.received);
    try testing.expectEqual(@as(u32, 511), b.ack);
}

test "a segment larger than the peer's MSS is refused" {
    var c = established();
    c.snd_mss = 8;

    // **The peer told us what it can take**, and exceeding it is how a segment
    // gets fragmented or dropped by something in the middle. Refusing is the
    // caller's cue to split it.
    try testing.expectEqual(@as(?Action, null), c.send("more than eight bytes", 0));
    try testing.expect(c.send("eight!!!", 0) != null);
}

test "data and a FIN both move the sequence, and both are acknowledged" {
    var c = established();
    _ = try c.onSegment(peerData(501, 101, "abcde"), 10);
    try testing.expectEqual(@as(u32, 506), c.rcv_nxt);

    // Their FIN follows the data it came after. **It consumes one sequence
    // number of its own**, so the ack is 507.
    const a = (try c.onSegment(peerSeg(Flag.fin | Flag.ack, 506, 101, null), 20)).?;
    try testing.expectEqual(@as(u32, 507), a.ack);
    try testing.expectEqual(State.close_wait, c.state);
    try testing.expect(c.peer_closed);
}

// ---------------------------------------------------------------------------
// Rung N8 — accepting a connection
// ---------------------------------------------------------------------------

/// A client's opening SYN, from the Mac to us.
fn clientSyn(seq: u32) Segment {
    var s = peerSeg(Flag.syn, seq, 0, 1460);
    s.dest_port = 4242;
    return s;
}

test "a listener accepts a SYN and completes the handshake" {
    var c = Connection{};
    c.listen(4242, 7000);
    try testing.expectEqual(State.listen, c.state);

    // Their SYN. **The four-tuple is learned here** — a listener has no remote
    // address until somebody names one.
    const sa = (try c.onSegment(clientSyn(500), 0)).?;
    try testing.expectEqual(State.syn_received, c.state);
    try testing.expectEqual(mac_ip, c.remote_ip);
    try testing.expectEqual(@as(u16, 4242), c.remote_port);

    // The SYN-ACK, shaped as the capture's macOS one is: our ISN, their
    // sequence plus one, and an MSS of our own.
    try testing.expectEqual(Flag.syn | Flag.ack, sa.flags);
    try testing.expectEqual(@as(u32, 7000), sa.seq);
    try testing.expectEqual(@as(u32, 501), sa.ack);
    try testing.expectEqual(@as(?u16, 1460), sa.mss);

    // Their ack completes it. **7001, because our SYN consumed one.**
    var client_ack = peerSeg(Flag.ack, 501, 7001, null);
    client_ack.dest_port = 4242;
    try testing.expectEqual(@as(?Action, null), try c.onSegment(client_ack, 5));
    try testing.expectEqual(State.established, c.state);
}

test "an ack that does not complete our handshake is refused" {
    var c = Connection{};
    c.listen(4242, 7000);
    _ = try c.onSegment(clientSyn(500), 0);

    // **§3.10.7.4 wants SND.UNA < SEG.ACK =< SND.NXT**, and our SYN consumed a
    // sequence number, so 7000 is not it. Accepting any ack here lets a stale
    // segment complete a handshake that never happened — and afterwards the
    // connection is indistinguishable from a real one.
    var stale = peerSeg(Flag.ack, 501, 7000, null);
    stale.dest_port = 4242;
    try testing.expectEqual(@as(?Action, null), try c.onSegment(stale, 5));
    try testing.expectEqual(State.syn_received, c.state);
}

test "a listener matches a SYN from anyone, and nothing else" {
    var c = Connection{};
    c.listen(4242, 7000);

    try testing.expect(c.matches(clientSyn(1)));

    // **Not a stray ACK.** A listening socket has no four-tuple yet, so
    // matching on the local port alone would hand it segments from
    // connections it has never heard of.
    var stray = peerSeg(Flag.ack, 1, 1, null);
    stray.dest_port = 4242;
    try testing.expect(!c.matches(stray));

    // Nor a SYN-ACK, which belongs to somebody else's active open.
    var synack = peerSeg(Flag.syn | Flag.ack, 1, 1, 1460);
    synack.dest_port = 4242;
    try testing.expect(!c.matches(synack));

    // Nor another port.
    var other = clientSyn(1);
    other.dest_port = 80;
    try testing.expect(!c.matches(other));
}

test "a reset mid-handshake returns the listener to listening" {
    var c = Connection{};
    c.listen(4242, 7000);
    _ = try c.onSegment(clientSyn(500), 0);
    try testing.expectEqual(State.syn_received, c.state);

    // **Back to LISTEN, not CLOSED, and no error** (§3.10.7.4). The listener
    // never asked for this connection, so there is nothing to report — and a
    // server that closed here would stop listening because one client changed
    // its mind.
    var client_rst = peerSeg(Flag.rst, 501, 7001, null);
    client_rst.dest_port = 4242;
    try testing.expectEqual(@as(?Action, null), try c.onSegment(client_rst, 5));
    try testing.expectEqual(State.listen, c.state);

    // And it can still accept the next one.
    _ = try c.onSegment(clientSyn(900), 10);
    try testing.expectEqual(State.syn_received, c.state);
    try testing.expectEqual(@as(u32, 901), c.rcv_nxt);
}

test "a client that vanishes mid-handshake does not strand the listener" {
    var c = Connection{};
    c.listen(4242, 7000);
    _ = try c.onSegment(clientSyn(500), 0);

    // The SYN-ACK goes again while nobody answers.
    var t: u64 = 0;
    var seen: u32 = 0;
    while (seen < syn_attempts - 1) : (seen += 1) {
        t += syn_rto_ms;
        const again = (try c.tick(t)).?;
        try testing.expectEqual(Flag.syn | Flag.ack, again.flags);
        try testing.expectEqual(@as(u32, 7000), again.seq);
    }

    // Then it gives up — **and goes back to listening rather than closing.**
    // A half-open connection that held the socket would mean one abandoned
    // client took the server down until it was restarted.
    t += syn_rto_ms;
    try testing.expectEqual(@as(?Action, null), try c.tick(t));
    try testing.expectEqual(State.listen, c.state);
}

test "a duplicated SYN is answered again, not ignored" {
    var c = Connection{};
    c.listen(4242, 7000);
    _ = try c.onSegment(clientSyn(500), 0);

    // Their SYN arrives twice — theirs was duplicated, or our SYN-ACK was
    // lost. **Answering costs one segment; ignoring it means waiting for
    // whichever timer fires first.**
    const again = (try c.onSegment(clientSyn(500), 1)).?;
    try testing.expectEqual(Flag.syn | Flag.ack, again.flags);
    try testing.expectEqual(@as(u32, 501), again.ack);

    // A *different* SYN is a different connection, and this socket is taken.
    try testing.expectEqual(@as(?Action, null), try c.onSegment(clientSyn(900), 2));
    try testing.expectEqual(@as(u32, 501), c.rcv_nxt);
}

test "an accepted connection carries data like any other" {
    var c = Connection{};
    c.listen(4242, 7000);
    _ = try c.onSegment(clientSyn(500), 0);
    var client_ack = peerSeg(Flag.ack, 501, 7001, null);
    client_ack.dest_port = 4242;
    _ = try c.onSegment(client_ack, 5);

    // **The point of N8 is that N7 keeps working above it.** A server reached
    // this state by a different route, and nothing downstream should care.
    var data = peerData(501, 7001, "GET / HTTP/1.0\r\n");
    data.dest_port = 4242;
    const a = (try c.onSegment(data, 10)).?;
    try testing.expectEqualSlices(u8, "GET / HTTP/1.0\r\n", c.received);
    try testing.expectEqual(@as(u32, 517), a.ack);

    try testing.expect(c.writable());
    const out = c.send("HTTP/1.0 200 OK\r\n", 11).?;
    try testing.expectEqual(@as(u32, 7001), out.seq);
}
