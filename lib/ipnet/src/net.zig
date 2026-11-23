//! Ethernet, ARP, IPv4 and UDP — formatting and parsing only.
//!
//! Everything here is a pure function over caller-supplied bytes: no device, no
//! state, no allocator, so all of it is testable on the host and none of it can
//! invent a packet. The address state that used to live here as a `NetStack`
//! belongs to `ipstack.zig`, which is the only thing that knows what this board
//! is actually called on the network.
//!
//! **The defaults that used to be here were fabrications**, and they are worth
//! naming because they read as configuration: `default_mac` was
//! `02:00:00:00:00:01`, an invented locally-administered address for a chip
//! that reports its own; `default_ip` was `192.168.1.100` on a network that
//! does not exist here; and `gateway_mac` defaulted to broadcast, which is not
//! a router. Code that used them produced well-formed frames addressed to
//! nobody. Every address in this file now arrives as an argument.

const std = @import("std");

pub const eth_header_len: usize = 14;
pub const ipv4_header_len: usize = 20;
pub const udp_header_len: usize = 8;

pub const EtherType = struct {
    pub const ipv4: u16 = 0x0800;
    pub const arp: u16 = 0x0806;
};

pub const IpProtocol = struct {
    pub const icmp: u8 = 1;
    pub const tcp: u8 = 6;
    pub const udp: u8 = 17;
};

pub const broadcast_mac: [6]u8 = @splat(0xFF);
pub const broadcast_ip: [4]u8 = @splat(0xFF);
pub const any_ip: [4]u8 = @splat(0);

/// Calculate the 16-bit one's-complement Internet Checksum (RFC 1071).
pub fn computeChecksum(data: []const u8) u16 {
    var sum: u32 = 0;
    var i: usize = 0;

    while (i + 1 < data.len) : (i += 2) {
        sum += std.mem.readInt(u16, data[i..][0..2], .big);
    }

    if (i < data.len) {
        sum += @as(u32, data[i]) << 8;
    }

    while ((sum >> 16) > 0) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return ~@as(u16, @truncate(sum));
}

/// The ones-complement sum carried across several buffers, for a checksum whose
/// input is not contiguous.
///
/// **UDP, TCP and ICMP all need this and only ICMP does not.** A UDP checksum
/// covers a *pseudo-header* — source address, destination address, a zero byte,
/// the protocol, and the UDP length — which exists nowhere in the frame and has
/// to be summed alongside the header and payload that do. Building a scratch
/// copy of all three to hand `computeChecksum` a single slice is the obvious
/// implementation and the wrong one: it needs a buffer the size of the largest
/// datagram, in a package with no allocator.
///
/// **The odd-length rule is why this cannot simply chain calls.** A ones
/// complement sum pads to an even length *once, at the end of the whole
/// message* — RFC 768's "padded with zero octets at the end (if necessary)".
/// Summing each slice separately would pad in the middle of the data wherever
/// a slice happened to end odd, which changes the answer. So this carries the
/// dangling byte across the boundary.
pub fn checksumParts(parts: []const []const u8) u16 {
    var sum: u32 = 0;
    var odd_byte: ?u8 = null;

    for (parts) |part| {
        var i: usize = 0;
        if (odd_byte) |hi| {
            if (part.len == 0) continue;
            sum += (@as(u32, hi) << 8) | part[0];
            odd_byte = null;
            i = 1;
        }
        while (i + 1 < part.len) : (i += 2) {
            sum += std.mem.readInt(u16, part[i..][0..2], .big);
        }
        if (i < part.len) odd_byte = part[i];
    }
    if (odd_byte) |hi| sum += @as(u32, hi) << 8;

    while ((sum >> 16) > 0) sum = (sum & 0xFFFF) + (sum >> 16);
    return ~@as(u16, @truncate(sum));
}

/// The twelve bytes RFC 768 prefixes to a UDP datagram for checksum purposes:
/// source address, destination address, zero, protocol, and the UDP length.
///
/// It is never transmitted. It exists so that a datagram delivered to the wrong
/// host or the wrong protocol fails its checksum instead of being accepted —
/// "protection against misrouted datagrams", in the RFC's words.
pub fn pseudoHeader(src_ip: [4]u8, dest_ip: [4]u8, protocol: u8, length: u16) [12]u8 {
    var ph: [12]u8 = undefined;
    @memcpy(ph[0..4], &src_ip);
    @memcpy(ph[4..8], &dest_ip);
    ph[8] = 0;
    ph[9] = protocol;
    std.mem.writeInt(u16, ph[10..12], length, .big);
    return ph;
}

// ---------------------------------------------------------------------------
// ICMP — RFC 792
//
// Echo only. The board answering a ping is worth more than the eight bytes it
// costs: it is the instrument every rung above this one is debugged with, and
// until it existed a board that answered ARP but not ping looked broken to
// anybody who reached for the obvious tool first.
// ---------------------------------------------------------------------------

pub const IcmpType = struct {
    pub const echo_reply: u8 = 0;
    pub const echo_request: u8 = 8;
};

pub const icmp_header_len: usize = 8;

pub const Echo = struct {
    src_ip: [4]u8,
    src_mac: [6]u8,
    id: u16,
    seq: u16,
    /// Points into the caller's frame — the payload is echoed back unchanged,
    /// so it is never copied.
    payload: []const u8,
};

/// Parse a received frame as an ICMP echo **request** addressed to `our_ip`.
///
/// Rejects a reply as firmly as it rejects a different protocol: answering an
/// echo reply would make two hosts ping each other forever.
pub fn parseEchoRequest(frame: []const u8, our_ip: [4]u8) !Echo {
    if (frame.len < eth_header_len + ipv4_header_len + icmp_header_len) return error.PacketTooShort;
    if (std.mem.readInt(u16, frame[12..14], .big) != EtherType.ipv4) return error.NotIpv4;

    const ip = frame[eth_header_len..];
    const ihl: usize = @as(usize, ip[0] & 0x0F) * 4;
    if ((ip[0] >> 4) != 4 or ihl < ipv4_header_len) return error.NotIpv4;
    if (ip[9] != IpProtocol.icmp) return error.NotIcmp;

    // **Fragments are refused, not reassembled.** A fragmented echo request
    // carries only part of its payload, and replying with that part is worse
    // than not replying: the peer sees a corrupted answer rather than silence.
    const frag = std.mem.readInt(u16, ip[6..8], .big);
    if ((frag & 0x2000) != 0 or (frag & 0x1FFF) != 0) return error.Fragmented;

    const total_len = std.mem.readInt(u16, ip[2..4], .big);
    if (total_len < ihl + icmp_header_len) return error.PacketTooShort;
    if (eth_header_len + total_len > frame.len) return error.PacketTooShort;

    var dest_ip: [4]u8 = undefined;
    @memcpy(&dest_ip, ip[16..20]);
    if (!std.mem.eql(u8, &dest_ip, &our_ip)) return error.NotForUs;

    const icmp = frame[eth_header_len + ihl ..][0 .. total_len - ihl];
    if (icmp[0] != IcmpType.echo_request) return error.NotEchoRequest;
    if (icmp[1] != 0) return error.NotEchoRequest;

    // **The received checksum is checked**, because a reply to a corrupt
    // request is a reply to noise. `computeChecksum` over a message whose own
    // checksum field is still in place yields zero when it is correct.
    if (computeChecksum(icmp) != 0) return error.BadChecksum;

    var src_ip: [4]u8 = undefined;
    @memcpy(&src_ip, ip[12..16]);
    var src_mac: [6]u8 = undefined;
    @memcpy(&src_mac, frame[6..12]);

    return .{
        .src_ip = src_ip,
        .src_mac = src_mac,
        .id = std.mem.readInt(u16, icmp[4..6], .big),
        .seq = std.mem.readInt(u16, icmp[6..8], .big),
        .payload = icmp[icmp_header_len..],
    };
}

/// Build the echo reply to a parsed request.
///
/// RFC 792: "the source and destination addresses are simply reversed, the type
/// code changed to 0, and the checksum recomputed." The identifier, sequence
/// number and payload come back **unchanged** — that is how `ping` matches a
/// reply to what it sent, and altering any of them makes the reply invisible to
/// the tool it exists to satisfy.
pub fn buildEchoReply(e: Echo, our_mac: [6]u8, our_ip: [4]u8, out_buf: []u8) ![]u8 {
    const total = eth_header_len + ipv4_header_len + icmp_header_len + e.payload.len;
    if (out_buf.len < total) return error.BufferTooSmall;

    @memcpy(out_buf[0..6], &e.src_mac);
    @memcpy(out_buf[6..12], &our_mac);
    std.mem.writeInt(u16, out_buf[12..14], EtherType.ipv4, .big);

    const ip = out_buf[eth_header_len..][0..ipv4_header_len];
    ip[0] = 0x45;
    ip[1] = 0x00;
    std.mem.writeInt(u16, ip[2..4], @intCast(ipv4_header_len + icmp_header_len + e.payload.len), .big);
    std.mem.writeInt(u16, ip[4..6], 0, .big);
    std.mem.writeInt(u16, ip[6..8], 0x4000, .big);
    ip[8] = 64;
    ip[9] = IpProtocol.icmp;
    std.mem.writeInt(u16, ip[10..12], 0, .big);
    @memcpy(ip[12..16], &our_ip);
    @memcpy(ip[16..20], &e.src_ip);
    std.mem.writeInt(u16, ip[10..12], computeChecksum(ip), .big);

    const icmp = out_buf[eth_header_len + ipv4_header_len ..][0 .. icmp_header_len + e.payload.len];
    icmp[0] = IcmpType.echo_reply;
    icmp[1] = 0;
    std.mem.writeInt(u16, icmp[2..4], 0, .big);
    std.mem.writeInt(u16, icmp[4..6], e.id, .big);
    std.mem.writeInt(u16, icmp[6..8], e.seq, .big);
    @memcpy(icmp[icmp_header_len..], e.payload);
    // **ICMP's checksum has no pseudo-header** — it covers the ICMP message and
    // nothing else, unlike UDP's and TCP's. Adding one is a plausible mistake
    // that produces a reply `ping` silently discards.
    std.mem.writeInt(u16, icmp[2..4], computeChecksum(icmp), .big);

    return out_buf[0..total];
}

// ---------------------------------------------------------------------------
// ARP — RFC 826
//
// The smallest thing that proves the data path in both directions: a request is
// 42 bytes and needs no IP stack above it, and the reply comes back addressed
// to this station rather than broadcast, so it cannot be one of the frames the
// network was already sending.
// ---------------------------------------------------------------------------

pub const arp_frame_len: usize = 42;

pub const ArpOp = struct {
    pub const request: u16 = 1;
    pub const reply: u16 = 2;
};

pub const Arp = struct {
    op: u16,
    sender_mac: [6]u8,
    sender_ip: [4]u8,
    target_mac: [6]u8,
    target_ip: [4]u8,
};

/// Build an ARP request — "who has `target_ip`, tell `sender_ip`".
///
/// Broadcast at the Ethernet layer, because the answer is what identifies the
/// host that has the address.
pub fn buildArpRequest(
    sender_mac: [6]u8,
    sender_ip: [4]u8,
    target_ip: [4]u8,
    out_buf: []u8,
) ![]u8 {
    return buildArp(.{
        .op = ArpOp.request,
        .sender_mac = sender_mac,
        .sender_ip = sender_ip,
        // What is being asked for, so it goes out zero.
        .target_mac = @splat(0),
        .target_ip = target_ip,
    }, broadcast_mac, out_buf);
}

/// Build the ARP reply to `request`, claiming `our_ip` for `our_mac`.
///
/// **Answering matters more than it looks.** A host that asks and never answers
/// works only until the peer's cache expires — and then the peer asks who has
/// our address, hears nothing, and stops sending. Nothing errors; the traffic
/// just stops.
pub fn buildArpReply(
    request: Arp,
    our_mac: [6]u8,
    our_ip: [4]u8,
    out_buf: []u8,
) ![]u8 {
    return buildArp(.{
        .op = ArpOp.reply,
        .sender_mac = our_mac,
        .sender_ip = our_ip,
        .target_mac = request.sender_mac,
        .target_ip = request.sender_ip,
    }, request.sender_mac, out_buf);
}

fn buildArp(a: Arp, dest_mac: [6]u8, out_buf: []u8) ![]u8 {
    if (out_buf.len < arp_frame_len) return error.BufferTooSmall;
    const f = out_buf[0..arp_frame_len];

    @memcpy(f[0..6], &dest_mac);
    @memcpy(f[6..12], &a.sender_mac);
    std.mem.writeInt(u16, f[12..14], EtherType.arp, .big);

    std.mem.writeInt(u16, f[14..16], 1, .big); // hardware type: Ethernet
    std.mem.writeInt(u16, f[16..18], EtherType.ipv4, .big); // protocol type
    f[18] = 6; // hardware address length
    f[19] = 4; // protocol address length
    std.mem.writeInt(u16, f[20..22], a.op, .big);

    @memcpy(f[22..28], &a.sender_mac);
    @memcpy(f[28..32], &a.sender_ip);
    @memcpy(f[32..38], &a.target_mac);
    @memcpy(f[38..42], &a.target_ip);

    return f;
}

/// Read any ARP frame — request or reply. The caller decides which it wanted;
/// `parseArpReply` is the narrower form.
pub fn parseArp(frame: []const u8) !Arp {
    if (frame.len < arp_frame_len) return error.PacketTooShort;
    if (std.mem.readInt(u16, frame[12..14], .big) != EtherType.arp) return error.NotArp;
    if (std.mem.readInt(u16, frame[14..16], .big) != 1) return error.NotEthernetArp;
    if (std.mem.readInt(u16, frame[16..18], .big) != EtherType.ipv4) return error.NotIpv4Arp;

    var a: Arp = undefined;
    a.op = std.mem.readInt(u16, frame[20..22], .big);
    @memcpy(&a.sender_mac, frame[22..28]);
    @memcpy(&a.sender_ip, frame[28..32]);
    @memcpy(&a.target_mac, frame[32..38]);
    @memcpy(&a.target_ip, frame[38..42]);
    return a;
}

/// Read an ARP reply, rejecting anything that is not one.
///
/// **A request is not a reply**, including our own coming back as a broadcast
/// echo — which does happen, and reads as a successful answer to a parser that
/// checks only the ethertype.
pub fn parseArpReply(frame: []const u8) !Arp {
    const a = try parseArp(frame);
    if (a.op != ArpOp.reply) return error.NotArpReply;
    return a;
}

// ---------------------------------------------------------------------------
// IPv4 / UDP
// ---------------------------------------------------------------------------

/// Everything a UDP frame needs, named rather than defaulted.
///
/// DHCP is the reason all six are parameters: its first exchange goes out from
/// `0.0.0.0` to `255.255.255.255` at the broadcast MAC, which no set of
/// sensible defaults would produce.
pub const UdpFrame = struct {
    dest_mac: [6]u8,
    src_mac: [6]u8,
    src_ip: [4]u8,
    dest_ip: [4]u8,
    src_port: u16,
    dest_port: u16,
    payload: []const u8,
};

pub fn buildUdpFrame(f: UdpFrame, out_buf: []u8) ![]u8 {
    const total = eth_header_len + ipv4_header_len + udp_header_len + f.payload.len;
    if (out_buf.len < total) return error.BufferTooSmall;

    @memcpy(out_buf[0..6], &f.dest_mac);
    @memcpy(out_buf[6..12], &f.src_mac);
    std.mem.writeInt(u16, out_buf[12..14], EtherType.ipv4, .big);

    const ip = out_buf[eth_header_len..][0..ipv4_header_len];
    ip[0] = 0x45; // version 4, IHL 5 words
    ip[1] = 0x00; // DSCP / ECN
    std.mem.writeInt(u16, ip[2..4], @intCast(ipv4_header_len + udp_header_len + f.payload.len), .big);
    std.mem.writeInt(u16, ip[4..6], 0, .big); // identification
    std.mem.writeInt(u16, ip[6..8], 0x4000, .big); // don't fragment
    ip[8] = 64; // TTL
    ip[9] = IpProtocol.udp;
    std.mem.writeInt(u16, ip[10..12], 0, .big); // checksum, computed below
    @memcpy(ip[12..16], &f.src_ip);
    @memcpy(ip[16..20], &f.dest_ip);
    std.mem.writeInt(u16, ip[10..12], computeChecksum(ip), .big);

    const udp = out_buf[eth_header_len + ipv4_header_len ..][0..udp_header_len];
    std.mem.writeInt(u16, udp[0..2], f.src_port, .big);
    std.mem.writeInt(u16, udp[2..4], f.dest_port, .big);
    const udp_len: u16 = @intCast(udp_header_len + f.payload.len);
    std.mem.writeInt(u16, udp[4..6], udp_len, .big);
    std.mem.writeInt(u16, udp[6..8], 0, .big);

    @memcpy(out_buf[eth_header_len + ipv4_header_len + udp_header_len ..][0..f.payload.len], f.payload);

    // **Computed, as of rung N4.** It was zero — legal in IPv4, meaning "not
    // computed", and accepted by every server this stack had met. A peer
    // entitled to demand one would have been within its rights, and the same
    // pseudo-header arithmetic is what TCP needs, so writing it twice is how
    // the two drift apart.
    //
    // The checksum covers the pseudo-header, the UDP header with its own
    // checksum field zeroed, and the payload — as one ones-complement sum,
    // padded at the very end, which is why it goes through `checksumParts`
    // rather than three calls.
    const ph = pseudoHeader(f.src_ip, f.dest_ip, IpProtocol.udp, udp_len);
    const sum = checksumParts(&.{ &ph, udp, f.payload });
    // **Zero on the wire means "no checksum here", so a computed zero has to be
    // sent as all ones** — RFC 768. The two are equal in ones-complement
    // arithmetic and are not equal to a receiver deciding whether to check.
    std.mem.writeInt(u16, udp[6..8], if (sum == 0) 0xFFFF else sum, .big);

    return out_buf[0..total];
}

pub const UdpDatagram = struct {
    src_mac: [6]u8,
    src_ip: [4]u8,
    dest_ip: [4]u8,
    src_port: u16,
    dest_port: u16,
    payload: []const u8,
};

/// Parse a received frame as IPv4/UDP.
///
/// **Bounded by the IP header's own total-length field**, not by the frame
/// length: Ethernet pads short frames to 60 bytes, so a 20-byte UDP payload
/// arrives with trailing bytes that belong to nobody. A parser that measured
/// the payload by the frame would hand those to the caller — and for a protocol
/// like DHCP, whose parser walks options until it finds `end`, that is a walk
/// through padding.
pub fn parseUdpFrame(frame: []const u8) !UdpDatagram {
    if (frame.len < eth_header_len + ipv4_header_len + udp_header_len) return error.PacketTooShort;
    if (std.mem.readInt(u16, frame[12..14], .big) != EtherType.ipv4) return error.NotIpv4;

    const ip = frame[eth_header_len..];
    if ((ip[0] >> 4) != 4) return error.NotIpv4;
    const ihl: usize = @as(usize, ip[0] & 0x0F) * 4;
    if (ihl < ipv4_header_len) return error.BadIpHeader;
    if (ip[9] != IpProtocol.udp) return error.NotUdp;

    const total_len: usize = std.mem.readInt(u16, ip[2..4], .big);
    if (total_len < ihl + udp_header_len) return error.BadIpHeader;
    if (eth_header_len + total_len > frame.len) return error.PacketTooShort;

    // A fragment carries only part of the datagram; the UDP header is only in
    // the first one, and reassembly is not something this driver does.
    const frag = std.mem.readInt(u16, ip[6..8], .big);
    if ((frag & 0x1FFF) != 0) return error.IpFragment;

    const udp = ip[ihl..];
    const udp_len: usize = std.mem.readInt(u16, udp[4..6], .big);
    if (udp_len < udp_header_len or ihl + udp_len > total_len) return error.BadUdpLength;

    var d: UdpDatagram = undefined;
    @memcpy(&d.src_mac, frame[6..12]);
    @memcpy(&d.src_ip, ip[12..16]);
    @memcpy(&d.dest_ip, ip[16..20]);
    d.src_port = std.mem.readInt(u16, udp[0..2], .big);
    d.dest_port = std.mem.readInt(u16, udp[2..4], .big);
    d.payload = udp[udp_header_len..udp_len];
    return d;
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const test_mac = [6]u8{ 0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F };
const gw_mac = [6]u8{ 0xE4, 0x5E, 0x1B, 0xFF, 0xEE, 0xD1 };

// ---------------------------------------------------------------------------
// Rung N4 — ICMP echo, and the UDP checksum
// ---------------------------------------------------------------------------

test "a UDP datagram checksums to zero at the receiver" {
    var buf: [128]u8 = undefined;
    const payload = "hello, checksum";
    const f = UdpFrame{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 209 },
        .dest_ip = .{ 129, 6, 15, 28 },
        .src_port = 50123,
        .dest_port = 123,
        .payload = payload,
    };
    const frame = try buildUdpFrame(f, &buf);

    // **The receiver's test, not the sender's.** Summing a message that still
    // carries its own checksum gives zero when that checksum is right — which
    // is what a peer does, and is a different assertion from recomputing the
    // value the same way the builder did and finding it matches itself.
    const udp = frame[eth_header_len + ipv4_header_len ..];
    const ph = pseudoHeader(f.src_ip, f.dest_ip, IpProtocol.udp, @intCast(udp.len));
    try testing.expectEqual(@as(u16, 0), checksumParts(&.{ &ph, udp }));

    // And it is not the zero that means "I did not bother".
    try testing.expect(std.mem.readInt(u16, frame[eth_header_len + ipv4_header_len + 6 ..][0..2], .big) != 0);
}

test "an odd-length payload pads once, at the end" {
    // **The trap `checksumParts` exists for.** A ones-complement sum pads to an
    // even length once for the whole message; padding at every slice boundary
    // instead gives a different answer, and an odd payload is what exposes it —
    // the UDP header is even, so the dangling byte lands at the very end.
    var buf: [128]u8 = undefined;
    const f = UdpFrame{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 209 },
        .dest_ip = .{ 8, 8, 8, 8 },
        .src_port = 1234,
        .dest_port = 53,
        .payload = "odd",
    };
    const frame = try buildUdpFrame(f, &buf);
    const udp = frame[eth_header_len + ipv4_header_len ..];
    const ph = pseudoHeader(f.src_ip, f.dest_ip, IpProtocol.udp, @intCast(udp.len));
    try testing.expectEqual(@as(u16, 0), checksumParts(&.{ &ph, udp }));

    // Split across the boundary the same bytes must give the same sum. A
    // chained implementation fails here and passes the test above.
    try testing.expectEqual(
        checksumParts(&.{ &ph, udp }),
        checksumParts(&.{ &ph, udp[0..7], udp[7..] }),
    );
}

test "the checksum covers the addresses, which is the point of the pseudo-header" {
    var buf: [128]u8 = undefined;
    var f = UdpFrame{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 209 },
        .dest_ip = .{ 8, 8, 8, 8 },
        .src_port = 1234,
        .dest_port = 53,
        .payload = "x",
    };
    const a = try buildUdpFrame(f, &buf);
    const csum_a = std.mem.readInt(u16, a[eth_header_len + ipv4_header_len + 6 ..][0..2], .big);

    var buf2: [128]u8 = undefined;
    f.dest_ip = .{ 8, 8, 4, 4 };
    const b = try buildUdpFrame(f, &buf2);
    const csum_b = std.mem.readInt(u16, b[eth_header_len + ipv4_header_len + 6 ..][0..2], .big);

    // **"Protection against misrouted datagrams"** — RFC 768's own words. A
    // checksum over the UDP header and payload alone would be identical here,
    // and a datagram delivered to the wrong host would pass.
    try testing.expect(csum_a != csum_b);
}

/// An echo request as `ping` sends one, with its checksum already correct.
///
/// **`pub` and named `test...` because `ipstack.zig`'s tests need it too**, and
/// a second copy there would be a second chance to build a subtly different
/// request — which is how a parser comes to be tested only against its own
/// author's idea of the format.
pub fn testEchoRequest(id: u16, seq: u16, payload: []const u8, our_ip: [4]u8, out: []u8) []u8 {
    const total = eth_header_len + ipv4_header_len + icmp_header_len + payload.len;
    @memcpy(out[0..6], &test_mac);
    @memcpy(out[6..12], &gw_mac);
    std.mem.writeInt(u16, out[12..14], EtherType.ipv4, .big);
    const ip = out[eth_header_len..][0..ipv4_header_len];
    ip[0] = 0x45;
    ip[1] = 0;
    std.mem.writeInt(u16, ip[2..4], @intCast(ipv4_header_len + icmp_header_len + payload.len), .big);
    std.mem.writeInt(u16, ip[4..6], 0, .big);
    std.mem.writeInt(u16, ip[6..8], 0x4000, .big);
    ip[8] = 64;
    ip[9] = IpProtocol.icmp;
    std.mem.writeInt(u16, ip[10..12], 0, .big);
    @memcpy(ip[12..16], &[4]u8{ 192, 168, 86, 1 });
    @memcpy(ip[16..20], &our_ip);
    std.mem.writeInt(u16, ip[10..12], computeChecksum(ip), .big);
    const icmp = out[eth_header_len + ipv4_header_len ..][0 .. icmp_header_len + payload.len];
    icmp[0] = IcmpType.echo_request;
    icmp[1] = 0;
    std.mem.writeInt(u16, icmp[2..4], 0, .big);
    std.mem.writeInt(u16, icmp[4..6], id, .big);
    std.mem.writeInt(u16, icmp[6..8], seq, .big);
    @memcpy(icmp[icmp_header_len..], payload);
    std.mem.writeInt(u16, icmp[2..4], computeChecksum(icmp), .big);
    return out[0..total];
}

test "an echo request is answered with its own id, sequence and payload" {
    const our_ip = [4]u8{ 192, 168, 86, 209 };
    var req_buf: [128]u8 = undefined;
    const req = testEchoRequest(0xABCD, 7, "abcdefghijklmnop", our_ip, &req_buf);

    const e = try parseEchoRequest(req, our_ip);
    try testing.expectEqual(@as(u16, 0xABCD), e.id);
    try testing.expectEqual(@as(u16, 7), e.seq);
    try testing.expectEqualSlices(u8, "abcdefghijklmnop", e.payload);

    var rep_buf: [128]u8 = undefined;
    const rep = try buildEchoReply(e, test_mac, our_ip, &rep_buf);

    const icmp = rep[eth_header_len + ipv4_header_len ..];
    try testing.expectEqual(IcmpType.echo_reply, icmp[0]);
    try testing.expectEqual(@as(u8, 0), icmp[1]);
    // **Unchanged, all three.** `ping` matches a reply by id and sequence and
    // compares the payload; altering any of them makes the reply invisible to
    // the tool this rung exists to satisfy.
    try testing.expectEqual(@as(u16, 0xABCD), std.mem.readInt(u16, icmp[4..6], .big));
    try testing.expectEqual(@as(u16, 7), std.mem.readInt(u16, icmp[6..8], .big));
    try testing.expectEqualSlices(u8, "abcdefghijklmnop", icmp[icmp_header_len..]);

    // The reply's own checksum, checked the way the peer checks it. **ICMP has
    // no pseudo-header** — adding one produces a reply `ping` silently drops.
    try testing.expectEqual(@as(u16, 0), computeChecksum(icmp));

    // Addresses reversed, RFC 792.
    try testing.expectEqualSlices(u8, &our_ip, rep[eth_header_len + 12 ..][0..4]);
    try testing.expectEqualSlices(u8, &[4]u8{ 192, 168, 86, 1 }, rep[eth_header_len + 16 ..][0..4]);
    try testing.expectEqualSlices(u8, &gw_mac, rep[0..6]);
}

test "an echo request this stack must not answer" {
    const our_ip = [4]u8{ 192, 168, 86, 209 };
    var buf: [128]u8 = undefined;

    // Addressed to somebody else.
    _ = testEchoRequest(1, 1, "x", .{ 192, 168, 86, 99 }, &buf);
    try testing.expectError(error.NotForUs, parseEchoRequest(buf[0..60], our_ip));

    // An echo *reply*. Answering one makes two hosts ping each other forever.
    const req = testEchoRequest(1, 1, "x", our_ip, &buf);
    const icmp_at = eth_header_len + ipv4_header_len;
    buf[icmp_at] = IcmpType.echo_reply;
    std.mem.writeInt(u16, buf[icmp_at + 2 ..][0..2], 0, .big);
    std.mem.writeInt(u16, buf[icmp_at + 2 ..][0..2], computeChecksum(req[icmp_at..]), .big);
    try testing.expectError(error.NotEchoRequest, parseEchoRequest(req, our_ip));

    // A corrupted one. Replying to noise is replying to noise.
    const req2 = testEchoRequest(1, 1, "x", our_ip, &buf);
    buf[icmp_at + icmp_header_len] ^= 0xFF;
    try testing.expectError(error.BadChecksum, parseEchoRequest(req2, our_ip));

    // A fragment carries only part of its payload, and echoing that part back
    // is worse than silence — the peer sees a corrupt answer rather than none.
    const req3 = testEchoRequest(1, 1, "x", our_ip, &buf);
    std.mem.writeInt(u16, buf[eth_header_len + 6 ..][0..2], 0x2000, .big);
    try testing.expectError(error.Fragmented, parseEchoRequest(req3, our_ip));
}

test "Internet Checksum calculation (RFC 1071)" {
    // Standard test header: 4500 003c 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c
    const ip_header = [_]u8{
        0x45, 0x00, 0x00, 0x3c,
        0x1c, 0x46, 0x40, 0x00,
        0x40, 0x06, 0x00, 0x00,
        0xac, 0x10, 0x0a, 0x63,
        0xac, 0x10, 0x0a, 0x0c,
    };
    try testing.expectEqual(@as(u16, 0xb1e6), computeChecksum(&ip_header));
}

test "an ARP request is 42 bytes, broadcast, with the target hardware address blank" {
    var buf: [64]u8 = @splat(0xAA);
    const f = try buildArpRequest(test_mac, .{ 192, 168, 86, 200 }, .{ 192, 168, 86, 1 }, &buf);

    try testing.expectEqual(arp_frame_len, f.len);
    try testing.expectEqualSlices(u8, &broadcast_mac, f[0..6]);
    try testing.expectEqualSlices(u8, &test_mac, f[6..12]);
    try testing.expectEqualSlices(u8, &.{ 0x08, 0x06 }, f[12..14]);
    // htype 1, ptype 0x0800, hlen 6, plen 4, op 1.
    try testing.expectEqualSlices(u8, &.{ 0, 1, 0x08, 0x00, 6, 4, 0, 1 }, f[14..22]);
    try testing.expectEqualSlices(u8, &test_mac, f[22..28]);
    try testing.expectEqualSlices(u8, &.{ 192, 168, 86, 200 }, f[28..32]);
    // The answer is what fills this in; sending it non-zero asks a different
    // question than "who has".
    try testing.expectEqualSlices(u8, &.{ 0, 0, 0, 0, 0, 0 }, f[32..38]);
    try testing.expectEqualSlices(u8, &.{ 192, 168, 86, 1 }, f[38..42]);
}

test "our own broadcast request is not mistaken for a reply" {
    var buf: [64]u8 = @splat(0);
    const req = try buildArpRequest(test_mac, .{ 192, 168, 86, 200 }, .{ 192, 168, 86, 1 }, &buf);

    // Right ethertype, wrong opcode. A parser that checked only the ethertype
    // would report the board's own request as the gateway answering.
    try testing.expectError(error.NotArpReply, parseArpReply(req));
}

test "an ARP reply is unicast back to whoever asked, with the addresses swapped" {
    var req_buf: [64]u8 = @splat(0);
    const req = try buildArpRequest(gw_mac, .{ 192, 168, 86, 1 }, .{ 192, 168, 86, 200 }, &req_buf);
    const asked = try parseArp(req);

    var buf: [64]u8 = @splat(0);
    const f = try buildArpReply(asked, test_mac, .{ 192, 168, 86, 200 }, &buf);

    // **Unicast, not broadcast.** A reply sent to ff:ff:ff:ff:ff:ff is still
    // answered correctly by most stacks, which is exactly why getting it wrong
    // survives testing and shows up later as broadcast noise.
    try testing.expectEqualSlices(u8, &gw_mac, f[0..6]);
    try testing.expectEqualSlices(u8, &test_mac, f[6..12]);

    const r = try parseArpReply(f);
    try testing.expectEqual(ArpOp.reply, r.op);
    try testing.expectEqual(test_mac, r.sender_mac);
    try testing.expectEqual([4]u8{ 192, 168, 86, 200 }, r.sender_ip);
    try testing.expectEqual(gw_mac, r.target_mac);
    try testing.expectEqual([4]u8{ 192, 168, 86, 1 }, r.target_ip);
}

test "a UDP frame carries the checksum over the IP header only" {
    var buf: [128]u8 = @splat(0);
    const payload = "Hello NTP Network!";
    const f = try buildUdpFrame(.{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 200 },
        .dest_ip = .{ 129, 6, 15, 28 },
        .src_port = 12345,
        .dest_port = 123,
        .payload = payload,
    }, &buf);

    try testing.expectEqual(eth_header_len + ipv4_header_len + udp_header_len + payload.len, f.len);
    try testing.expectEqualSlices(u8, &gw_mac, f[0..6]);
    try testing.expectEqual(@as(u16, 0x0800), std.mem.readInt(u16, f[12..14], .big));

    // A correct IPv4 header checksums to zero over itself, which is a stronger
    // check than recomputing the same expression the code did.
    try testing.expectEqual(@as(u16, 0), computeChecksum(f[eth_header_len..][0..ipv4_header_len]));

    const d = try parseUdpFrame(f);
    try testing.expectEqual([4]u8{ 192, 168, 86, 200 }, d.src_ip);
    try testing.expectEqual([4]u8{ 129, 6, 15, 28 }, d.dest_ip);
    try testing.expectEqual(@as(u16, 12345), d.src_port);
    try testing.expectEqual(@as(u16, 123), d.dest_port);
    try testing.expectEqualStrings(payload, d.payload);
}

test "an Ethernet-padded frame yields the payload the sender sent, not the padding" {
    // Ethernet's 60-byte minimum pads anything shorter. A four-byte payload
    // arrives in a frame with 14 bytes of filler after it, and a parser that
    // measured by the frame would return 18 bytes.
    var buf: [64]u8 = @splat(0xEE);
    const sent = [_]u8{ 1, 2, 3, 4 };
    const f = try buildUdpFrame(.{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 200 },
        .dest_ip = .{ 192, 168, 86, 1 },
        .src_port = 68,
        .dest_port = 67,
        .payload = &sent,
    }, &buf);
    try testing.expectEqual(@as(usize, 46), f.len);

    const padded = buf[0..60];
    const d = try parseUdpFrame(padded);
    try testing.expectEqual(@as(usize, 4), d.payload.len);
    try testing.expectEqualSlices(u8, &sent, d.payload);
}

test "a frame that is not IPv4 UDP is refused rather than decoded" {
    var buf: [64]u8 = @splat(0);
    const arp = try buildArpRequest(test_mac, .{ 192, 168, 86, 200 }, .{ 192, 168, 86, 1 }, &buf);
    try testing.expectError(error.NotIpv4, parseUdpFrame(arp));

    var udp_buf: [64]u8 = @splat(0);
    const f = try buildUdpFrame(.{
        .dest_mac = gw_mac,
        .src_mac = test_mac,
        .src_ip = .{ 192, 168, 86, 200 },
        .dest_ip = .{ 192, 168, 86, 1 },
        .src_port = 68,
        .dest_port = 67,
        .payload = &.{ 1, 2, 3, 4 },
    }, &udp_buf);

    // ICMP, at the same length and shape.
    var icmp: [64]u8 = @splat(0);
    @memcpy(icmp[0..f.len], f);
    icmp[eth_header_len + 9] = IpProtocol.icmp;
    try testing.expectError(error.NotUdp, parseUdpFrame(icmp[0..f.len]));

    // A truncated frame, which is what a torn read looks like.
    try testing.expectError(error.PacketTooShort, parseUdpFrame(f[0 .. f.len - 8]));
}
