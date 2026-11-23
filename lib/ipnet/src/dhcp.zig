//! DHCP client messages — RFC 2131, formatting and parsing only.
//!
//! Four messages get this board an address: the client broadcasts a DISCOVER,
//! a server broadcasts an OFFER, the client broadcasts a REQUEST naming which
//! offer it took, and the server sends ACK (or NAK). Everything here is a pure
//! function; `ipstack.zig` runs the exchange.
//!
//! ## What is easy to get wrong here
//!
//! **The transaction id is the only thing tying a reply to our request.** Other
//! clients' exchanges are broadcast across the same network and arrive here
//! looking exactly like ours. A reply accepted on shape alone configures this
//! board with someone else's address — and it will work for a while, which is
//! the worst kind of wrong.
//!
//! **A DHCP message is padded to 300 bytes.** The floor is BOOTP's, not
//! DHCP's, and the options that matter are long finished by then. Servers and
//! relay agents that drop short messages exist, and a client that never pads
//! works on the network it was written on.
//!
//! **The broadcast flag exists for exactly this state.** The client has no
//! address, so a server replying by unicast would have to ARP for an address
//! nobody owns yet. Bit 15 of `flags` asks it to broadcast instead. Servers are
//! free to ignore it, so the parser accepts a reply that arrives either way —
//! it filters on the transaction id and the hardware address, never on where
//! the frame was addressed.

const std = @import("std");

pub const client_port: u16 = 68;
pub const server_port: u16 = 67;

/// Fixed part of a BOOTP message, up to and including the magic cookie.
pub const header_len: usize = 240;

/// **A message is padded to this.** See the header comment: the number is
/// BOOTP's minimum, and nothing in the options needs it.
pub const min_message_len: usize = 300;

/// The largest reply this client will parse. RFC 2131's floor for what a client
/// must accept.
pub const max_message_len: usize = 576;

const magic_cookie = [4]u8{ 0x63, 0x82, 0x53, 0x63 };

const op_request: u8 = 1;
const op_reply: u8 = 2;
const htype_ethernet: u8 = 1;
const hlen_ethernet: u8 = 6;

/// `flags` bit 15 — "reply by broadcast, I have no address yet".
const flag_broadcast: u16 = 0x8000;

pub const MessageType = enum(u8) {
    discover = 1,
    offer = 2,
    request = 3,
    decline = 4,
    ack = 5,
    nak = 6,
    release = 7,
    _,
};

const Opt = struct {
    const pad: u8 = 0;
    const subnet_mask: u8 = 1;
    const router: u8 = 3;
    const dns: u8 = 6;
    const hostname: u8 = 12;
    const requested_ip: u8 = 50;
    const lease_time: u8 = 51;
    const message_type: u8 = 53;
    const server_id: u8 = 54;
    const param_request: u8 = 55;
    const client_id: u8 = 61;
    const end: u8 = 255;
};

/// What a server told us. Every field except `ip` may be absent from a reply,
/// and absent is zero rather than a guess — a fabricated default netmask is how
/// a driver ends up unable to explain its own routing.
pub const Lease = struct {
    ip: [4]u8 = @splat(0),
    mask: [4]u8 = @splat(0),
    gateway: [4]u8 = @splat(0),
    dns: [4]u8 = @splat(0),
    server_id: [4]u8 = @splat(0),
    lease_secs: u32 = 0,
};

pub const Reply = struct {
    kind: MessageType,
    lease: Lease,
};

/// Writes options into a buffer, refusing to run past the end.
const Writer = struct {
    buf: []u8,
    at: usize,

    fn byte(self: *Writer, v: u8) !void {
        if (self.at >= self.buf.len) return error.BufferTooSmall;
        self.buf[self.at] = v;
        self.at += 1;
    }

    fn option(self: *Writer, code: u8, data: []const u8) !void {
        if (data.len > 255) return error.OptionTooLong;
        try self.byte(code);
        try self.byte(@intCast(data.len));
        for (data) |b| try self.byte(b);
    }
};

/// The options every request carries: which parameters we want back, who we
/// are, and what to call us.
///
/// **The hostname is not decoration.** It is what the router's client table
/// displays, and a name this driver chose is a name the SDK's earlier lease
/// cannot account for. Before there was a DHCP client the router still listed
/// the board as `PicoW`, remembered from the SDK build's lease on the same MAC
/// — a name that looked like evidence and was not.
fn writeCommon(w: *Writer, mac: [6]u8, hostname: []const u8) !void {
    try w.option(Opt.param_request, &.{
        Opt.subnet_mask,
        Opt.router,
        Opt.dns,
        Opt.lease_time,
        Opt.server_id,
    });

    // Client identifier: hardware type, then the address.
    var client_id: [7]u8 = undefined;
    client_id[0] = htype_ethernet;
    @memcpy(client_id[1..7], &mac);
    try w.option(Opt.client_id, &client_id);

    if (hostname.len > 0) try w.option(Opt.hostname, hostname);
    try w.byte(Opt.end);

    // Pad to the BOOTP floor. The bytes are zero, which is option 0 — "pad" —
    // so a server parsing past `end` still sees something legal.
    while (w.at < min_message_len) try w.byte(Opt.pad);
}

fn writeHeader(buf: []u8, xid: u32, mac: [6]u8, ciaddr: [4]u8) void {
    @memset(buf[0..header_len], 0);
    buf[0] = op_request;
    buf[1] = htype_ethernet;
    buf[2] = hlen_ethernet;
    buf[3] = 0; // hops
    std.mem.writeInt(u32, buf[4..8], xid, .big);
    std.mem.writeInt(u16, buf[8..10], 0, .big); // secs
    std.mem.writeInt(u16, buf[10..12], flag_broadcast, .big);
    @memcpy(buf[12..16], &ciaddr);
    // yiaddr, siaddr, giaddr stay zero: the server fills the first and the
    // other two are for relays, which a station on its own subnet has none of.
    @memcpy(buf[28..34], &mac);
    @memcpy(buf[236..240], &magic_cookie);
}

/// DISCOVER — "is there a server, and what would you give me?"
pub fn buildDiscover(mac: [6]u8, xid: u32, hostname: []const u8, out: []u8) ![]u8 {
    if (out.len < min_message_len) return error.BufferTooSmall;
    writeHeader(out, xid, mac, @splat(0));

    var w = Writer{ .buf = out, .at = header_len };
    try w.option(Opt.message_type, &.{@backingInt(MessageType.discover)});
    try writeCommon(&w, mac, hostname);
    return out[0..w.at];
}

/// REQUEST — "I accept that offer, from that server."
///
/// **Both options are load-bearing and neither is the header.** `requested_ip`
/// goes in option 50 rather than `ciaddr`, which stays zero until the address
/// is actually held; `server_id` is what tells the other servers on the network
/// that their offers were declined. A REQUEST missing them is a second
/// DISCOVER as far as the server is concerned.
pub fn buildRequest(
    mac: [6]u8,
    xid: u32,
    requested_ip: [4]u8,
    server_id: [4]u8,
    hostname: []const u8,
    out: []u8,
) ![]u8 {
    if (out.len < min_message_len) return error.BufferTooSmall;
    writeHeader(out, xid, mac, @splat(0));

    var w = Writer{ .buf = out, .at = header_len };
    try w.option(Opt.message_type, &.{@backingInt(MessageType.request)});
    try w.option(Opt.requested_ip, &requested_ip);
    try w.option(Opt.server_id, &server_id);
    try writeCommon(&w, mac, hostname);
    return out[0..w.at];
}

/// REQUEST again, to keep an address we already hold — RENEWING at T1 and
/// REBINDING at T2.
///
/// **This is not the same message as `buildRequest`, and sending that one
/// instead mostly works**, which is what makes it a trap rather than an
/// omission. Three differences, all of them from RFC 2131 §4.3.2:
///
///  * `ciaddr` carries the address, because we hold it now. In a SELECTING
///    request it is zero and the address goes in option 50.
///  * **Option 50 is omitted.** A server that sees both a `ciaddr` and a
///    requested-address option is entitled to treat the message as malformed.
///  * **Option 54 is omitted.** Naming a server is how a SELECTING client
///    declines the other offers; a renewing client is not choosing, it is
///    asking the server it already has.
///
/// The difference between renewing and rebinding is not in the message at all
/// — it is where it is sent. `ipstack` unicasts this to the server at T1 and
/// broadcasts the identical bytes at T2.
pub fn buildRenew(mac: [6]u8, xid: u32, ciaddr: [4]u8, hostname: []const u8, out: []u8) ![]u8 {
    if (out.len < min_message_len) return error.BufferTooSmall;
    writeHeader(out, xid, mac, ciaddr);

    var w = Writer{ .buf = out, .at = header_len };
    try w.option(Opt.message_type, &.{@backingInt(MessageType.request)});
    try writeCommon(&w, mac, hostname);
    return out[0..w.at];
}

/// Read a server's reply, or say why it is not one of ours.
///
/// Checks, in order: it is a reply, for Ethernet, for **our** transaction, with
/// **our** hardware address, carrying the cookie and a message type. Dropping
/// any one of those admits another client's exchange.
pub fn parseReply(payload: []const u8, mac: [6]u8, xid: u32) !Reply {
    if (payload.len < header_len) return error.MessageTooShort;
    if (payload[0] != op_reply) return error.NotAReply;
    if (payload[1] != htype_ethernet or payload[2] != hlen_ethernet) return error.NotEthernet;
    if (std.mem.readInt(u32, payload[4..8], .big) != xid) return error.WrongTransaction;
    if (!std.mem.eql(u8, payload[28..34], &mac)) return error.NotOurHardwareAddress;
    if (!std.mem.eql(u8, payload[236..240], &magic_cookie)) return error.NoMagicCookie;

    var lease = Lease{};
    @memcpy(&lease.ip, payload[16..20]); // yiaddr

    var kind: ?MessageType = null;
    var at: usize = header_len;
    while (at < payload.len) {
        const code = payload[at];
        if (code == Opt.end) break;
        if (code == Opt.pad) {
            at += 1;
            continue;
        }
        // Length byte, then the body — both have to be inside the message, and
        // a server that truncates one is not a reason to read past the end.
        if (at + 2 > payload.len) return error.TruncatedOption;
        const len: usize = payload[at + 1];
        const body_at = at + 2;
        if (body_at + len > payload.len) return error.TruncatedOption;
        const body = payload[body_at..][0..len];

        switch (code) {
            Opt.message_type => if (len >= 1) {
                kind = @fromBackingInt(@intCast(body[0]));
            },
            Opt.subnet_mask => if (len >= 4) @memcpy(&lease.mask, body[0..4]),
            // A router option may list several; the first is the default route.
            Opt.router => if (len >= 4) @memcpy(&lease.gateway, body[0..4]),
            Opt.dns => if (len >= 4) @memcpy(&lease.dns, body[0..4]),
            Opt.server_id => if (len >= 4) @memcpy(&lease.server_id, body[0..4]),
            Opt.lease_time => if (len >= 4) {
                lease.lease_secs = std.mem.readInt(u32, body[0..4], .big);
            },
            else => {},
        }
        at = body_at + len;
    }

    return .{ .kind = kind orelse return error.NoMessageType, .lease = lease };
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const test_mac = [6]u8{ 0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F };
const test_xid: u32 = 0xDEAD_BEEF;

/// A server's reply, assembled the way a server assembles one — so the parser
/// is tested against the shape it will actually meet rather than against the
/// builder's own output.
fn serverReply(
    out: []u8,
    kind: MessageType,
    xid: u32,
    mac: [6]u8,
    yiaddr: [4]u8,
    options: []const u8,
) []u8 {
    @memset(out, 0);
    out[0] = op_reply;
    out[1] = htype_ethernet;
    out[2] = hlen_ethernet;
    std.mem.writeInt(u32, out[4..8], xid, .big);
    @memcpy(out[16..20], &yiaddr);
    @memcpy(out[28..34], &mac);
    @memcpy(out[236..240], &magic_cookie);
    out[header_len] = Opt.message_type;
    out[header_len + 1] = 1;
    out[header_len + 2] = @backingInt(kind);
    @memcpy(out[header_len + 3 ..][0..options.len], options);
    return out[0 .. header_len + 3 + options.len];
}

test "a DISCOVER is a BOOTREQUEST for this MAC, padded to the BOOTP floor" {
    var buf: [max_message_len]u8 = @splat(0xAA);
    const m = try buildDiscover(test_mac, test_xid, "zig-pico2w", &buf);

    try testing.expectEqual(min_message_len, m.len);
    try testing.expectEqual(@as(u8, 1), m[0]); // BOOTREQUEST
    try testing.expectEqual(@as(u8, 1), m[1]); // Ethernet
    try testing.expectEqual(@as(u8, 6), m[2]);
    try testing.expectEqual(test_xid, std.mem.readInt(u32, m[4..8], .big));
    // The broadcast flag: without an address, a unicast reply would have to be
    // ARPed for.
    try testing.expectEqual(@as(u16, 0x8000), std.mem.readInt(u16, m[10..12], .big));
    // ciaddr, yiaddr, siaddr, giaddr are all zero in a DISCOVER.
    try testing.expectEqualSlices(u8, &@as([16]u8, @splat(0)), m[12..28]);
    try testing.expectEqualSlices(u8, &test_mac, m[28..34]);
    try testing.expectEqualSlices(u8, &magic_cookie, m[236..240]);
    try testing.expectEqualSlices(u8, &.{ 53, 1, 1 }, m[240..243]);
}

test "a DISCOVER names itself, so the lease can be told apart from an older one" {
    var buf: [max_message_len]u8 = @splat(0);
    const m = try buildDiscover(test_mac, test_xid, "zig-pico2w", &buf);
    // Option 12. The router displays this, and a name we chose is evidence the
    // previous tenant of this MAC cannot supply.
    const at = std.mem.indexOf(u8, m, "zig-pico2w") orelse return error.HostnameMissing;
    try testing.expectEqual(@as(u8, 12), m[at - 2]);
    try testing.expectEqual(@as(u8, 10), m[at - 1]);
}

test "a REQUEST carries the offered address and the server it came from" {
    var buf: [max_message_len]u8 = @splat(0);
    const m = try buildRequest(
        test_mac,
        test_xid,
        .{ 192, 168, 86, 42 },
        .{ 192, 168, 86, 1 },
        "zig-pico2w",
        &buf,
    );

    try testing.expectEqualSlices(u8, &.{ 53, 1, 3 }, m[240..243]);
    // Option 50 — requested address. **Not `ciaddr`**, which stays zero until
    // the address is actually held.
    try testing.expectEqualSlices(u8, &.{ 50, 4, 192, 168, 86, 42 }, m[243..249]);
    // Option 54 — which server's offer. This is what declines the others.
    try testing.expectEqualSlices(u8, &.{ 54, 4, 192, 168, 86, 1 }, m[249..255]);
    try testing.expectEqualSlices(u8, &@as([4]u8, @splat(0)), m[12..16]);
}

test "a renewal carries the address in ciaddr and names no server" {
    var buf: [max_message_len]u8 = @splat(0);
    const m = try buildRenew(test_mac, test_xid, .{ 192, 168, 86, 42 }, "zig-pico2w", &buf);

    // Still a REQUEST.
    try testing.expectEqualSlices(u8, &.{ 53, 1, 3 }, m[240..243]);
    // **The address is in the header now**, not in an option.
    try testing.expectEqualSlices(u8, &.{ 192, 168, 86, 42 }, m[12..16]);

    // And neither option 50 nor option 54 appears anywhere in the message —
    // which is the whole difference from the initial REQUEST, and the reason a
    // client that reuses that one is subtly wrong rather than broken.
    var at: usize = header_len;
    while (at + 2 <= m.len) {
        const code = m[at];
        if (code == Opt.end) break;
        if (code == Opt.pad) {
            at += 1;
            continue;
        }
        try testing.expect(code != Opt.requested_ip);
        try testing.expect(code != Opt.server_id);
        at += 2 + @as(usize, m[at + 1]);
    }

    // The initial REQUEST, for contrast: zero ciaddr, both options present.
    var other: [max_message_len]u8 = @splat(0);
    const initial = try buildRequest(test_mac, test_xid, .{ 192, 168, 86, 42 }, .{ 192, 168, 86, 1 }, "zig-pico2w", &other);
    try testing.expectEqualSlices(u8, &@as([4]u8, @splat(0)), initial[12..16]);
    try testing.expectEqual(@as(u8, Opt.requested_ip), initial[243]);
    try testing.expectEqual(@as(u8, Opt.server_id), initial[249]);
}

test "an OFFER yields the address, mask, router, lease and server id" {
    var raw: [max_message_len]u8 = undefined;
    const m = serverReply(&raw, .offer, test_xid, test_mac, .{ 192, 168, 86, 42 }, &.{
        Opt.subnet_mask, 4, 255, 255, 255, 0,
        Opt.router,      4, 192, 168, 86,  1,
        Opt.lease_time, 4, 0,   0,   0x0E, 0x10, // 3600
        Opt.server_id,  4, 192, 168, 86,   1,
        Opt.dns,        4, 8,   8,   8,    8,
        Opt.end,
    });

    const r = try parseReply(m, test_mac, test_xid);
    try testing.expectEqual(MessageType.offer, r.kind);
    try testing.expectEqual([4]u8{ 192, 168, 86, 42 }, r.lease.ip);
    try testing.expectEqual([4]u8{ 255, 255, 255, 0 }, r.lease.mask);
    try testing.expectEqual([4]u8{ 192, 168, 86, 1 }, r.lease.gateway);
    try testing.expectEqual([4]u8{ 192, 168, 86, 1 }, r.lease.server_id);
    try testing.expectEqual([4]u8{ 8, 8, 8, 8 }, r.lease.dns);
    try testing.expectEqual(@as(u32, 3600), r.lease.lease_secs);
}

test "another client's exchange is refused, however well-formed" {
    var raw: [max_message_len]u8 = undefined;

    // Same shape, someone else's transaction. This is the one that matters:
    // these are broadcast, so they genuinely arrive here.
    const other_xid = serverReply(&raw, .offer, 0x1234_5678, test_mac, .{ 192, 168, 86, 99 }, &.{Opt.end});
    try testing.expectError(error.WrongTransaction, parseReply(other_xid, test_mac, test_xid));

    const other_mac = serverReply(&raw, .offer, test_xid, .{ 1, 2, 3, 4, 5, 6 }, .{ 192, 168, 86, 99 }, &.{Opt.end});
    try testing.expectError(error.NotOurHardwareAddress, parseReply(other_mac, test_mac, test_xid));

    // Our own DISCOVER, echoed back by the broadcast we sent it on.
    var ours: [max_message_len]u8 = @splat(0);
    const disco = try buildDiscover(test_mac, test_xid, "zig-pico2w", &ours);
    try testing.expectError(error.NotAReply, parseReply(disco, test_mac, test_xid));
}

test "a NAK is a result, not a parse failure" {
    var raw: [max_message_len]u8 = undefined;
    const m = serverReply(&raw, .nak, test_xid, test_mac, @splat(0), &.{Opt.end});

    // The distinction is worth keeping: a NAK means the server said no and the
    // client must start over, which is different from a reply we could not read.
    const r = try parseReply(m, test_mac, test_xid);
    try testing.expectEqual(MessageType.nak, r.kind);
}

test "a truncated option is refused rather than read past" {
    var raw: [max_message_len]u8 = undefined;
    const m = serverReply(&raw, .ack, test_xid, test_mac, .{ 192, 168, 86, 42 }, &.{
        Opt.subnet_mask, 4, 255, 255, // two bytes short, and no end option
    });
    try testing.expectError(error.TruncatedOption, parseReply(m, test_mac, test_xid));

    // A reply with no message type at all is not a DHCP reply, whatever else
    // it carries.
    var bare: [header_len]u8 = @splat(0);
    bare[0] = op_reply;
    bare[1] = htype_ethernet;
    bare[2] = hlen_ethernet;
    std.mem.writeInt(u32, bare[4..8], test_xid, .big);
    @memcpy(bare[28..34], &test_mac);
    @memcpy(bare[236..240], &magic_cookie);
    try testing.expectError(error.NoMessageType, parseReply(&bare, test_mac, test_xid));
}

test "options are read whatever order they arrive in, and pad bytes are skipped" {
    var raw: [max_message_len]u8 = undefined;
    const m = serverReply(&raw, .ack, test_xid, test_mac, .{ 192, 168, 86, 42 }, &.{
        // zig fmt: off
        Opt.pad,        Opt.pad,
        Opt.server_id,  4,
        192,            168,
        86,             1,
        Opt.pad,        Opt.lease_time,
        4,              0,
        0,              0x1C,
        0x20, // 7200
        Opt.subnet_mask,
        4,              255,
        255,            255,
        0,              Opt.end,
        // zig fmt: on
    });

    const r = try parseReply(m, test_mac, test_xid);
    try testing.expectEqual(MessageType.ack, r.kind);
    try testing.expectEqual(@as(u32, 7200), r.lease.lease_secs);
    try testing.expectEqual([4]u8{ 192, 168, 86, 1 }, r.lease.server_id);
}
