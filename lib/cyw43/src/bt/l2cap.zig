//! L2CAP framing — the four bytes between an ACL packet and ATT.
//!
//! Small enough to fold into its neighbours and kept separate anyway, because
//! it is a real layer with a real rule: **the length counts the payload, not
//! the header**, and getting that wrong shortens every message by four bytes in
//! a way the peer reports as a malformed PDU rather than a length error.
//!
//! On BLE the only channels that matter are fixed ones — there is no channel
//! negotiation here, and nothing in this file opens or closes anything.

const std = @import("std");

pub const header_len: usize = 4;

/// Fixed channel identifiers. BLE assigns these rather than negotiating them.
pub const cid = struct {
    /// The Attribute Protocol — everything rung 20 does.
    pub const att: u16 = 0x0004;
    /// Connection parameter updates and the like.
    pub const signalling: u16 = 0x0005;
    /// Pairing. Not implemented: this driver has no security manager, so a
    /// phone that tries to bond gets nothing.
    pub const smp: u16 = 0x0006;
};

pub const Frame = struct {
    cid: u16,
    payload: []const u8,
};

/// Wrap a payload. Returns the slice to hand to `hci.buildAcl`.
pub fn build(channel: u16, payload: []const u8, out: []u8) ![]u8 {
    const total = header_len + payload.len;
    if (out.len < total) return error.BufferTooSmall;
    // **The length is the payload's**, which is what makes this worth its own
    // function rather than four inline lines that eventually include the
    // header.
    std.mem.writeInt(u16, out[0..2], @intCast(payload.len), .little);
    std.mem.writeInt(u16, out[2..4], channel, .little);
    @memcpy(out[header_len..][0..payload.len], payload);
    return out[0..total];
}

pub fn parse(buf: []const u8) !Frame {
    if (buf.len < header_len) return error.Malformed;
    const len: usize = std.mem.readInt(u16, buf[0..2], .little);
    if (header_len + len > buf.len) return error.Malformed;
    return .{
        .cid = std.mem.readInt(u16, buf[2..4], .little),
        .payload = buf[header_len..][0..len],
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the length field counts the payload, not the whole frame" {
    var buf: [32]u8 = @splat(0xAA);
    const f = try build(cid.att, &.{ 0x02, 0x17, 0x00 }, &buf);

    try testing.expectEqual(@as(usize, 7), f.len);
    // Three, not seven. A frame whose length includes its own header is
    // reported by the peer as a malformed PDU, which sends the reader looking
    // at ATT rather than at this.
    try testing.expectEqual(@as(u16, 3), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(cid.att, std.mem.readInt(u16, f[2..4], .little));

    const p = try parse(f);
    try testing.expectEqual(cid.att, p.cid);
    try testing.expectEqualSlices(u8, &.{ 0x02, 0x17, 0x00 }, p.payload);
}

test "a frame claiming more than it carries is refused" {
    var buf: [8]u8 = @splat(0);
    std.mem.writeInt(u16, buf[0..2], 32, .little);
    std.mem.writeInt(u16, buf[2..4], cid.att, .little);
    try testing.expectError(error.Malformed, parse(&buf));
    try testing.expectError(error.Malformed, parse(buf[0..2]));
}
