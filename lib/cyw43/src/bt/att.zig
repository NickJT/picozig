//! The Attribute Protocol and a small GATT server — rung 20.
//!
//! A GATT server is an **array of attributes**, each with a handle, a type and
//! a value, and a protocol for a client to walk it. That is genuinely all it
//! is: the apparent complexity is in the walk, because a client discovers what
//! is there by asking repeatedly and being told "nothing more" at the end.
//!
//! This file is pure — a table in, a response out, no device — so the whole of
//! it is testable against the requests a phone actually sends.
//!
//! ## What a phone does when it connects
//!
//! nRF Connect runs this the moment the connection completes, which is why
//! rung 19 showed a spinner: with nothing answering, it waits and gives up.
//!
//!  1. `Exchange MTU` — how big may a message be.
//!  2. `Read By Group Type` for `0x2800`, repeatedly, to find the services.
//!  3. `Read By Type` for `0x2803` within each service, to find the
//!     characteristics.
//!  4. `Find Information` to find the descriptors, which is how it locates the
//!     `0x2902` configuration descriptor a client writes to turn notifications
//!     on.
//!  5. `Read` and `Write` on the values themselves.
//!
//! **Every one of those must end with an error response**, not with silence.
//! `AttributeNotFound` is how a client learns it has reached the end, and a
//! server that simply stops answering leaves it waiting for the timeout —
//! which looks exactly like a server that crashed.
//!
//! ## The 128-bit UUID rule that shapes the responses
//!
//! `Read By Group Type` and `Read By Type` answer with a **list of equal-sized
//! entries** and one length byte covering all of them. A service with a 16-bit
//! UUID and one with a 128-bit UUID therefore cannot share a response. This
//! server sends one entry per response, which is legal, simple, and slightly
//! chatty — and avoids the class of bug where a mixed table produces a response
//! whose entries do not line up with its own length byte.

const std = @import("std");

pub const opcode = struct {
    pub const error_response: u8 = 0x01;
    pub const exchange_mtu_req: u8 = 0x02;
    pub const exchange_mtu_rsp: u8 = 0x03;
    pub const find_information_req: u8 = 0x04;
    pub const find_information_rsp: u8 = 0x05;
    pub const read_by_type_req: u8 = 0x08;
    pub const read_by_type_rsp: u8 = 0x09;
    pub const read_req: u8 = 0x0A;
    pub const read_rsp: u8 = 0x0B;
    pub const read_by_group_type_req: u8 = 0x10;
    pub const read_by_group_type_rsp: u8 = 0x11;
    pub const write_req: u8 = 0x12;
    pub const write_rsp: u8 = 0x13;
    pub const write_cmd: u8 = 0x52;
    pub const handle_value_notification: u8 = 0x1B;
};

/// The error codes a client acts on. `attribute_not_found` is the important
/// one: it is not a failure, it is how discovery terminates.
pub const err = struct {
    pub const invalid_handle: u8 = 0x01;
    pub const read_not_permitted: u8 = 0x02;
    pub const write_not_permitted: u8 = 0x03;
    pub const invalid_pdu: u8 = 0x04;
    pub const request_not_supported: u8 = 0x06;
    pub const attribute_not_found: u8 = 0x0A;
    pub const invalid_attribute_value_length: u8 = 0x0D;
    pub const unlikely_error: u8 = 0x0E;
};

/// Standard declaration types, and the descriptor a client writes to subscribe.
pub const uuid16 = struct {
    pub const primary_service: u16 = 0x2800;
    pub const characteristic: u16 = 0x2803;
    /// Client Characteristic Configuration. Writing 0x0001 here is a client
    /// saying "send me notifications".
    pub const cccd: u16 = 0x2902;
};

pub const props = struct {
    pub const read: u8 = 0x02;
    pub const write_no_response: u8 = 0x04;
    pub const write: u8 = 0x08;
    pub const notify: u8 = 0x10;
};

/// The default ATT MTU. A client may ask for more; this server keeps 23,
/// because every value it serves fits and a larger MTU would mean handling
/// fragmentation for no gain.
pub const default_mtu: u16 = 23;

/// An attribute's type: either a 16-bit assigned number or a 128-bit custom
/// one. Kept as a tagged union rather than always-128-bit because the
/// declarations are genuinely 16-bit and a response's entries must all be the
/// same size.
pub const Uuid = union(enum) {
    short: u16,
    long: [16]u8,

    pub fn len(self: Uuid) usize {
        return switch (self) {
            .short => 2,
            .long => 16,
        };
    }

    pub fn write(self: Uuid, out: []u8) void {
        switch (self) {
            .short => |v| std.mem.writeInt(u16, out[0..2], v, .little),
            // 128-bit UUIDs go out little-endian too, which is why the
            // constants below are written reversed.
            .long => |v| @memcpy(out[0..16], &v),
        }
    }

    pub fn eql(self: Uuid, other: Uuid) bool {
        return switch (self) {
            .short => |a| switch (other) {
                .short => |b| a == b,
                .long => false,
            },
            .long => |a| switch (other) {
                .short => false,
                .long => |b| std.mem.eql(u8, &a, &b),
            },
        };
    }
};

/// One entry in the table.
pub const Attribute = struct {
    handle: u16,
    type: Uuid,
    /// The bytes this attribute holds.
    ///
    /// **Mutable, and it has to be.** A write lands here, and a read returns
    /// the same slice — so a client writing 1 to the LED and then reading it
    /// back sees 1. The first version of this was `[]const u8`: writes were
    /// detected and reported and the bytes went nowhere, so the LED byte stayed
    /// zero and the board dutifully logged `LED OFF` for every write. The host
    /// tests passed, because they only ever checked which handle was written.
    /// It took a phone and a light that would not come on.
    ///
    /// An application points this at storage it also updates itself — the
    /// counter below is rewritten in place every second — so the two directions
    /// share one buffer by construction rather than by copying.
    value: []u8,
    readable: bool = true,
    writable: bool = false,
    /// For a characteristic value: the handle of its declaration's group, used
    /// to bound `Read By Group Type`.
    group_end: u16 = 0,
};

/// What the server decided to do with a request.
pub const Outcome = struct {
    /// The response to send, or empty for a request that takes none — a Write
    /// Command is the only one of those here.
    response: []const u8,
    /// Set when a write landed on an attribute, so the caller can act on it.
    written_handle: ?u16 = null,
};

/// A GATT server over a caller-supplied table.
///
/// The table is borrowed, not owned, and the values inside it are slices the
/// caller keeps live — which lets an application point an attribute at a
/// counter it updates in place, without this file knowing anything about it.
pub const Server = struct {
    table: []const Attribute,
    mtu: u16 = default_mtu,
    /// Set by a client writing 0x0001 to the CCCD. **Notifications before this
    /// are a protocol violation**, and a phone that receives one may drop the
    /// connection rather than display it.
    notifications_enabled: bool = false,
    cccd_handle: u16 = 0,

    /// Scratch for the response. An ATT response never exceeds the MTU, and
    /// the MTU here is 23.
    buf: [64]u8 = @splat(0),

    pub fn handle(self: *Server, req: []const u8) Outcome {
        if (req.len < 1) return self.errorFor(0, 0, err.invalid_pdu);
        const op = req[0];

        return switch (op) {
            opcode.exchange_mtu_req => self.exchangeMtu(req),
            opcode.read_by_group_type_req => self.readByGroupType(req),
            opcode.read_by_type_req => self.readByType(req),
            opcode.find_information_req => self.findInformation(req),
            opcode.read_req => self.read(req),
            opcode.write_req => self.write(req, true),
            opcode.write_cmd => self.write(req, false),
            // **An unsupported request still gets an answer.** Silence is
            // indistinguishable from a dead server, and a client waits out its
            // timeout before deciding.
            else => self.errorFor(op, 0, err.request_not_supported),
        };
    }

    fn errorFor(self: *Server, op: u8, h: u16, code: u8) Outcome {
        self.buf[0] = opcode.error_response;
        self.buf[1] = op;
        std.mem.writeInt(u16, self.buf[2..4], h, .little);
        self.buf[4] = code;
        return .{ .response = self.buf[0..5] };
    }

    fn exchangeMtu(self: *Server, req: []const u8) Outcome {
        if (req.len < 3) return self.errorFor(req[0], 0, err.invalid_pdu);
        // The client's MTU is noted and not adopted: this server answers with
        // its own, and the smaller of the two is what both sides use.
        self.buf[0] = opcode.exchange_mtu_rsp;
        std.mem.writeInt(u16, self.buf[1..3], self.mtu, .little);
        return .{ .response = self.buf[0..3] };
    }

    /// Primary service discovery. One service per response — see the header.
    fn readByGroupType(self: *Server, req: []const u8) Outcome {
        if (req.len < 7) return self.errorFor(req[0], 0, err.invalid_pdu);
        const start = std.mem.readInt(u16, req[1..3], .little);
        const end = std.mem.readInt(u16, req[3..5], .little);
        const want = readUuid(req[5..]) orelse return self.errorFor(req[0], start, err.invalid_pdu);

        for (self.table) |a| {
            if (a.handle < start or a.handle > end) continue;
            if (!a.type.eql(want)) continue;

            const entry_len = 4 + a.value.len;
            self.buf[0] = opcode.read_by_group_type_rsp;
            self.buf[1] = @intCast(entry_len);
            std.mem.writeInt(u16, self.buf[2..4], a.handle, .little);
            std.mem.writeInt(u16, self.buf[4..6], a.group_end, .little);
            @memcpy(self.buf[6..][0..a.value.len], a.value);
            return .{ .response = self.buf[0 .. 6 + a.value.len] };
        }
        // The end of the walk, and the client is waiting to be told.
        return self.errorFor(req[0], start, err.attribute_not_found);
    }

    /// Characteristic discovery, and anything else found by type.
    fn readByType(self: *Server, req: []const u8) Outcome {
        if (req.len < 7) return self.errorFor(req[0], 0, err.invalid_pdu);
        const start = std.mem.readInt(u16, req[1..3], .little);
        const end = std.mem.readInt(u16, req[3..5], .little);
        const want = readUuid(req[5..]) orelse return self.errorFor(req[0], start, err.invalid_pdu);

        for (self.table) |a| {
            if (a.handle < start or a.handle > end) continue;
            if (!a.type.eql(want)) continue;

            const entry_len = 2 + a.value.len;
            self.buf[0] = opcode.read_by_type_rsp;
            self.buf[1] = @intCast(entry_len);
            std.mem.writeInt(u16, self.buf[2..4], a.handle, .little);
            @memcpy(self.buf[4..][0..a.value.len], a.value);
            return .{ .response = self.buf[0 .. 4 + a.value.len] };
        }
        return self.errorFor(req[0], start, err.attribute_not_found);
    }

    /// Descriptor discovery: handle and type, for everything in range. This is
    /// how a client finds the CCCD it needs to write to subscribe.
    fn findInformation(self: *Server, req: []const u8) Outcome {
        if (req.len < 5) return self.errorFor(req[0], 0, err.invalid_pdu);
        const start = std.mem.readInt(u16, req[1..3], .little);
        const end = std.mem.readInt(u16, req[3..5], .little);

        for (self.table) |a| {
            if (a.handle < start or a.handle > end) continue;
            // Format 1 is 16-bit UUIDs, 2 is 128-bit. One attribute per
            // response keeps the two from having to share.
            switch (a.type) {
                .short => |u| {
                    self.buf[0] = opcode.find_information_rsp;
                    self.buf[1] = 0x01;
                    std.mem.writeInt(u16, self.buf[2..4], a.handle, .little);
                    std.mem.writeInt(u16, self.buf[4..6], u, .little);
                    return .{ .response = self.buf[0..6] };
                },
                .long => |u| {
                    self.buf[0] = opcode.find_information_rsp;
                    self.buf[1] = 0x02;
                    std.mem.writeInt(u16, self.buf[2..4], a.handle, .little);
                    @memcpy(self.buf[4..20], &u);
                    return .{ .response = self.buf[0..20] };
                },
            }
        }
        return self.errorFor(req[0], start, err.attribute_not_found);
    }

    fn read(self: *Server, req: []const u8) Outcome {
        if (req.len < 3) return self.errorFor(req[0], 0, err.invalid_pdu);
        const h = std.mem.readInt(u16, req[1..3], .little);

        for (self.table) |a| {
            if (a.handle != h) continue;
            if (!a.readable) return self.errorFor(req[0], h, err.read_not_permitted);

            self.buf[0] = opcode.read_rsp;
            // Truncated to the MTU, which is what a client expects and what it
            // uses Read Blob to continue past. Nothing here is that long.
            const n = @min(a.value.len, self.mtu - 1);
            @memcpy(self.buf[1..][0..n], a.value[0..n]);
            return .{ .response = self.buf[0 .. 1 + n] };
        }
        return self.errorFor(req[0], h, err.invalid_handle);
    }

    fn write(self: *Server, req: []const u8, wants_response: bool) Outcome {
        if (req.len < 3) return self.errorFor(req[0], 0, err.invalid_pdu);
        const h = std.mem.readInt(u16, req[1..3], .little);
        const value = req[3..];

        for (self.table) |a| {
            if (a.handle != h) continue;
            if (!a.writable) return self.errorFor(req[0], h, err.write_not_permitted);

            // The CCCD is the server's own business: a client writing 0x0001
            // is subscribing, and nothing above needs to know how that is
            // spelled.
            if (h == self.cccd_handle and value.len >= 2) {
                self.notifications_enabled = (value[0] & 0x01) != 0;
            }

            // **Store it.** Everything else here is bookkeeping; this is the
            // write.
            if (value.len > a.value.len) {
                return self.errorFor(req[0], h, err.invalid_attribute_value_length);
            }
            @memcpy(a.value[0..value.len], value);

            if (!wants_response) return .{ .response = &.{}, .written_handle = h };
            self.buf[0] = opcode.write_rsp;
            return .{ .response = self.buf[0..1], .written_handle = h };
        }
        return self.errorFor(req[0], h, err.invalid_handle);
    }

    /// Build a notification. Returns null if the client has not subscribed —
    /// **which is a refusal, not an oversight**: an unsolicited notification is
    /// a protocol violation and a phone may drop the connection over it.
    pub fn notify(self: *Server, h: u16, value: []const u8, out: []u8) ?[]u8 {
        if (!self.notifications_enabled) return null;
        if (out.len < 3 + value.len) return null;
        out[0] = opcode.handle_value_notification;
        std.mem.writeInt(u16, out[1..3], h, .little);
        @memcpy(out[3..][0..value.len], value);
        return out[0 .. 3 + value.len];
    }
};

/// A UUID at the end of a request: two bytes or sixteen, and nothing else.
fn readUuid(bytes: []const u8) ?Uuid {
    if (bytes.len == 2) return .{ .short = std.mem.readInt(u16, bytes[0..2], .little) };
    if (bytes.len == 16) {
        var u: [16]u8 = undefined;
        @memcpy(&u, bytes[0..16]);
        return .{ .long = u };
    }
    return null;
}

// ---------------------------------------------------------------------------
// Host unit tests
//
// The requests below are the ones nRF Connect actually sends, in the order it
// sends them, so this suite is a rehearsal of the discovery that rung 19's
// spinner was waiting for.
// ---------------------------------------------------------------------------

const testing = std.testing;

/// A service UUID, little-endian on the wire — so the bytes are the reverse of
/// how the UUID is written down.
const service_uuid_const = [16]u8{
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
};
const counter_uuid_const = [16]u8{
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e,
};

// The table holds mutable slices, so every value it points at is `var` —
// including the UUIDs, which never change but must live somewhere writable-
// typed for the table to reference them uniformly.
var service_uuid = service_uuid_const;
var counter_uuid = counter_uuid_const;
var counter_value = [_]u8{ 0, 0, 0, 0 };
var led_value = [_]u8{0};
var counter_decl: [19]u8 = [_]u8{ props.read | props.notify, 0x03, 0x00 } ++ counter_uuid_const;
var led_decl: [19]u8 = [_]u8{ props.read | props.write, 0x06, 0x00 } ++ counter_uuid_const;
var cccd_value = [_]u8{ 0, 0 };

fn testTable() []const Attribute {
    const S = struct {
        var table = [_]Attribute{
            .{ .handle = 1, .type = .{ .short = uuid16.primary_service }, .value = &service_uuid, .group_end = 6 },
            .{ .handle = 2, .type = .{ .short = uuid16.characteristic }, .value = &counter_decl },
            .{ .handle = 3, .type = .{ .long = counter_uuid_const }, .value = &counter_value },
            .{ .handle = 4, .type = .{ .short = uuid16.cccd }, .value = &cccd_value, .writable = true },
            .{ .handle = 5, .type = .{ .short = uuid16.characteristic }, .value = &led_decl },
            .{ .handle = 6, .type = .{ .long = counter_uuid_const }, .value = &led_value, .writable = true },
        };
    };
    return &S.table;
}

fn newServer() Server {
    return .{ .table = testTable(), .cccd_handle = 4 };
}

test "the MTU exchange is answered, and the server keeps its own" {
    var s = newServer();
    // A phone asking for 517, which they do.
    const out = s.handle(&.{ opcode.exchange_mtu_req, 0x05, 0x02 });
    try testing.expectEqual(opcode.exchange_mtu_rsp, out.response[0]);
    try testing.expectEqual(default_mtu, std.mem.readInt(u16, out.response[1..3], .little));
}

test "service discovery finds the service, then says there are no more" {
    var s = newServer();
    // Read By Group Type, 0x0001..0xFFFF, type 0x2800.
    var req = [_]u8{ opcode.read_by_group_type_req, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28 };
    const first = s.handle(&req);

    try testing.expectEqual(opcode.read_by_group_type_rsp, first.response[0]);
    try testing.expectEqual(@as(u8, 20), first.response[1]); // 4 + a 16-byte UUID
    try testing.expectEqual(@as(u16, 1), std.mem.readInt(u16, first.response[2..4], .little));
    try testing.expectEqual(@as(u16, 6), std.mem.readInt(u16, first.response[4..6], .little));
    try testing.expectEqualSlices(u8, &service_uuid, first.response[6..22]);

    // Asking again from past the end. **This must be an error response**, not
    // silence: it is how the client learns the walk is finished, and a server
    // that stops answering leaves it waiting out its timeout.
    req[1] = 0x07;
    const done = s.handle(&req);
    try testing.expectEqual(opcode.error_response, done.response[0]);
    try testing.expectEqual(err.attribute_not_found, done.response[4]);
}

test "characteristic discovery returns the declaration, value handle included" {
    var s = newServer();
    const out = s.handle(&.{ opcode.read_by_type_req, 0x01, 0x00, 0x06, 0x00, 0x03, 0x28 });

    try testing.expectEqual(opcode.read_by_type_rsp, out.response[0]);
    try testing.expectEqual(@as(u16, 2), std.mem.readInt(u16, out.response[2..4], .little));
    // Properties, then the handle the value lives at — which is what the client
    // reads and subscribes to, and it is not the declaration's own handle.
    try testing.expectEqual(props.read | props.notify, out.response[4]);
    try testing.expectEqual(@as(u16, 3), std.mem.readInt(u16, out.response[5..7], .little));
}

test "descriptor discovery finds the CCCD, which is how subscribing works" {
    var s = newServer();
    const out = s.handle(&.{ opcode.find_information_req, 0x04, 0x00, 0x06, 0x00 });

    try testing.expectEqual(opcode.find_information_rsp, out.response[0]);
    try testing.expectEqual(@as(u8, 0x01), out.response[1]); // 16-bit format
    try testing.expectEqual(@as(u16, 4), std.mem.readInt(u16, out.response[2..4], .little));
    try testing.expectEqual(uuid16.cccd, std.mem.readInt(u16, out.response[4..6], .little));
}

test "a read returns the value, and an unknown handle is refused" {
    var s = newServer();
    counter_value = .{ 0x2A, 0x00, 0x00, 0x00 };

    const out = s.handle(&.{ opcode.read_req, 0x03, 0x00 });
    try testing.expectEqual(opcode.read_rsp, out.response[0]);
    try testing.expectEqualSlices(u8, &.{ 0x2A, 0x00, 0x00, 0x00 }, out.response[1..5]);

    const bad = s.handle(&.{ opcode.read_req, 0x99, 0x00 });
    try testing.expectEqual(opcode.error_response, bad.response[0]);
    try testing.expectEqual(err.invalid_handle, bad.response[4]);
}

test "a write to a read-only attribute is refused rather than ignored" {
    var s = newServer();
    // Handle 3 is the counter: readable and notifiable, not writable.
    const out = s.handle(&.{ opcode.write_req, 0x03, 0x00, 0x01 });
    try testing.expectEqual(opcode.error_response, out.response[0]);
    try testing.expectEqual(err.write_not_permitted, out.response[4]);
    try testing.expectEqual(@as(?u16, null), out.written_handle);
}

test "a write actually lands in the attribute, and a read gets it back" {
    var s = newServer();
    led_value = .{0};

    const out = s.handle(&.{ opcode.write_req, 0x06, 0x00, 0x01 });
    try testing.expectEqual(opcode.write_rsp, out.response[0]);
    try testing.expectEqual(@as(?u16, 6), out.written_handle);

    // **The byte, not just the handle.** Reporting the write and dropping the
    // value passes any test that checks only `written_handle` — and on
    // hardware it is an LED that never comes on however many times the phone
    // writes to it.
    try testing.expectEqual(@as(u8, 1), led_value[0]);
    const back = s.handle(&.{ opcode.read_req, 0x06, 0x00 });
    try testing.expectEqual(@as(u8, 1), back.response[1]);

    // And a write longer than the attribute is refused rather than truncated
    // or written past.
    const long = s.handle(&.{ opcode.write_req, 0x06, 0x00, 1, 2, 3 });
    try testing.expectEqual(err.invalid_attribute_value_length, long.response[4]);

    // A Write Command takes no response at all, and still reports the write.
    const cmd = s.handle(&.{ opcode.write_cmd, 0x06, 0x00, 0x00 });
    try testing.expectEqual(@as(usize, 0), cmd.response.len);
    try testing.expectEqual(@as(?u16, 6), cmd.written_handle);
}

test "notifications are refused until the client subscribes" {
    var s = newServer();
    var out: [32]u8 = undefined;

    // **Before the CCCD write there is no subscription**, and sending anyway is
    // a protocol violation a phone may drop the connection over.
    try testing.expectEqual(@as(?[]u8, null), s.notify(3, &.{ 1, 2, 3, 4 }, &out));

    _ = s.handle(&.{ opcode.write_req, 0x04, 0x00, 0x01, 0x00 });
    try testing.expect(s.notifications_enabled);

    const n = s.notify(3, &.{ 1, 2, 3, 4 }, &out) orelse return error.NoNotification;
    try testing.expectEqual(opcode.handle_value_notification, n[0]);
    try testing.expectEqual(@as(u16, 3), std.mem.readInt(u16, n[1..3], .little));
    try testing.expectEqualSlices(u8, &.{ 1, 2, 3, 4 }, n[3..7]);

    // And unsubscribing stops them again.
    _ = s.handle(&.{ opcode.write_req, 0x04, 0x00, 0x00, 0x00 });
    try testing.expectEqual(@as(?[]u8, null), s.notify(3, &.{ 1, 2, 3, 4 }, &out));
}

test "an unsupported request is answered, because silence reads as a dead server" {
    var s = newServer();
    // Read Blob, which this server does not implement.
    const out = s.handle(&.{ 0x0C, 0x03, 0x00, 0x00, 0x00 });
    try testing.expectEqual(opcode.error_response, out.response[0]);
    try testing.expectEqual(@as(u8, 0x0C), out.response[1]);
    try testing.expectEqual(err.request_not_supported, out.response[4]);

    // And a truncated one is an invalid PDU rather than a crash.
    const short = s.handle(&.{opcode.read_req});
    try testing.expectEqual(err.invalid_pdu, short.response[4]);
}
