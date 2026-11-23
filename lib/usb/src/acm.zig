//! A CDC-ACM serial port, assembled.
//!
//! Puts the pieces from `cdc.zig` together into the descriptor blob a host
//! expects and answers the three class requests, so an application says how
//! many endpoints it wants and gets `/dev/ttyACM*` at the other end.
//!
//! ## The shape of an ACM device
//!
//! ```
//! configuration
//!   interface 0 — Communications / ACM
//!     header, call management, ACM, union   (the functional descriptors)
//!     endpoint: interrupt IN                (notifications; never used here)
//!   interface 1 — CDC Data
//!     endpoint: bulk OUT                    (host to device)
//!     endpoint: bulk IN                     (device to host)
//! ```
//!
//! Sixty-seven bytes, so the configuration descriptor is **the first thing here
//! that does not fit one 64-byte packet** and needs `device.zig`'s multi-packet
//! control path to be right.
//!
//! **The notification endpoint is declared and never written to.** A host
//! expects it to exist because the class says so, and a serial port that never
//! reports a change of carrier or a framing error has nothing to send on it.
//! Declaring it and leaving it idle is correct, not a stub.

const std = @import("std");
const proto = @import("usb.zig");
const cdc = @import("cdc.zig");

/// Which endpoint numbers to use. Defaults follow the usual convention and
/// nothing depends on them beyond internal consistency.
pub const Endpoints = struct {
    notify: u4 = 1,
    data_out: u4 = 2,
    data_in: u4 = 3,
};

pub const total_length: u16 = 9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7;

comptime {
    if (total_length != 67) @compileError("an ACM configuration is 67 bytes");
}

/// The whole configuration blob, built at comptime.
pub fn configuration(comptime eps: Endpoints, comptime max_power_ma: u16) [total_length]u8 {
    return proto.ConfigurationDescriptor.bytes(.{
        .wTotalLength = total_length,
        .bNumInterfaces = 2,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bMaxPower = @intCast(max_power_ma / 2), // the field is in 2 mA units
    }) ++ proto.InterfaceDescriptor.bytes(.{
        .bInterfaceNumber = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = cdc.comm_interface_class,
        .bInterfaceSubClass = cdc.acm_subclass,
        .bInterfaceProtocol = cdc.acm_protocol_none,
        .iInterface = 0,
    }) ++ cdc.headerDescriptor(0x0120) ++
        cdc.callManagementDescriptor(0x00, 1) ++
        cdc.acmDescriptor(cdc.acm_caps_line_coding) ++
        cdc.unionDescriptor(0, 1) ++
        proto.EndpointDescriptor.bytes(.{
            .bEndpointAddress = proto.endpointAddress(eps.notify, true),
            .bmAttributes = @backingInt(proto.TransferType.interrupt),
            .wMaxPacketSize = 8,
            .bInterval = 16,
        }) ++ proto.InterfaceDescriptor.bytes(.{
        .bInterfaceNumber = 1,
        .bNumEndpoints = 2,
        .bInterfaceClass = cdc.data_interface_class,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    }) ++ proto.EndpointDescriptor.bytes(.{
        .bEndpointAddress = proto.endpointAddress(eps.data_out, false),
        .bmAttributes = @backingInt(proto.TransferType.bulk),
        .wMaxPacketSize = 64,
        .bInterval = 0,
    }) ++ proto.EndpointDescriptor.bytes(.{
        .bEndpointAddress = proto.endpointAddress(eps.data_in, true),
        .bmAttributes = @backingInt(proto.TransferType.bulk),
        .wMaxPacketSize = 64,
        .bInterval = 0,
    });
}

/// The line settings and control state a host has asked for.
///
/// **A serial port over USB obeys none of this, and should not pretend to**:
/// there is no UART on the other side whose baud could be wrong. The values are
/// remembered so `GET_LINE_CODING` returns what was set, which is what a host
/// checks, and `dtr` is worth watching because it says whether anything is
/// listening.
pub const State = struct {
    line_coding: cdc.LineCoding = .{
        .dwDTERate = 115200,
        .bCharFormat = @backingInt(cdc.StopBits.one),
        .bParityType = @backingInt(cdc.Parity.none),
        .bDataBits = 8,
    },
    dtr: bool = false,
    rts: bool = false,
    /// Scratch for `GET_LINE_CODING`'s reply, which must outlive the handler.
    reply: [7]u8 = undefined,

    /// Answer a CDC class request. Wire this to `Device.onClassRequest`.
    pub fn onClassRequest(self: *State, s: proto.Setup, data: []const u8) ?[]const u8 {
        return switch (@as(cdc.Request, @fromBackingInt(@intCast(s.bRequest)))) {
            .set_line_coding => {
                if (data.len < 7) return null;
                self.line_coding = cdc.LineCoding.parse(data[0..7].*);
                return &.{};
            },
            .get_line_coding => {
                self.reply = self.line_coding.bytes();
                return &self.reply;
            },
            .set_control_line_state => {
                self.dtr = cdc.dtrAsserted(s.wValue);
                self.rts = s.wValue & cdc.control_rts != 0;
                return &.{};
            },
            // SEND_BREAK is not advertised by `acmDescriptor` above, so a host
            // should not send one — acknowledge rather than stall if it does.
            .send_break => &.{},
            _ => null,
        };
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the configuration blob's wTotalLength describes itself" {
    // The mistake that makes a host read nine bytes, ask for wTotalLength more,
    // and get back something that does not parse.
    const blob = configuration(.{}, 100);
    try testing.expectEqual(@as(usize, 67), blob.len);
    const stated = @as(u16, blob[2]) | (@as(u16, blob[3]) << 8);
    try testing.expectEqual(@as(u16, blob.len), stated);
}

test "the blob declares two interfaces, communications then data" {
    const blob = configuration(.{}, 100);
    try testing.expectEqual(@as(u8, 2), blob[4]); // bNumInterfaces
    // Interface 0 starts at offset 9.
    try testing.expectEqual(@as(u8, 4), blob[10]); // INTERFACE descriptor type
    try testing.expectEqual(cdc.comm_interface_class, blob[14]);
    try testing.expectEqual(cdc.acm_subclass, blob[15]);
    // Interface 1 follows the four functional descriptors and the notification
    // endpoint: 9 + 9 + 5 + 5 + 4 + 5 + 7 = 44.
    try testing.expectEqual(@as(u8, 4), blob[45]);
    try testing.expectEqual(@as(u8, 1), blob[46]); // bInterfaceNumber
    try testing.expectEqual(cdc.data_interface_class, blob[49]);
}

test "bMaxPower is expressed in 2 mA units" {
    // 100 mA must appear as 50. Writing the milliamps straight in asks for
    // 200 mA, which a bus-powered hub may refuse.
    const blob = configuration(.{}, 100);
    try testing.expectEqual(@as(u8, 50), blob[8]);
}

test "the functional descriptors sit between the two interfaces" {
    const blob = configuration(.{}, 100);
    // Header at 18, then call management, ACM, union — each tagged 0x24.
    try testing.expectEqualSlices(u8, &cdc.headerDescriptor(0x0120), blob[18..23]);
    try testing.expectEqualSlices(u8, &cdc.callManagementDescriptor(0x00, 1), blob[23..28]);
    try testing.expectEqualSlices(u8, &cdc.acmDescriptor(cdc.acm_caps_line_coding), blob[28..32]);
    try testing.expectEqualSlices(u8, &cdc.unionDescriptor(0, 1), blob[32..37]);
}

test "SET_LINE_CODING is remembered and GET_LINE_CODING returns it" {
    var st = State{};
    const s = proto.Setup.parse(.{ 0x21, 0x20, 0, 0, 0, 0, 7, 0 });
    // 9600 8N1 — the payload captured off the Debug Probe.
    const reply = st.onClassRequest(s, &.{ 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 });
    try testing.expect(reply != null);
    try testing.expectEqual(@as(usize, 0), reply.?.len); // acknowledged, no data
    try testing.expectEqual(@as(u32, 9600), st.line_coding.dwDTERate);

    const get = proto.Setup.parse(.{ 0xa1, 0x21, 0, 0, 0, 0, 7, 0 });
    const back = st.onClassRequest(get, &.{}).?;
    try testing.expectEqualSlices(u8, &.{ 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 }, back);
}

test "SET_CONTROL_LINE_STATE tracks DTR, which says a terminal is open" {
    var st = State{};
    try testing.expect(!st.dtr);
    _ = st.onClassRequest(proto.Setup.parse(.{ 0x21, 0x22, 0x03, 0, 0, 0, 0, 0 }), &.{});
    try testing.expect(st.dtr);
    try testing.expect(st.rts);
    _ = st.onClassRequest(proto.Setup.parse(.{ 0x21, 0x22, 0x00, 0, 0, 0, 0, 0 }), &.{});
    try testing.expect(!st.dtr);
}

test "a short SET_LINE_CODING is stalled rather than half-applied" {
    var st = State{};
    const s = proto.Setup.parse(.{ 0x21, 0x20, 0, 0, 0, 0, 7, 0 });
    try testing.expectEqual(@as(?[]const u8, null), st.onClassRequest(s, &.{ 0x80, 0x25 }));
    // And the previous setting is untouched.
    try testing.expectEqual(@as(u32, 115200), st.line_coding.dwDTERate);
}

test "an unknown class request is stalled" {
    var st = State{};
    const s = proto.Setup.parse(.{ 0x21, 0x99, 0, 0, 0, 0, 0, 0 });
    try testing.expectEqual(@as(?[]const u8, null), st.onClassRequest(s, &.{}));
}
