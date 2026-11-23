//! CDC-ACM: the class that makes a device appear as a serial port.
//!
//! `CDC120.pdf` defines the class and then defers the interesting part to
//! `PSTN120.pdf`, which is where Abstract Control Model actually lives. Both are
//! in `../reference/`.
//!
//! ## An ACM device is two interfaces, not one
//!
//! **This is the thing that surprises people.** A serial port over USB is a
//! *communications* interface carrying an interrupt endpoint for notifications,
//! plus a *data* interface carrying the bulk pair that moves bytes — and a Union
//! functional descriptor tying them together so the host knows they are one
//! device rather than two. Omit the Union and the host loads a driver for each
//! half separately, and neither works.
//!
//! ## Verified against a working device, not just the specification
//!
//! Every constant here is checked against the Raspberry Pi Debug Probe's own
//! configuration descriptor, captured over `usbmon`. That is a CDC-ACM device this workspace uses
//! every day, so its descriptors are known-good by the strongest available
//! standard: they work.

const std = @import("std");
const proto = @import("usb.zig");

/// Interface class codes. `CDC120.pdf` §4.
pub const comm_interface_class: u8 = 0x02;
pub const data_interface_class: u8 = 0x0a;

/// Abstract Control Model — `02h`, the subclass `CDC120.pdf` hands to
/// `PSTN120.pdf`.
pub const acm_subclass: u8 = 0x02;

/// **Protocol 0, not 1.** `01h` claims AT-command support, and a host may then
/// expect the device to answer them. The Debug Probe declares 0 and so does
/// this; there is nothing to be gained by claiming a protocol you do not speak.
pub const acm_protocol_none: u8 = 0x00;

/// Class-specific descriptor types. `CDC120.pdf` Table 12.
pub const cs_interface: u8 = 0x24;
pub const cs_endpoint: u8 = 0x25;

/// Functional descriptor subtypes. `CDC120.pdf` Table 13.
pub const FunctionalSubtype = enum(u8) {
    header = 0x00,
    call_management = 0x01,
    abstract_control_management = 0x02,
    union_ = 0x06,
    _,
};

/// The three class requests an ACM device must answer. `PSTN120.pdf` §6.3 —
/// **not** `CDC120.pdf`, which is where anyone would look first and where these
/// requests are not defined.
pub const Request = enum(u8) {
    set_line_coding = 0x20,
    get_line_coding = 0x21,
    set_control_line_state = 0x22,
    send_break = 0x23,
    _,
};

// ---------------------------------------------------------------------------
// Functional descriptors
// ---------------------------------------------------------------------------

/// Header. Always first among the functional descriptors.
pub fn headerDescriptor(bcd_cdc: u16) [5]u8 {
    return .{
        5,                         cs_interface,            @backingInt(FunctionalSubtype.header),
        @truncate(bcd_cdc & 0xff), @truncate(bcd_cdc >> 8),
    };
}

/// Call Management. `bmCapabilities` 0 means the device does not handle call
/// management itself, which is what a plain serial port wants.
pub fn callManagementDescriptor(caps: u8, data_interface: u8) [5]u8 {
    return .{
        5,    cs_interface,   @backingInt(FunctionalSubtype.call_management),
        caps, data_interface,
    };
}

/// Abstract Control Management. `bmCapabilities` bit 1 advertises support for
/// `SET_LINE_CODING`, `GET_LINE_CODING` and `SET_CONTROL_LINE_STATE`; bit 2
/// advertises `SEND_BREAK`. **The Debug Probe declares `0x06`** — both — and so
/// does `acm_caps_line_and_break` below.
pub const acm_caps_line_coding: u8 = 0x02;
pub const acm_caps_send_break: u8 = 0x04;
pub const acm_caps_line_and_break: u8 = acm_caps_line_coding | acm_caps_send_break;

pub fn acmDescriptor(caps: u8) [4]u8 {
    return .{ 4, cs_interface, @backingInt(FunctionalSubtype.abstract_control_management), caps };
}

/// Union — the descriptor that makes two interfaces one device.
pub fn unionDescriptor(control_interface: u8, data_interface: u8) [5]u8 {
    return .{
        5,                 cs_interface,   @backingInt(FunctionalSubtype.union_),
        control_interface, data_interface,
    };
}

// ---------------------------------------------------------------------------
// Line coding
// ---------------------------------------------------------------------------

pub const StopBits = enum(u8) { one = 0, one_and_half = 1, two = 2 };
pub const Parity = enum(u8) { none = 0, odd = 1, even = 2, mark = 3, space = 4 };

/// The seven bytes carried by `SET_LINE_CODING` and `GET_LINE_CODING`.
/// `PSTN120.pdf` §6.3.11.
///
/// **Seven bytes, not eight.** `dwDTERate` is a 32-bit value at offset 0
/// followed by three single bytes, so any struct that pads to a multiple of
/// four is the wrong size and the host rejects the transfer.
pub const LineCoding = packed struct {
    dwDTERate: u32,
    bCharFormat: u8,
    bParityType: u8,
    bDataBits: u8,

    pub fn bytes(self: LineCoding) [7]u8 {
        return @bitCast(self);
    }

    pub fn parse(raw: [7]u8) LineCoding {
        return @bitCast(raw);
    }
};

comptime {
    if (@bitSizeOf(LineCoding) != 7 * 8) @compileError("line coding must be 7 bytes");
}

/// `SET_CONTROL_LINE_STATE`'s `wValue`. **Bit 0, DTR, is how a device learns a
/// terminal has opened the port** — and the reason output can vanish: a device
/// that writes before DTR is asserted is writing into a port nobody is reading.
pub const control_dtr: u16 = 1 << 0;
pub const control_rts: u16 = 1 << 1;

pub fn dtrAsserted(w_value: u16) bool {
    return w_value & control_dtr != 0;
}

// ---------------------------------------------------------------------------
// Tests — against the Debug Probe's real descriptors
// ---------------------------------------------------------------------------

const testing = std.testing;

/// The CDC portion of the Raspberry Pi Debug Probe's configuration descriptor,
/// captured over `usbmon` on 2026-08-29.
///
/// Interface 1 (Communications/ACM), its four functional descriptors, its
/// notification endpoint, then interface 2 (CDC Data) and its bulk pair.
const probe_cdc_descriptors = [_]u8{
    0x09, 0x04, 0x01, 0x00, 0x01, 0x02, 0x02, 0x00, 0x06, // interface 1: class 2, subclass 2
    0x05, 0x24, 0x00, 0x20, 0x01, // header, bcdCDC 0x0120
    0x05, 0x24, 0x01, 0x00, 0x02, // call management, data interface 2
    0x04, 0x24, 0x02, 0x06, // ACM, caps 0x06
    0x05, 0x24, 0x06, 0x01, 0x02, // union: control 1, data 2
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x10, // EP 0x81 interrupt, 64, interval 16
    0x09, 0x04, 0x02, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // interface 2: class 0x0a
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, // EP 0x02 bulk OUT
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00, // EP 0x83 bulk IN
};

test "the four functional descriptors match a working CDC device byte for byte" {
    // Reconstructed here and compared against the captured bytes. If a subtype
    // or a length is wrong, this fails — and the alternative is finding out
    // from a host that declines to load a driver and does not say why.
    try testing.expectEqualSlices(u8, probe_cdc_descriptors[9..14], &headerDescriptor(0x0120));
    try testing.expectEqualSlices(u8, probe_cdc_descriptors[14..19], &callManagementDescriptor(0x00, 2));
    try testing.expectEqualSlices(u8, probe_cdc_descriptors[19..23], &acmDescriptor(acm_caps_line_and_break));
    try testing.expectEqualSlices(u8, probe_cdc_descriptors[23..28], &unionDescriptor(1, 2));
}

test "the probe's ACM capabilities are the two this module names" {
    // 0x06 is line coding plus send break. Decoded from the capture rather than
    // assumed, because a device advertising a capability it lacks is a device
    // the host will eventually ask about it.
    try testing.expectEqual(@as(u8, 0x06), probe_cdc_descriptors[22]);
    try testing.expectEqual(@as(u8, 0x06), acm_caps_line_and_break);
    try testing.expect(acm_caps_line_and_break & acm_caps_line_coding != 0);
}

test "the interface classes are what the capture shows" {
    // Interface 1 is Communications/ACM, interface 2 is CDC Data. Getting these
    // the wrong way round produces a device that enumerates and does nothing.
    try testing.expectEqual(comm_interface_class, probe_cdc_descriptors[5]);
    try testing.expectEqual(acm_subclass, probe_cdc_descriptors[6]);
    try testing.expectEqual(acm_protocol_none, probe_cdc_descriptors[7]);
    try testing.expectEqual(data_interface_class, probe_cdc_descriptors[40]); // interface 2's bInterfaceClass
}

test "the notification endpoint is interrupt, the data pair is bulk" {
    // EP 0x81 interrupt with a 16 ms interval; EP 0x02 OUT and 0x83 IN bulk.
    try testing.expectEqual(@as(u8, 0x81), probe_cdc_descriptors[30]);
    try testing.expectEqual(@as(u8, @backingInt(proto.TransferType.interrupt)), probe_cdc_descriptors[31]);
    try testing.expectEqual(@as(u8, 0x10), probe_cdc_descriptors[34]); // bInterval
    try testing.expectEqual(@as(u8, @backingInt(proto.TransferType.bulk)), probe_cdc_descriptors[47]); // EP 0x02's bmAttributes
}

test "line coding is seven bytes and matches a real SET_LINE_CODING" {
    // `80 25 00 00 00 00 08` — captured off the Debug Probe during enumeration.
    // 0x00002580 = 9600 baud, 1 stop bit, no parity, 8 data bits.
    const captured = [7]u8{ 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
    const lc = LineCoding.parse(captured);
    try testing.expectEqual(@as(u32, 9600), lc.dwDTERate);
    try testing.expectEqual(@as(u8, @backingInt(StopBits.one)), lc.bCharFormat);
    try testing.expectEqual(@as(u8, @backingInt(Parity.none)), lc.bParityType);
    try testing.expectEqual(@as(u8, 8), lc.bDataBits);
    // And back again, byte for byte.
    try testing.expectEqualSlices(u8, &captured, &lc.bytes());
}

test "line coding round-trips at a rate that is not 9600" {
    const lc = LineCoding{
        .dwDTERate = 115200,
        .bCharFormat = @backingInt(StopBits.one),
        .bParityType = @backingInt(Parity.none),
        .bDataBits = 8,
    };
    const b = lc.bytes();
    try testing.expectEqual(@as(usize, 7), b.len);
    // 115200 = 0x0001C200, little-endian on the wire.
    try testing.expectEqualSlices(u8, &.{ 0x00, 0xc2, 0x01, 0x00 }, b[0..4]);
    try testing.expectEqual(@as(u32, 115200), LineCoding.parse(b).dwDTERate);
}

test "the class requests are PSTN's codes, and DTR is bit 0" {
    try testing.expectEqual(@as(u8, 0x20), @backingInt(Request.set_line_coding));
    try testing.expectEqual(@as(u8, 0x21), @backingInt(Request.get_line_coding));
    try testing.expectEqual(@as(u8, 0x22), @backingInt(Request.set_control_line_state));
    try testing.expect(dtrAsserted(control_dtr));
    try testing.expect(!dtrAsserted(control_rts));
    try testing.expect(dtrAsserted(control_dtr | control_rts));
}

test "the captured SETUP for SET_LINE_CODING decodes as a class request" {
    // `21 20 0000 0001 0007` from the trace — class request, to interface 1,
    // seven bytes of payload. Exactly the line coding above.
    const s = proto.Setup.parse(.{ 0x21, 0x20, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00 });
    try testing.expectEqual(proto.RequestKind.class, s.kind());
    try testing.expectEqual(@as(u8, @backingInt(Request.set_line_coding)), s.bRequest);
    try testing.expectEqual(@as(u16, 1), s.wIndex);
    try testing.expectEqual(@as(u16, 7), s.wLength);
}
