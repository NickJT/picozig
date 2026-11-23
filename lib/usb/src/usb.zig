//! USB device protocol: descriptors and the standard requests of `usb_20.pdf`
//! Chapter 9.
//!
//! **This half knows nothing about the RP2350.** The controller lives in
//! `picozig`'s `rp2350.usb`; everything here is wire format and would be
//! identical on any chip. The split is deliberate and is the same one, for the
//! same reason, as chip-versus-board: what would be identical on any chip does
//! not import one, and is host-testable because of it.
//!
//! ## Descriptors are byte arrays, not structs
//!
//! Every descriptor here is a `packed struct` with an explicit size assertion
//! and a `bytes()` that hands back the wire form. The temptation is `extern
//! struct`, and it is a trap: the configuration descriptor is **9 bytes with a
//! `u16` in the middle**, so C ABI alignment pads it to 10 and the host reads
//! nine bytes of descriptor followed by one byte of nothing. `packed` has no
//! padding by construction, and `comptime` assertions below hold every
//! descriptor to the size Chapter 9 gives it.
//!
//! ## What is verified, and how
//!
//! The device descriptor's layout is checked against **a real one, captured off
//! real silicon**: the RP2350 bootrom's own, captured over `usbmon`. That is a stronger check than re-reading
//! the specification table, because it catches a field order that is plausible
//! and wrong.

const std = @import("std");

/// The device state machine, which binds these descriptors to picozig's
/// controller. Kept in its own file so that everything above stays free of any
/// chip at all.
pub const cdc = @import("cdc.zig");
pub const acm = @import("acm.zig");
pub const serial = @import("serial.zig");
pub const SerialPort = serial.SerialPort;
pub const device = @import("device.zig");
pub const Device = device.Device;
pub const Descriptors = device.Descriptors;

// ---------------------------------------------------------------------------
// Setup packets
// ---------------------------------------------------------------------------

/// The eight bytes of a control transfer's SETUP stage. `usb_20.pdf` §9.3.
pub const Setup = packed struct {
    bmRequestType: u8,
    bRequest: u8,
    wValue: u16,
    wIndex: u16,
    wLength: u16,

    pub fn parse(raw: [8]u8) Setup {
        return .{
            .bmRequestType = raw[0],
            .bRequest = raw[1],
            .wValue = @as(u16, raw[2]) | (@as(u16, raw[3]) << 8),
            .wIndex = @as(u16, raw[4]) | (@as(u16, raw[5]) << 8),
            .wLength = @as(u16, raw[6]) | (@as(u16, raw[7]) << 8),
        };
    }

    /// Which way the *data* stage goes. Bit 7 of `bmRequestType`, and it is
    /// named from the host's point of view like everything else in USB.
    pub fn dataIn(self: Setup) bool {
        return self.bmRequestType & 0x80 != 0;
    }

    /// Request category — bits 6:5. Standard requests are the only ones this
    /// module answers; class and vendor requests belong to whatever sits above.
    pub fn kind(self: Setup) RequestKind {
        return @fromBackingInt(@intCast((self.bmRequestType >> 5) & 0x3));
    }

    pub fn request(self: Setup) Request {
        return @fromBackingInt(@intCast(self.bRequest));
    }

    /// For `GET_DESCRIPTOR`, `wValue`'s high byte is the type and its low byte
    /// the index. Reading them the other way round is a classic, and produces a
    /// request for descriptor type 0 that a device rightly stalls.
    pub fn descriptorType(self: Setup) DescriptorType {
        return @fromBackingInt(@as(u8, @truncate(self.wValue >> 8)));
    }

    pub fn descriptorIndex(self: Setup) u8 {
        return @truncate(self.wValue & 0xff);
    }
};

pub const RequestKind = enum(u2) { standard = 0, class = 1, vendor = 2, reserved = 3 };

/// `usb_20.pdf` Table 9-4. Values 2 and 4 are reserved and deliberately absent.
pub const Request = enum(u8) {
    get_status = 0,
    clear_feature = 1,
    set_feature = 3,
    set_address = 5,
    get_descriptor = 6,
    set_descriptor = 7,
    get_configuration = 8,
    set_configuration = 9,
    get_interface = 10,
    set_interface = 11,
    synch_frame = 12,
    _,
};

/// `usb_20.pdf` Table 9-5.
pub const DescriptorType = enum(u8) {
    device = 1,
    configuration = 2,
    string = 3,
    interface = 4,
    endpoint = 5,
    device_qualifier = 6,
    other_speed_configuration = 7,
    interface_power = 8,
    _,
};

/// Transfer type, matching both `bmAttributes` in an endpoint descriptor and
/// the RP2350's `BUFFER_TYPE` field, which is why no translation is needed
/// between a descriptor and the hardware it configures.
pub const TransferType = enum(u2) { control = 0, isochronous = 1, bulk = 2, interrupt = 3 };

// ---------------------------------------------------------------------------
// Descriptors
// ---------------------------------------------------------------------------

/// `usb_20.pdf` Table 9-8. Eighteen bytes.
pub const DeviceDescriptor = packed struct {
    bLength: u8 = 18,
    bDescriptorType: u8 = @backingInt(DescriptorType.device),
    bcdUSB: u16,
    bDeviceClass: u8,
    bDeviceSubClass: u8,
    bDeviceProtocol: u8,
    bMaxPacketSize0: u8,
    idVendor: u16,
    idProduct: u16,
    bcdDevice: u16,
    iManufacturer: u8,
    iProduct: u8,
    iSerialNumber: u8,
    bNumConfigurations: u8,

    pub fn bytes(self: DeviceDescriptor) [18]u8 {
        return @bitCast(self);
    }
};

/// `usb_20.pdf` Table 9-10. Nine bytes — **the one that C ABI padding would
/// silently make ten.**
pub const ConfigurationDescriptor = packed struct {
    bLength: u8 = 9,
    bDescriptorType: u8 = @backingInt(DescriptorType.configuration),
    /// The length of this descriptor **and every interface and endpoint
    /// descriptor that follows it**, not of this one alone. A host reads nine
    /// bytes, learns the total from here, then asks again for all of it; a
    /// wrong value means the second read is truncated or over-long, and the
    /// device looks malformed rather than mis-sized.
    wTotalLength: u16,
    bNumInterfaces: u8,
    bConfigurationValue: u8,
    iConfiguration: u8,
    /// Bit 7 is reserved and **must be set**. A zero here is rejected by some
    /// hosts and shrugged at by others, which is the worst kind of bug.
    bmAttributes: u8 = 0x80,
    /// In 2 mA units, so 50 means 100 mA.
    bMaxPower: u8,

    pub fn bytes(self: ConfigurationDescriptor) [9]u8 {
        return @bitCast(self);
    }
};

/// `usb_20.pdf` Table 9-12. Nine bytes.
pub const InterfaceDescriptor = packed struct {
    bLength: u8 = 9,
    bDescriptorType: u8 = @backingInt(DescriptorType.interface),
    bInterfaceNumber: u8,
    bAlternateSetting: u8 = 0,
    /// **Endpoint zero is not counted.** Every interface has it implicitly.
    bNumEndpoints: u8,
    bInterfaceClass: u8,
    bInterfaceSubClass: u8,
    bInterfaceProtocol: u8,
    iInterface: u8,

    pub fn bytes(self: InterfaceDescriptor) [9]u8 {
        return @bitCast(self);
    }
};

/// `usb_20.pdf` Table 9-13. Seven bytes.
pub const EndpointDescriptor = packed struct {
    bLength: u8 = 7,
    bDescriptorType: u8 = @backingInt(DescriptorType.endpoint),
    /// Number in bits 3:0, direction in bit 7 — `0x81` is EP1 IN.
    bEndpointAddress: u8,
    /// Bits 1:0 are the transfer type; the rest matter only for isochronous.
    bmAttributes: u8,
    wMaxPacketSize: u16,
    /// Polling interval in frames. Ignored for bulk, required for interrupt.
    bInterval: u8,

    pub fn bytes(self: EndpointDescriptor) [7]u8 {
        return @bitCast(self);
    }
};

/// An endpoint address as a descriptor carries it.
pub fn endpointAddress(num: u4, dir_in: bool) u8 {
    return @as(u8, num) | (if (dir_in) @as(u8, 0x80) else 0);
}

comptime {
    // Chapter 9's sizes, asserted at compile time. `packed` makes these exact;
    // `extern` would make three of the four wrong.
    if (@bitSizeOf(Setup) != 64) @compileError("Setup must be 8 bytes");
    if (@bitSizeOf(DeviceDescriptor) != 18 * 8) @compileError("device descriptor must be 18 bytes");
    if (@bitSizeOf(ConfigurationDescriptor) != 9 * 8) @compileError("configuration descriptor must be 9 bytes");
    if (@bitSizeOf(InterfaceDescriptor) != 9 * 8) @compileError("interface descriptor must be 9 bytes");
    if (@bitSizeOf(EndpointDescriptor) != 7 * 8) @compileError("endpoint descriptor must be 7 bytes");
}

// ---------------------------------------------------------------------------
// String descriptors
// ---------------------------------------------------------------------------

/// String descriptor zero: the list of language IDs the device supports.
/// `usb_20.pdf` Table 9-15. 0x0409 is US English.
pub const lang_id_en_us: u16 = 0x0409;

pub fn languageDescriptor(comptime lang: u16) [4]u8 {
    return .{ 4, @backingInt(DescriptorType.string), @truncate(lang & 0xff), @truncate(lang >> 8) };
}

/// A string descriptor, built at comptime from ASCII.
///
/// **USB string descriptors are UTF-16LE**, so each character costs two bytes
/// and an ASCII string handed over unconverted enumerates as every other letter
/// followed by a null. This widens as it copies.
pub fn stringDescriptor(comptime text: []const u8) [2 + text.len * 2]u8 {
    comptime {
        // A descriptor's length is one byte, so this is the real ceiling.
        if (2 + text.len * 2 > 255) @compileError("string descriptor too long");
        for (text) |c| {
            if (c > 0x7f) @compileError(
                "stringDescriptor widens ASCII to UTF-16LE and cannot carry byte " ++
                    std.fmt.comptimePrint("0x{x}", .{c}) ++ " — encode UTF-16 yourself",
            );
        }
    }
    var out: [2 + text.len * 2]u8 = undefined;
    out[0] = @intCast(out.len);
    out[1] = @backingInt(DescriptorType.string);
    for (text, 0..) |c, i| {
        out[2 + i * 2] = c;
        out[3 + i * 2] = 0;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test {
    _ = device;
    _ = cdc;
    _ = acm;
    _ = serial;
}

/// The RP2350 bootrom's own device descriptor, captured over `usbmon` on
/// 2026-08-29 (frame 2 of the BOOTSEL enumeration).
///
/// **This is the fixture that matters.** It is not a transcription of Table
/// 9-8; it is eighteen bytes a real RP2350 put on a real wire, so a field order
/// that is plausible and wrong fails here rather than on the bench.
const rp2350_bootrom_device_descriptor = [18]u8{
    0x12, 0x01, 0x10, 0x02, 0x00, 0x00, 0x00, 0x40, 0x8a,
    0x2e, 0x0f, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
};

test "the device descriptor serialises exactly as real silicon emits it" {
    const d = DeviceDescriptor{
        .bcdUSB = 0x0210,
        .bDeviceClass = 0,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize0 = 64,
        .idVendor = 0x2e8a,
        .idProduct = 0x000f,
        .bcdDevice = 0x0100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
    };
    try testing.expectEqualSlices(u8, &rp2350_bootrom_device_descriptor, &d.bytes());
}

test "the captured descriptor decodes back to the values tshark reported" {
    // The other direction: the bytes, read as fields, must give what an
    // independent dissector said they meant. Two implementations agreeing on
    // eighteen bytes is worth more than either alone.
    const b = rp2350_bootrom_device_descriptor;
    try testing.expectEqual(@as(u8, 18), b[0]);
    try testing.expectEqual(@backingInt(DescriptorType.device), b[1]);
    try testing.expectEqual(@as(u16, 0x0210), @as(u16, b[2]) | (@as(u16, b[3]) << 8));
    try testing.expectEqual(@as(u8, 64), b[7]);
    try testing.expectEqual(@as(u16, 0x2e8a), @as(u16, b[8]) | (@as(u16, b[9]) << 8));
    try testing.expectEqual(@as(u16, 0x000f), @as(u16, b[10]) | (@as(u16, b[11]) << 8));
    try testing.expectEqual(@as(u8, 1), b[17]);
}

test "multi-byte descriptor fields are little-endian on the wire" {
    // idVendor 0x2e8a appears as 8a 2e. A big-endian slip here enumerates the
    // device as vendor 0x8a2e, which is nobody, and the host loads no driver.
    const d = DeviceDescriptor{
        .bcdUSB = 0x0200,
        .bDeviceClass = 0,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize0 = 64,
        .idVendor = 0x2e8a,
        .idProduct = 0x000f,
        .bcdDevice = 0x0100,
        .iManufacturer = 0,
        .iProduct = 0,
        .iSerialNumber = 0,
        .bNumConfigurations = 1,
    };
    const b = d.bytes();
    try testing.expectEqual(@as(u8, 0x8a), b[8]);
    try testing.expectEqual(@as(u8, 0x2e), b[9]);
}

test "the configuration descriptor is nine bytes, not ten" {
    // The whole reason these are `packed`. An `extern struct` with a u16 at
    // offset 2 pads to 10, and the host then reads one byte of rubbish where
    // the next descriptor should start.
    const c = ConfigurationDescriptor{
        .wTotalLength = 32,
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bMaxPower = 50,
    };
    const b = c.bytes();
    try testing.expectEqual(@as(usize, 9), b.len);
    try testing.expectEqual(@as(u8, 9), b[0]);
    try testing.expectEqual(@as(u8, 32), b[2]);
    try testing.expectEqual(@as(u8, 0), b[3]);
    // bmAttributes bit 7 is reserved-and-must-be-set, and defaults that way.
    try testing.expectEqual(@as(u8, 0x80), b[7]);
    try testing.expectEqual(@as(u8, 50), b[8]); // 100 mA, in 2 mA units
}

test "interface and endpoint descriptors are nine and seven bytes" {
    const i = InterfaceDescriptor{
        .bInterfaceNumber = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xff,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    };
    try testing.expectEqual(@as(usize, 9), i.bytes().len);

    const e = EndpointDescriptor{
        .bEndpointAddress = endpointAddress(1, false),
        .bmAttributes = @backingInt(TransferType.bulk),
        .wMaxPacketSize = 64,
        .bInterval = 0,
    };
    const b = e.bytes();
    try testing.expectEqual(@as(usize, 7), b.len);
    try testing.expectEqual(@as(u8, 0x01), b[2]); // EP1 OUT
    try testing.expectEqual(@as(u8, 2), b[3]); // bulk
    try testing.expectEqual(@as(u8, 64), b[4]); // 64, little-endian
    try testing.expectEqual(@as(u8, 0), b[5]);
}

test "endpoint addresses put direction in bit 7" {
    // dev_lowlevel's own pair, so this is checkable against the reference.
    try testing.expectEqual(@as(u8, 0x01), endpointAddress(1, false)); // EP1 OUT
    try testing.expectEqual(@as(u8, 0x82), endpointAddress(2, true)); // EP2 IN
    try testing.expectEqual(@as(u8, 0x80), endpointAddress(0, true)); // EP0 IN
}

test "a SETUP packet parses the way the captured enumeration reads" {
    // From bootsel-enum.pcapng: `80 06 0100 0000 0040` — GET_DESCRIPTOR of the
    // device descriptor, 64 bytes, the very first request of any enumeration.
    const s = Setup.parse(.{ 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00 });
    try testing.expectEqual(Request.get_descriptor, s.request());
    try testing.expectEqual(RequestKind.standard, s.kind());
    try testing.expect(s.dataIn());
    try testing.expectEqual(DescriptorType.device, s.descriptorType());
    try testing.expectEqual(@as(u8, 0), s.descriptorIndex());
    try testing.expectEqual(@as(u16, 64), s.wLength);
}

test "SET_ADDRESS from the captured trace decodes to address 5" {
    // `00 05 0005 0000 0000` — the moment the bootrom stopped answering on
    // address 0. Direction is host-to-device and there is no data stage.
    const s = Setup.parse(.{ 0x00, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 });
    try testing.expectEqual(Request.set_address, s.request());
    try testing.expect(!s.dataIn());
    try testing.expectEqual(@as(u16, 5), s.wValue);
    try testing.expectEqual(@as(u16, 0), s.wLength);
}

test "the configuration descriptor request is told apart by wValue's high byte" {
    // `80 06 0200 0000 0062` — the same bRequest as the device descriptor, and
    // only wValue's high byte distinguishes them. Reading the low byte instead
    // asks for descriptor type 0, which does not exist.
    const s = Setup.parse(.{ 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x62, 0x00 });
    try testing.expectEqual(DescriptorType.configuration, s.descriptorType());
    try testing.expectEqual(@as(u16, 98), s.wLength);
}

test "a class request is not mistaken for a standard one" {
    // `21 20 ...` is SET_LINE_CODING — CDC-ACM, captured off the Debug Probe.
    // bRequest 0x20 is meaningless as a standard request, so the kind bits are
    // what stop this module answering it.
    const s = Setup.parse(.{ 0x21, 0x20, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00 });
    try testing.expectEqual(RequestKind.class, s.kind());
    try testing.expect(!s.dataIn());
    try testing.expectEqual(@as(u16, 1), s.wIndex); // interface 1
    try testing.expectEqual(@as(u16, 7), s.wLength);
}

test "string descriptors are UTF-16LE with a length byte" {
    const d = stringDescriptor("Zig");
    try testing.expectEqual(@as(usize, 8), d.len);
    try testing.expectEqual(@as(u8, 8), d[0]);
    try testing.expectEqual(@backingInt(DescriptorType.string), d[1]);
    try testing.expectEqualSlices(u8, &.{ 'Z', 0, 'i', 0, 'g', 0 }, d[2..]);
}

test "string descriptor zero carries the language id" {
    const d = languageDescriptor(lang_id_en_us);
    try testing.expectEqualSlices(u8, &.{ 4, 3, 0x09, 0x04 }, &d);
}
