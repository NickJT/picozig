//! USB bulk loopback — rung 2's proof.
//!
//! Presents a device with two bulk endpoints, EP1 OUT and EP2 IN, and sends
//! back whatever it is given. **This is deliberately the same shape as the C
//! SDK's `dev_lowlevel`**, so its behaviour can be compared directly against a
//! reference that is known to work, and so `dev_lowlevel_loopback.py` drives it
//! unmodified.
//!
//! ## The clock is the example's own decision, and has to be
//!
//! `board.clock_plan` is `null` on both boards here — clk_sys runs from the
//! 12 MHz crystal, which `adc.zig` explains at length is the right default.
//! **USB cannot use it.** RP2350-E12 requires clk_sys above 48 MHz, and
//! `clocks.initUsb` refuses at compile time below that. So this example
//! declares 150 MHz for itself rather than changing a board file: needing a
//! fast clock is a property of doing USB, not of the board.
//!
//! ## Recovery
//!
//! Nothing here stops clk_sys or sleeps, so the board cannot wedge. If it ever
//! stops answering, the usual ladder applies — power cycle, then BOOTSEL and
//! any good UF2.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const usb = @import("usb");

comptime {
    _ = &boot;
}

const ctl = rp2350.usb;

/// 150 MHz — the SDK's default, comfortably past E12's 48 MHz floor, and a
/// frequency `Plan.forSysHz` has a test pinning.
const plan = rp2350.clocks.Plan.require(150_000_000);
const sys_hz = plan.sys_hz;

// ---------------------------------------------------------------------------
// Descriptors
// ---------------------------------------------------------------------------

/// **Vendor 0x0000, product 0x0001 are placeholders**, exactly as
/// `dev_lowlevel` uses. They are not registered to anyone and must not be
/// shipped; borrowing a real vendor's ID for a test device would be worse.
const device_desc = usb.DeviceDescriptor{
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0000,
    .idProduct = 0x0001,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

const ep1_out_addr = usb.endpointAddress(1, false);
const ep2_in_addr = usb.endpointAddress(2, true);

/// Configuration, interface and both endpoints as one blob — which is what the
/// host asks for, and what `wTotalLength` has to describe.
const config_blob = usb.ConfigurationDescriptor.bytes(.{
    .wTotalLength = 9 + 9 + 7 + 7,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bMaxPower = 50, // 100 mA, in 2 mA units
}) ++ usb.InterfaceDescriptor.bytes(.{
    .bInterfaceNumber = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0xff, // vendor-specific: no host driver claims it
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
}) ++ usb.EndpointDescriptor.bytes(.{
    .bEndpointAddress = ep1_out_addr,
    .bmAttributes = @backingInt(usb.TransferType.bulk),
    .wMaxPacketSize = 64,
    .bInterval = 0,
}) ++ usb.EndpointDescriptor.bytes(.{
    .bEndpointAddress = ep2_in_addr,
    .bmAttributes = @backingInt(usb.TransferType.bulk),
    .wMaxPacketSize = 64,
    .bInterval = 0,
});

const lang_desc = usb.languageDescriptor(usb.lang_id_en_us);
const mfr_desc = usb.stringDescriptor("picozig");
const product_desc = usb.stringDescriptor("USB loopback");

const descriptors = usb.Descriptors{
    .device = &device_desc.bytes(),
    .configuration = &config_blob,
    .strings = &.{ &lang_desc, &mfr_desc, &product_desc },
};

comptime {
    descriptors.verify();
}

// ---------------------------------------------------------------------------
// Self-test block, read over SWD at 0x20000000
// ---------------------------------------------------------------------------

pub const watch_magic: u32 = 0x05B10090;

pub const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// Rung reached: 1 clocks, 2 clk_usb, 3 controller up, 4 connected,
    /// 5 first SETUP seen, 6 configured, 7 first byte looped back.
    rung: u32,
    bus_resets: u32,
    setups: u32,
    configured: u32,
    address: u32,
    rx_packets: u32,
    tx_packets: u32,
    last_len: u32,
    clk_usb_khz: u32,
    err: u32,
};

export var watch: Watch linksection(".selftest") = undefined;

fn note(rung: u32) void {
    if (rung > watch.rung) watch.rung = rung;
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

var dev = usb.Device.init(descriptors);

var ep1_out = ctl.Endpoint{
    .num = 1,
    .dir = .out,
    .transfer = .bulk,
    .buf = 0, // filled in by the allocator at start-up
};

var ep2_in = ctl.Endpoint{
    .num = 2,
    .dir = .in,
    .transfer = .bulk,
    .buf = 0,
};

/// One packet in flight, which is all a 64-byte loopback needs.
var packet: [64]u8 = undefined;

fn onEndpoint(ep: u4, dir: ctl.Dir) void {
    if (ep == 1 and dir == .out) {
        const got = ctl.readInto(ep1_out, &packet);
        watch.rx_packets += 1;
        watch.last_len = @intCast(got.len);
        // Send it straight back on EP2 IN, then re-arm EP1 OUT for the next.
        ctl.send(&ep2_in, got);
        note(7);
    } else if (ep == 2 and dir == .in) {
        watch.tx_packets += 1;
        ctl.receive(&ep1_out, 64);
    }
}

pub fn main() noreturn {
    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;

    rp2350.clocks.init(plan);
    rp2350.gpio.init();
    note(1);

    rp2350.uart.init(.uart0, .{
        .inst = .uart0,
        .tx = board.uart_tx,
        .rx = board.uart_rx,
        .clk_hz = sys_hz,
    });
    rp2350.uart.print(.uart0, "\n\nusb-loopback: clk_sys {d} Hz\n", .{sys_hz});

    // clk_usb at 48 MHz. This is where RP2350-E12 is enforced: pass a sys_hz
    // at or below 48 MHz and the line below is a compile error, not a bug.
    rp2350.clocks.initUsb(sys_hz);
    note(2);

    watch.clk_usb_khz = rp2350.clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_usb) orelse 0;
    rp2350.uart.print(.uart0, "clk_usb measured {d} kHz (want 48000)\n", .{watch.clk_usb_khz});

    // Controller up, then endpoints, then present to the host — in that order.
    // A device that appears before its endpoints exist is asked about them
    // immediately and gets one chance to answer.
    usb.Device.start();
    note(3);

    var alloc = ctl.BufferAllocator{};
    ep1_out.buf = alloc.alloc(64).?;
    ep2_in.buf = alloc.alloc(64).?;
    ctl.configure(ep1_out);
    ctl.configure(ep2_in);

    dev.onEndpoint = &onEndpoint;
    dev.connect();
    note(4);

    // Armed to receive the host's first bulk OUT.
    ctl.receive(&ep1_out, 64);

    rp2350.uart.print(.uart0, "connected, polling\n", .{});

    var last_report: u32 = 0;
    while (true) {
        dev.service();

        watch.ticks +%= 1;
        // `bus_resets` and `setups` are left at zero deliberately: `Device`
        // keeps no counters, and a field that always reads zero is better
        // understood as absent than as a measurement saying nothing happened.
        watch.configured = if (dev.configured) 1 else 0;
        watch.address = dev.address;
        if (dev.configured) note(6);
        if (dev.address != 0) note(5);

        // A line a second or so, so a UART watcher sees progress.
        const now: u32 = @truncate(rp2350.timer.now64());
        if (now -% last_report > 1_000_000) {
            last_report = now;
            rp2350.uart.print(
                .uart0,
                "rung {d} addr {d} cfg {d} rx {d} tx {d} last {d}\n",
                .{ watch.rung, watch.address, watch.configured, watch.rx_packets, watch.tx_packets, watch.last_len },
            );
        }
    }
}
