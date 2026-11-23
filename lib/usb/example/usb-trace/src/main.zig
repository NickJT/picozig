//! **usb-trace — watch a USB device being enumerated.**
//!
//! This is a CDC-ACM serial port exactly like `example/serial`, with one thing
//! added: it narrates every control transfer over the *SWD* UART as it happens.
//! Plug the board into a host, watch the probe's console, and the whole
//! conversation goes past.
//!
//! ```
//! SETUP 80 06 0100 0000 len=64  GET_DESCRIPTOR device         -> sent 18
//! SETUP 00 05 0003 0000 len=0   SET_ADDRESS 3                 -> acked
//! SETUP 80 06 0200 0000 len=67  GET_DESCRIPTOR configuration  -> sent 67
//! SETUP 21 20 0000 0000 len=7   [class] SET_LINE_CODING       -> awaiting data
//! SETUP 21 20 0000 0000 len=7   [class] SET_LINE_CODING       -> acked
//! ```
//!
//! ## Who this is for
//!
//! **Not the application writer.** If you want a serial port, take
//! `example/serial` — it is shorter and this one's chatter would only be in the
//! way. This is for understanding what the layer underneath is doing, and for
//! the day a device of your own will not come up.
//!
//! ## What the trace teaches that prose does not
//!
//! **Hosts do not enumerate the same way, and this is how you find that out.**
//! Two real sequences, both captured on this firmware:
//!
//! macOS 26.6 (read off the SWD console, 2026-08-29, after `reset run`):
//!
//! ```
//! SET_ADDRESS 3
//! GET_DESCRIPTOR device        len=8    <- just enough to learn bMaxPacketSize0
//! GET_DESCRIPTOR device        len=18   <- now the whole thing
//! GET_DESCRIPTOR string 2      len=2    <- product, length byte first
//! GET_DESCRIPTOR string 2      len=36
//! GET_DESCRIPTOR string 1      len=2    <- then manufacturer
//! GET_DESCRIPTOR string 1      len=16
//! GET_DESCRIPTOR configuration len=9    <- header, to learn wTotalLength
//! GET_DESCRIPTOR configuration len=67   <- then all of it
//! SET_CONFIGURATION 1
//! ```
//!
//! Linux 6.18 (`usbmon`, the BOOTSEL enumeration):
//!
//! ```
//! GET_DESCRIPTOR device        len=64   <- at address 0, before addressing
//! SET_ADDRESS
//! GET_DESCRIPTOR device        len=18
//! ...
//! ```
//!
//! Things worth taking from that:
//!
//! * **The "read a prefix, then read it all" pattern is everywhere** — device
//!   descriptor, every string, the configuration. The short read exists to
//!   learn a length before committing to it. A device that answers the short
//!   read and botches the long one fails in a way that looks like "it almost
//!   worked".
//! * **The prefix length is the host's choice, not the specification's.** Linux
//!   asks for 64, macOS for 8. Both are legal; anything that assumed one of
//!   them would work on one machine and not the other.
//! * **`SET_ADDRESS` is acked from the OLD address.** The reply goes out as
//!   address 0; only when it completes does the device answer as the new one.
//!   Doing it the other way round mostly works, which is worse than never.
//! * **A control write appears twice**, as `awaiting data` then `acked` — the
//!   SETUP, the data the host then sends, and a zero-length status are three
//!   transactions. Open the port to see `SET_LINE_CODING` do this.
//! * **Stalls are not always faults.** `DEVICE_QUALIFIER` is refused on
//!   purpose: it is a High Speed question and this is a Full Speed device.
//!
//! ## `reset run` is not a cable pull
//!
//! The macOS trace above starts at `SET_ADDRESS` with **no address-0 descriptor
//! read at all**, because it followed an SWD `reset run` rather than a replug.
//! The host never saw a disconnect — only a device that went quiet briefly — so
//! it re-addressed one it already knew instead of enumerating from scratch.
//!
//! **Unplug and replug to see a full enumeration.** If you are comparing this
//! trace against a capture, compare like with like, or you will conclude the
//! device skipped a step it was never asked for.
//!
//! ## The tracer reports its own losses
//!
//! Each line checks `uart.dropped_bytes` and appends a warning if it moved. At
//! 115200 a line takes roughly 5 ms to leave the wire and a host can issue
//! control transfers faster than that, so a console that cannot keep up is a
//! real possibility. **A gap that announces itself is a fact; a gap that does
//! not is a wrong conclusion waiting to happen** — and this file's own header
//! was wrong about the macOS sequence until the trace showed otherwise.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const usb = @import("usb");

comptime {
    _ = &boot;
}

const ctl = rp2350.usb;

const plan = rp2350.clocks.Plan.require(150_000_000);
const sys_hz = plan.sys_hz;
const eps = usb.acm.Endpoints{ .notify = 1, .data_out = 2, .data_in = 3 };

const device_desc = usb.DeviceDescriptor{
    .bcdUSB = 0x0200,
    .bDeviceClass = usb.cdc.comm_interface_class,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0000,
    .idProduct = 0x0003,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

const config_blob = usb.acm.configuration(eps, 100);
const lang_desc = usb.languageDescriptor(usb.lang_id_en_us);
const mfr_desc = usb.stringDescriptor("picozig");
const product_desc = usb.stringDescriptor("picozig usb-trace");

const descriptors = usb.Descriptors{
    .device = &device_desc.bytes(),
    .configuration = &config_blob,
    .strings = &.{ &lang_desc, &mfr_desc, &product_desc },
};

comptime {
    descriptors.verify();
}

// ---------------------------------------------------------------------------
// Naming things, so the trace reads as a conversation
// ---------------------------------------------------------------------------
//
// Deliberately here and not in the library: an application never needs it, and
// `lib/usb` is kept lean for the people who only want a serial port.

fn requestName(s: usb.Setup) []const u8 {
    if (s.kind() == .class) {
        return switch (@as(usb.cdc.Request, @fromBackingInt(@intCast(s.bRequest)))) {
            .set_line_coding => "SET_LINE_CODING",
            .get_line_coding => "GET_LINE_CODING",
            .set_control_line_state => "SET_CONTROL_LINE_STATE",
            .send_break => "SEND_BREAK",
            _ => "unknown class request",
        };
    }
    if (s.kind() != .standard) return "vendor request";
    return switch (s.request()) {
        .get_status => "GET_STATUS",
        .clear_feature => "CLEAR_FEATURE",
        .set_feature => "SET_FEATURE",
        .set_address => "SET_ADDRESS",
        .get_descriptor => "GET_DESCRIPTOR",
        .set_descriptor => "SET_DESCRIPTOR",
        .get_configuration => "GET_CONFIGURATION",
        .set_configuration => "SET_CONFIGURATION",
        .get_interface => "GET_INTERFACE",
        .set_interface => "SET_INTERFACE",
        .synch_frame => "SYNCH_FRAME",
        _ => "unknown standard request",
    };
}

fn descriptorName(t: usb.DescriptorType) []const u8 {
    return switch (t) {
        .device => "device",
        .configuration => "configuration",
        .string => "string",
        .interface => "interface",
        .endpoint => "endpoint",
        .device_qualifier => "device_qualifier",
        .other_speed_configuration => "other_speed_configuration",
        .interface_power => "interface_power",
        _ => "unknown",
    };
}

fn actionName(a: usb.device.Action) []const u8 {
    return switch (a) {
        .sent => "sent",
        .acked => "acked",
        .stalled => "STALLED",
        .awaiting_data => "awaiting data",
    };
}

/// Bytes the UART dropped as of the previous trace line.
///
/// **A tracer that cannot keep up is worse than no tracer**, because it reports
/// a conversation with holes in it and does not say so. picozig's UART counts
/// what it could not fit in the TX FIFO (`uart.dropped_bytes`), and each line
/// below checks whether that number moved. At 115200 a trace line takes about
/// 5 ms to leave the wire, and a host can issue control transfers a good deal
/// faster than that — so this is a real risk, not a theoretical one.
var last_dropped: u32 = 0;

/// The whole point of this example.
fn onSetupTrace(s: usb.Setup, action: usb.device.Action, len: u16) void {
    const uart = rp2350.uart;
    uart.print(.uart0, "SETUP {x:0>2} {x:0>2} {x:0>4} {x:0>4} len={d:<3} ", .{
        s.bmRequestType, s.bRequest, s.wValue, s.wIndex, s.wLength,
    });
    if (s.kind() == .class) uart.print(.uart0, "[class] ", .{});
    uart.print(.uart0, "{s}", .{requestName(s)});

    // The detail that makes the line worth reading.
    switch (s.kind()) {
        .standard => switch (s.request()) {
            .get_descriptor => uart.print(.uart0, " {s}", .{descriptorName(s.descriptorType())}),
            .set_address, .set_configuration => uart.print(.uart0, " {d}", .{s.wValue}),
            else => {},
        },
        else => {},
    }

    if (action == .sent) {
        uart.print(.uart0, " -> {s} {d}", .{ actionName(action), len });
    } else {
        uart.print(.uart0, " -> {s}", .{actionName(action)});
    }

    // Say so if the console could not keep up. A gap in a trace that announces
    // itself is a fact; a gap that does not is a wrong conclusion waiting.
    const dropped = uart.dropped_bytes;
    if (dropped != last_dropped) {
        uart.print(.uart0, "   [!! {d} UART bytes dropped]", .{dropped - last_dropped});
        last_dropped = dropped;
    }
    uart.print(.uart0, "\n", .{});
}

// ---------------------------------------------------------------------------

var acm_state = usb.acm.State{};
var dev = usb.Device.init(descriptors);

var ep_notify = ctl.Endpoint{ .num = eps.notify, .dir = .in, .transfer = .interrupt, .buf = 0, .max_packet = 8 };
var ep_out = ctl.Endpoint{ .num = eps.data_out, .dir = .out, .transfer = .bulk, .buf = 0 };
var ep_in = ctl.Endpoint{ .num = eps.data_in, .dir = .in, .transfer = .bulk, .buf = 0 };

var packet: [64]u8 = undefined;
var in_flight: u32 = 0;

fn onClassRequest(s: usb.Setup, data: []const u8) ?[]const u8 {
    return acm_state.onClassRequest(s, data);
}

fn onEndpoint(ep: u4, dir: ctl.Dir) void {
    if (ep == eps.data_out and dir == .out) {
        const got = ctl.readInto(ep_out, &packet);
        rp2350.uart.print(.uart0, "BULK  out {d} bytes\n", .{got.len});
        in_flight = @intCast(got.len);
        ctl.send(&ep_in, got); // echo, so the port is usable while tracing
    } else if (ep == eps.data_in and dir == .in) {
        rp2350.uart.print(.uart0, "BULK  in  {d} bytes taken\n", .{in_flight});
        in_flight = 0;
        ctl.receive(&ep_out, 64);
    }
}

pub fn main() noreturn {
    rp2350.clocks.init(plan);
    rp2350.gpio.init();
    rp2350.uart.init(.uart0, .{
        .inst = .uart0,
        .tx = board.uart_tx,
        .rx = board.uart_rx,
        .clk_hz = sys_hz,
    });

    rp2350.uart.print(.uart0, "\n\n=== usb-trace ===\nclk_sys {d} Hz\n", .{sys_hz});
    rp2350.clocks.initUsb(sys_hz);
    rp2350.uart.print(.uart0, "clk_usb {d} kHz (48000 wanted)\n", .{
        rp2350.clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_usb) orelse 0,
    });

    usb.Device.start();

    var alloc = ctl.BufferAllocator{};
    ep_notify.buf = alloc.alloc(8).?;
    ep_out.buf = alloc.alloc(64).?;
    ep_in.buf = alloc.alloc(64).?;
    ctl.configure(ep_notify);
    ctl.configure(ep_out);
    ctl.configure(ep_in);

    dev.onEndpoint = &onEndpoint;
    dev.onClassRequest = &onClassRequest;
    dev.onSetupTrace = &onSetupTrace;

    rp2350.uart.print(.uart0, "presenting to the host...\n\n", .{});
    dev.connect();
    ctl.receive(&ep_out, 64);

    // Nothing is printed from here on except what the host causes, so the
    // console is the conversation and nothing else.
    while (true) dev.service();
}
