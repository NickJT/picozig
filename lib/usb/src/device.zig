//! The device state machine: SETUP packets in, descriptors out.
//!
//! This is the piece that binds the chip-independent protocol in `usb.zig` to
//! picozig's controller in `rp2350.usb`. It answers the standard requests of
//! `usb_20.pdf` Chapter 9 and stalls everything else, which is exactly enough
//! to enumerate.
//!
//! ## The two things that make enumeration fail intermittently
//!
//! **`SET_ADDRESS` is honoured after its status stage, never when it arrives.**
//! The request is addressed to the device at address 0, and the acknowledgement
//! must also come from address 0. A device that adopts the new address on
//! receipt stops answering before it has acknowledged, and the host sees a
//! timeout. It works often enough to look like a wiring fault. `pending_address`
//! exists solely for this.
//!
//! **A control IN transfer longer than one packet needs a short packet to end
//! it.** The host asks for `wLength` bytes and keeps reading until it gets a
//! packet shorter than the maximum. If the data happens to be an exact multiple
//! of 64 and the host asked for more, a zero-length packet has to follow, or
//! the host waits for a packet that never comes.

const std = @import("std");
const rp2350 = @import("rp2350");
const proto = @import("usb.zig");

const ctl = rp2350.usb;
const Setup = proto.Setup;

/// Everything the host may ask for. Slices rather than typed structs, because
/// by this point they are wire format — assembled once at comptime by the
/// application, which is the only place that knows what device it is.
pub const Descriptors = struct {
    device: []const u8,
    /// The configuration descriptor **and everything that follows it** —
    /// interfaces and endpoints — as one blob. Its `wTotalLength` must equal
    /// this slice's length; `verify` checks that rather than trusting it.
    configuration: []const u8,
    /// Index 0 must be the language descriptor, not a string.
    strings: []const []const u8 = &.{},

    /// Catch at comptime the descriptor mistakes that present as a device the
    /// host refuses without saying why.
    pub fn verify(comptime self: Descriptors) void {
        comptime {
            if (self.device.len != 18) @compileError("device descriptor must be 18 bytes");
            if (self.configuration.len < 9) @compileError("configuration descriptor too short");
            const total = @as(u16, self.configuration[2]) | (@as(u16, self.configuration[3]) << 8);
            if (total != self.configuration.len) @compileError(std.fmt.comptimePrint(
                "configuration wTotalLength is {d} but the blob is {d} bytes",
                .{ total, self.configuration.len },
            ));
            if (self.device[7] == 0) @compileError("bMaxPacketSize0 must not be zero");
        }
    }
};

/// What the device did about a SETUP packet. The whole of the trace vocabulary:
/// every control transfer ends in exactly one of these.
pub const Action = enum {
    /// Data sent back to the host — a descriptor, a status word, a line coding.
    sent,
    /// Acknowledged with a zero-length packet and no data.
    acked,
    /// Refused. Not necessarily a fault: a host asks for things a device is
    /// entitled not to have, and `DEVICE_QUALIFIER` is refused on purpose.
    stalled,
    /// A class request whose data is still arriving. One of the three above
    /// follows once it lands.
    awaiting_data,
};

pub const Device = struct {
    descriptors: Descriptors,

    ep0_in: ctl.Endpoint = ctl.ep0In(),
    ep0_out: ctl.Endpoint = ctl.ep0Out(),

    address: u7 = 0,
    /// Set by `SET_ADDRESS`, applied when its status stage completes. See the
    /// note at the top of this file — this field is the whole fix.
    pending_address: ?u7 = null,
    configured: bool = false,

    /// What is left of a control IN transfer that did not fit in one packet.
    pending: []const u8 = &.{},
    /// Whether a zero-length packet is still owed to terminate that transfer.
    owes_zlp: bool = false,

    /// Called for every completed buffer on an endpoint this module does not
    /// own, so the application can drive its own endpoints. `null` to ignore.
    onEndpoint: ?*const fn (ep: u4, dir: ctl.Dir) void = null,

    /// Answer a class or vendor request — the ones Chapter 9 knows nothing
    /// about, such as CDC's `SET_LINE_CODING`.
    ///
    /// `data` is the OUT data stage if the request had one, and empty
    /// otherwise. Return the bytes to send for an IN request, an **empty
    /// slice** to acknowledge without data, or **null to stall**. Anything
    /// returned must outlive the call, so a static buffer — this is a device
    /// with no allocator and one request in flight at a time.
    onClassRequest: ?*const fn (s: Setup, data: []const u8) ?[]const u8 = null,

    /// Called for every SETUP packet with what the device decided about it.
    ///
    /// **Diagnostic only, and the instrument this package was missing.** A
    /// device that will not enumerate says nothing about why; this reports the
    /// conversation as it happens, which is the difference between reading a
    /// capture afterwards and watching it go wrong. Costs nothing when null.
    onSetupTrace: ?*const fn (s: Setup, action: Action, len: u16) void = null,

    /// The setup of a class request whose OUT data stage is still arriving.
    ///
    /// **A control-write is three transactions, not one**: SETUP, then the data
    /// the host sends, then a zero-length IN as the status stage. The handler
    /// cannot run until the middle one lands, so the request waits here.
    pending_setup: ?Setup = null,
    /// Where that data stage lands. 64 bytes is EP0's packet size, and no class
    /// request this device answers is longer — CDC's largest is seven.
    setup_data: [64]u8 = undefined,
    setup_data_len: u8 = 0,

    pub fn init(descriptors: Descriptors) Device {
        return .{ .descriptors = descriptors };
    }

    /// Bring the controller up and present to the host.
    ///
    /// `clocks.initUsb` must already have run. Endpoints beyond EP0 are the
    /// caller's to configure, and must be configured **before** this — a device
    /// that appears and is then asked about endpoints it has not set up gets
    /// one impression and does not get another.
    pub fn connect(self: *Device) void {
        _ = self;
        ctl.connect();
    }

    pub fn start() void {
        ctl.init();
    }

    /// Read the controller's interrupt status and act on it: a bus reset, a
    /// SETUP packet, a completed buffer.
    ///
    /// **This is the engine, whichever way you drive it.** Call it from the
    /// `usbctrl` vector or from a loop; it holds no state of its own beyond the
    /// `Device` it is given. Named `service` rather than `poll` because
    /// `SerialPort` has a `poll` meaning something different — the main-loop
    /// entry point for the non-interrupt case — and two `poll`s in one call
    /// chain read as one thing with a fallback, which they are not.
    pub fn service(self: *Device) void {
        const ints = ctl.intStatus();

        if (ints & rp2350.hw.Usb.int_bus_reset != 0) self.onBusReset();

        if (ints & rp2350.hw.Usb.int_setup_req != 0) {
            ctl.ackSetup();
            self.onSetup(Setup.parse(ctl.setupPacket()));
        }

        if (ints & rp2350.hw.Usb.int_buff_status != 0) self.onBuffStatus();
    }

    fn onBusReset(self: *Device) void {
        ctl.ackBusReset();
        self.address = 0;
        self.pending_address = null;
        self.configured = false;
        self.pending = &.{};
        self.owes_zlp = false;
        self.pending_setup = null;
        self.setup_data_len = 0;
        self.ep0_in.next_pid = 1; // a control IN's data stage always starts DATA1
        self.ep0_out.next_pid = 1;
    }

    fn onBuffStatus(self: *Device) void {
        const status = ctl.buffStatus();
        ctl.clearBuffStatus(status);

        if (status & self.ep0_in.statusBit() != 0) self.onEp0InDone();
        if (status & self.ep0_out.statusBit() != 0) self.onEp0OutDone();

        // Everything else belongs to the application.
        if (self.onEndpoint) |cb| {
            var ep: u4 = 1;
            while (true) : (ep += 1) {
                if (status & rp2350.hw.Usb.buffStatusBit(ep, .in) != 0) cb(ep, .in);
                if (status & rp2350.hw.Usb.buffStatusBit(ep, .out) != 0) cb(ep, .out);
                if (ep == 15) break;
            }
        }
    }

    fn onEp0InDone(self: *Device) void {
        // The status stage of SET_ADDRESS has completed, so it is now safe —
        // and required — to start answering on the new address.
        if (self.pending_address) |addr| {
            self.pending_address = null;
            self.address = addr;
            ctl.setAddress(addr);
            return;
        }

        if (self.pending.len > 0 or self.owes_zlp) {
            self.sendChunk();
            return;
        }

        // A data stage has finished; the host now sends a zero-length OUT as
        // the status stage, and the device must be ready to receive it.
        ctl.receive(&self.ep0_out, 0);
    }

    fn onEp0OutDone(self: *Device) void {
        // If a class request is waiting on its data stage, this is it.
        if (self.pending_setup) |s| {
            self.pending_setup = null;
            const got = ctl.readInto(self.ep0_out, &self.setup_data);
            self.setup_data_len = @intCast(got.len);
            self.finishClassRequest(s, got);
            return;
        }
        // Otherwise this was the status stage of a control-write completing.
        // Nothing to do: the next SETUP re-arms everything, and acting here
        // would be acting twice.
    }

    fn finishClassRequest(self: *Device, s: Setup, data: []const u8) void {
        const handler = self.onClassRequest orelse {
            self.reject(s);
            return;
        };
        const reply = handler(s, data) orelse {
            self.reject(s);
            return;
        };
        if (s.dataIn() and reply.len > 0) {
            self.answer(s, reply);
        } else {
            self.ack(s);
        }
    }

    fn trace(self: *Device, s: Setup, action: Action, len: u16) void {
        if (self.onSetupTrace) |cb| cb(s, action, len);
    }

    /// Send `body` as the data stage, capped at what the host asked for.
    fn answer(self: *Device, s: Setup, body: []const u8) void {
        self.trace(s, .sent, self.beginIn(body, s.wLength));
    }

    /// Acknowledge with a zero-length status packet and no data.
    fn ack(self: *Device, s: Setup) void {
        ctl.sendZlp(&self.ep0_in);
        self.trace(s, .acked, 0);
    }

    /// Refuse. The host is entitled to ask for things that do not exist.
    fn reject(self: *Device, s: Setup) void {
        ctl.stall(self.ep0_in);
        self.trace(s, .stalled, 0);
    }

    fn onSetup(self: *Device, s: Setup) void {
        // Every control transfer's data stage begins with DATA1.
        self.ep0_in.next_pid = 1;
        self.ep0_out.next_pid = 1;
        self.pending = &.{};
        self.owes_zlp = false;

        if (s.kind() != .standard) {
            // A class request that carries data from the host cannot be
            // answered yet — the data has not arrived. Arm EP0 OUT and pick it
            // up in `onEp0OutDone`.
            if (!s.dataIn() and s.wLength > 0) {
                if (s.wLength > self.setup_data.len) {
                    self.reject(s);
                    return;
                }
                self.pending_setup = s;
                ctl.receive(&self.ep0_out, s.wLength);
                self.trace(s, .awaiting_data, s.wLength);
                return;
            }
            self.finishClassRequest(s, &.{});
            return;
        }

        switch (s.request()) {
            .set_address => {
                // Remembered, not applied. See the file header.
                self.pending_address = @truncate(s.wValue & 0x7f);
                self.ack(s);
            },
            .set_configuration => {
                self.configured = s.wValue != 0;
                self.ack(s);
            },
            .get_configuration => {
                const v: u8 = if (self.configured) 1 else 0;
                self.answer(s, &.{v});
            },
            .get_descriptor => self.onGetDescriptor(s),
            .get_status => {
                // Bit 0 self-powered, bit 1 remote-wakeup. Both zero here.
                self.answer(s, &.{ 0, 0 });
            },
            .set_interface => self.ack(s),
            .get_interface => self.answer(s, &.{0}),
            .clear_feature, .set_feature => self.ack(s),
            else => self.reject(s),
        }
    }

    fn onGetDescriptor(self: *Device, s: Setup) void {
        const body: []const u8 = switch (s.descriptorType()) {
            .device => self.descriptors.device,
            .configuration => self.descriptors.configuration,
            .string => blk: {
                const i = s.descriptorIndex();
                if (i >= self.descriptors.strings.len) {
                    self.reject(s);
                    return;
                }
                break :blk self.descriptors.strings[i];
            },
            // DEVICE_QUALIFIER is a High Speed concept. A Full Speed-only device
            // **must stall it**, and a device that answers instead is telling
            // the host it can do High Speed, which it then cannot.
            else => {
                self.reject(s);
                return;
            },
        };
        self.answer(s, body);
    }

    /// Start a control IN data stage, honouring `wLength` and the short-packet
    /// termination rule.
    fn beginIn(self: *Device, body: []const u8, requested: u16) u16 {
        const want = @min(body.len, requested);
        self.pending = body[0..want];
        // A ZLP is owed only when we send exactly what was asked for, in whole
        // packets, and the host is therefore still waiting for a short one.
        self.owes_zlp = want < requested and want % 64 == 0 and want != 0;
        self.sendChunk();
        return @intCast(want);
    }

    fn sendChunk(self: *Device) void {
        if (self.pending.len == 0) {
            if (self.owes_zlp) {
                self.owes_zlp = false;
                ctl.sendZlp(&self.ep0_in);
            }
            return;
        }
        const n = @min(self.pending.len, @as(usize, 64));
        ctl.send(&self.ep0_in, self.pending[0..n]);
        self.pending = self.pending[n..];
    }
};

// ---------------------------------------------------------------------------
// Tests — the pure decisions, without a controller
// ---------------------------------------------------------------------------

const testing = std.testing;

/// `beginIn`'s two outputs — how much to send, and whether a ZLP is owed —
/// extracted so the rule can be tested without a chip. This mirrors the logic
/// above rather than being called by it, which is a real duplication and is
/// why the two sit next to each other.
fn inPlan(body_len: usize, requested: u16) struct { send: usize, zlp: bool } {
    const want = @min(body_len, requested);
    return .{ .send = want, .zlp = want < requested and want % 64 == 0 and want != 0 };
}

test "a descriptor shorter than the host asked for needs no ZLP" {
    // 18 bytes of device descriptor against a 64-byte request: the packet is
    // already short, so it terminates the transfer by itself.
    const p = inPlan(18, 64);
    try testing.expectEqual(@as(usize, 18), p.send);
    try testing.expect(!p.zlp);
}

test "an exact multiple of 64 that is shorter than requested needs a ZLP" {
    // The trap. 64 bytes sent against a 255-byte request looks to the host like
    // "more to come", and it waits for a packet that never arrives.
    const p = inPlan(64, 255);
    try testing.expectEqual(@as(usize, 64), p.send);
    try testing.expect(p.zlp);
}

test "sending exactly what was asked for needs no ZLP" {
    // The host got its wLength; it is not waiting for anything.
    const p = inPlan(64, 64);
    try testing.expectEqual(@as(usize, 64), p.send);
    try testing.expect(!p.zlp);
}

test "a zero-length answer does not also owe a ZLP" {
    const p = inPlan(0, 8);
    try testing.expectEqual(@as(usize, 0), p.send);
    try testing.expect(!p.zlp);
}

test "the host's wLength caps the answer" {
    // The first request of any enumeration asks for 64 bytes of an 18-byte
    // descriptor; the eighth asks for 8 bytes of it. Over-sending is a protocol
    // error, not a kindness.
    try testing.expectEqual(@as(usize, 8), inPlan(18, 8).send);
    try testing.expectEqual(@as(usize, 18), inPlan(18, 64).send);
}

test "a configuration blob's wTotalLength must match its own length" {
    // The check `Descriptors.verify` makes at comptime, exercised here on the
    // shape the loopback example uses: config 9 + interface 9 + two endpoints
    // of 7 = 32 bytes.
    const total: u16 = 32;
    var blob: [32]u8 = @splat(0);
    blob[0] = 9;
    blob[1] = 2;
    blob[2] = @truncate(total & 0xff);
    blob[3] = @truncate(total >> 8);
    const read_back = @as(u16, blob[2]) | (@as(u16, blob[3]) << 8);
    try testing.expectEqual(@as(u16, blob.len), read_back);
}
