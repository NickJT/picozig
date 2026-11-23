//! `SerialPort` — a USB serial port in about five lines of application code.
//!
//! ```zig
//! const Port = usb.SerialPort(.{ .product = "my thing" });
//! var port: Port = .{};
//!
//! pub fn main() noreturn {
//!     rp2350.clocks.init(plan);
//!     port.start(plan.sys_hz);
//!     var buf: [256]u8 = undefined;
//!     var out = port.writer(&buf);
//!     while (true) out.print("t={d}\n", .{t}) catch {};
//! }
//!
//! pub const interrupts = struct {
//!     pub fn usbctrl() callconv(.c) void { port.onIrq(); }
//! };
//! ```
//!
//! **This is the whole of what most applications should need.** Descriptors,
//! endpoints, the class requests and the control transfers are all underneath.
//! If you need to change any of them, drop to `Device` + `acm` directly — that
//! path stays open and `example/serial` shows it.
//!
//! ## It never blocks, and it drops rather than buffering when nobody is there
//!
//! Two decisions worth stating, because they are the ones that differ from a
//! UART and they are not negotiable defaults chosen at random.
//!
//! **`write` never blocks.** A UART writes into the void quite happily; a USB
//! device with no host cannot. Blocking would mean a battery-powered board
//! hanging because nobody plugged a cable in, which is a far worse failure than
//! losing a log line. `write` returns how much it accepted and `dropped` counts
//! the rest.
//!
//! **Output produced while the port is closed is discarded, not queued.** The
//! tempting alternative — buffer it and deliver on open — means the first thing
//! a terminal shows is history, and history that looks like the present is how
//! an afternoon gets lost. This workspace has already paid for that once: a
//! serial backlog that survived a reflash passed the old firmware's output off
//! as the new one's. If early-boot output
//! matters, print it to a UART, which is what a UART is good at.

const std = @import("std");
const rp2350 = @import("rp2350");
const proto = @import("usb.zig");
const acm = @import("acm.zig");
const cdc = @import("cdc.zig");
const dev_mod = @import("device.zig");

const ctl = rp2350.usb;

/// A byte queue between `write` and the endpoint that drains it.
///
/// **Single producer, single consumer, and that is a correctness requirement
/// rather than a description.** Once servicing runs in the `usbctrl` interrupt,
/// each ring is touched from two contexts at once:
///
/// | Ring | Producer | Consumer |
/// |---|---|---|
/// | `tx` | `write`, in thread context | the interrupt |
/// | `rx` | the interrupt | `read`, in thread context |
///
/// So `push` writes only `head` and `pop` writes only `tail`, and neither ever
/// touches the other's index. **An earlier version kept a shared `count` that
/// both incremented**, which is fine while polled and a race the moment an
/// interrupt is involved — the sort that costs a byte once an hour and looks
/// like a host problem.
///
/// One slot is always left empty so that `head == tail` means empty and never
/// full; that is the price of not needing a third variable both sides write.
pub fn Ring(comptime capacity: usize) type {
    return struct {
        const Self = @This();
        const slots = capacity + 1;

        buf: [slots]u8 = undefined,
        /// Written only by `push`.
        head: usize = 0,
        /// Written only by `pop`.
        tail: usize = 0,

        pub fn len(self: *const Self) usize {
            const h = @atomicLoad(usize, &self.head, .acquire);
            const t = @atomicLoad(usize, &self.tail, .acquire);
            return (h + slots - t) % slots;
        }

        pub fn free(self: *const Self) usize {
            return capacity - self.len();
        }

        pub fn isEmpty(self: *const Self) bool {
            return @atomicLoad(usize, &self.head, .acquire) ==
                @atomicLoad(usize, &self.tail, .acquire);
        }

        /// Append what fits. Returns how many bytes were taken — **which may be
        /// fewer than offered, and that is the caller's to notice.**
        pub fn push(self: *Self, bytes: []const u8) usize {
            const t = @atomicLoad(usize, &self.tail, .acquire);
            var h = self.head;
            var n: usize = 0;
            for (bytes) |b| {
                const next = (h + 1) % slots;
                if (next == t) break; // full
                self.buf[h] = b;
                h = next;
                n += 1;
            }
            // Publish the new head only after the bytes are in place.
            @atomicStore(usize, &self.head, h, .release);
            return n;
        }

        /// Remove up to `dest.len` bytes. Returns how many.
        pub fn pop(self: *Self, dest: []u8) usize {
            const h = @atomicLoad(usize, &self.head, .acquire);
            var t = self.tail;
            var n: usize = 0;
            while (n < dest.len and t != h) {
                dest[n] = self.buf[t];
                t = (t + 1) % slots;
                n += 1;
            }
            @atomicStore(usize, &self.tail, t, .release);
            return n;
        }
    };
}

pub const Config = struct {
    /// **Placeholders.** Not registered to anyone; a shipped product needs its
    /// own, and borrowing a real vendor's would be worse than using zero.
    vendor_id: u16 = 0x0000,
    product_id: u16 = 0x0002,
    manufacturer: []const u8 = "picozig",
    product: []const u8 = "picozig serial",
    max_power_ma: u16 = 100,
    endpoints: acm.Endpoints = .{},
    /// How much output may be in flight before `write` starts dropping.
    tx_bytes: usize = 512,
    /// How much input may arrive before it is read.
    rx_bytes: usize = 256,
    /// Service USB from the `usbctrl` interrupt rather than from a call in the
    /// main loop.
    ///
    /// **On by default, and it changes what the application must do**: the
    /// vector belongs to the root module, so the application forwards it —
    ///
    /// ```zig
    /// pub const interrupts = struct {
    ///     pub fn usbctrl() callconv(.c) void { port.onIrq(); }
    /// };
    /// ```
    ///
    /// Forgetting that is not silent. `boot.zig` points every unclaimed vector
    /// at a trap that records which one fired, so a missing forward is a fault
    /// with a number on it rather than a port that quietly stops answering.
    interrupt_driven: bool = true,
};

pub fn SerialPort(comptime cfg: Config) type {
    return struct {
        const Self = @This();

        const device_desc = proto.DeviceDescriptor{
            .bcdUSB = 0x0200,
            // A single-function CDC device declares the class here, so the host
            // pairs the two interfaces without an association descriptor.
            .bDeviceClass = cdc.comm_interface_class,
            .bDeviceSubClass = 0,
            .bDeviceProtocol = 0,
            .bMaxPacketSize0 = 64,
            .idVendor = cfg.vendor_id,
            .idProduct = cfg.product_id,
            .bcdDevice = 0x0100,
            .iManufacturer = 1,
            .iProduct = 2,
            .iSerialNumber = 0,
            .bNumConfigurations = 1,
        };

        const device_bytes = device_desc.bytes();
        const config_bytes = acm.configuration(cfg.endpoints, cfg.max_power_ma);
        const lang_bytes = proto.languageDescriptor(proto.lang_id_en_us);
        const mfr_bytes = proto.stringDescriptor(cfg.manufacturer);
        const product_bytes = proto.stringDescriptor(cfg.product);

        const descriptors = dev_mod.Descriptors{
            .device = &device_bytes,
            .configuration = &config_bytes,
            .strings = &.{ &lang_bytes, &mfr_bytes, &product_bytes },
        };

        comptime {
            descriptors.verify();
        }

        /// The one port this program has. **A singleton, on purpose**: the
        /// callbacks in `Device` carry no context, and one USB serial port per
        /// program is the case this whole package is narrow for. Anything more
        /// composite belongs on `Device` + `acm` directly.
        var active: ?*Self = null;

        dev: dev_mod.Device = dev_mod.Device.init(descriptors),
        line: acm.State = .{},

        ep_notify: ctl.Endpoint = .{ .num = cfg.endpoints.notify, .dir = .in, .transfer = .interrupt, .buf = 0, .max_packet = 8 },
        ep_out: ctl.Endpoint = .{ .num = cfg.endpoints.data_out, .dir = .out, .transfer = .bulk, .buf = 0 },
        ep_in: ctl.Endpoint = .{ .num = cfg.endpoints.data_in, .dir = .in, .transfer = .bulk, .buf = 0 },

        tx: Ring(cfg.tx_bytes) = .{},
        rx: Ring(cfg.rx_bytes) = .{},
        /// Bytes `write` could not take. **Read it.** A silent drop is the
        /// failure this counter exists to make visible.
        dropped: u32 = 0,
        /// Whether a packet is with the controller awaiting the host.
        tx_busy: bool = false,
        packet: [64]u8 = undefined,

        /// Bring up clk_usb and the controller, then present to the host.
        ///
        /// `sys_hz` is the board's clk_sys and is **checked, not used** —
        /// RP2350-E12 requires it above 48 MHz and `clocks.initUsb` refuses to
        /// compile below that.
        pub fn start(self: *Self, comptime sys_hz: u32) void {
            active = self;
            rp2350.clocks.initUsb(sys_hz);
            dev_mod.Device.start();

            var alloc = ctl.BufferAllocator{};
            self.ep_notify.buf = alloc.alloc(8).?;
            self.ep_out.buf = alloc.alloc(64).?;
            self.ep_in.buf = alloc.alloc(64).?;
            ctl.configure(self.ep_notify);
            ctl.configure(self.ep_out);
            ctl.configure(self.ep_in);

            self.dev.onEndpoint = &onEndpoint;
            self.dev.onClassRequest = &onClassRequest;

            if (cfg.interrupt_driven) {
                // Enabled *before* the pull-up: the host may begin asking
                // questions the instant the device appears, and an interrupt
                // armed afterwards would miss the first of them.
                rp2350.nvic.enable(rp2350.nvic.IrqNum.usbctrl);
            }

            self.dev.connect();
            ctl.receive(&self.ep_out, 64);
        }

        /// Service the controller from the `usbctrl` vector. Forward it from
        /// the application's `interrupts` namespace — see `Config`.
        pub fn onIrq(self: *Self) void {
            self.dev.service();
            self.serviceTx();
        }

        /// Service the controller from the main loop.
        ///
        /// **Only for `interrupt_driven = false`.** Calling this as well as
        /// forwarding the vector puts two consumers on the TX ring, which is
        /// exactly what `Ring`'s single-consumer rule forbids.
        pub fn poll(self: *Self) void {
            comptime {
                if (cfg.interrupt_driven) @compileError(
                    "poll() is for interrupt_driven = false. This port is " ++
                        "interrupt-driven: forward the usbctrl vector to onIrq() instead.",
                );
            }
            self.dev.service();
            self.serviceTx();
        }

        /// Whether a terminal has the port open — `DTR`, which is the only
        /// signal a host gives that anything is listening.
        pub fn isOpen(self: *const Self) bool {
            return self.dev.configured and self.line.dtr;
        }

        /// Queue `bytes`. Returns how many were taken; never blocks.
        pub fn write(self: *Self, bytes: []const u8) usize {
            if (!self.isOpen()) {
                self.dropped +%= @intCast(bytes.len);
                return 0;
            }
            const n = self.tx.push(bytes);
            self.dropped +%= @intCast(bytes.len - n);
            self.kick();
            return n;
        }

        /// Start the queued bytes moving.
        ///
        /// **Interrupt-driven, this pends the interrupt rather than servicing
        /// directly**, so that the handler stays the only consumer of the TX
        /// ring. Doing the work here instead would be a second consumer racing
        /// the first, and the symptom would be a byte lost now and then.
        fn kick(self: *Self) void {
            if (cfg.interrupt_driven) {
                rp2350.nvic.setPending(rp2350.nvic.IrqNum.usbctrl);
            } else {
                self.serviceTx();
            }
        }

        /// Take what has arrived. Returns the bytes written into `dest`.
        pub fn read(self: *Self, dest: []u8) []u8 {
            return dest[0..self.rx.pop(dest)];
        }

        fn serviceTx(self: *Self) void {
            if (self.tx_busy or self.tx.isEmpty()) return;
            if (!self.dev.configured) return;
            const n = self.tx.pop(&self.packet);
            self.tx_busy = true;
            ctl.send(&self.ep_in, self.packet[0..n]);
        }

        fn onEndpoint(ep: u4, dir: ctl.Dir) void {
            const self = active orelse return;
            if (ep == cfg.endpoints.data_out and dir == .out) {
                var tmp: [64]u8 = undefined;
                const got = ctl.readInto(self.ep_out, &tmp);
                _ = self.rx.push(got); // a full rx ring drops, as write does
                ctl.receive(&self.ep_out, 64);
            } else if (ep == cfg.endpoints.data_in and dir == .in) {
                self.tx_busy = false;
                self.serviceTx();
            }
        }

        fn onClassRequest(s: proto.Setup, data: []const u8) ?[]const u8 {
            const self = active orelse return null;
            return self.line.onClassRequest(s, data);
        }

        // -------------------------------------------------------------------
        // std.Io.Writer
        // -------------------------------------------------------------------

        /// Wrap this port as a `std.Io.Writer`, the same shape as
        /// `rp2350.uart.writerFor`, so code can take a writer and not care
        /// which it got.
        ///
        /// **Pass a real buffer.** A zero-length one makes `std.Io.Writer` call
        /// `drain` once per formatted fragment — see `uart.writerFor`, where
        /// that cost a bench session.
        pub fn writer(self: *Self, buffer: []u8) std.Io.Writer {
            active = self;
            return .{ .buffer = buffer, .vtable = &vtable };
        }

        const vtable: std.Io.Writer.VTable = .{ .drain = drain };

        /// The `std.Io.Writer` drain contract, copied in shape from
        /// `uart.drainWith` — including the part that matters: bytes already in
        /// `w.buffer` are emitted first and **do not count** toward the return
        /// value, because the caller counted them when they went in.
        fn drain(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
            const self = active orelse return error.WriteFailed;

            if (w.end > 0) {
                _ = self.write(w.buffer[0..w.end]);
                w.end = 0;
            }

            var written: usize = 0;
            for (data[0 .. data.len - 1]) |slice| {
                _ = self.write(slice);
                written += slice.len;
            }

            // The last element repeats `splat` times, "which may be zero" — so
            // a splat of 0 writes it no times and counts it not at all.
            const last = data[data.len - 1];
            var i: usize = 0;
            while (i < splat) : (i += 1) {
                _ = self.write(last);
                written += last.len;
            }
            return written;
        }
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the ring takes what fits and reports what it took" {
    var r = Ring(8){};
    try testing.expectEqual(@as(usize, 8), r.free());
    try testing.expectEqual(@as(usize, 5), r.push("hello"));
    try testing.expectEqual(@as(usize, 5), r.len());
    // Only three bytes of room left, so only three are taken -- and the caller
    // is told, which is what makes the drop visible rather than silent.
    try testing.expectEqual(@as(usize, 3), r.push("world"));
    try testing.expectEqual(@as(usize, 0), r.free());
    try testing.expectEqual(@as(usize, 0), r.push("more"));
}

test "the ring returns bytes in order across a wrap" {
    var r = Ring(8){};
    _ = r.push("abcdef");
    var dst: [4]u8 = undefined;
    try testing.expectEqual(@as(usize, 4), r.pop(&dst));
    try testing.expectEqualSlices(u8, "abcd", &dst);
    // Now push past the end of the backing array, so head and tail straddle it.
    try testing.expectEqual(@as(usize, 6), r.push("ghijkl"));
    var all: [8]u8 = undefined;
    const n = r.pop(&all);
    try testing.expectEqualSlices(u8, "efghijkl", all[0..n]);
    try testing.expect(r.isEmpty());
}

test "popping an empty ring yields nothing rather than stale bytes" {
    var r = Ring(4){};
    var dst: [4]u8 = undefined;
    try testing.expectEqual(@as(usize, 0), r.pop(&dst));
    _ = r.push("ab");
    try testing.expectEqual(@as(usize, 2), r.pop(&dst));
    try testing.expectEqual(@as(usize, 0), r.pop(&dst));
}

test "a ring survives many wraps without drifting" {
    // The modulo arithmetic is the part that rots quietly: an off-by-one shows
    // up only after the indices have gone round.
    var r = Ring(5){};
    var dst: [3]u8 = undefined;
    var expect: u8 = 0;
    for (0..200) |i| {
        const b: u8 = @intCast(i % 251);
        try testing.expectEqual(@as(usize, 1), r.push(&.{b}));
        if (r.len() == 3) {
            _ = r.pop(&dst);
            for (dst) |got| {
                try testing.expectEqual(expect % 251, got);
                expect +%= 1;
            }
        }
    }
}
