//! UART debug output over the ARM PL011 (both UART0 and UART1).
//!
//! Requires `clocks.init()` first — the baud divisors are computed against
//! clk_peri, which that routine sources from clk_sys (the 12 MHz crystal).

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// Bytes dropped because the TX FIFO never drained.
///
/// **The console is the one place where reporting an error is the wrong
/// answer.** Every diagnostic in this workspace goes out over this path, so a
/// `uart` that returns errors would put a `try` on every print and make the
/// logging harder to use than the bug it is there to find. A `uart` that spins
/// forever is worse still: a stuck transmitter takes the whole firmware with
/// it, and the symptom is a board that stopped mid-line.
///
/// So the waits below are bounded and **drop the byte**, counting it here. A
/// console that loses output is a degraded console; one that hangs is a dead
/// board. Read this over SWD, or print it — if it is non-zero, the log above it
/// has holes and should not be read as complete.
pub var dropped_bytes: u32 = 0;
const gpio = @import("gpio.zig");

/// GPIO function 2 is UART on every pin that has one (function table, §9.4).
const func_uart: u32 = 2;

pub const default_baud: u32 = 115_200;

/// Which PL011 instance. Same shape as `spi.Instance` and `i2c.Instance`.
pub const Instance = enum(u1) {
    uart0 = 0,
    uart1 = 1,

    pub fn base(self: Instance) u32 {
        return switch (self) {
            .uart0 => hw.uart0_base,
            .uart1 => hw.uart1_base,
        };
    }

    pub fn resetBit(self: Instance) u32 {
        return switch (self) {
            .uart0 => hw.Resets.uart0,
            .uart1 => hw.Resets.uart1,
        };
    }
};

/// Everything this driver needs from the outside world. Pins and clock rate are
/// parameters rather than reach-throughs so the chip layer stays board-agnostic.
pub const Config = struct {
    inst: Instance = .uart0,
    tx: gpio.Pin,
    rx: gpio.Pin,
    /// clk_peri, which `clocks.init()` sources from clk_sys.
    clk_hz: u32,
    baud: u32 = default_baud,
};

/// Divisors for the PL011's 16.6 fixed-point baud generator, from clk_peri.
const Divisors = struct {
    ibrd: u32,
    fbrd: u32,

    fn compute(clk_hz: u32, baud: u32) Divisors {
        const div = (8 * clk_hz / baud) + 1;
        var ibrd = div >> 7;
        var fbrd: u32 = 0;
        if (ibrd == 0) {
            ibrd = 1;
        } else if (ibrd >= 65535) {
            ibrd = 65535;
        } else {
            fbrd = (div & 0x7f) >> 1;
        }
        return .{ .ibrd = ibrd, .fbrd = fbrd };
    }

    /// The baud rate these divisors actually produce.
    fn actualBaud(self: Divisors, clk_hz: u32) u32 {
        return (4 * clk_hz) / (64 * self.ibrd + self.fbrd);
    }
};

pub fn init(inst: Instance, cfg: Config) void {
    const b = inst.base();
    hw.Resets.unreset(inst.resetBit());

    // Pins first. TX is driven by the peripheral; RX gets a pull-up.
    gpio.initFunctionOutput(cfg.tx, func_uart);
    gpio.initFunctionInput(cfg.rx, func_uart, .up);

    // Configure while disabled.
    hw.write(hw.Uart.cr(b), 0);

    const d = Divisors.compute(cfg.clk_hz, cfg.baud);
    hw.write(hw.Uart.ibrd(b), d.ibrd);
    hw.write(hw.Uart.fbrd(b), d.fbrd);

    // The PL011 latches IBRD/FBRD only when LCR_H is written.
    hw.write(hw.Uart.lcr_h(b), hw.Uart.lcr_wlen_8 | hw.Uart.lcr_fen);

    hw.write(hw.Uart.cr(b), hw.Uart.cr_uarten | hw.Uart.cr_txe | hw.Uart.cr_rxe);
}

/// Blocking single-byte write, bounded. Drops the byte rather than hanging —
/// see `dropped_bytes`.
pub fn putc(inst: Instance, c: u8) void {
    const b = inst.base();
    var s = bounded.Spin.init(bounded.transfer_polls);
    while (hw.read(hw.Uart.fr(b)) & hw.Uart.fr_txff != 0) {
        s.tick() catch {
            dropped_bytes +%= 1;
            return;
        };
    }
    hw.write(hw.Uart.dr(b), c);
}

/// Non-blocking single-byte write. Returns false when the TX FIFO is full,
/// having written nothing.
///
/// **This is what a firmware with a real-time loop has to use.** `putc` spins
/// until the FIFO drains, and at 115200 baud a few hundred characters of log
/// hold the caller for tens of milliseconds. A DMA fed from that same loop goes
/// on swapping throughout and replays whatever buffer it has, which for audio
/// is plainly audible — and it does not appear in a late-fill counter that only
/// brackets the fill itself, so the log looks clean while the output is not.
pub fn tryPutc(inst: Instance, c: u8) bool {
    const b = inst.base();
    if (hw.read(hw.Uart.fr(b)) & hw.Uart.fr_txff != 0) return false;
    hw.write(hw.Uart.dr(b), c);
    return true;
}

/// Non-blocking single-byte read. Returns null if RX FIFO is empty.
pub fn tryGetc(inst: Instance) ?u8 {
    const b = inst.base();
    if (hw.read(hw.Uart.fr(b)) & hw.Uart.fr_rxfe != 0) {
        return null;
    }
    return @truncate(hw.read(hw.Uart.dr(b)));
}

/// Blocking single-byte read.
pub fn getc(inst: Instance) u8 {
    while (true) {
        if (tryGetc(inst)) |c| return c;
    }
}

pub fn write(inst: Instance, bytes: []const u8) void {
    for (bytes) |c| {
        // Terminals want CRLF; the firmware emits LF.
        if (c == '\n') putc(inst, '\r');
        putc(inst, c);
    }
}

/// Block until everything queued has actually left the wire.
///
/// Bounded on the same reasoning as `putc`: this is what a firmware calls
/// before a reset or a power-down, and a transmitter that never goes idle would
/// otherwise turn "flush the log and sleep" into "never sleep".
pub fn drainTx(inst: Instance) void {
    var s = bounded.Spin.init(bounded.transfer_polls);
    while (hw.read(hw.Uart.fr(inst.base())) & hw.Uart.fr_busy != 0) {
        s.tick() catch return;
    }
}

// ---------------------------------------------------------------------------
// std.Io.Writer adapter
// ---------------------------------------------------------------------------

/// Wrap a UART as a `std.Io.Writer`, using `buffer` for formatting.
///
/// **Pass a real buffer.** This used to hard-code `&.{}`, and a zero-length
/// buffer makes `std.Io.Writer` call `drain` once per formatted *fragment*: a
/// five-argument line measured **11 drain calls**, so one log line became
/// eleven separate bursts into the TX FIFO. See `print` for what that was
/// suspected of costing.
pub fn writerFor(inst: Instance, buffer: []u8) std.Io.Writer {
    return .{
        .buffer = buffer,
        .vtable = switch (inst) {
            .uart0 => &uart0_vtable,
            .uart1 => &uart1_vtable,
        },
    };
}

const uart0_vtable: std.Io.Writer.VTable = .{ .drain = drain0 };
const uart1_vtable: std.Io.Writer.VTable = .{ .drain = drain1 };

/// The `std.Io.Writer` drain contract, with byte emission left to `sink`.
///
/// **Split out so it can be host-tested.** The real sink writes to a UART data
/// register and cannot run on a laptop, but the part that was wrong is pure
/// ordering — and a bug here is invisible until a board goes quiet.
///
/// The contract, from `std.Io.Writer.VTable.drain`:
///
///   * "`buffer[0..end]` is consumed first, followed by each slice of `data` in
///     order."
///   * "This function modifies `Writer.end` and `Writer.buffer`."
///   * "Number of bytes consumed from `data` is returned, **excluding bytes
///     from `buffer`**."
///
/// **All three halves of that were missed on 2026-08-25 and it cost a bench
/// session.** This drain wrote only `data` and never looked at `w.buffer` or
/// reset `w.end`. That was harmless for as long as `writerFor` passed a
/// zero-length buffer, because nothing was ever buffered — and then `print` was
/// given a real 256-byte one, at which point `flush` had buffered bytes that
/// nothing would consume, `end` never fell to zero, and the *first* `uart.print`
/// of every firmware in this workspace never returned. A board that flashed,
/// verified, ran and blinked a pin, in perfect silence.
///
/// The lesson worth more than the fix: **the host test written alongside that
/// change used `.buffer = &.{}`**, faithfully reproducing the behaviour being
/// replaced, so it could not have caught this. A test that does not exercise
/// the configuration you are shipping is not evidence about it.
fn drainWith(w: *std.Io.Writer, data: []const []const u8, splat: usize, sink: anytype) usize {
    // Buffered bytes first, and they do **not** count toward the return value —
    // the caller already counted them as written when they went into `buffer`.
    if (w.end > 0) {
        sink.emit(w.buffer[0..w.end]);
        w.end = 0;
    }

    var written: usize = 0;

    for (data[0 .. data.len - 1]) |slice| {
        sink.emit(slice);
        written += slice.len;
    }

    // The last element is repeated `splat` times, "which may be zero" — so a
    // splat of 0 must write it no times and count it not at all.
    const last = data[data.len - 1];
    var i: usize = 0;
    while (i < splat) : (i += 1) {
        sink.emit(last);
        written += last.len;
    }

    return written;
}

/// Emits to a real UART. The other implementation of this shape lives in the
/// tests below.
const UartSink = struct {
    inst: Instance,
    fn emit(self: UartSink, bytes: []const u8) void {
        write(self.inst, bytes);
    }
};

fn drain0(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
    return drainWith(w, data, splat, UartSink{ .inst = .uart0 });
}

fn drain1(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
    return drainWith(w, data, splat, UartSink{ .inst = .uart1 });
}

/// How much of a formatted line is assembled before it reaches the wire.
///
/// Sized for the longest line in this workspace with room over; a line longer
/// than this is not truncated, it simply drains more than once.
pub const print_buffer_bytes = 256;

/// Formatting failures. **`print` cannot return an error** — it is called from
/// ISRs, from `noreturn` paths and from a hundred places where a `try` would
/// obscure the diagnostic being printed — so a failure counts here instead of
/// vanishing.
///
/// Non-zero means some line in the log is incomplete. Print it at the end of a
/// report, or read it over SWD.
pub var print_errors: u32 = 0;

/// Convenience wrapper for formatting against a specific UART instance.
///
/// ## The truncation this was blamed for
///
/// `app/bt-speaker`'s notes record a five-argument line of ~84 characters
/// arriving cut mid-literal, and attribute it to the zero-length buffer this
/// used to pass, with the error swallowed by `catch {}`.
///
/// **Measured on the host, 2026-08-25: that mechanism is disproved.** A
/// zero-length buffer truncates nothing. Lines of 5, 4, 10 and 16 arguments,
/// widths to 301 characters, and every format specifier in use all came out
/// byte-exact with `print` returning no error at all. Length and argument count
/// are not the limit, and there was no error for `catch {}` to swallow.
///
/// **What the measurement did show** is that a zero-length buffer turns one
/// line into one `drain` call per fragment — 11 for that five-argument line —
/// and long many-argument lines produce the most. That reconciles the two
/// observations that seemed to conflict: a per-burst loss on the bench UART
/// would preferentially hit exactly those lines, and would cut them at a
/// fragment boundary rather than scattering losses across the log. It is a
/// hypothesis and not a finding; the original call site has since been split
/// into two prints, so the failure cannot be reproduced from the tree as it
/// stands.
///
/// Both halves are now closed off regardless of which was right: the buffer is
/// real, so a line is one burst rather than eleven, and a failure is counted in
/// `print_errors` rather than discarded.
pub fn print(inst: Instance, comptime fmt: []const u8, args: anytype) void {
    var buffer: [print_buffer_bytes]u8 = undefined;
    var w = writerFor(inst, &buffer);
    w.print(fmt, args) catch {
        print_errors +%= 1;
    };
    w.flush() catch {
        print_errors +%= 1;
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

// ---------------------------------------------------------------------------
// The drain contract
// ---------------------------------------------------------------------------
//
// These exist because the previous version of `drainInstance` passed every test
// in this file while being unable to emit a single byte through a buffered
// writer. The tests all used a zero-length buffer, which is the one
// configuration in which the bug is invisible.

/// A `drainWith` sink that collects into memory, so the ordering can be checked
/// on a laptop. Same shape as `UartSink`; that is the point.
const CollectSink = struct {
    out: *std.ArrayList(u8),
    gpa: std.mem.Allocator,
    fn emit(self: CollectSink, bytes: []const u8) void {
        self.out.appendSlice(self.gpa, bytes) catch unreachable;
    }
};

fn collectDrain(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
    const sink = CollectSink{ .out = test_sink.?, .gpa = testing.allocator };
    return drainWith(w, data, splat, sink);
}

var test_sink: ?*std.ArrayList(u8) = null;
const collect_vtable: std.Io.Writer.VTable = .{ .drain = collectDrain };

test "drain consumes the buffer first, and does not count it as data" {
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    test_sink = &out;
    defer test_sink = null;

    var buf: [16]u8 = undefined;
    var w: std.Io.Writer = .{ .buffer = &buf, .vtable = &collect_vtable };

    // Four bytes already buffered, as `print` would leave them.
    @memcpy(buf[0..4], "abcd");
    w.end = 4;

    const n = try w.vtable.drain(&w, &.{"XY"}, 1);

    // Buffer first, then the data slice.
    try testing.expectEqualSlices(u8, "abcdXY", out.items);
    // **Two, not six.** Buffered bytes were already counted as written when
    // they went into the buffer; returning six makes the writer believe it has
    // sent four bytes it has not.
    try testing.expectEqual(@as(usize, 2), n);
    // And `end` must fall to zero, or `flush` never terminates — which is
    // exactly how this failed on hardware.
    try testing.expectEqual(@as(usize, 0), w.end);
}

test "a buffered print reaches the sink, and flush terminates" {
    // **The regression test proper.** With a real buffer, `print` puts the
    // formatted bytes in `w.buffer` and only `flush` pushes them out. A drain
    // that ignores the buffer leaves `end` non-zero forever and `flush` spins.
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    test_sink = &out;
    defer test_sink = null;

    var buf: [print_buffer_bytes]u8 = undefined;
    var w: std.Io.Writer = .{ .buffer = &buf, .vtable = &collect_vtable };

    try w.print("up {d}s, rung {d}, {s}\n", .{ 42, 0x19, "ok" });
    try w.flush();

    try testing.expectEqualSlices(u8, "up 42s, rung 25, ok\n", out.items);
    try testing.expectEqual(@as(usize, 0), w.end);
}

test "a line longer than the buffer drains more than once, losing nothing" {
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    test_sink = &out;
    defer test_sink = null;

    // Deliberately small, so the multi-drain path runs without a 256-byte line.
    var buf: [8]u8 = undefined;
    var w: std.Io.Writer = .{ .buffer = &buf, .vtable = &collect_vtable };

    const long = "0123456789abcdefghijklmnopqrstuvwxyz";
    try w.print("{s}", .{long});
    try w.flush();

    try testing.expectEqualSlices(u8, long, out.items);
    try testing.expectEqual(@as(usize, 0), w.end);
}

test "a splat of zero writes the last slice no times and counts it not at all" {
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(testing.allocator);
    test_sink = &out;
    defer test_sink = null;

    var w: std.Io.Writer = .{ .buffer = &.{}, .vtable = &collect_vtable };
    const n = try w.vtable.drain(&w, &.{ "keep", "drop" }, 0);

    try testing.expectEqualSlices(u8, "keep", out.items);
    try testing.expectEqual(@as(usize, 4), n);
}

test "Divisors: 12 MHz clk_peri at 115200 lands well inside UART tolerance" {
    const d = Divisors.compute(12_000_000, 115_200);
    try testing.expectEqual(@as(u32, 6), d.ibrd);
    try testing.expectEqual(@as(u32, 33), d.fbrd);

    const actual = d.actualBaud(12_000_000);
    try testing.expectEqual(@as(u32, 115_107), actual);

    const err_ppm = (@as(u64, 115_200 - actual) * 1_000_000) / 115_200;
    try testing.expect(err_ppm < 10_000);
}

test "Divisors: clamps rather than emitting a nonsense rate" {
    const fast = Divisors.compute(12_000_000, 100_000_000);
    try testing.expectEqual(@as(u32, 1), fast.ibrd);
    try testing.expectEqual(@as(u32, 0), fast.fbrd);

    const slow = Divisors.compute(12_000_000, 1);
    try testing.expectEqual(@as(u32, 65535), slow.ibrd);
    try testing.expectEqual(@as(u32, 0), slow.fbrd);
}

test "Instance addresses match addressmap.h" {
    try testing.expectEqual(@as(u32, 0x4007_0000), Instance.uart0.base());
    try testing.expectEqual(@as(u32, 0x4007_8000), Instance.uart1.base());
}
