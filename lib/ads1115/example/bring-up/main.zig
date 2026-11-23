//! bring-up — Rung 0 of `lib/ads1115`: one breakout on its own, no front end.
//!
//! Wiring (Pico LiPo 2, breakout powered from 3V3):
//!
//!   breakout V     3V3   pin 36
//!   breakout S     GND   pin 38   (S is presumed ground — check with a meter first)
//!   breakout SDA   GP4   pin 6    (or the Qw/ST connector)
//!   breakout SCL   GP5   pin 7
//!   breakout Alert GP6   pin 9    internal pull-up on; the pin is open drain
//!   breakout A0    GND   } shorted together, for the 12-bit test
//!   breakout A1    GND   }
//!   breakout ADDR  as it comes — every address is scanned
//!   Debug Probe    GP0 TX, GP1 RX, GND
//!
//! What it answers, in order, then idles:
//!
//!   1. **Which address answers.** All four are scanned, so a board with ADDR
//!      tied on board and one with it floating both show what they are.
//!   2. **The Config register.** `8583` only on the first boot after power-up:
//!      the part is not reset when the Pico is, so a reflash reads whatever the
//!      last run left. Information, not a pass or fail.
//!   3. **Threshold round trip**, then the reset values written back.
//!   4. **16-bit or 12-bit.** 256 single shots of a shorted input at ±0.256 V and
//!      860 SPS, where the datasheet's noise is ~4.6 codes peak to peak. An
//!      ADS1015 left-justifies 12 bits, so the low four bits of every code are
//!      zero; the OR of them across all 256 says which part this is, whatever
//!      the package is marked.
//!   5. **The real data rate, at all eight settings**, by timing ALERT/RDY's
//!      ~8 µs conversion-ready pulses for about a second each. The datasheet
//!      allows ±10 %, and the AC correction needs the true figure. Each single
//!      shot's busy time is printed too, as a second opinion that does not
//!      depend on the pin.
//!
//! The part is left powered down in its reset configuration.
//!
//! **Read results over the UART, not a watch step.** Halting the core mid-run
//! skews step 5's timing.
//!
//!     zig build swd

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const ads1115 = @import("ads1115");

const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;
const silicon = rp2350.silicon;

comptime {
    _ = &boot;
}

const sda_pin: rp2350.Pin = 4;
const scl_pin: rp2350.Pin = 5;
const alert_pin: rp2350.Pin = 6;
const timeout_polls: u32 = 10_000;

/// `lib/ads1115`'s bus contract over `rp2350.i2c`.
///
/// **STOP between the pointer write and the read, not a repeated START.** The
/// part accepts either (§7.5.3), and `rp2350.i2c`'s repeated-start path is
/// host-tested only — a first-ever driver should not also be the first proof
/// of a bus feature.
const PicoBus = struct {
    inst: i2c.Instance,

    pub fn writeRegs(self: PicoBus, addr: u7, start_reg: u8, data: []const u8) !void {
        var buf: [3]u8 = undefined;
        if (data.len + 1 > buf.len) return error.BufferOverflow;
        buf[0] = start_reg;
        @memcpy(buf[1 .. data.len + 1], data);
        try i2c.write(self.inst, addr, buf[0 .. data.len + 1], true, timeout_polls);
    }

    pub fn readRegs(self: PicoBus, addr: u7, start_reg: u8, dest: []u8) !void {
        const ptr = [1]u8{start_reg};
        try i2c.write(self.inst, addr, &ptr, true, timeout_polls);
        try i2c.read(self.inst, addr, dest, true, timeout_polls);
    }
};

const Adc = ads1115.Driver(PicoBus);

var out_buf: [256]u8 = undefined;
var out: std.Io.Writer = undefined;

fn say(comptime fmt: []const u8, args: anytype) void {
    out.print(fmt, args) catch {};
    out.flush() catch {};
}

fn park() noreturn {
    say("parked\n", .{});
    while (true) timer.delayMs(1000);
}

/// Busy until `OS` reads 1, in microseconds, or null after `limit_us`.
fn waitIdle(adc: *Adc, limit_us: u64) !?u64 {
    const start = timer.now64();
    while (true) {
        if (!(try adc.readStatus()).converting) return timer.now64() - start;
        if (timer.now64() - start > limit_us) return null;
    }
}

/// The time for `n` RDY falling edges after the first, in microseconds, or
/// null if the pin stops pulsing.
fn timeEdges(n: u32, limit_us: u64) ?u64 {
    const start = timer.now64();
    var prev = gpio.get(alert_pin);
    var seen: u32 = 0;
    var first: u64 = 0;
    while (true) {
        const now = gpio.get(alert_pin);
        if (prev and !now) {
            const t = timer.now64();
            if (seen == 0) first = t;
            if (seen == n) return t - first;
            seen += 1;
        }
        prev = now;
        if (timer.now64() - start > limit_us) return null;
    }
}

pub fn main() noreturn {
    rp2350.clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());
    out = uart.writerFor(.uart0, &out_buf);

    say("\n=== ads1115 bring-up ===  board {s}  chip {s}\n", .{ board.name, silicon.idHex() });

    i2c.init(.i2c0, .{ .sda = sda_pin, .scl = scl_pin, .speed = .fast_400k, .clk_hz = board.sys_hz });
    gpio.initInput(alert_pin, .up);

    // 1. Address scan.
    var found: ?ads1115.Address = null;
    for (std.enums.values(ads1115.Address)) |a| {
        const ack = i2c.probe(.i2c0, @backingInt(a), timeout_polls);
        say("1 addr 0x{x:0>2} ({t}): {s}\n", .{ @backingInt(a), a, if (ack) "ACK" else "-" });
        if (ack and found == null) found = a;
    }
    const addr = found orelse {
        say("1 FAIL no ADS111x at any address\n", .{});
        park();
    };

    var adc = Adc.init(.{ .inst = .i2c0 }, addr);
    run(&adc) catch |err| {
        say("FAIL {t}\n", .{err});
        park();
    };
    park();
}

fn run(adc: *Adc) !void {
    // 2. Config as found.
    const cfg_raw = try adc.readRegister(.config);
    say("2 config 0x{x:0>4} ({s})\n", .{ cfg_raw, if (cfg_raw == ads1115.reset_config) "power-up value" else "not the power-up value: a previous run's, or not an ADS1115" });

    // 3. Threshold round trip.
    try adc.setThresholds(0x1234, 0x5678);
    const lo = try adc.readRegister(.lo_thresh);
    const hi = try adc.readRegister(.hi_thresh);
    say("3 thresholds wrote 1234/5678 read {x:0>4}/{x:0>4}: {s}\n", .{ lo, hi, if (lo == 0x1234 and hi == 0x5678) "ok" else "FAIL" });
    try adc.writeRegister(.lo_thresh, ads1115.reset_lo_thresh);
    try adc.writeRegister(.hi_thresh, ads1115.reset_hi_thresh);

    // 4. 16-bit or 12-bit.
    const shot = ads1115.Config{ .mux = .ain0_ain1, .range = .fsr_0v256, .rate = .sps860 };
    var low_bits: u16 = 0;
    var min: i16 = std.math.maxInt(i16);
    var max: i16 = std.math.minInt(i16);
    var busy_min: u64 = std.math.maxInt(u64);
    var busy_max: u64 = 0;
    for (0..256) |_| {
        try adc.startSingle(shot);
        const busy = (try waitIdle(adc, 20_000)) orelse {
            say("4 FAIL single shot never finished\n", .{});
            return;
        };
        busy_min = @min(busy_min, busy);
        busy_max = @max(busy_max, busy);
        const code = try adc.readConversion();
        low_bits |= @as(u16, @bitCast(code)) & 0xF;
        min = @min(min, code);
        max = @max(max, code);
    }
    say("4 shorted input, 256 shots at 0.256 V/860 SPS: codes {d}..{d}, low-nibble OR 0x{x}: {s}\n", .{
        min, max, low_bits, if (low_bits != 0) "16-bit (ADS1115)" else "12-bit (ADS1015) or a stuck input",
    });
    say("4 single-shot busy {d}..{d} us (860 SPS nominal 1163 us + ~25 us wake)\n", .{ busy_min, busy_max });

    // 5. Data rate at every setting, from the RDY pulses.
    try adc.setReadyThresholds();
    for (std.enums.values(ads1115.DataRate)) |rate| {
        const nominal = rate.nominalSps();
        try adc.configure(.{ .mux = .ain0_ain1, .range = .fsr_0v256, .mode = .continuous, .rate = rate, .comp_queue = .after_1 });
        // The conversion in flight finishes on the old rate; let two go by.
        if (timeEdges(2, 1_000_000) == null) {
            say("5 {d} SPS: FAIL no RDY pulses on GP{d}\n", .{ nominal, alert_pin });
            continue;
        }
        const n: u32 = nominal; // ~1 s at every rate
        const dt = timeEdges(n, 3_000_000) orelse {
            say("5 {d} SPS: FAIL RDY stopped\n", .{nominal});
            continue;
        };
        const milli_sps = @as(u64, n) * 1_000_000_000 / dt;
        const err_ppm = @divTrunc((@as(i64, @intCast(milli_sps)) - @as(i64, nominal) * 1000) * 1000, @as(i64, nominal));
        say("5 {d} SPS nominal: {d}.{d:0>3} SPS measured over {d} periods, {d} ppm\n", .{
            nominal, milli_sps / 1000, milli_sps % 1000, n, err_ppm,
        });
    }

    // Leave it as power-up left it.
    try adc.writeRegister(.config, ads1115.reset_config & 0x7FFF);
    try adc.writeRegister(.lo_thresh, ads1115.reset_lo_thresh);
    try adc.writeRegister(.hi_thresh, ads1115.reset_hi_thresh);
    say("done; part left in single-shot power-down, reset configuration\n", .{});
}
