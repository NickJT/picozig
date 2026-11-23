//! Generic WS2812 (NeoPixel) PIO driver.
//!
//! Drives a chain of WS2812 RGB LEDs over a single GPIO pin using RP2350 PIO.

const std = @import("std");
const rp2350 = @import("rp2350");

const pio = rp2350.pio;

/// Everything here can fail only one way: the state machine stopped consuming.
pub const Error = rp2350.bounded.Error;
const gpio = rp2350.gpio;
const timer = rp2350.timer;

/// WS2812 RGB Color in 8-bit R, G, B components.
pub const Color = struct {
    r: u8 = 0,
    g: u8 = 0,
    b: u8 = 0,

    /// Convert to 24-bit GRB word formatted for MSB-first transmission.
    pub fn grbWord(self: Color) u32 {
        const grb = (@as(u32, self.g) << 16) | (@as(u32, self.r) << 8) | @as(u32, self.b);
        return grb << 8;
    }
};

/// Convert HSV to RGB Color.
/// - `hue_deg`: 0..359 degrees
/// - `s`: Saturation 0..255
/// - `v`: Value/Brightness 0..255
pub fn hsvToRgb(hue_deg: u16, s: u8, v: u8) Color {
    if (s == 0) return .{ .r = v, .g = v, .b = v };

    const h = @as(u32, hue_deg % 360);
    const region = h / 60;
    const remainder = (h % 60) * 255 / 60;

    const p: u8 = @intCast((@as(u32, v) * (255 - s)) / 255);
    const q: u8 = @intCast((@as(u32, v) * (255 - ((@as(u32, s) * remainder) / 255))) / 255);
    const t: u8 = @intCast((@as(u32, v) * (255 - ((@as(u32, s) * (255 - remainder)) / 255))) / 255);

    return switch (region) {
        0 => .{ .r = v, .g = t, .b = p },
        1 => .{ .r = q, .g = v, .b = p },
        2 => .{ .r = p, .g = v, .b = t },
        3 => .{ .r = p, .g = q, .b = v },
        4 => .{ .r = t, .g = p, .b = v },
        else => .{ .r = v, .g = p, .b = q },
    };
}

/// 4-instruction WS2812 PIO program (800 kHz at 8 MHz PIO clock).
///
/// 10 PIO cycles per bit (1.25 µs):
/// - Bit 1: 7 cycles HIGH, 3 cycles LOW
/// - Bit 0: 3 cycles HIGH, 7 cycles LOW
pub const program = [_]u16{
    // 0: out x, 1         side 0 [2] (3 cycles low)
    pio.out(.x, 1, pio.sideSet(1, 0, 2)),
    // 1: jmp !x, do_zero  side 1 [2] (3 cycles high; if x==0 jump to inst 3)
    pio.jmp(.x_zero, 3, pio.sideSet(1, 1, 2)),
    // 2: jmp 0            side 1 [3] (4 cycles high, total 7 cycles high; jump to inst 0)
    pio.jmp(.always, 0, pio.sideSet(1, 1, 3)),
    // 3: nop              side 0 [3] (4 cycles low, total 7 cycles low; wraps to inst 0)
    pio.nop(pio.sideSet(1, 0, 3)),
};

/// Generic WS2812 Driver for `num_leds` LEDs on `pin`.
pub fn Driver(comptime pin: gpio.Pin, comptime num_leds: usize) type {
    return struct {
        const Self = @This();

        block: pio.Block,
        sm: u3,
        colors: [num_leds]Color = @as([num_leds]Color, @splat(.{})),

        pub fn init(block: pio.Block, sm: u3, clk_sys_freq: u32) Error!Self {
            gpio.initFunctionOutput(pin, block.funcsel());

            pio.init(block);
            pio.setConsecutivePindirs(block, sm, pin, 1, true);

            // PIO clock = 8 MHz (125 ns per cycle -> 10 cycles = 1.25 µs = 800 kHz)
            const clkdiv: u32 = @intCast((@as(u64, clk_sys_freq) << 16) / 8_000_000);

            const cfg = pio.SmConfig{
                .wrap_bottom = 0,
                .wrap_top = @intCast(program.len - 1),
                .sideset_base = pin,
                .sideset_count = 1,
                .out_base = pin,
                .out_count = 1,
                .out_shiftdir = .left,
                .autopull = true,
                .pull_threshold = 24,
                .clkdiv = clkdiv,
            };

            pio.loadProgram(block, 0, &program);
            pio.configure(block, sm, cfg);
            pio.prepare(block, sm, 0);
            pio.setEnabled(block, sm, true);

            var driver = Self{
                .block = block,
                .sm = sm,
            };
            try driver.clear();
            return driver;
        }

        pub fn setRgb(self: *Self, index: usize, r: u8, g: u8, b: u8) void {
            if (index >= num_leds) return;
            self.colors[index] = .{ .r = r, .g = g, .b = b };
        }

        pub fn setHsv(self: *Self, index: usize, hue_deg: u16, s: u8, v: u8) void {
            if (index >= num_leds) return;
            self.colors[index] = hsvToRgb(hue_deg, s, v);
        }

        pub fn setAll(self: *Self, color: Color) void {
            for (&self.colors) |*c| {
                c.* = color;
            }
        }

        pub fn clear(self: *Self) Error!void {
            self.setAll(.{ .r = 0, .g = 0, .b = 0 });
            try self.show();
        }

        /// Push the whole strip out of the state machine.
        ///
        /// **Bounded since 2026-08-25.** A strip is fed one word per LED with
        /// no flow control other than the FIFO, so a machine that stops
        /// consuming — a bad program, a clock that never started, a block held
        /// in reset — used to hang the caller here with the lights stuck on
        /// whatever they were last showing. Now it reports.
        pub fn show(self: *const Self) Error!void {
            for (self.colors) |color| {
                try pio.write(self.block, self.sm, color.grbWord());
            }
            // WS2812 latch reset requires data line low for > 50 µs (using 300 µs for safety).
            timer.delayUs(300);
        }
    };
}

// ---------------------------------------------------------------------------
// Host Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "Color grbWord formats GRB into MSB 24 bits" {
    const red = Color{ .r = 0xFF, .g = 0x00, .b = 0x00 };
    // Green=00, Red=FF, Blue=00 -> 0x00FF0000
    try testing.expectEqual(@as(u32, 0x00FF0000), red.grbWord());

    const green = Color{ .r = 0x00, .g = 0xFF, .b = 0x00 };
    // Green=FF, Red=00, Blue=00 -> 0xFF000000
    try testing.expectEqual(@as(u32, 0xFF000000), green.grbWord());

    const blue = Color{ .r = 0x00, .g = 0x00, .b = 0xFF };
    // Green=00, Red=00, Blue=FF -> 0x0000FF00
    try testing.expectEqual(@as(u32, 0x0000FF00), blue.grbWord());
}

test "hsvToRgb basic primary colors" {
    // Red (Hue 0)
    const r = hsvToRgb(0, 255, 255);
    try testing.expectEqual(@as(u8, 255), r.r);
    try testing.expectEqual(@as(u8, 0), r.g);
    try testing.expectEqual(@as(u8, 0), r.b);

    // Green (Hue 120)
    const g = hsvToRgb(120, 255, 255);
    try testing.expectEqual(@as(u8, 0), g.r);
    try testing.expectEqual(@as(u8, 255), g.g);
    try testing.expectEqual(@as(u8, 0), g.b);

    // Blue (Hue 240)
    const b = hsvToRgb(240, 255, 255);
    try testing.expectEqual(@as(u8, 0), b.r);
    try testing.expectEqual(@as(u8, 0), b.g);
    try testing.expectEqual(@as(u8, 255), b.b);
}
