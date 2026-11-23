//! Generic PWM Servo Motor Driver.
//!
//! Drives RC servos using RP2350 hardware PWM (`rp2350.pwm`) at 50 Hz (20 ms period).

const std = @import("std");
const rp2350 = @import("rp2350");

const gpio = rp2350.gpio;
const pwm = rp2350.pwm;

/// Generic Servo Driver for a specific GPIO pin.
pub fn Driver(comptime pin: gpio.Pin) type {
    return struct {
        const Self = @This();

        slice: pwm.Slice,
        channel: pwm.Channel,
        top: u16 = 0,
        clk_hz: u32 = 0,

        pub fn init(clk_sys_hz: u32) Self {
            pwm.init();

            const slice = pwm.sliceOf(pin);
            const channel = pwm.channelOf(pin);

            pwm.initPin(pin);

            // 50 Hz target frequency (20,000 µs period)
            const cfg = pwm.configureHz(clk_sys_hz, 50) orelse {
                @panic("Failed to configure 50 Hz PWM for servo");
            };

            pwm.configure(slice, cfg);

            var driver = Self{
                .slice = slice,
                .channel = channel,
                .top = cfg.top,
                .clk_hz = clk_sys_hz,
            };

            // Default to neutral (90 degrees / 1500 us)
            driver.setPulseWidthUs(1500);
            pwm.enable(slice);

            return driver;
        }

        /// Set pulse width in microseconds (typically 500..2500 µs; 1000 µs = 0°, 2000 µs = 180°).
        pub fn setPulseWidthUs(self: *Self, us: u16) void {
            // Period is 20,000 µs (50 Hz). Level counts = (us / 20000) * (top + 1).
            const level = (@as(u32, us) * (@as(u32, self.top) + 1)) / 20_000;
            pwm.setLevel(self.slice, self.channel, @intCast(@min(level, self.top)));
        }

        /// Set angle in degrees (0..180° mapped to 1000..2000 µs).
        pub fn setAngleDeg(self: *Self, deg: u8) void {
            const clamped = @min(deg, 180);
            const us: u16 = @intCast(1000 + (@as(u32, clamped) * 1000) / 180);
            self.setPulseWidthUs(us);
        }

        /// Set position percentage (0..100% mapped to 1000..2000 µs).
        pub fn setPercent(self: *Self, percent: u8) void {
            const clamped = @min(percent, 100);
            const us: u16 = @intCast(1000 + @as(u32, clamped) * 10);
            self.setPulseWidthUs(us);
        }

        pub fn enable(self: *const Self) void {
            pwm.enable(self.slice);
        }

        pub fn disable(self: *const Self) void {
            pwm.disable(self.slice);
        }
    };
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "angle to pulse width conversion" {
    // 0 deg -> 1000 us
    const us0: u16 = @intCast(1000 + (@as(u32, 0) * 1000) / 180);
    try testing.expectEqual(@as(u16, 1000), us0);

    // 90 deg -> 1500 us
    const us90: u16 = @intCast(1000 + (@as(u32, 90) * 1000) / 180);
    try testing.expectEqual(@as(u16, 1500), us90);

    // 180 deg -> 2000 us
    const us180: u16 = @intCast(1000 + (@as(u32, 180) * 1000) / 180);
    try testing.expectEqual(@as(u16, 2000), us180);
}

test "percentage to pulse width conversion" {
    // 0% -> 1000 us
    const us0: u16 = 1000 + @as(u16, 0) * 10;
    try testing.expectEqual(@as(u16, 1000), us0);

    // 50% -> 1500 us
    const us50: u16 = 1000 + @as(u16, 50) * 10;
    try testing.expectEqual(@as(u16, 1500), us50);

    // 100% -> 2000 us
    const us100: u16 = 1000 + @as(u16, 100) * 10;
    try testing.expectEqual(@as(u16, 2000), us100);
}
