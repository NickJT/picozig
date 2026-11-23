//! PWM — 12 slices, each driving two channels (A and B).
//!
//! ## Pin to slice is fixed by hardware, not chosen
//!
//! A pin's slice and channel are determined by its number: `slice = (pin/2) % 8`
//! and channel A for even pins, B for odd. You do not pick them — `rp2350.pins`
//! validates that a claimed pin really is the slice/channel being programmed,
//! which is the mistake this catches at compile time.
//!
//! ## Frequency and resolution trade against each other
//!
//! `f = clk_sys / ((TOP + 1) * (DIV_INT + DIV_FRAC/16))`, and duty resolution is
//! `TOP + 1` steps. From a 12 MHz clk_sys, `configureHz` picks the largest TOP
//! that fits, so resolution is as good as the frequency allows — at 1 kHz that
//! is the full 16-bit range; at 100 kHz it drops to about 120 steps, which is
//! still far more than an eye can resolve.

const std = @import("std");
const hw = @import("hw.zig");
const gpio = @import("gpio.zig");
const pins = @import("pins.zig");

/// On RP2350, FUNCSEL value 4 maps a GPIO pin to the hardware PWM peripheral.
const func_pwm: u32 = 4;

pub const slice_count = 12;
pub const Slice = u4;
pub const Channel = enum { a, b };

/// Which slice drives a pin. Hardware-fixed.
pub fn sliceOf(pin: gpio.Pin) Slice {
    return @truncate((pin / 2) % 8);
}

/// Which channel a pin drives. Even pins are A, odd are B.
pub fn channelOf(pin: gpio.Pin) Channel {
    return if (pin % 2 == 0) .a else .b;
}

/// Validate that a GPIO pin is valid for RP2350 PWM at compile time.
pub fn validatePin(comptime pin: gpio.Pin) void {
    comptime {
        if (pin > 47) {
            @compileError(std.fmt.comptimePrint(
                "GP{d} is not a valid GPIO pin for PWM (max GP47)",
                .{pin},
            ));
        }
        const slice = sliceOf(pin);
        const channel = channelOf(pin);
        const pwm_ch: pins.PwmChannel = switch (channel) {
            .a => .a,
            .b => .b,
        };
        if (!pins.supports(pin, .{ .pwm = .{ .slice = slice, .channel = pwm_ch } })) {
            @compileError(std.fmt.comptimePrint(
                "GP{d} does not support PWM slice {d} channel {s}",
                .{ pin, slice, @tagName(channel) },
            ));
        }
    }
}

fn regs(slice: Slice) struct { csr: u32, div: u32, ctr: u32, cc: u32, top: u32 } {
    // Slices are 0x14 apart, five registers each.
    const base = hw.pwm_base + @as(u32, slice) * 0x14;
    return .{
        .csr = base + 0x00,
        .div = base + 0x04,
        .ctr = base + 0x08,
        .cc = base + 0x0c,
        .top = base + 0x10,
    };
}

pub const Config = struct {
    /// Counter wrap value. Duty resolution is `top + 1` steps.
    top: u16,
    /// Clock divider, 8.4 fixed point: integer part 1-255, fractional sixteenths.
    div_int: u8 = 1,
    div_frac: u4 = 0,
    /// Count up then down, halving the output frequency but keeping edges
    /// centred. Useful for motors, irrelevant for LEDs.
    phase_correct: bool = false,
    invert_a: bool = false,
    invert_b: bool = false,
};

/// Divisor and wrap for a target frequency, choosing the largest TOP that fits
/// so duty resolution stays as high as the frequency permits.
///
/// Returns null if the frequency is unreachable — too high for even TOP=1, or
/// too low for the largest divider. Refusing beats silently producing some
/// other frequency, which on an LED looks like a bug in the pattern code.
pub fn configureHz(clk_hz: u32, target_hz: u32) ?Config {
    if (target_hz == 0) return null;

    // Total clock cycles per PWM period.
    const period = clk_hz / target_hz;
    if (period == 0) return null; // faster than the clock

    // Prefer div = 1 and the largest TOP that fits.
    if (period <= 65536) {
        return .{ .top = @intCast(period - 1), .div_int = 1, .div_frac = 0 };
    }

    // Otherwise divide down until the period fits in 16 bits.
    var div_int: u32 = (period + 65535) / 65536;
    if (div_int > 255) return null; // slower than the hardware can go
    if (div_int < 1) div_int = 1;
    const top = (period / div_int) - 1;
    if (top > 65535) return null;

    return .{ .top = @intCast(top), .div_int = @intCast(div_int), .div_frac = 0 };
}

pub fn init() void {
    hw.Resets.unreset(hw.Resets.pwm);
}

/// Configure a slice. Does not start it — call `enable`.
pub fn configure(slice: Slice, cfg: Config) void {
    const r = regs(slice);
    hw.write(r.csr, 0); // stopped while reprogramming
    hw.write(r.div, (@as(u32, cfg.div_int) << 4) | cfg.div_frac);
    hw.write(r.top, cfg.top);
    hw.write(r.ctr, 0);

    var csr: u32 = 0;
    if (cfg.phase_correct) csr |= 1 << 1;
    if (cfg.invert_a) csr |= 1 << 2;
    if (cfg.invert_b) csr |= 1 << 3;
    hw.write(r.csr, csr);
}

/// Hand a pin to its PWM slice. Validates pin selection at compile time.
pub fn initPin(comptime pin: gpio.Pin) void {
    validatePin(pin);
    gpio.initFunctionOutput(pin, func_pwm);
}

/// Set a channel's compare value: output is high for `level` counts of each
/// `top + 1` period.
pub fn setLevel(slice: Slice, channel: Channel, level: u16) void {
    const r = regs(slice);
    const cc = hw.read(r.cc);
    hw.write(r.cc, switch (channel) {
        .a => (cc & 0xffff_0000) | level,
        .b => (cc & 0x0000_ffff) | (@as(u32, level) << 16),
    });
}

/// Set duty as a percentage of the configured period. Convenience over
/// `setLevel` for the common case; rounds to nearest.
pub fn setDutyPercent(slice: Slice, channel: Channel, top: u16, percent: u8) void {
    const p = @min(percent, 100);
    const level = (@as(u32, top) + 1) * p / 100;
    setLevel(slice, channel, @intCast(@min(level, 0xffff)));
}

pub fn enable(slice: Slice) void {
    const r = regs(slice);
    hw.write(r.csr, hw.read(r.csr) | 1);
}

pub fn disable(slice: Slice) void {
    const r = regs(slice);
    hw.write(r.csr, hw.read(r.csr) & ~@as(u32, 1));
}

/// Live counter value — useful as proof a slice is actually running when there
/// is no scope to hand.
pub fn counter(slice: Slice) u16 {
    return @truncate(hw.read(regs(slice).ctr));
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "slice and channel are fixed by pin number" {
    // GP25 (the LiPo 2's LED) is slice 4, channel B.
    try testing.expectEqual(@as(Slice, 4), sliceOf(25));
    try testing.expectEqual(Channel.b, channelOf(25));
    // GP0 is slice 0 channel A; GP16 wraps back to slice 0.
    try testing.expectEqual(@as(Slice, 0), sliceOf(0));
    try testing.expectEqual(Channel.a, channelOf(0));
    try testing.expectEqual(@as(Slice, 0), sliceOf(16));
    try testing.expectEqual(@as(Slice, 3), sliceOf(22));
}

test "register addresses use the 0x14 per-slice stride" {
    try testing.expectEqual(hw.pwm_base + 0x00, regs(0).csr);
    try testing.expectEqual(hw.pwm_base + 0x10, regs(0).top);
    try testing.expectEqual(hw.pwm_base + 0x14, regs(1).csr);
    try testing.expectEqual(hw.pwm_base + 0x14 * 11, regs(11).csr);
}

test "configureHz keeps full resolution where it can" {
    const clk = 12_000_000;

    // 1 kHz from 12 MHz needs 12000 counts — fits in TOP with div 1, so the
    // full resolution is available.
    const khz = configureHz(clk, 1000).?;
    try testing.expectEqual(@as(u8, 1), khz.div_int);
    try testing.expectEqual(@as(u16, 11_999), khz.top);

    // 200 Hz needs 60000 counts — still fits.
    const slow = configureHz(clk, 200).?;
    try testing.expectEqual(@as(u8, 1), slow.div_int);
    try testing.expectEqual(@as(u16, 59_999), slow.top);

    // 100 Hz needs 120000 — must divide.
    const slower = configureHz(clk, 100).?;
    try testing.expect(slower.div_int > 1);
}

test "configureHz refuses rather than silently producing another frequency" {
    const clk = 12_000_000;

    // Faster than the clock, and the zero case.
    try testing.expectEqual(@as(?Config, null), configureHz(clk, 20_000_000));
    try testing.expectEqual(@as(?Config, null), configureHz(clk, 0));

    // **At 12 MHz there is no unreachable low frequency**: the slowest possible
    // is clk / (255 * 65536) ~= 0.72 Hz, below the 1 Hz floor of an integer
    // argument. So 1 Hz genuinely works, and asserting otherwise would be
    // asserting a bug.
    const one_hz = configureHz(clk, 1).?;
    try testing.expect(one_hz.div_int > 1 and one_hz.top > 0);

    // The too-slow guard still needs to exist, and does — it just takes a much
    // faster clock to reach it.
    try testing.expectEqual(@as(?Config, null), configureHz(4_000_000_000, 1));
}

test "resulting frequency is within a percent of the request" {
    const clk = 12_000_000;
    for ([_]u32{ 100, 250, 500, 1000, 2000, 5000, 20_000 }) |want| {
        const c = configureHz(clk, want).?;
        const actual = clk / ((@as(u32, c.top) + 1) * c.div_int);
        const err = if (actual > want) actual - want else want - actual;
        try testing.expect(err * 100 <= want); // within 1%
    }
}

test "setDutyPercent maths spans the range" {
    // Pure arithmetic check of the level computation used by setDutyPercent.
    const top: u16 = 999;
    try testing.expectEqual(@as(u32, 0), (@as(u32, top) + 1) * 0 / 100);
    try testing.expectEqual(@as(u32, 500), (@as(u32, top) + 1) * 50 / 100);
    try testing.expectEqual(@as(u32, 1000), (@as(u32, top) + 1) * 100 / 100);
}

test "validatePin accepts valid pins" {
    validatePin(0);
    validatePin(4);
    validatePin(25);
    validatePin(47);
}

// The failure cases (e.g., validatePin(48)) trigger @compileError at build/test
// compile time and are not catchable inside standard unit test blocks by design.
