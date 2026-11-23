//! Compile-time pin and function validation.
//!
//! **This is the reason this library exists rather than being a thinner wrapper
//! over registers.** On RP2350 every pin mistake fails silently: assign SCK to a
//! pin that cannot do SCK and the peripheral configures cleanly, the pad sits
//! there, and nothing happens. Claim one pin for two peripherals and whichever
//! ran last quietly wins. Neither produces an error, a fault, or any diagnostic
//! at all.
//!
//! Both are decidable from static data, so both are caught at **compile time**
//! here — something a C HAL structurally cannot do, because the function table
//! is documentation there and data here.
//!
//! Deliberately *validation*, not *generation*: a board declares what it has
//! wired and this checks it. Generating the setup code from the same
//! declaration is possible and might come later, but validation delivers nearly
//! all of the safety for a fraction of the API disruption.
//!
//! Everything in this file is comptime-only and costs zero flash.

const std = @import("std");
const hw = @import("hw.zig");

pub const Pin = hw.Pin;

// ---------------------------------------------------------------------------
// The function table
// ---------------------------------------------------------------------------
// Encoded from the RP2350 datasheet's GPIO function-select table, all 48 rows
// checked rather than extrapolated — which mattered, see the UART note below.

pub const SpiRole = enum { rx, csn, sck, tx };
pub const UartRole = enum { tx, rx, cts, rts };
pub const I2cRole = enum { sda, scl };
pub const PwmChannel = enum { a, b };

pub const Function = union(enum) {
    /// Software-driven pin. Available on every GPIO.
    sio,
    spi: struct { instance: u1, role: SpiRole },
    uart: struct { instance: u1, role: UartRole },
    i2c: struct { instance: u1, role: I2cRole },
    pwm: struct { slice: u3, channel: PwmChannel },
    pio: u2,
    /// Analogue input. The pad takes no digital function and its input buffer
    /// is disabled — see `adc.initPin`. Spelled out rather than reusing `none`
    /// so a claims list says what a pin is *for*, not merely what it is not.
    analog,
    /// Explicitly unconnected.
    none,
};

/// SPI instance for a pin: eight-pin blocks alternating SPI0/SPI1.
fn spiInstance(pin: Pin) u1 {
    return @truncate((pin / 8) % 2);
}

/// UART instance for a pin.
///
/// **This one is NOT a simple alternation, and assuming it was would have been
/// a bug.** Four-pin groups run UART0, UART1, UART1, UART0 and then repeat —
/// so pins 8-11 are UART1 where naive alternation predicts UART0. Verified
/// against every row of the datasheet table rather than extrapolated from the
/// first few, precisely because the first few look regular.
fn uartInstance(pin: Pin) u1 {
    const by_quad = [4]u1{ 0, 1, 1, 0 };
    return by_quad[(pin / 4) % 4];
}

/// I2C instance: pin pairs alternating I2C0/I2C1.
fn i2cInstance(pin: Pin) u1 {
    return @truncate((pin / 2) % 2);
}

/// PWM slice: pin pairs, eight slices, wrapping.
fn pwmSlice(pin: Pin) u3 {
    return @truncate((pin / 2) % 8);
}

/// The ADC channel behind `pin`, or null if it has none.
///
/// **Package-dependent, and the two are not the same pins**: QFN-60 puts ADC0-3
/// on GP26-29, QFN-80 puts ADC0-7 on GP40-47 (RP2350 datasheet, package pinout
/// tables). `max_pin` identifies the package, the same argument `validate`
/// already takes, so this file still holds no board knowledge — only chip.
///
/// The trap this exists for: Pimoroni's RP2350B boards silkscreen the analogue
/// pads `26`/`27`/`28` for Pico compatibility and wire them to GP40/41/42
/// through a series resistor. Code written against the silkscreen configures
/// GP26, which on that package is an ordinary digital pin, and then reads a
/// converter input that is not connected to anything. There is no diagnostic.
pub fn adcChannel(pin: Pin, max_pin: Pin) ?u4 {
    return switch (max_pin) {
        29 => if (pin >= 26 and pin <= 29) @intCast(pin - 26) else null,
        47 => if (pin >= 40 and pin <= 47) @intCast(pin - 40) else null,
        else => null,
    };
}

/// The FUNCSEL value that gives `pin` the requested `function`, or null if this
/// pin physically cannot do it.
pub fn funcselFor(pin: Pin, function: Function) ?u32 {
    return switch (function) {
        .sio => hw.IoBank0.func_sio,
        .analog, .none => hw.IoBank0.func_null,

        .spi => |s| blk: {
            if (spiInstance(pin) != s.instance) break :blk null;
            if (@as(SpiRole, @fromBackingInt(@intCast(pin % 4))) != s.role) break :blk null;
            break :blk hw.IoBank0.func_spi;
        },

        .uart => |u| blk: {
            if (uartInstance(pin) != u.instance) break :blk null;
            if (@as(UartRole, @fromBackingInt(@intCast(pin % 4))) != u.role) break :blk null;
            break :blk hw.IoBank0.func_uart;
        },

        .i2c => |i| blk: {
            if (i2cInstance(pin) != i.instance) break :blk null;
            const role: I2cRole = if (pin % 2 == 0) .sda else .scl;
            if (role != i.role) break :blk null;
            break :blk 3;
        },

        .pwm => |p| blk: {
            if (pwmSlice(pin) != p.slice) break :blk null;
            const ch: PwmChannel = if (pin % 2 == 0) .a else .b;
            if (ch != p.channel) break :blk null;
            break :blk 4;
        },

        // PIO can drive any pin; FUNCSEL 6/7/8 select the instance.
        .pio => |n| 6 + @as(u32, n),
    };
}

pub fn supports(pin: Pin, function: Function) bool {
    return funcselFor(pin, function) != null;
}

// ---------------------------------------------------------------------------
// Board pin claims
// ---------------------------------------------------------------------------

/// One pin, claimed for one purpose. `what` is free text used only in the
/// compile error, so a failure names the thing the human recognises ("nRF24
/// SCK") rather than a pin number.
pub const Claim = struct {
    pin: Pin,
    function: Function,
    what: []const u8,
};

/// Validate a board's complete pin map at compile time.
///
/// Checks, in order:
///   1. every pin exists in this package,
///   2. every pin can actually perform the function claimed for it,
///   3. no pin is claimed twice.
///
/// `max_pin` is the package limit — 29 for QFN-60, 47 for QFN-80. Passing it in
/// rather than reading it keeps this file free of any board knowledge.
///
/// Call from a `comptime` block in the board file. A failure is a build error
/// naming the pin and the purpose, not a runtime surprise.
pub fn validate(comptime claims: []const Claim, comptime max_pin: Pin) void {
    comptime {
        for (claims) |c| {
            if (c.pin > max_pin) {
                @compileError(std.fmt.comptimePrint(
                    "pin GP{d} ({s}) does not exist in this package (max GP{d})",
                    .{ c.pin, c.what, max_pin },
                ));
            }
            // `.analog` cannot be checked by `supports` — it maps to FUNC_NULL,
            // which every pin can do — so it needs the package to answer, and
            // `max_pin` is the package. Without this an `.analog` claim on GP26
            // passes on an RP2350B, where GP26 has no converter behind it at
            // all: the pad configures, the ADC reads a different pin, and
            // nothing anywhere reports a fault. See `adcChannel`.
            if (c.function == .analog and adcChannel(c.pin, max_pin) == null) {
                @compileError(std.fmt.comptimePrint(
                    "pin GP{d} has no ADC channel in this package (max GP{d}) — claimed for '{s}'." ++
                        " On QFN-60 the analogue pins are GP26-29; on QFN-80 they are GP40-47",
                    .{ c.pin, max_pin, c.what },
                ));
            }
            if (!supports(c.pin, c.function)) {
                @compileError(std.fmt.comptimePrint(
                    "pin GP{d} cannot do {s} — claimed for '{s}'",
                    .{ c.pin, @tagName(c.function), c.what },
                ));
            }
        }
        for (claims, 0..) |a, i| {
            for (claims[i + 1 ..]) |b| {
                if (a.pin == b.pin) {
                    @compileError(std.fmt.comptimePrint(
                        "pin GP{d} claimed twice: '{s}' and '{s}'",
                        .{ a.pin, a.what, b.what },
                    ));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "SPI: the pins this project actually uses" {
    // GP16/17/18/19 = SPI0 RX/CSn/SCK/TX, straight off the datasheet row.
    try testing.expect(supports(16, .{ .spi = .{ .instance = 0, .role = .rx } }));
    try testing.expect(supports(17, .{ .spi = .{ .instance = 0, .role = .csn } }));
    try testing.expect(supports(18, .{ .spi = .{ .instance = 0, .role = .sck } }));
    try testing.expect(supports(19, .{ .spi = .{ .instance = 0, .role = .tx } }));

    // The mistake this exists to catch: SCK on a pin that only does RX.
    try testing.expect(!supports(16, .{ .spi = .{ .instance = 0, .role = .sck } }));
    // And the right role on the wrong instance.
    try testing.expect(!supports(16, .{ .spi = .{ .instance = 1, .role = .rx } }));

    // Eight-pin blocks alternate: 12 is SPI1, 20 is back to SPI0.
    try testing.expect(supports(12, .{ .spi = .{ .instance = 1, .role = .rx } }));
    try testing.expect(supports(20, .{ .spi = .{ .instance = 0, .role = .rx } }));
}

test "UART: the irregular instance mapping, spot-checked against every quad" {
    // This is the table that would have been wrong if derived from the first
    // few rows. Each entry is a datasheet row.
    const expect_uart1 = [_]Pin{ 4, 8, 9, 10, 11, 20, 24, 36, 40 };
    for (expect_uart1) |p| {
        try testing.expectEqual(@as(u1, 1), uartInstance(p));
    }
    const expect_uart0 = [_]Pin{ 0, 12, 16, 28, 32, 44, 47 };
    for (expect_uart0) |p| {
        try testing.expectEqual(@as(u1, 0), uartInstance(p));
    }

    // The console wiring: GP0 = UART0 TX, GP1 = UART0 RX.
    try testing.expect(supports(0, .{ .uart = .{ .instance = 0, .role = .tx } }));
    try testing.expect(supports(1, .{ .uart = .{ .instance = 0, .role = .rx } }));
    // Swapping TX and RX is a real and easy mistake; it must not validate.
    try testing.expect(!supports(0, .{ .uart = .{ .instance = 0, .role = .rx } }));
}

test "I2C and PWM follow their regular patterns" {
    try testing.expect(supports(16, .{ .i2c = .{ .instance = 0, .role = .sda } }));
    try testing.expect(supports(17, .{ .i2c = .{ .instance = 0, .role = .scl } }));
    try testing.expect(supports(18, .{ .i2c = .{ .instance = 1, .role = .sda } }));
    // SDA on an odd pin is impossible.
    try testing.expect(!supports(17, .{ .i2c = .{ .instance = 0, .role = .sda } }));

    try testing.expect(supports(0, .{ .pwm = .{ .slice = 0, .channel = .a } }));
    try testing.expect(supports(1, .{ .pwm = .{ .slice = 0, .channel = .b } }));
    try testing.expect(supports(22, .{ .pwm = .{ .slice = 3, .channel = .a } }));
    try testing.expect(!supports(22, .{ .pwm = .{ .slice = 3, .channel = .b } }));
}

test "SIO and PIO are available on every pin" {
    for (0..48) |i| {
        const p: Pin = @intCast(i);
        try testing.expect(supports(p, .sio));
        try testing.expect(supports(p, .{ .pio = 0 }));
        try testing.expect(supports(p, .{ .pio = 2 }));
    }
    try testing.expectEqual(@as(?u32, 5), funcselFor(3, .sio));
    try testing.expectEqual(@as(?u32, 8), funcselFor(3, .{ .pio = 2 }));
}

test "validate accepts a correct map" {
    // Exactly this project's wiring — a regression test on the real board file.
    const claims = [_]Claim{
        .{ .pin = 0, .function = .{ .uart = .{ .instance = 0, .role = .tx } }, .what = "debug TX" },
        .{ .pin = 1, .function = .{ .uart = .{ .instance = 0, .role = .rx } }, .what = "debug RX" },
        .{ .pin = 16, .function = .{ .spi = .{ .instance = 0, .role = .rx } }, .what = "nRF24 MISO" },
        .{ .pin = 17, .function = .sio, .what = "nRF24 CSN" },
        .{ .pin = 18, .function = .{ .spi = .{ .instance = 0, .role = .sck } }, .what = "nRF24 SCK" },
        .{ .pin = 19, .function = .{ .spi = .{ .instance = 0, .role = .tx } }, .what = "nRF24 MOSI" },
        .{ .pin = 20, .function = .sio, .what = "nRF24 CE" },
        .{ .pin = 21, .function = .sio, .what = "nRF24 IRQ" },
    };
    validate(&claims, 47);
}

// The failure cases cannot be unit-tested — @compileError is not catchable, by
// design. They are verified by temporarily breaking board.zig and reading the
// build error each one produces.

test "adcChannel is package-dependent, which is the whole point" {
    // QFN-60 (RP2350A, Pico 2): GP26-29 are ADC0-3.
    try testing.expectEqual(@as(?u4, 0), adcChannel(26, 29));
    try testing.expectEqual(@as(?u4, 3), adcChannel(29, 29));
    try testing.expectEqual(@as(?u4, null), adcChannel(25, 29));

    // QFN-80 (RP2350B, Pico LiPo 2): GP40-47 are ADC0-7, and GP26 is not one.
    try testing.expectEqual(@as(?u4, 0), adcChannel(40, 47));
    try testing.expectEqual(@as(?u4, 7), adcChannel(47, 47));
    // **The failing case this check exists for.** A claim written from a
    // Pimoroni RP2350B board's silkscreen, where the pad is marked `26 / A0`.
    try testing.expectEqual(@as(?u4, null), adcChannel(26, 47));
    try testing.expectEqual(@as(?u4, null), adcChannel(27, 47));
}
