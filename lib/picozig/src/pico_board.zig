//! What every Pico board shares, so each board package states only its own facts.
//!
//! A board is its own package — `lib/pico2`, `lib/pico2w`, `lib/pico-lipo2`,
//! `lib/pico-plus2w` — exporting one `board` module. That module imports this
//! one for three things: the list of boards, the **reference wiring** the
//! examples use, and the helpers that turn a board's facts into validated pin
//! claims and ready-made peripheral configs.
//!
//! Depends on the chip layer only. Nothing here knows which board it is on;
//! the board module hands it `sys_hz` and its own pins.

const std = @import("std");
const rp2350 = @import("rp2350");

const Pin = rp2350.Pin;
const Claim = rp2350.pins.Claim;

/// The boards there is a package for.
pub const Kind = enum {
    /// Pimoroni Pico LiPo 2, RP2350B (QFN-80, 48 GPIOs).
    pico_lipo2,
    /// Raspberry Pi Pico 2 W, RP2350A (QFN-60, 30 GPIOs).
    pico2w,
    /// Raspberry Pi Pico 2, RP2350A (QFN-60, 30 GPIOs) — no radio.
    pico2,
    /// Pimoroni Pico Plus 2 W (PIM722), RP2350B (QFN-80, 48 GPIOs).
    pico_plus2w,
};

// ---------------------------------------------------------------------------
// The reference wiring
// ---------------------------------------------------------------------------
//
// **A wiring convention, not a board fact.** What this workspace's examples
// plug in, identical on every Pico board because every pin is inside QFN-60's
// 0-29 range. Nothing here is soldered to any board; an application wiring its
// own peripherals validates against `soldered_claims` instead and takes any of
// these pins it likes.

/// Debug UART. GP0/GP1 are UART0 TX/RX under function 2, and are what the
/// Raspberry Pi Debug Probe's UART cable expects: probe TX (orange) -> GP1,
/// probe RX (yellow) -> GP0.
pub const uart_tx: Pin = 0;
pub const uart_rx: Pin = 1;

/// SPI0 for a radio or display module. GP16/18/19 take SPI0 under function 1.
///
/// **CSN is a plain GPIO, not the PL022's hardware CSn**, and that is required
/// rather than a simplification: the PL022 asserts chip select per *frame*, so
/// it would deassert between every byte, whereas an nRF24 or a CC1101 needs CSN
/// held low for a whole command-plus-payload transaction and treats the rising
/// edge as "command complete". Wiring the radio to SPI0 CSn on GP17 would
/// produce a radio that ACKs nothing and looks dead.
pub const spi_miso: Pin = 16; // SPI0 RX
pub const spi_csn: Pin = 17; // plain SIO output — see above
pub const spi_sck: Pin = 18; // SPI0 SCK
pub const spi_mosi: Pin = 19; // SPI0 TX

/// nRF24 mode control (standby vs TX/RX) — plain output.
pub const nrf_ce: Pin = 20;
/// nRF24 interrupt, active low — plain input, pull-**UP** (the direction
/// erratum E9 leaves working; see `rp2350.silicon`).
pub const nrf_irq: Pin = 21;

/// A CC1101 on the same module socket reuses the nRF24's two pins: these are
/// **aliases, not new assignments** — only one radio is ever fitted, and the
/// pins are claimed once. Naming them twice is what lets CC1101 code read as
/// CC1101 code rather than as nRF24 code borrowed.
///
/// The CC1101 has no CE: mode is set by SPI strobes, and these carry its
/// configurable GDO status outputs instead.
pub const cc_gdo0: Pin = nrf_ce;
pub const cc_gdo2: Pin = nrf_irq;

/// A pin that exists and is unconnected on every supported board, used for the
/// GPIO loopback self-test. GP15 is inside QFN-60's range and clear of the
/// radio's GP23/24/25/29 on the W boards.
pub const selftest_pin: Pin = 15;

/// The reference wiring as claims, for a board to validate alongside its own.
pub const reference_pinout: []const Claim = &.{
    .{ .pin = uart_tx, .function = .{ .uart = .{ .instance = 0, .role = .tx } }, .what = "debug console TX" },
    .{ .pin = uart_rx, .function = .{ .uart = .{ .instance = 0, .role = .rx } }, .what = "debug console RX" },
    // **SPI0, not "the nRF24's".** Radio and display drivers both use this bus.
    // Labelling four working pins after one peripheral is a trap: the next
    // reader frees them and silently breaks the others.
    .{ .pin = spi_miso, .function = .{ .spi = .{ .instance = 0, .role = .rx } }, .what = "SPI0 MISO (cc1101, displays, nrf24)" },
    .{ .pin = spi_csn, .function = .sio, .what = "SPI0 CSn — plain GPIO, see above" },
    .{ .pin = spi_sck, .function = .{ .spi = .{ .instance = 0, .role = .sck } }, .what = "SPI0 SCK (cc1101, displays, nrf24)" },
    .{ .pin = spi_mosi, .function = .{ .spi = .{ .instance = 0, .role = .tx } }, .what = "SPI0 MOSI (cc1101, displays, nrf24)" },
    .{ .pin = nrf_ce, .function = .sio, .what = "nRF24 CE (nrf24 example only)" },
    .{ .pin = nrf_irq, .function = .sio, .what = "nRF24 IRQ (nrf24 example only)" },
    .{ .pin = selftest_pin, .function = .sio, .what = "GPIO loopback self-test" },
};

// ---------------------------------------------------------------------------
// Helpers a board module calls with its own facts
// ---------------------------------------------------------------------------

/// The onboard hardware a Pico board can have, as the board states it.
pub const Onboard = struct {
    led: ?Pin,
    user_sw: ?Pin,
    vbat_sense: ?Pin,
    vbus_sense: ?Pin,
    vsys_sense: ?Pin,
    smps_power_save: ?Pin,
};

/// **What is soldered to a board**, as claims: its onboard hardware, then the
/// pins it commits to hardware nothing here drives (`reserved`), claimed purely
/// so that nothing else can take them.
///
/// Without the reserved list the validator cannot know those pins are spoken
/// for — reassigning an LED to GP47 compiled happily before it existed, which
/// would have meant fighting the PSRAM for a pin with no diagnostic at all.
pub fn solderedClaims(comptime on: Onboard, comptime reserved: []const Claim) []const Claim {
    const claims = comptime blk: {
        var list: []const Claim = &.{};
        if (on.led) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .sio, .what = "onboard LED" }};
        if (on.user_sw) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .sio, .what = "user switch" }};
        if (on.vbat_sense) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .analog, .what = "VBAT sense (through the /3 divider)" }};
        if (on.vbus_sense) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .sio, .what = "VBUS sense" }};
        if (on.vsys_sense) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .analog, .what = "VSYS sense (through the /3 divider)" }};
        if (on.smps_power_save) |p| list = list ++ [_]Claim{.{ .pin = p, .function = .sio, .what = "SMPS power-save control — see the warning on that declaration" }};
        break :blk list ++ reserved;
    };
    return claims;
}

/// The debug console at the board's clk_sys.
pub fn uartConfig(sys_hz: u32) rp2350.uart.Config {
    return .{ .inst = .uart0, .tx = uart_tx, .rx = uart_rx, .clk_hz = sys_hz };
}

/// SPI0 as wired to the reference module socket, at the board's clk_sys.
pub fn spiConfig(sys_hz: u32) rp2350.spi.Config {
    return .{ .miso = spi_miso, .sck = spi_sck, .mosi = spi_mosi, .clk_hz = sys_hz };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;
const wired = [_]Pin{ uart_tx, uart_rx, spi_miso, spi_csn, spi_sck, spi_mosi, nrf_ce, nrf_irq, selftest_pin };

test "the reference wiring fits every board, QFN-60 included" {
    for (wired) |p| try testing.expect(p <= 29);
}

test "no reference pin is assigned twice" {
    for (wired, 0..) |a, i| {
        for (wired[i + 1 ..]) |b| try testing.expect(a != b);
    }
}

test "soldered claims keep their order: onboard hardware, then reserved" {
    const claims = solderedClaims(.{
        .led = 25,
        .user_sw = null,
        .vbat_sense = null,
        .vbus_sense = 24,
        .vsys_sense = null,
        .smps_power_save = null,
    }, &.{.{ .pin = 47, .function = .none, .what = "RESERVED" }});
    try testing.expectEqual(@as(usize, 3), claims.len);
    try testing.expectEqual(@as(Pin, 25), claims[0].pin);
    try testing.expectEqual(@as(Pin, 24), claims[1].pin);
    try testing.expectEqual(@as(Pin, 47), claims[2].pin);
}
