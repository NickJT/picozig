//! Raspberry Pi Pico 2 W — RP2350A, QFN-60, 30 GPIOs, and a CYW43439 radio on
//! GP23/24/25/29.
//!
//! **The radio owns more than it appears to.** The LED, VBUS sense and VSYS
//! sense that a Pico 2 has on CPU pads are all behind the CYW43 here, reachable
//! only through its firmware, and the four pads it uses are not spare. Every
//! one of those differences presents as "nothing happened", which is why the
//! board is a package rather than a comment.

const std = @import("std");
const rp2350 = @import("rp2350");
const pico = @import("pico_board");

const Pin = rp2350.Pin;

pub const Kind = pico.Kind;
pub const kind: Kind = .pico2w;
pub const name: []const u8 = "Raspberry Pi Pico 2 W (RP2350A)";

/// The package this build assumes. **Nothing checks this automatically — an
/// image has to ask**, by comparing it with `silicon.package()`. It *is* used
/// at comptime by the pin validation below.
pub const expected_package: rp2350.silicon.Package = .qfn60;

/// Whether GPIO32-47 exist.
pub const has_high_gpio: bool = false;

/// Null: clk_sys straight off the 12 MHz crystal. An application wanting more
/// — an A2DP sink does — sets its own plan.
pub const clock_plan: ?rp2350.clocks.Plan = null;

/// clk_sys after `clocks.init(clock_plan)`, in Hz.
pub const sys_hz: u32 = if (clock_plan) |p| p.sys_hz else rp2350.clocks.crystal_sys_hz;

// ---------------------------------------------------------------------------
// Onboard peripherals
// ---------------------------------------------------------------------------

/// **Null, and this is the trap the board package exists for**: the LED hangs
/// off the CYW43's WL_GPIO0, reachable only by uploading the radio's firmware.
/// GP25 is not spare either; it is the CYW43's chip select. So nothing drives
/// an LED without the radio, and verification goes through the SWD-readable
/// self-test block instead.
pub const led: ?Pin = null;
/// The SMPS mode pin is GP23, the radio's.
pub const smps_power_save: ?Pin = null;
/// Behind the radio, on the CYW43's WL_GPIO2.
pub const vbus_sense: ?Pin = null;
/// No battery and no charger.
pub const vbat_sense: ?Pin = null;
pub const vbat_adc_channel: ?u4 = null;
/// GP29 would read VSYS/3, but it is the radio's clock.
pub const vsys_sense: ?Pin = null;
pub const vsys_adc_channel: ?u4 = null;
/// BOOTSEL is not a GPIO — it is read through the QSPI CS pad, a different
/// mechanism and not modelled here.
pub const user_sw: ?Pin = null;

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

pub const uart_tx = pico.uart_tx;
pub const uart_rx = pico.uart_rx;
pub const spi_miso = pico.spi_miso;
pub const spi_csn = pico.spi_csn;
pub const spi_sck = pico.spi_sck;
pub const spi_mosi = pico.spi_mosi;
pub const nrf_ce = pico.nrf_ce;
pub const nrf_irq = pico.nrf_irq;
pub const cc_gdo0 = pico.cc_gdo0;
pub const cc_gdo2 = pico.cc_gdo2;
pub const selftest_pin = pico.selftest_pin;

/// What is soldered to this board: the radio's four pads, which nothing here
/// drives but which must not be handed to anything else.
pub const soldered_claims = pico.solderedClaims(.{
    .led = led,
    .user_sw = user_sw,
    .vbat_sense = vbat_sense,
    .vbus_sense = vbus_sense,
    .vsys_sense = vsys_sense,
    .smps_power_save = smps_power_save,
}, &.{
    .{ .pin = 23, .function = .none, .what = "RESERVED: CYW43 power/control" },
    .{ .pin = 24, .function = .none, .what = "RESERVED: CYW43 data" },
    .{ .pin = 25, .function = .none, .what = "RESERVED: CYW43 chip select" },
    .{ .pin = 29, .function = .none, .what = "RESERVED: CYW43 clock" },
});

/// The examples' wiring convention, which every Pico board shares.
pub const reference_pinout = pico.reference_pinout;

/// Everything, checked at compile time.
pub const claims = soldered_claims ++ reference_pinout;

comptime {
    rp2350.pins.validate(claims, expected_package.maxGpio());
}

/// The debug console, wired as above.
pub fn uartConfig() rp2350.uart.Config {
    return pico.uartConfig(sys_hz);
}

/// SPI0 as wired to the reference module socket.
pub fn spiConfig() rp2350.spi.Config {
    return pico.spiConfig(sys_hz);
}

test {
    std.testing.refAllDecls(@This());
}
