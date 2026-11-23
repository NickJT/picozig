//! Raspberry Pi Pico 2 — RP2350A, QFN-60, 30 GPIOs, **no radio**.
//!
//! Not a Pico 2 W with the radio ignored. The difference is four pins the W
//! spends on the CYW43 and this board puts to its own use: GP23 the SMPS
//! power-save control, GP24 VBUS sense, GP25 the LED, and **GP29 as ADC3
//! reading VSYS/3**. Under the W's map all four are reserved, which costs an
//! LED, a quarter of the converter channels, and — for an ADC application,
//! which is why this board exists here — the documented control over ADC supply
//! ripple. See `smps_power_save`.

const std = @import("std");
const rp2350 = @import("rp2350");
const pico = @import("pico_board");

const Pin = rp2350.Pin;

pub const Kind = pico.Kind;
pub const kind: Kind = .pico2;
pub const name: []const u8 = "Raspberry Pi Pico 2 (RP2350A)";

/// The package this build assumes. **Nothing checks this automatically — an
/// image has to ask**, by comparing it with `silicon.package()`. It *is* used
/// at comptime by the pin validation below.
pub const expected_package: rp2350.silicon.Package = .qfn60;

/// Whether GPIO32-47 exist.
pub const has_high_gpio: bool = false;

/// Null: clk_sys straight off the 12 MHz crystal.
pub const clock_plan: ?rp2350.clocks.Plan = null;

/// clk_sys after `clocks.init(clock_plan)`, in Hz.
pub const sys_hz: u32 = if (clock_plan) |p| p.sys_hz else rp2350.clocks.crystal_sys_hz;

// ---------------------------------------------------------------------------
// Onboard peripherals
// ---------------------------------------------------------------------------

/// GP25, driven directly by the CPU — the ordinary case, and the one the W
/// does not have.
pub const led: ?Pin = 25;

/// **The SMPS power-save control. DO NOT DRIVE IT HIGH — see the warning.**
///
/// > **2026-09-06, measured on a bench Pico 2: driving this pad high took 3V3
/// > to 4.65 V**, against the RP2350's 3.135–3.63 V range. Same board, same
/// > firmware, same wiring — the only variable was whether this pad was driven,
/// > confirmed by flashing a stock `advanced_uart.uf2` (3.27 V) and then an ADC
/// > application with and without it.
/// >
/// > Nothing external can explain it: **GP23 is not on the 40-pin header**, so
/// > the path is entirely between the RP2350 and the RT6150. Either that
/// > board's regulator is faulty, or a Pico 2 does this at the very light load
/// > of a 12 MHz core. **Which one is not established**, and finding out costs
/// > a board if the answer is the second.
/// >
/// > The failure is silent and total: `ADC_VREF` is 3V3, so every conversion on
/// > the chip is scaled by 1.4 and nothing reports an error. It took hours to
/// > find, and the datasheet advice below is what led there.
///
/// The datasheet's own words, recorded but **not endorsed**:
///
/// Pico 2 datasheet, on ADC performance: *"Driving high the SMPS mode pin
/// (GPIO23), to force the power supply into PWM mode, can greatly reduce the
/// inherent ripple of the SMPS at light load, and therefore the ripple on the
/// ADC supply."* Low is the board's default and is PFM, which is more
/// efficient and noisier.
///
/// Exposed rather than reserved so that a caller *can* reach it, and so that
/// this warning has somewhere to live. An application should default to never
/// configuring the pad at all; at reset it is an input with a pull-down, which
/// is PFM and is what the board ships with.
///
/// If it is ever tried again: meter on 3V3, hand on the power.
pub const smps_power_save: ?Pin = 23;

/// GP24, a logic input: high while VBUS is present.
pub const vbus_sense: ?Pin = 24;

/// No battery and no charger; `vsys_sense` is the only rail here.
pub const vbat_sense: ?Pin = null;
pub const vbat_adc_channel: ?u4 = null;

/// **GP29 = ADC3, reading VSYS/3** (Pico 2 datasheet, "A few RP2350 GPIO pins
/// are used for internal board functions").
pub const vsys_sense: ?Pin = 29;
/// GP26..29 are ADC0..3 on QFN-60, so GP29 is channel 3.
pub const vsys_adc_channel: ?u4 = 3;

/// The ADC sees **VSYS / 3**.
pub const vsys_divider: u32 = 3;

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

/// What is soldered to this board. **Nothing is reserved**: every pin the
/// Pico 2 commits to onboard hardware is modelled above as the thing it is —
/// LED, VBUS sense, VSYS sense, SMPS control — so each is claimed with a real
/// function and stays usable.
pub const soldered_claims = pico.solderedClaims(.{
    .led = led,
    .user_sw = user_sw,
    .vbat_sense = vbat_sense,
    .vbus_sense = vbus_sense,
    .vsys_sense = vsys_sense,
    .smps_power_save = smps_power_save,
}, &.{});

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
