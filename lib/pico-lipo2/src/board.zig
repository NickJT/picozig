//! Pimoroni Pico LiPo 2 (PIM775) — RP2350B, QFN-80, 48 GPIOs, a LiPo charger,
//! a battery-sense divider, a user switch and 8 MB of PSRAM.
//!
//! **The point of a board file is to state facts, not take parameters.** The
//! divider ratio, the charge current, which pin the LED is on — all of these
//! are knowable, so they are recorded here rather than asked of the caller.
//! Depends on the chip layer and `pico_board`; the chip layer never depends on
//! this.

const std = @import("std");
const rp2350 = @import("rp2350");
const pico = @import("pico_board");

const Pin = rp2350.Pin;

pub const Kind = pico.Kind;
pub const kind: Kind = .pico_lipo2;
pub const name: []const u8 = "Pimoroni Pico LiPo 2 (RP2350B)";

/// The package this build assumes. **Nothing checks this automatically — an
/// image has to ask**, by comparing it with `silicon.package()`. It *is* used
/// at comptime by the pin validation below, which is why a wrong value is
/// over-restrictive rather than dangerous.
pub const expected_package: rp2350.silicon.Package = .qfn80;

/// Whether GPIO32-47 exist — i.e. whether SIO's HI register bank can be
/// exercised against real pads.
pub const has_high_gpio: bool = true;

/// How this board runs clk_sys. Null means straight off the crystal with no
/// PLL: the board's decision, not the chip layer's, which only knows how to
/// reach a frequency. Hand it to `clocks.init`.
pub const clock_plan: ?rp2350.clocks.Plan = null;

/// clk_sys after `clocks.init(clock_plan)`, in Hz. **Every divisor in the
/// system derives from this**, so it follows the plan rather than being
/// maintained beside it.
pub const sys_hz: u32 = if (clock_plan) |p| p.sys_hz else rp2350.clocks.crystal_sys_hz;

// ---------------------------------------------------------------------------
// Onboard peripherals
// ---------------------------------------------------------------------------

/// Onboard LED, on a CPU pad.
pub const led: ?Pin = 25;
/// No SMPS power-save pin on this board's regulator.
pub const smps_power_save: ?Pin = null;
/// USB power present, a logic input.
pub const vbus_sense: ?Pin = 24;
/// Battery voltage, through the divider below. **ADC channel 3** — ADC0..7 are
/// GP40..47 on the RP2350B.
pub const vbat_sense: ?Pin = 43;
/// ADC channel behind `vbat_sense`, kept next to the pin so the two cannot
/// drift apart.
pub const vbat_adc_channel: ?u4 = 3;
/// No separate VSYS sense: `vbat_sense` reads the cell itself.
pub const vsys_sense: ?Pin = null;
pub const vsys_adc_channel: ?u4 = null;
/// User switch, active low. GP45 — it does not exist on QFN-60.
pub const user_sw: ?Pin = 45;

/// VBAT sense divider, read off the official schematic — this ratio is
/// published nowhere else. R19 = 1M5 top, R20 = 750k bottom, so the ADC sees
/// **VBAT / 3**. A full 4.2 V cell reads about 1.4 V against the 3.3 V
/// reference, comfortably in range.
pub const vbat_divider: u32 = 3;

/// **The divider is RC-filtered, and it must be allowed to settle before the
/// first reading is believed.**
///
/// The schematic puts C20 = 100 nF across the divider output. Against the
/// 500 kOhm Thevenin source that is a time constant of **50 ms**, so ~250 ms is
/// needed for 99%. Measured the hard way: a boot-time reading came back as
/// 192 mV where the true battery was 4.25 V — 4.5% of final, exactly what an RC
/// curve gives about 2.3 ms after power-on, which is when boot reaches the ADC.
///
/// A wrong-but-plausible small number is the worst kind of wrong for a battery
/// gauge: it reads as "flat" and would trigger a low-battery alarm on a fully
/// charged cell.
///
/// **Only cold boot is affected.** The divider is permanently connected, so the
/// cap stays charged as long as the battery is; a reading taken after a dormant
/// wake needs no wait.
pub const vbat_settle_ms: u32 = 250;

/// The divider is permanently connected with no switching FET, so it draws
/// VBAT / 2.25 MOhm ~= 1.9 uA continuously — a floor that cannot be designed
/// away, and belongs in the power budget from the start.
pub const vbat_divider_drain_na: u32 = 1900;

/// LiPo charger: MCP73831 with R18 = 4k7 on PROG, giving ~213 mA
/// (1000 V / R_PROG).
pub const charge_current_ma: u32 = 213;

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

/// What is soldered to this board, and nothing else. An application wiring its
/// own peripherals validates against this rather than `claims`.
pub const soldered_claims = pico.solderedClaims(.{
    .led = led,
    .user_sw = user_sw,
    .vbat_sense = vbat_sense,
    .vbus_sense = vbus_sense,
    .vsys_sense = vsys_sense,
    .smps_power_save = smps_power_save,
}, &.{
    .{ .pin = 47, .function = .none, .what = "RESERVED: PSRAM chip select" },
});

/// The examples' wiring convention, which every Pico board shares.
pub const reference_pinout = pico.reference_pinout;

/// Everything: the board's own hardware plus the reference pinout. Checked at
/// **compile time** — that each pin exists in this package, can perform the
/// function claimed, and is claimed once. All three are silent failures at
/// runtime on this chip.
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test {
    testing.refAllDecls(@This());
}

test "pins land in the SIO register bank their wiring requires" {
    // VBAT sense and the user switch are above GPIO31 — the entire reason
    // `Pin` is a u6. Narrowed back to u5, these would not compile.
    try testing.expect(vbat_sense.? >= 32);
    try testing.expect(user_sw.? >= 32);
    try testing.expect(led.? < 32);
    try testing.expect(vbus_sense.? < 32);
}

test "battery divider keeps a full cell inside the ADC reference" {
    // 4.2 V through the divider must land under 3.3 V, or readings clip and the
    // telemetry silently saturates near full charge.
    const full_cell_mv: u32 = 4200;
    try testing.expect(full_cell_mv / vbat_divider < 3300);
}
