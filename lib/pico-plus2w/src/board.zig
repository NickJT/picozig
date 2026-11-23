//! Pimoroni Pico Plus 2 W (PIM722) — RP2350B, QFN-80, 48 GPIOs, an RM2 radio
//! module, a LiPo charger and 8 MB of PSRAM.
//!
//! **The radio is an RM2 module, not a bare CYW43439** — the same four pins
//! (GP23/24/25/29) and, where it matters, the same circuit: `WL_D` joins `DI`
//! directly, `DO` through 470R and `IRQ` through 10K, which is electrically
//! what a Pico 2 W does with `CMD`/`DATA0`/`DATA1`. The resistors are the whole
//! of the bus arbitration: whoever drives through the least resistance wins, so
//! the host at 0R beats the chip's data at 470R, which beats the interrupt at
//! 10K — and the interrupt can move the line only when both others have let
//! go, which is an idle bus.

const std = @import("std");
const rp2350 = @import("rp2350");
const pico = @import("pico_board");

const Pin = rp2350.Pin;

pub const Kind = pico.Kind;
pub const kind: Kind = .pico_plus2w;
pub const name: []const u8 = "Pimoroni Pico Plus 2 W (RP2350B)";

/// The package this build assumes. **Nothing checks this automatically — an
/// image has to ask**, by comparing it with `silicon.package()`. It *is* used
/// at comptime by the pin validation below.
pub const expected_package: rp2350.silicon.Package = .qfn80;

/// Whether GPIO32-47 exist.
pub const has_high_gpio: bool = true;

/// Null: clk_sys straight off the 12 MHz crystal.
pub const clock_plan: ?rp2350.clocks.Plan = null;

/// clk_sys after `clocks.init(clock_plan)`, in Hz.
pub const sys_hz: u32 = if (clock_plan) |p| p.sys_hz else rp2350.clocks.crystal_sys_hz;

// ---------------------------------------------------------------------------
// Onboard peripherals
// ---------------------------------------------------------------------------

/// **Null, the Pico 2 W's trap one module further out**: the schematic wires
/// the LED to the RM2's own `B1 GPIO0` as net `LED_WLGP0`, so it is behind the
/// radio firmware rather than on a CPU pad. GP25 is the RM2's chip select.
pub const led: ?Pin = null;
/// The SMPS mode pin is the radio's here.
pub const smps_power_save: ?Pin = null;

/// Behind the radio: `VBUS_SENSE` lands on the RM2's `C3 GPIO2`, matching the
/// SDK's `CYW43_WL_GPIO_VBUS_PIN 2`.
///
/// **And it is a logic input, not an analogue one.** R14/R15 are 10k and 10k,
/// so the divider halves 5 V VBUS to 2.5 V — not a measurement, a level shift
/// into something that reads a 1 or a 0.
pub const vbus_sense: ?Pin = null;

/// **Null on purpose, and not because there is no pin.** GP43/ADC3 carries
/// `VSYS_SENSE`, not the cell — VSYS is the rail after the power path, so it
/// reads the battery only while nothing else is powering the board. It is
/// `vsys_sense` below rather than a second meaning for this name.
pub const vbat_sense: ?Pin = null;
pub const vbat_adc_channel: ?u4 = null;

/// **VSYS**, through the divider below — the rail after the power path, which
/// is the battery when running on one and about 5 V when running from USB.
///
/// Distinct from `vbat_sense` on purpose: a caller that wants a *state of
/// charge* must know which of the two it is reading.
pub const vsys_sense: ?Pin = 43;
/// GP40..47 are ADC0..7 on the RP2350B, so GP43 is channel 3.
pub const vsys_adc_channel: ?u4 = 3;

/// VSYS sense divider, read off the schematic: **R17 = 150k top, R16 = 75k
/// bottom**, so the ADC sees **VSYS / 3**. A 5 V USB rail reads 1.67 V and a
/// full 4.2 V cell 1.4 V, both comfortably inside a 3.3 V reference.
pub const vsys_divider: u32 = 3;

/// **18.7 uA at 4.2 V, continuously, and nothing switches it off** — ten times
/// a 2.25 MOhm battery divider's, and the largest standing load recorded for
/// any board here.
///
/// **`Q1` does not gate it, which was the hopeful reading and is wrong.** The
/// transistor's terminals, read off the schematic: gate to **3V3**, drain to
/// the **R16/R17 junction**, source to **`VSYS_SENSE`**. So it sits between the
/// divider and the ADC pin — and R17-to-R16 is a series path from VSYS to
/// ground that does not pass through it.
///
/// What `Q1` is for is the case where **VSYS is present and 3V3 is not**: a
/// battery on a board that is switched off. The divider midpoint sits at 1.4 V
/// while the RP2350 has no supply, and without the transistor that would drive
/// GP43's ESD diode into the dead 3V3 rail — back-powering the chip through an
/// I/O pin. With the gate at 0 V the FET is off and the pin is isolated. It is
/// protection, not power saving.
pub const vsys_divider_drain_na: u32 = 18_700;

/// **`Q1` is ohmic in the whole working range, so the ADC reads `VSYS / 3` with
/// no offset to correct.** The topology — an N-channel FET with its gate on a
/// fixed rail and the signal on its source — is the shape of a source follower,
/// which would subtract a temperature-dependent Vgs from every reading. It is
/// not one here: at a full 4.2 V cell the source sits at 1.4 V, so Vgs is
/// 1.9 V, and the ADC draws no steady current, so the drop across the channel
/// is nil. It would leave that region only above about 7.5 V on VSYS.
///
/// **Source impedance is 150k || 75k = 50 kOhm**, with no filter capacitor on
/// the net, so a reading can be believed immediately — there is no settle time.
pub const vsys_source_ohms: u32 = 50_000;

/// GP45/ADC5, package pin 56, net `USER_SW`, active low.
pub const user_sw: ?Pin = 45;

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

/// What is soldered to this board: the RM2's four pads and the PSRAM's chip
/// select, which nothing here drives but which must not be handed out.
pub const soldered_claims = pico.solderedClaims(.{
    .led = led,
    .user_sw = user_sw,
    .vbat_sense = vbat_sense,
    .vbus_sense = vbus_sense,
    .vsys_sense = vsys_sense,
    .smps_power_save = smps_power_save,
}, &.{
    .{ .pin = 23, .function = .none, .what = "RESERVED: RM2 WL_ON" },
    .{ .pin = 24, .function = .none, .what = "RESERVED: RM2 WL_D (DI direct, DO 470R, IRQ 10K)" },
    .{ .pin = 25, .function = .none, .what = "RESERVED: RM2 WL_CS" },
    .{ .pin = 29, .function = .none, .what = "RESERVED: RM2 WL_CLK" },
    .{ .pin = 47, .function = .none, .what = "RESERVED: PSRAM chip select" },
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
