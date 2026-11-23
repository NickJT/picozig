//! 12-bit SAR ADC.
//!
//! ## The clock decision, which is not the obvious one
//!
//! The datasheet says clk_adc **"must be 48MHz"**, normally from PLL_USB, giving
//! 500 kS/s (96 clk_adc cycles per conversion). This project runs off the 12 MHz
//! crystal with no PLL at all, deliberately — so `initClock` sources clk_adc
//! from XOSC and conversions take 8 us instead of 2 us.
//!
//! That is out of the datasheet's stated spec, and it is still the right choice
//! here, for a reason that inverts the usual worry:
//!
//! * **Sample rate is irrelevant.** This ADC reads a battery voltage every few
//!   seconds at most. 125 kS/s is four orders of magnitude more than needed.
//! * **The slower clock actively helps.** The Pico LiPo 2's VBAT divider
//!   (1M5 / 750k) presents a Thevenin source impedance of **500 kOhm**, which is
//!   enormous for a switched-capacitor SAR input — the sample-and-hold has to
//!   charge through it. A longer aperture is exactly what a high-impedance
//!   source needs, so 12 MHz gives *better* settling than 48 MHz would.
//! * **Running a PLL to read a battery is the wrong trade** on a device whose
//!   entire idle budget is a few hundred microamps.
//!
//! Expect a residual downward bias on high-impedance channels regardless; see
//! `readAveraged` and the note on discarding the first conversion.
//!
//! **That argument is about a default, not about a limit.** An instrument that
//! wants the datasheet's 500 kS/s says so: `initClockFromPllSys` sources
//! clk_adc from the system PLL and `startStream` runs the FIFO into DMA. The
//! two entry points exist side by side because the right clock is a property of
//! the application, and a driver that decided it for everyone would be wrong
//! for one of them.
//!
//! ## Hazard: touching the ADC with clk_adc stopped is a bus fault
//!
//! The datasheet lists "accessed ADC whilst clk_adc [is not running]" as a cause
//! of an APB timeout fault. `init` sets the clock up first for that reason, and
//! nothing here should be called before it.

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");
const gpio = @import("gpio.zig");
const silicon = @import("silicon.zig");

/// ADC channel. On QFN-80 (RP2350B) channels 0-7 are GP40-47; on QFN-60
/// (RP2350A) channels 0-3 are GP26-29. The temperature sensor is the channel
/// *after* the last external one, which differs by package — see `tempChannel`.
pub const Channel = u4;

/// Reference voltage in millivolts. The RP2350 ADC references ADC_VREF, which
/// on the supported boards is a filtered copy of the 3.3 V rail.
///
/// **Nominal, and per-chip rather than per-board.** Measured 2026-09-06 on the
/// bench Pico 2 (`9BCEDF98B4655F9D`): **3234 mV**, so every absolute voltage
/// computed from this constant is **2.02% high** there. ADC_VREF also sits
/// ~35 mV under 3V3 on that board, being filtered rather than the same node.
///
/// A caller that needs better than a couple of percent must measure its own
/// board and supply the figure, for example as a `-Dvref-mv` build option. Two
/// known voltages on two channels are enough, and a resistor ladder off 3V3 is
/// self-checking because the ladder and the reference share a rail.
pub const vref_mv: u32 = 3300;

/// Full scale of the 12-bit result.
pub const full_scale: u32 = 4096;

/// The temperature sensor's channel number, which is **package-dependent**:
/// AINSEL=4 on QFN-60, AINSEL=8 on QFN-80. Reading the wrong one gives a
/// plausible-looking voltage from an unconnected pin rather than an error, so
/// this is asked of the silicon rather than assumed.
pub fn tempChannel() Channel {
    return switch (silicon.package()) {
        .qfn60 => 4,
        .qfn80 => 8,
    };
}

/// Point clk_adc at the crystal and enable it. See the header for why XOSC
/// rather than a 48 MHz PLL.
///
/// Separate from `init` because it touches the clocks block rather than the ADC,
/// and because a caller that has a PLL running may want to source it differently.
pub fn initClock() void {
    hw.write(hw.Clocks.clk_adc_div, hw.Clocks.div_by_1);
    hw.write(
        hw.Clocks.clk_adc_ctrl,
        hw.Clocks.adc_auxsrc_xosc | hw.Clocks.adc_enable,
    );
}

/// The exact integer that divides `sys_hz` to 48 MHz, or a compile error.
///
/// **`CLK_ADC_DIV.INT` is four bits wide with no fractional part**, so the only
/// system clocks that can feed the ADC at all are 48 MHz times 1 through 15.
/// 144 MHz divides by 3; 150 MHz divides by nothing — 150/3 is 50 MHz, four
/// percent out of specification, and there is no fractional field to rescue it.
/// Getting this wrong is otherwise silent: the converter runs at the wrong rate
/// and every sample interval is a lie.
pub fn pllSysDivisor(comptime sys_hz: u32) u4 {
    comptime {
        if (sys_hz % required_clk_hz != 0) @compileError(std.fmt.comptimePrint(
            "clk_adc must be {d} Hz and CLK_ADC_DIV.INT is a 4-bit integer with no fraction, " ++
                "so clk_sys must be an exact multiple of it. {d} Hz is not: {d}/{d} = {d}.{d:0>3}. " ++
                "144 MHz works (divisor 3); 150 MHz cannot.",
            .{ required_clk_hz, sys_hz, sys_hz, required_clk_hz, sys_hz / required_clk_hz, (sys_hz % required_clk_hz) * 1000 / required_clk_hz },
        ));
        const n = sys_hz / required_clk_hz;
        if (n < 1 or n > 15) @compileError(std.fmt.comptimePrint(
            "CLK_ADC_DIV.INT is 4 bits, so the divisor must be 1..15; {d} Hz needs {d}.",
            .{ sys_hz, n },
        ));
    }
    return @intCast(sys_hz / required_clk_hz);
}

/// clk_adc as the datasheet requires it.
pub const required_clk_hz: u32 = 48_000_000;

/// Conversion time in clk_adc cycles, free-running.
pub const cycles_per_conversion: u32 = 96;

/// Point clk_adc at the **system** PLL, for an application that needs the
/// datasheet's 500 kS/s rather than the crystal's 125.
///
/// Pass `board.sys_hz`. The divisor is checked at compile time, so a board
/// whose clock cannot reach 48 MHz fails the build instead of the bench.
pub fn initClockFromPllSys(comptime sys_hz: u32) void {
    const n = pllSysDivisor(sys_hz);
    hw.write(hw.Clocks.clk_adc_div, hw.Clocks.adcDiv(n));
    hw.write(
        hw.Clocks.clk_adc_ctrl,
        hw.Clocks.adc_auxsrc_pll_sys | hw.Clocks.adc_enable,
    );
}

/// Bring the ADC up. Call `initClock` first — or let this do it.
///
/// Bounded: an ADC whose clock never arrived never reports ready, and on a
/// battery device this is the path that reads VBAT — a battery device that hangs while
/// measuring its own battery is a particularly unhelpful failure.
pub fn init() bounded.Error!void {
    initClock();
    try enable();
}

/// Unreset and power up the converter, leaving clk_adc alone.
///
/// **Separate from `init` because the aux mux feeding clk_adc is not
/// glitchless.** An application that wants the PLL source calls
/// `initClockFromPllSys` and then this, so the clock is settled before the
/// converter sees it — switching underneath a running ADC is a glitch nothing
/// downstream would attribute correctly.
pub fn enable() bounded.Error!void {
    hw.Resets.unreset(hw.Resets.adc);

    hw.write(hw.Adc.cs, hw.Adc.cs_en);
    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(hw.Adc.cs) & hw.Adc.cs_ready == 0) try s.tick();
}

/// Configure a GPIO as an analogue input: digital input buffer off, no pulls.
///
/// Clearing IE is not optional — a pad left digitally enabled while an analogue
/// signal sits at a mid-rail voltage is precisely erratum E9's ~120 uA leakage
/// case (see `silicon.zig`), and it would also load the very divider being
/// measured.
pub fn initPin(pin: gpio.Pin) void {
    hw.write(hw.PadsBank0.pad(pin), hw.PadsBank0.od | hw.PadsBank0.iso);
    hw.write(hw.IoBank0.ctrl(pin), hw.IoBank0.func_null);
    gpio.deisolate(pin);
}

/// Enable or disable the on-die temperature sensor. It costs bias current, so
/// leave it off unless a reading is being taken.
pub fn setTempSensor(on: bool) void {
    const cs = hw.read(hw.Adc.cs);
    hw.write(hw.Adc.cs, if (on) cs | hw.Adc.cs_ts_en else cs & ~hw.Adc.cs_ts_en);
}

/// One conversion, blocking and bounded. Returns the raw 12-bit result.
pub fn readRaw(ch: Channel) bounded.Error!u12 {
    const cs = hw.read(hw.Adc.cs) & ~hw.Adc.cs_ainsel_mask;
    hw.write(hw.Adc.cs, cs | hw.Adc.csAinsel(ch) | hw.Adc.cs_start_once);
    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(hw.Adc.cs) & hw.Adc.cs_ready == 0) try s.tick();
    return @truncate(hw.read(hw.Adc.result));
}

/// Average `n` conversions, **discarding the first**.
///
/// Both halves matter on this board. The discard covers the sample-and-hold
/// still carrying charge from whatever channel was read previously — with a
/// 500 kOhm source it does not fully settle in one aperture, so the first
/// reading after a channel change is biased toward the old one. Averaging then
/// suppresses the ~1 LSB of noise that any SAR shows.
pub fn readAveraged(ch: Channel, n: u8) bounded.Error!u12 {
    std.debug.assert(n > 0);
    _ = try readRaw(ch); // settle the mux and S/H; result deliberately dropped
    var sum: u32 = 0;
    var i: u8 = 0;
    while (i < n) : (i += 1) sum += try readRaw(ch);
    return @truncate(sum / n);
}

// ---------------------------------------------------------------------------
// Pure conversion helpers — host-testable, no hardware involved
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Streaming: FIFO, pacing and DMA
// ---------------------------------------------------------------------------

/// FIFO depth, in samples. Datasheet §12.4.3.4.
pub const fifo_depth: u4 = 8;

/// The pacing divisor. `int` is cycles per conversion **minus one**, with a
/// first-order delta-sigma fraction in 256ths.
pub const Div = struct {
    int: u16 = 0,
    frac: u8 = 0,

    /// Free-running: a conversion every `cycles_per_conversion` cycles.
    pub const free: Div = .{};
};

/// The divisor that paces `clk_hz` down to `rate_hz`.
///
/// **Returns `null` when the rate is faster than the converter**, rather than a
/// divisor the hardware will quietly ignore. A trigger that arrives while a
/// conversion is in progress is dropped, so any `int` below 95 means
/// free-running whatever the caller intended — and a sample rate that is
/// silently not the one you asked for is the kind of error that survives all
/// the way into a published number.
pub fn divForRate(clk_hz: u32, rate_hz: u32) ?Div {
    if (rate_hz == 0) return null;
    // Cycles per conversion, in 256ths, so the fraction is exact arithmetic.
    const per_256: u64 = (@as(u64, clk_hz) * 256) / rate_hz;
    if (per_256 < @as(u64, cycles_per_conversion) * 256) return null;
    const whole = per_256 / 256;
    if (whole - 1 > std.math.maxInt(u16)) return null;
    return .{ .int = @intCast(whole - 1), .frac = @intCast(per_256 % 256) };
}

/// The rate a divisor actually produces, in samples per second, rounded.
///
/// The inverse of `divForRate`, and present so a caller can assert the rate it
/// got rather than the rate it asked for.
pub fn rateForDiv(clk_hz: u32, div: Div) u32 {
    const per_256: u64 = (@as(u64, div.int) + 1) * 256 + div.frac;
    const eff = @max(per_256, @as(u64, cycles_per_conversion) * 256);
    return @intCast((@as(u64, clk_hz) * 256 + eff / 2) / eff);
}

pub const StreamConfig = struct {
    /// The first channel converted, and the only one when `rrobin` is zero.
    first: Channel,
    /// One bit per channel; zero disables round-robin. **`first` is converted
    /// once even if its own bit is clear** — see `hw.Adc.csRrobin`.
    rrobin: u9 = 0,
    /// Pacing. Free-running by default, which is 500 kS/s at 48 MHz.
    div: Div = .free,
    /// Right-shift results to 8 bits, for byte-wide DMA into a deeper buffer.
    shift: bool = false,
    /// FIFO level at which DREQ and IRQ assert. **1 for DMA**, so a transfer
    /// starts as soon as one sample exists; higher only suits interrupts.
    thresh: u4 = 1,
    /// Tag each sample with its conversion-error flag. On by default: the
    /// datasheet requires discarding failed conversions, and a driver that
    /// makes that impossible has decided something it should not.
    err_in_fifo: bool = true,
    /// Assert DMA requests. The one reason to turn this off is polling.
    dreq: bool = true,
};

/// Configure and start free-running conversion into the FIFO.
///
/// **Start the DMA channel before calling this.** Datasheet §12.4.3.5 gives the
/// order and getting it wrong is silent: samples accumulate, the FIFO fills, and
/// the run reports an overrun that is the driver's fault rather than the link's.
pub fn startStream(cfg: StreamConfig) void {
    // Drain anything a previous user left behind, so `overran` reports this run.
    stopStream();

    hw.write(hw.Adc.div, hw.Adc.divWord(cfg.div.int, cfg.div.frac));
    // **`OVER` and `UNDER` are sticky and write-1-to-clear, so they are cleared
    // here as part of configuring.** Without this a single legitimate overrun
    // is inherited by every later stream, and `overran()` then reports a fault
    // that belongs to a run which finished minutes ago — measured on the bench,
    // where it made a 10 kS/s stream with 100 us of slack per sample claim to
    // have overflowed an eight-deep FIFO.
    hw.write(hw.Adc.fcs, hw.Adc.fcs_en |
        hw.Adc.fcs_over | hw.Adc.fcs_under |
        hw.Adc.fcsThresh(cfg.thresh) |
        (if (cfg.dreq) hw.Adc.fcs_dreq_en else 0) |
        (if (cfg.shift) hw.Adc.fcs_shift else 0) |
        (if (cfg.err_in_fifo) hw.Adc.fcs_err else 0));

    const cs = hw.read(hw.Adc.cs) & ~(hw.Adc.cs_ainsel_mask | hw.Adc.cs_rrobin_mask);
    hw.write(hw.Adc.cs, cs | hw.Adc.csAinsel(cfg.first) | hw.Adc.csRrobin(cfg.rrobin));
    hw.setBits(hw.Adc.cs, hw.Adc.cs_start_many);
}

/// Stop converting and leave the ADC fit for one-shot use.
///
/// The datasheet's own sequence: clear START_MANY, poll READY so the conversion
/// in flight completes, then drain. **The FIFO is disabled at the end** — it is
/// off by default precisely so that `readRaw` does not quietly fill it.
///
/// `OVER` and `UNDER` deliberately survive, so a caller can stop a stream and
/// then ask whether it lost anything. `startStream` clears them.
pub fn stopStream() void {
    hw.clrBits(hw.Adc.cs, hw.Adc.cs_start_many);

    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(hw.Adc.cs) & hw.Adc.cs_ready == 0) s.tick() catch break;

    while (hw.read(hw.Adc.fcs) & hw.Adc.fcs_empty == 0) _ = hw.read(hw.Adc.fifo);
    hw.write(hw.Adc.fcs, 0);
}

/// Samples waiting in the FIFO.
pub fn fifoLevel() u4 {
    return hw.Adc.fcsLevel(hw.read(hw.Adc.fcs));
}

/// Whether a conversion completed with the FIFO full, losing it. Sticky.
///
/// **Read this on every run and report it.** An overrun does not slow the
/// stream down or corrupt a sample; it removes one, and nothing downstream can
/// tell that the record has a hole in it unless this is carried with the data.
pub fn overran() bool {
    return hw.read(hw.Adc.fcs) & hw.Adc.fcs_over != 0;
}

pub fn clearOverrun() void {
    hw.setBits(hw.Adc.fcs, hw.Adc.fcs_over | hw.Adc.fcs_under);
}

/// One FIFO entry, split into its parts.
///
/// `err` means the SAR failed to converge and `value` is undefined — not
/// merely noisy. The datasheet says to discard it, in a Caution box.
pub const Sample = struct {
    value: u12,
    err: bool,

    pub fn decode(word: u32) Sample {
        return .{
            .value = @intCast(word & hw.Adc.fifo_val_mask),
            .err = word & hw.Adc.fifo_err != 0,
        };
    }
};

/// Read one sample from the FIFO without blocking. Null when empty.
pub fn pop() ?Sample {
    if (hw.read(hw.Adc.fcs) & hw.Adc.fcs_empty != 0) return null;
    return Sample.decode(hw.read(hw.Adc.fifo));
}

/// Raw count to millivolts at the pin.
pub fn rawToMv(raw: u12) u32 {
    return (@as(u32, raw) * vref_mv) / full_scale;
}

/// Raw count to millivolts before an external divider — e.g. the battery
/// voltage rather than the divided voltage the pin actually sees.
pub fn rawToMvThrough(raw: u12, divider: u32) u32 {
    return rawToMv(raw) * divider;
}

/// On-die temperature in centi-degrees Celsius (i.e. 2350 = 23.50 C).
///
/// From the datasheet: Vbe is typically 0.706 V at 27 C with a slope of
/// -1.721 mV/C, so T = 27 - (V - 0.706) / 0.001721.
///
/// Integer arithmetic throughout, in microvolts, to keep this off the FPU —
/// the firmware is built soft-float on purpose (see build.zig).
///
/// **Accuracy is poor without per-chip calibration** — the datasheet's figures
/// are typical, not trimmed, so absolute error of several degrees is normal.
/// Adequate for "is the enclosure absurdly hot", useless as a thermometer.
pub fn rawToTempCentiC(raw: u12) i32 {
    const uv: i32 = @intCast((@as(u32, raw) * vref_mv * 1000) / full_scale);
    const vbe_27_uv: i32 = 706_000;
    const slope_uv_per_c: i32 = 1721;
    // 27 C minus the offset, scaled by 100 for centi-degrees.
    return 2700 - @divTrunc((uv - vbe_27_uv) * 100, slope_uv_per_c);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "rawToMv spans the reference correctly" {
    try testing.expectEqual(@as(u32, 0), rawToMv(0));
    try testing.expectEqual(@as(u32, vref_mv / 2), rawToMv(2048));
    // Full scale is 4095, one count below vref.
    try testing.expect(rawToMv(4095) > 3299 - 1 and rawToMv(4095) <= vref_mv);
}

test "rawToMvThrough recovers a battery voltage across the /3 divider" {
    // A 4.2 V cell divided by 3 is 1.400 V at the pin, which is raw
    // 1400 * 4096 / 3300 = 1737.
    const raw: u12 = 1737;
    const mv = rawToMvThrough(raw, 3);

    // Tolerance is set by integer truncation, not by hopefulness: rawToMv
    // truncates, costing up to one LSB (~0.81 mV), and the divider then
    // multiplies that error by 3. So ~2.5 mV is the worst case and 4 mV is a
    // safe bound. One LSB of the *battery* voltage is ~2.4 mV, which is far
    // finer than anything a battery gauge needs.
    try testing.expect(mv >= 4200 - 4 and mv <= 4200 + 4);

    // And a flat-ish 3.3 V cell.
    const raw33: u12 = @intCast((1100 * full_scale) / vref_mv);
    const mv33 = rawToMvThrough(raw33, 3);
    try testing.expect(mv33 > 3290 and mv33 < 3310);
}

test "rawToTempCentiC: the datasheet's own reference point" {
    // 0.706 V must give exactly 27.00 C.
    const raw_at_27: u12 = @intCast((706 * full_scale) / vref_mv);
    const t = rawToTempCentiC(raw_at_27);
    try testing.expect(t > 2650 and t < 2750);

    // Slope is negative: a higher voltage means a colder die.
    try testing.expect(rawToTempCentiC(raw_at_27 + 50) < t);
    try testing.expect(rawToTempCentiC(raw_at_27 - 50) > t);
}

test "tempChannel differs by package, which is why it is not a constant" {
    // Cannot call tempChannel() on the host (it reads a register), but the
    // mapping it encodes is asserted here so a future edit cannot quietly
    // swap them: QFN-60 -> 4, QFN-80 -> 8.
    try testing.expectEqual(@as(Channel, 4), switch (silicon.Package.qfn60) {
        .qfn60 => @as(Channel, 4),
        .qfn80 => @as(Channel, 8),
    });
    try testing.expectEqual(@as(Channel, 8), switch (silicon.Package.qfn80) {
        .qfn60 => @as(Channel, 4),
        .qfn80 => @as(Channel, 8),
    });
}

test "pllSysDivisor: 144 MHz reaches 48, and the arithmetic is why the plan says 144" {
    try std.testing.expectEqual(@as(u4, 3), pllSysDivisor(144_000_000));
    try std.testing.expectEqual(@as(u4, 1), pllSysDivisor(48_000_000));
    try std.testing.expectEqual(@as(u4, 2), pllSysDivisor(96_000_000));
    try std.testing.expectEqual(@as(u4, 4), pllSysDivisor(192_000_000));
    // 150 MHz is the one that does not, and it is a compile error rather than a
    // 50 MHz clk_adc — which is what makes the guard worth having.
}

test "divForRate: free-running is the floor, and faster is refused" {
    const clk = required_clk_hz;
    // 500 kS/s is exactly 96 cycles, the fastest the converter goes.
    const at_max = divForRate(clk, 500_000).?;
    try std.testing.expectEqual(@as(u16, 95), at_max.int);
    try std.testing.expectEqual(@as(u8, 0), at_max.frac);

    // Anything faster has no divisor. Returning null rather than a value the
    // hardware ignores is the whole point.
    try std.testing.expect(divForRate(clk, 500_001) == null);
    try std.testing.expect(divForRate(clk, 1_000_000) == null);
    try std.testing.expect(divForRate(clk, 0) == null);
}

test "divForRate: the datasheet's own worked example" {
    // "setting DIV.INT to 47999 runs the ADC at 1 kS/s, if running from a
    // 48 MHz clock" — §12.4.3.2.
    const d = divForRate(required_clk_hz, 1000).?;
    try std.testing.expectEqual(@as(u16, 47_999), d.int);
    try std.testing.expectEqual(@as(u8, 0), d.frac);
}

test "divForRate and rateForDiv are inverses across the useful range" {
    const clk = required_clk_hz;
    for ([_]u32{ 1000, 8000, 44_100, 48_000, 96_000, 192_000, 250_000, 500_000 }) |want| {
        const d = divForRate(clk, want).?;
        const got = rateForDiv(clk, d);
        // Within a part in ten thousand: the delta-sigma fraction is 8 bits.
        const err = if (got > want) got - want else want - got;
        try std.testing.expect(err * 10_000 <= want);
    }
}

test "rateForDiv clamps to the conversion time rather than believing the divisor" {
    // A divisor the hardware would ignore must not produce an impossible rate.
    try std.testing.expectEqual(@as(u32, 500_000), rateForDiv(required_clk_hz, .{ .int = 0 }));
    try std.testing.expectEqual(@as(u32, 500_000), rateForDiv(required_clk_hz, .{ .int = 10 }));
    // And the crystal default, which is where the one-shot driver runs.
    try std.testing.expectEqual(@as(u32, 125_000), rateForDiv(12_000_000, .free));
}

test "Sample.decode separates a failed conversion from a small reading" {
    const good = Sample.decode(0x0abc);
    try std.testing.expectEqual(@as(u12, 0xabc), good.value);
    try std.testing.expect(!good.err);

    // The error flag is bit 15, well clear of the 12-bit value — so a sample
    // that failed is not merely a sample that read low.
    const bad = Sample.decode(0x8000 | 0x0007);
    try std.testing.expectEqual(@as(u12, 7), bad.value);
    try std.testing.expect(bad.err);
}

test "the FCS word a stream builds is the one the datasheet prescribes" {
    // §12.4.3.5: FIFO enabled, DREQ enabled, threshold 1.
    const w = hw.Adc.fcs_en | hw.Adc.fcs_dreq_en | hw.Adc.fcsThresh(1) | hw.Adc.fcs_err;
    try std.testing.expectEqual(@as(u32, 0x0100_000d), w);
    try std.testing.expectEqual(@as(u4, 1), hw.Adc.fcsLevel(0x0001_0000));
    try std.testing.expectEqual(@as(u4, 8), hw.Adc.fcsLevel(0x0008_0000));
}

test "round-robin masks land in CS bits 24:16" {
    try std.testing.expectEqual(@as(u32, 0x0006_0000), hw.Adc.csRrobin(0x006));
    try std.testing.expectEqual(@as(u32, 0x01ff_0000), hw.Adc.csRrobin(0x1ff));
    // And they do not collide with AINSEL.
    try std.testing.expectEqual(@as(u32, 0), hw.Adc.csRrobin(0x1ff) & hw.Adc.cs_ainsel_mask);
}
