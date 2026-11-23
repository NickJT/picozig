//! SPI (ARM PL022 SSP) master — both instances.
//!
//! Motorola frame format, MSB first, any of the four clock modes (mode 0 by
//! default). Frame size is configurable: 8 bits is
//! what the nRF24 and CC1101 require, and **9 bits** is what an ST7701-family
//! panel controller wants on its 3-wire config port, where the extra bit
//! carries the data/command flag that a 4-wire device would put on a D/C pin.
//!
//! **This took an `Instance` parameter on 2026-07-30**, having been bound to
//! SPI0 since it was written. That was right while only one instance was wired
//! to anything; a display panel on SPI1 made it stop being. The
//! shape follows `i2c.zig`, which has had two instances from the start.
//!
//! Requires `clocks.init()` first: the bit rate is derived from clk_peri, which
//! that routine sources from clk_sys.
//!
//! CSN and CE are **not** handled here. CSN is a plain GPIO deliberately (see
//! board.zig for why the PL022's hardware chip select is unusable with this
//! radio), and CE is a mode signal with nothing to do with SPI at all. Both
//! belong to the nRF24 driver that will sit on top of this.

const std = @import("std");
const hw = @import("hw.zig");
const gpio = @import("gpio.zig");

/// GPIO function 1 is SPI on every pin that has it (function table, §9.4).
const func_spi: u32 = 1;

/// Which PL022. Same shape as `i2c.Instance`, for the same reason.
pub const Instance = enum(u1) {
    spi0 = 0,
    spi1 = 1,

    pub fn base(self: Instance) u32 {
        return switch (self) {
            .spi0 => hw.spi0_base,
            .spi1 => hw.spi1_base,
        };
    }

    pub fn resetBit(self: Instance) u32 {
        return switch (self) {
            .spi0 => hw.Resets.spi0,
            .spi1 => hw.Resets.spi1,
        };
    }
};

/// Conservative default. The nRF24L01+ tolerates up to 10 MHz and clk_peri can
/// give 6, but nothing here is throughput-bound — an 8-byte radio packet at
/// even 1 MHz costs 64 us against a receive window measured in tens of
/// milliseconds. Starting slow removes SPI signal integrity from the list of
/// suspects while the radio is first being brought up; raise it later if there
/// is ever a reason to.
pub const default_hz: u32 = 1_000_000;

/// The clock mode: SSPCR0's SPO (CPOL, bit 6) and SPH (CPHA, bit 7). SPO sets
/// SCK's idle level; SPH set captures data on the second clock edge rather
/// than the first. So mode 1 idles low, shifts out on rising edges and
/// captures on falling ones (RP2350 datasheet §12.3.4.9–12.3.4.13).
pub const Mode = enum(u2) {
    mode0 = 0b00,
    mode1 = 0b01,
    mode2 = 0b10,
    mode3 = 0b11,

    pub fn cr0Bits(self: Mode) u32 {
        const m = @backingInt(self);
        return (if (m & 0b10 != 0) hw.Spi.cr0_spo else 0) |
            (if (m & 0b01 != 0) hw.Spi.cr0_sph else 0);
    }
};

/// Pins and clock as parameters, not reach-throughs — see rp2350.zig's header.
///
/// Note what is absent: chip select. The PL022's hardware CSn asserts per
/// frame, which is unusable with devices that need it held across a whole
/// transaction (the nRF24 among them), so CSN is always a plain GPIO owned by
/// the device driver.
pub const Config = struct {
    /// **Optional**, because a write-only device has nothing to say. The
    /// ST7701's 3-wire port has no MISO at all, and there is no pin to give.
    miso: ?gpio.Pin,
    sck: gpio.Pin,
    mosi: gpio.Pin,
    /// clk_peri, which `clocks.init()` sources from clk_sys.
    clk_hz: u32,
    baud_hz: u32 = default_hz,
    /// Frame size, 4-16. Eight for a byte-oriented device; nine for an ST7701,
    /// where bit 8 is the data/command flag.
    data_bits: u5 = 8,
    mode: Mode = .mode0,
};

/// PL022 bit rate is `clk_peri / (CPSDVSR * (1 + SCR))`, where CPSDVSR is
/// **even**, 2-254, and SCR is 0-255.
///
/// The even-only prescaler is worth knowing: from a 12 MHz clk_peri the
/// achievable rates are 6, 3, 2, 1.5, 1.2, 1 MHz and so on — 4 MHz is simply
/// not reachable, since it would need a total divisor of 3. Asking for an
/// unreachable rate is not an error; the search below returns the fastest rate
/// that does not exceed the request.
pub const Divisors = struct {
    cpsdvsr: u32,
    scr: u32,

    /// Search taken from the SDK's `spi_set_baudrate`: smallest prescaler that
    /// can reach the target, then the largest post-divider that still does.
    pub fn compute(clk_hz: u32, target_hz: u32) Divisors {
        var prescale: u32 = 2;
        while (prescale <= 254) : (prescale += 2) {
            if (@as(u64, clk_hz) < @as(u64, prescale + 2) * 256 * target_hz) break;
        }
        if (prescale > 254) prescale = 254;

        var postdiv: u32 = 256;
        while (postdiv > 1) : (postdiv -= 1) {
            if (clk_hz / (prescale * (postdiv - 1)) > target_hz) break;
        }

        return .{ .cpsdvsr = prescale, .scr = postdiv - 1 };
    }

    pub fn actualHz(self: Divisors, clk_hz: u32) u32 {
        return clk_hz / (self.cpsdvsr * (1 + self.scr));
    }
};

pub fn init(inst: Instance, cfg: Config) void {
    std.debug.assert(cfg.data_bits >= 4 and cfg.data_bits <= 16);
    const b = inst.base();
    hw.Resets.unreset(inst.resetBit());

    // Disabled while configuring.
    hw.write(hw.Spi.cr1(b), 0);

    const d = Divisors.compute(cfg.clk_hz, cfg.baud_hz);
    hw.write(hw.Spi.cpsr(b), d.cpsdvsr);
    hw.write(
        hw.Spi.cr0(b),
        hw.Spi.cr0Dss(cfg.data_bits) | hw.Spi.cr0_frf_motorola | cfg.mode.cr0Bits() |
            hw.Spi.cr0Scr(d.scr),
    );

    // Pins. MISO gets a pull-**UP**, and the direction is load-bearing.
    //
    // An nRF24 tri-states MISO whenever CSN is high — which is nearly always,
    // since CSN is only asserted for a transaction. So this pin floats at idle
    // even with a module fitted. On A2 silicon that means erratum E9's ~120 uA
    // leakage, continuously, against a 200-400 uA total sleep budget.
    //
    // An earlier version used `.down` and claimed it as the E9 mitigation. That
    // was exactly backwards: the erratum says the internal pull-down is far too
    // weak to overcome the leakage and that the pull-up still works. The bug
    // would have presented as bad battery life with no obvious cause, and then
    // silently fixed itself on A4 silicon — see silicon.zig.
    //
    // Consequence for diagnostics: an absent module now reads as 0xff rather
    // than 0x00. The presence probe compares against a written pattern, so it
    // is unaffected either way.
    if (cfg.miso) |pin| gpio.initFunctionInput(pin, func_spi, .up);
    gpio.initFunctionOutput(cfg.sck, func_spi);
    gpio.initFunctionOutput(cfg.mosi, func_spi);

    hw.write(hw.Spi.cr1(b), hw.Spi.cr1_sse); // master, enabled
}

/// Full-duplex byte exchange. SPI has no notion of a read without a write, so
/// send a dummy byte when only the reply matters.
pub fn transferByte(inst: Instance, tx: u8) u8 {
    return @truncate(transferFrame(inst, tx));
}

/// Exchange one frame of whatever width `init` configured.
///
/// **This is the entry point for anything other than 8 bits.** The PL022's data
/// register is 16 bits wide regardless of frame size, with unused high bits
/// ignored on write and zero on read, so a 9-bit device is driven by passing a
/// `u16` here — bit 8 being the ST7701's data/command flag.
pub fn transferFrame(inst: Instance, tx: u16) u16 {
    const b = inst.base();
    while (hw.read(hw.Spi.sr(b)) & hw.Spi.sr_tnf == 0) {}
    hw.write(hw.Spi.dr(b), tx);
    while (hw.read(hw.Spi.sr(b)) & hw.Spi.sr_rne == 0) {}
    return @truncate(hw.read(hw.Spi.dr(b)));
}

/// Exchange a buffer. `rx` may alias `tx`.
pub fn transfer(inst: Instance, tx: []const u8, rx: []u8) void {
    std.debug.assert(tx.len == rx.len);
    for (tx, rx) |t, *r| r.* = transferByte(inst, t);
}

pub fn writeAll(inst: Instance, tx: []const u8) void {
    for (tx) |t| _ = transferByte(inst, t);
}

pub fn readAll(inst: Instance, rx: []u8, dummy: u8) void {
    for (rx) |*r| r.* = transferByte(inst, dummy);
}

/// Wait for the shift register to drain. Needed before deasserting a manually
/// driven CSN: the transmit FIFO going empty is not the same as the last bit
/// having left the pin, and dropping CSN early truncates the final byte.
pub fn waitIdle(inst: Instance) void {
    while (hw.read(hw.Spi.sr(inst.base())) & hw.Spi.sr_bsy != 0) {}
}

/// Self-test using the PL022's internal loopback (CR1.LBM), which ties the
/// transmit output back to the receive input inside the peripheral.
///
/// This checks the whole path — RESETS released, clock divisors programmed,
/// FIFOs moving, 8-bit framing — **with no radio attached and nothing wired**.
/// What it deliberately cannot check is the pins themselves: loopback happens
/// before the pad, so a mis-muxed SCK or a dead MOSI pad passes this and fails
/// against real hardware. It narrows the search; it does not end it.
pub fn selfTestLoopback(inst: Instance) bool {
    const b = inst.base();
    const saved_cr1 = hw.read(hw.Spi.cr1(b));

    // LBM may only be changed with the port disabled.
    hw.write(hw.Spi.cr1(b), 0);
    hw.write(hw.Spi.cr1(b), hw.Spi.cr1_lbm);
    hw.write(hw.Spi.cr1(b), hw.Spi.cr1_lbm | hw.Spi.cr1_sse);

    // Values chosen to catch a stuck bit, an inverted bit and a bit-order
    // error: 0x00 and 0xff pin the extremes, 0xa5/0x5a are each other's
    // complement, and 0x80/0x01 differ only in which end is set — a MSB/LSB
    // mix-up turns one into the other.
    const pattern = [_]u8{ 0xa5, 0x5a, 0x00, 0xff, 0x80, 0x01 };
    var ok = true;
    for (pattern) |byte| {
        if (transferByte(inst, byte) != byte) ok = false;
    }

    waitIdle(inst);
    hw.write(hw.Spi.cr1(b), 0);
    hw.write(hw.Spi.cr1(b), saved_cr1);
    return ok;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "Divisors: reachable rates from a 12 MHz clk_peri" {
    const clk = 12_000_000;

    // 6 MHz is the fastest possible: the prescaler bottoms out at 2.
    const fast = Divisors.compute(clk, 6_000_000);
    try testing.expectEqual(@as(u32, 2), fast.cpsdvsr);
    try testing.expectEqual(@as(u32, 0), fast.scr);
    try testing.expectEqual(@as(u32, 6_000_000), fast.actualHz(clk));

    // The default.
    const one = Divisors.compute(clk, 1_000_000);
    try testing.expectEqual(@as(u32, 1_000_000), one.actualHz(clk));
}

test "Divisors: an unreachable rate rounds DOWN, never up" {
    const clk = 12_000_000;

    // 4 MHz needs a total divisor of 3, impossible with an even prescaler.
    // Overshooting would clock the radio faster than asked, so the search must
    // settle for 3 MHz.
    const four = Divisors.compute(clk, 4_000_000);
    const actual = four.actualHz(clk);
    try testing.expectEqual(@as(u32, 3_000_000), actual);
    try testing.expect(actual <= 4_000_000);
}

test "Divisors: never exceeds the requested rate, across a wide sweep" {
    // The property that actually matters — an nRF24 clocked past its 10 MHz
    // limit fails in ways far harder to diagnose than one clocked slowly.
    const clk = 12_000_000;
    var target: u32 = 50_000;
    while (target <= 6_000_000) : (target += 50_000) {
        const d = Divisors.compute(clk, target);
        try testing.expect(d.cpsdvsr >= 2 and d.cpsdvsr <= 254);
        try testing.expect(d.cpsdvsr % 2 == 0);
        try testing.expect(d.scr <= 255);
        try testing.expect(d.actualHz(clk) <= target);
    }
}

test "DSS encodes frame size minus one" {
    // The off-by-one that gives 9-bit frames to a device expecting 8, which
    // then sees every byte shifted and answers nothing.
    try testing.expectEqual(@as(u32, 0b0111), hw.Spi.cr0Dss(8));
    try testing.expectEqual(@as(u32, 0b1000), hw.Spi.cr0Dss(9));
    try testing.expectEqual(@as(u32, 0b1111), hw.Spi.cr0Dss(16));
    // And the constant the 8-bit callers used before this was configurable
    // still agrees, so nothing changed underneath them.
    try testing.expectEqual(hw.Spi.cr0_dss_8bit, hw.Spi.cr0Dss(8));
}

test "Mode: CPOL is SPO (bit 6), CPHA is SPH (bit 7); mode 0 sets neither" {
    try testing.expectEqual(@as(u32, 0), Mode.mode0.cr0Bits());
    try testing.expectEqual(@as(u32, 1 << 7), Mode.mode1.cr0Bits());
    try testing.expectEqual(@as(u32, 1 << 6), Mode.mode2.cr0Bits());
    try testing.expectEqual(@as(u32, 0b11 << 6), Mode.mode3.cr0Bits());
    // The default is what every caller got before this was configurable.
    try testing.expectEqual(Mode.mode0, (Config{ .miso = null, .sck = undefined, .mosi = undefined, .clk_hz = 0 }).mode);
}

test "the two instances are 0x8000 apart, and neither is the other" {
    try testing.expectEqual(@as(u32, 0x4008_0000), Instance.spi0.base());
    try testing.expectEqual(@as(u32, 0x4008_8000), Instance.spi1.base());
    try testing.expect(Instance.spi0.resetBit() != Instance.spi1.resetBit());
    // Register offsets are the same block at either base.
    try testing.expectEqual(Instance.spi0.base() + 0x08, hw.Spi.dr(Instance.spi0.base()));
    try testing.expectEqual(Instance.spi1.base() + 0x08, hw.Spi.dr(Instance.spi1.base()));
}
