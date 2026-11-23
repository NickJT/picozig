//! TI ADS1115 — a 16-bit ΔΣ ADC on I2C, with an internal reference, a PGA and
//! a four-input multiplexer.
//!
//! `std` only, and generic over an I2C bus, so the host tests are the real test
//! and nothing here knows which chip is driving the bus.
//!
//! ## Where the field values come from
//!
//! **Every address, bit position and encoding below is from
//! `reference/ads1115.pdf` (SBAS444E)**: §8.1 for the register map, Table 7-2
//! for addresses, Table 7-1 for the LSB sizes, §7.5.3 for the bus protocol.
//!
//! ## What the datasheet says that is easy to get wrong
//!
//!   * **A config write in continuous mode does not take effect at once.** "The
//!     currently ongoing conversion completes with the previous configuration
//!     settings" (§7.4.2.2). The first result after a mux or range change is
//!     still the old one; discard it.
//!   * **The part is not reset when the controller is.** Config survives a
//!     reflash of the Pico, so `reset_config` is what a *power-up* reads, not
//!     what every boot reads.
//!   * **PGA codes 110b and 111b are also ±0.256 V**, so a config read back does
//!     not always re-encode to the bits that were written.
//!   * **`OS` reads inverted to how it writes**: writing 1 starts a conversion,
//!     reading 0 means one is running.
//!   * **ALERT/RDY becomes a conversion-ready pin** when Hi_thresh's MSB is 1 and
//!     Lo_thresh's MSB is 0 — read as two's complement, a high threshold below
//!     the low one — *and* `COMP_QUE` is anything but `11b`, which disables the
//!     pin (§8.1.4, §7.3.8). Both halves are needed. It is open drain.
//!   * **The output clips at 7FFFh and 8000h** (§7.5.4); a clipped code is a
//!     bound, not a reading.

const std = @import("std");

// ---------------------------------------------------------------------------
// Addressing — Table 7-2
// ---------------------------------------------------------------------------

/// The 7-bit target address, chosen by what ADDR is wired to. **The pin is
/// sampled continuously**, so a floating ADDR is an address that can change.
/// The datasheet says to prefer GND, VDD and SCL; SDA needs SDA held low 100 ns
/// past SCL falling.
pub const Address = enum(u7) {
    gnd = 0b1001000,
    vdd = 0b1001001,
    sda = 0b1001010,
    scl = 0b1001011,
};

// ---------------------------------------------------------------------------
// Registers — §8.1
// ---------------------------------------------------------------------------

/// The Address Pointer register's `P[1:0]`. Bits 7:2 are always written 0.
pub const Register = enum(u8) {
    conversion = 0b00,
    config = 0b01,
    lo_thresh = 0b10,
    hi_thresh = 0b11,
};

/// What the Config register holds after power-up (§8.1.3).
pub const reset_config: u16 = 0x8583;

/// Threshold reset values (§8.1.4).
pub const reset_lo_thresh: u16 = 0x8000;
pub const reset_hi_thresh: u16 = 0x7FFF;

/// The threshold pair that turns ALERT/RDY into a conversion-ready pin.
pub const ready_lo_thresh: u16 = 0x0000;
pub const ready_hi_thresh: u16 = 0x8000;

/// `MUX[2:0]`: AINP and AINN. The four single-ended settings measure against
/// GND, which is the only way to read a voltage that is not a difference.
pub const Mux = enum(u3) {
    ain0_ain1 = 0b000,
    ain0_ain3 = 0b001,
    ain1_ain3 = 0b010,
    ain2_ain3 = 0b011,
    ain0_gnd = 0b100,
    ain1_gnd = 0b101,
    ain2_gnd = 0b110,
    ain3_gnd = 0b111,
};

/// `PGA[2:0]`: the full-scale range. **±6.144 V and ±4.096 V describe the
/// scaling only** — no input may go beyond VDD + 0.3 V, whatever the range.
pub const Range = enum(u3) {
    fsr_6v144 = 0b000,
    fsr_4v096 = 0b001,
    fsr_2v048 = 0b010,
    fsr_1v024 = 0b011,
    fsr_0v512 = 0b100,
    /// Also what 110b and 111b decode to.
    fsr_0v256 = 0b101,

    /// Positive full scale in microvolts.
    pub fn fullScaleMicrovolts(self: Range) u32 {
        return switch (self) {
            .fsr_6v144 => 6_144_000,
            .fsr_4v096 => 4_096_000,
            .fsr_2v048 => 2_048_000,
            .fsr_1v024 => 1_024_000,
            .fsr_0v512 => 512_000,
            .fsr_0v256 => 256_000,
        };
    }

    /// One code, in picovolts — exact for every range, where nanovolts are not
    /// (±0.256 V is 7812.5 nV). Table 7-1: LSB = full scale / 2^15.
    pub fn lsbPicovolts(self: Range) u64 {
        return @as(u64, self.fullScaleMicrovolts()) * 1_000_000 / 32768;
    }
};

/// `DR[2:0]`. Nominal: the internal oscillator is specified only to ±10 %, so
/// anything that depends on the real rate has to measure it.
pub const DataRate = enum(u3) {
    sps8 = 0b000,
    sps16 = 0b001,
    sps32 = 0b010,
    sps64 = 0b011,
    sps128 = 0b100,
    sps250 = 0b101,
    sps475 = 0b110,
    sps860 = 0b111,

    pub fn nominalSps(self: DataRate) u16 {
        return switch (self) {
            .sps8 => 8,
            .sps16 => 16,
            .sps32 => 32,
            .sps64 => 64,
            .sps128 => 128,
            .sps250 => 250,
            .sps475 => 475,
            .sps860 => 860,
        };
    }
};

/// `MODE`.
pub const Mode = enum(u1) {
    continuous = 0,
    /// Also the power-down state between conversions.
    single_shot = 1,
};

/// `COMP_QUE[1:0]`: conversions past a threshold before ALERT/RDY asserts.
/// `disabled` also puts the pin in high impedance.
pub const ComparatorQueue = enum(u2) {
    after_1 = 0b00,
    after_2 = 0b01,
    after_4 = 0b10,
    disabled = 0b11,
};

/// The Config register less `OS`, which is a command when written and a status
/// when read, and so is an argument to `encode` and a field of `Status`.
/// Defaults are the power-up values.
pub const Config = struct {
    mux: Mux = .ain0_ain1,
    range: Range = .fsr_2v048,
    mode: Mode = .single_shot,
    rate: DataRate = .sps128,
    /// `COMP_MODE`: window rather than traditional.
    comp_window: bool = false,
    /// `COMP_POL`: ALERT/RDY active high rather than low.
    comp_active_high: bool = false,
    /// `COMP_LAT`.
    comp_latching: bool = false,
    comp_queue: ComparatorQueue = .disabled,

    /// `start` sets `OS`, which begins a conversion only from power-down.
    pub fn encode(self: Config, start: bool) u16 {
        return @as(u16, @intFromBool(start)) << 15 |
            @as(u16, @backingInt(self.mux)) << 12 |
            @as(u16, @backingInt(self.range)) << 9 |
            @as(u16, @backingInt(self.mode)) << 8 |
            @as(u16, @backingInt(self.rate)) << 5 |
            @as(u16, @intFromBool(self.comp_window)) << 4 |
            @as(u16, @intFromBool(self.comp_active_high)) << 3 |
            @as(u16, @intFromBool(self.comp_latching)) << 2 |
            @as(u16, @backingInt(self.comp_queue));
    }

    pub fn decode(raw: u16) Config {
        const pga: u3 = @truncate(raw >> 9);
        return .{
            .mux = @fromBackingInt(@as(u3, @truncate(raw >> 12))),
            .range = if (pga >= 0b101) .fsr_0v256 else @fromBackingInt(@intCast(pga)),
            .mode = @fromBackingInt(@as(u1, @truncate(raw >> 8))),
            .rate = @fromBackingInt(@as(u3, @truncate(raw >> 5))),
            .comp_window = raw & (1 << 4) != 0,
            .comp_active_high = raw & (1 << 3) != 0,
            .comp_latching = raw & (1 << 2) != 0,
            .comp_queue = @fromBackingInt(@as(u2, @truncate(raw))),
        };
    }
};

/// The Config register as read.
pub const Status = struct {
    config: Config,
    /// `OS` read as 0.
    converting: bool,

    pub fn decode(raw: u16) Status {
        return .{ .config = Config.decode(raw), .converting = raw & 0x8000 == 0 };
    }
};

// ---------------------------------------------------------------------------
// Codes
// ---------------------------------------------------------------------------

/// A code in nanovolts at the given range, truncated toward zero (only
/// ±0.256 V has a fraction to lose, half a nanovolt).
pub fn nanovolts(code: i16, range: Range) i64 {
    return @divTrunc(@as(i64, code) * @as(i64, @intCast(range.lsbPicovolts())), 1000);
}

/// True at either clip code: the input was at or beyond full scale.
pub fn isClipped(code: i16) bool {
    return code == std.math.maxInt(i16) or code == std.math.minInt(i16);
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// Generic ADS1115 driver over an I2C bus.
///
/// `I2cBus` must provide:
///   * `writeRegs(addr: u7, start_reg: u8, data: []const u8) !void`
///   * `readRegs(addr: u7, start_reg: u8, out: []u8) !void`
///
/// The shape `lib/ds3231` and `lib/rv3028` use. `readRegs` may put either a
/// STOP or a repeated START between the pointer write and the read — this part
/// accepts both (§7.5.3).
///
/// Registers are 16 bits, most significant byte first.
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,
        addr: u7,

        pub fn init(bus: I2cBus, address: Address) Self {
            return .{ .bus = bus, .addr = @backingInt(address) };
        }

        pub fn writeRegister(self: *Self, reg: Register, value: u16) !void {
            const bytes = [2]u8{ @truncate(value >> 8), @truncate(value) };
            try self.bus.writeRegs(self.addr, @backingInt(reg), &bytes);
        }

        pub fn readRegister(self: *Self, reg: Register) !u16 {
            var bytes: [2]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(reg), &bytes);
            return @as(u16, bytes[0]) << 8 | bytes[1];
        }

        /// Write the configuration without starting anything. In continuous
        /// mode the conversion in progress still finishes on the old settings.
        pub fn configure(self: *Self, config: Config) !void {
            try self.writeRegister(.config, config.encode(false));
        }

        /// Start one conversion in single-shot mode, whatever `config.mode`
        /// says: `OS` has no effect outside power-down. Done when
        /// `readStatus().converting` is false, about 25 µs plus one data period.
        pub fn startSingle(self: *Self, config: Config) !void {
            var c = config;
            c.mode = .single_shot;
            try self.writeRegister(.config, c.encode(true));
        }

        pub fn readStatus(self: *Self) !Status {
            return Status.decode(try self.readRegister(.config));
        }

        /// The last completed conversion. 0 until the first one after
        /// power-up completes.
        pub fn readConversion(self: *Self) !i16 {
            return @bitCast(try self.readRegister(.conversion));
        }

        /// Both thresholds, as written to the part. They compare against
        /// codes, so they are in the units of whatever range is in force.
        pub fn setThresholds(self: *Self, lo: i16, hi: i16) !void {
            try self.writeRegister(.lo_thresh, @bitCast(lo));
            try self.writeRegister(.hi_thresh, @bitCast(hi));
        }

        /// The threshold half of conversion-ready mode. The config half is
        /// `comp_queue` other than `.disabled`, which is the caller's to set.
        pub fn setReadyThresholds(self: *Self) !void {
            try self.writeRegister(.lo_thresh, ready_lo_thresh);
            try self.writeRegister(.hi_thresh, ready_hi_thresh);
        }
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the power-up config decodes to the defaults, and they re-encode to it" {
    const c = Config.decode(reset_config);
    try testing.expectEqual(Config{}, c);
    try testing.expectEqual(reset_config, (Config{}).encode(true));
    try testing.expect(!Status.decode(reset_config).converting);
}

test "every field lands on its own bits" {
    try testing.expectEqual(@as(u16, 0x8000), (Config{ .mux = .ain0_ain1, .range = .fsr_6v144, .mode = .continuous, .rate = .sps8, .comp_queue = .after_1 }).encode(true));
    const base = (Config{ .range = .fsr_6v144, .mode = .continuous, .rate = .sps8, .comp_queue = .after_1 }).encode(false);
    try testing.expectEqual(@as(u16, 0), base);
    try testing.expectEqual(@as(u16, 0x7000), (Config{ .mux = .ain3_gnd, .range = .fsr_6v144, .mode = .continuous, .rate = .sps8, .comp_queue = .after_1 }).encode(false));
    try testing.expectEqual(@as(u16, 0x0A00), (Config{ .range = .fsr_0v256, .mode = .continuous, .rate = .sps8, .comp_queue = .after_1 }).encode(false));
    try testing.expectEqual(@as(u16, 0x0100), (Config{ .range = .fsr_6v144, .mode = .single_shot, .rate = .sps8, .comp_queue = .after_1 }).encode(false));
    try testing.expectEqual(@as(u16, 0x00E0), (Config{ .range = .fsr_6v144, .mode = .continuous, .rate = .sps860, .comp_queue = .after_1 }).encode(false));
    try testing.expectEqual(@as(u16, 0x001C), (Config{ .range = .fsr_6v144, .mode = .continuous, .rate = .sps8, .comp_window = true, .comp_active_high = true, .comp_latching = true, .comp_queue = .after_1 }).encode(false));
    try testing.expectEqual(@as(u16, 0x0003), (Config{ .range = .fsr_6v144, .mode = .continuous, .rate = .sps8, .comp_queue = .disabled }).encode(false));
}

test "every config the encoder can produce decodes back to itself" {
    for (std.enums.values(Mux)) |mux| for (std.enums.values(Range)) |range| for (std.enums.values(DataRate)) |rate| {
        const c = Config{ .mux = mux, .range = range, .rate = rate, .mode = .continuous, .comp_latching = true };
        try testing.expectEqual(c, Config.decode(c.encode(false)));
    };
}

test "PGA codes 110b and 111b read as ±0.256 V" {
    try testing.expectEqual(Range.fsr_0v256, Config.decode(0b110 << 9).range);
    try testing.expectEqual(Range.fsr_0v256, Config.decode(0b111 << 9).range);
}

test "OS reads 0 while converting" {
    try testing.expect(Status.decode(reset_config & 0x7FFF).converting);
}

test "LSB sizes match Table 7-1" {
    try testing.expectEqual(@as(u64, 187_500_000), Range.fsr_6v144.lsbPicovolts());
    try testing.expectEqual(@as(u64, 125_000_000), Range.fsr_4v096.lsbPicovolts());
    try testing.expectEqual(@as(u64, 62_500_000), Range.fsr_2v048.lsbPicovolts());
    try testing.expectEqual(@as(u64, 31_250_000), Range.fsr_1v024.lsbPicovolts());
    try testing.expectEqual(@as(u64, 15_625_000), Range.fsr_0v512.lsbPicovolts());
    try testing.expectEqual(@as(u64, 7_812_500), Range.fsr_0v256.lsbPicovolts());
}

test "codes to nanovolts, at both ends and both signs" {
    try testing.expectEqual(@as(i64, 62_500), nanovolts(1, .fsr_2v048));
    try testing.expectEqual(@as(i64, -62_500), nanovolts(-1, .fsr_2v048));
    try testing.expectEqual(@as(i64, 2_047_937_500), nanovolts(std.math.maxInt(i16), .fsr_2v048));
    try testing.expectEqual(@as(i64, -6_144_000_000), nanovolts(std.math.minInt(i16), .fsr_6v144));
    try testing.expectEqual(@as(i64, 7_812), nanovolts(1, .fsr_0v256));
}

test "only the two end codes are clipped" {
    try testing.expect(isClipped(0x7FFF));
    try testing.expect(isClipped(-0x8000));
    try testing.expect(!isClipped(0x7FFE));
    try testing.expect(!isClipped(-0x7FFF));
}

const FakeBus = struct {
    regs: [4]u16 = .{ 0, reset_config, reset_lo_thresh, reset_hi_thresh },
    writes: u32 = 0,
    last_write: [2]u8 = .{ 0, 0 },

    fn writeRegs(self: *FakeBus, addr: u7, start_reg: u8, data: []const u8) !void {
        try testing.expectEqual(@backingInt(Address.gnd), addr);
        try testing.expectEqual(@as(usize, 2), data.len);
        try testing.expect(start_reg < 4);
        self.writes += 1;
        self.last_write = .{ data[0], data[1] };
        self.regs[start_reg] = @as(u16, data[0]) << 8 | data[1];
    }

    fn readRegs(self: *FakeBus, addr: u7, start_reg: u8, out: []u8) !void {
        try testing.expectEqual(@backingInt(Address.gnd), addr);
        try testing.expectEqual(@as(usize, 2), out.len);
        const v = self.regs[start_reg];
        out[0] = @truncate(v >> 8);
        out[1] = @truncate(v);
    }
};

test "registers go over the bus most significant byte first" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try dev.configure(.{ .mux = .ain3_gnd });
    try testing.expectEqual([2]u8{ 0x75, 0x83 }, bus.last_write);
    try testing.expectEqual(Mux.ain3_gnd, (try dev.readStatus()).config.mux);
}

test "a conversion reads back as a signed code" {
    var bus = FakeBus{};
    bus.regs[0] = 0xFFFF;
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try testing.expectEqual(@as(i16, -1), try dev.readConversion());
    bus.regs[0] = 0x8000;
    try testing.expectEqual(std.math.minInt(i16), try dev.readConversion());
}

test "startSingle sets OS and single-shot even when handed continuous" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try dev.startSingle(.{ .mode = .continuous, .rate = .sps860 });
    const raw = bus.regs[@backingInt(Register.config)];
    try testing.expect(raw & 0x8000 != 0);
    try testing.expectEqual(Mode.single_shot, Config.decode(raw).mode);
    try testing.expectEqual(DataRate.sps860, Config.decode(raw).rate);
}

test "configure never sets OS" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try dev.configure(.{});
    try testing.expectEqual(@as(u16, 0), bus.regs[@backingInt(Register.config)] & 0x8000);
}

test "conversion-ready thresholds: high MSB set, low MSB clear" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try dev.setReadyThresholds();
    try testing.expectEqual(@as(u16, 0x8000), bus.regs[@backingInt(Register.hi_thresh)]);
    try testing.expectEqual(@as(u16, 0x0000), bus.regs[@backingInt(Register.lo_thresh)]);
}

test "thresholds are stored as two's complement" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus, .gnd);
    try dev.setThresholds(-2, 1000);
    try testing.expectEqual(@as(u16, 0xFFFE), bus.regs[@backingInt(Register.lo_thresh)]);
    try testing.expectEqual(@as(u16, 1000), bus.regs[@backingInt(Register.hi_thresh)]);
}

test "the address is what ADDR is wired to" {
    try testing.expectEqual(@as(u7, 0x48), @backingInt(Address.gnd));
    try testing.expectEqual(@as(u7, 0x49), @backingInt(Address.vdd));
    try testing.expectEqual(@as(u7, 0x4A), @backingInt(Address.sda));
    try testing.expectEqual(@as(u7, 0x4B), @backingInt(Address.scl));
}
