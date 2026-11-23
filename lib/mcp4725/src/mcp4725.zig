//! Microchip MCP4725 12-Bit Digital-to-Analog Converter (DAC) Driver.
//!
//! Freestanding, hardware-agnostic Zig driver supporting Fast Mode write,
//! DAC register write, nonvolatile EEPROM write, power-down modes, and status reading.

const std = @import("std");
const rp2350 = @import("rp2350");

/// Standard 7-bit I2C addresses for MCP4725.
pub const i2c_address_default_a0_low: u7 = 0x60;
pub const i2c_address_default_a0_high: u7 = 0x61;

/// Power-down resistance selection per datasheet §5.2.
pub const PowerDownMode = enum(u2) {
    /// Normal operation (output buffer active)
    normal = 0b00,
    /// Power-down with 1 kΩ pull-down to GND
    res_1k = 0b01,
    /// Power-down with 100 kΩ pull-down to GND
    res_100k = 0b10,
    /// Power-down with 500 kΩ pull-down to GND
    res_500k = 0b11,
};

/// MCP4725 device status parsed from a 5-byte I2C read.
pub const Status = struct {
    /// True if EEPROM write is complete, false if write is in progress (RDY/BSY)
    ready: bool = true,
    /// True if power-on reset sequence has completed normally
    por: bool = true,
    /// Current power-down mode of the DAC register
    power_down: PowerDownMode = .normal,
    /// Current 12-bit value in the DAC register (0..4095)
    dac_value: u12 = 0,
    /// Power-down mode stored in EEPROM
    eeprom_power_down: PowerDownMode = .normal,
    /// 12-bit DAC value stored in EEPROM (0..4095)
    eeprom_dac_value: u12 = 0,
};

/// Convert target voltage to 12-bit DAC raw count (0..4095).
pub fn voltageToRaw(voltage: f32, vdd: f32) u12 {
    if (vdd <= 0.0) return 0;
    const clamped = std.math.clamp(voltage, 0.0, vdd);
    const raw_f = (clamped / vdd) * 4095.0 + 0.5;
    const raw_i: u32 = @intFromFloat(raw_f);
    return @intCast(@min(raw_i, 4095));
}

/// Convert 12-bit DAC raw count (0..4095) to output voltage.
pub fn rawToVoltage(raw: u12, vdd: f32) f32 {
    return (@as(f32, @floatFromInt(raw)) / 4095.0) * vdd;
}

/// Convenience transport adapter wrapping `rp2350.i2c` for RP2350 freestanding operation.
pub const PicoI2c = struct {
    inst: rp2350.i2c.Instance,
    timeout_polls: u32 = 10000,

    pub fn write(self: PicoI2c, addr: u7, bytes: []const u8) !void {
        try rp2350.i2c.write(self.inst, addr, bytes, true, self.timeout_polls);
    }

    pub fn read(self: PicoI2c, addr: u7, out: []u8) !void {
        try rp2350.i2c.read(self.inst, addr, out, true, self.timeout_polls);
    }
};

/// Generic MCP4725 Driver parameterized over an I2C bus interface.
///
/// `I2cBus` must provide:
/// - `write(addr: u7, bytes: []const u8) !void`
/// - `read(addr: u7, out: []u8) !void`
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,
        addr: u7,

        pub fn init(bus: I2cBus, addr: u7) Self {
            return Self{
                .bus = bus,
                .addr = addr,
            };
        }

        /// Fast Mode Write (2 bytes payload).
        ///
        /// Fast write updates the DAC register directly without affecting EEPROM.
        pub fn setRawFast(self: *Self, value: u12, pd: PowerDownMode) !void {
            const high_nibble: u8 = @intCast((value >> 8) & 0x0F);
            const pd_bits: u8 = @backingInt(pd);
            const buf = [2]u8{
                (pd_bits << 4) | high_nibble,
                @intCast(value & 0xFF),
            };
            try self.bus.write(self.addr, &buf);
        }

        /// Fast Mode helper for updating output voltage in normal mode.
        pub fn setVoltageFast(self: *Self, voltage: f32, vdd: f32) !void {
            const raw = voltageToRaw(voltage, vdd);
            try self.setRawFast(raw, .normal);
        }

        /// Write DAC Register (3 bytes payload, C2=0, C1=1, C0=0).
        ///
        /// Updates the DAC register and power-down configuration without writing EEPROM.
        pub fn setRawDac(self: *Self, value: u12, pd: PowerDownMode) !void {
            const pd_bits: u8 = @backingInt(pd);
            const cmd_byte: u8 = 0x40 | (pd_bits << 1);
            const high_byte: u8 = @intCast((value >> 4) & 0xFF);
            const low_byte: u8 = @intCast((value & 0x0F) << 4);

            const buf = [3]u8{ cmd_byte, high_byte, low_byte };
            try self.bus.write(self.addr, &buf);
        }

        /// Write DAC Register helper for updating output voltage.
        pub fn setVoltageDac(self: *Self, voltage: f32, vdd: f32) !void {
            const raw = voltageToRaw(voltage, vdd);
            try self.setRawDac(raw, .normal);
        }

        /// Write DAC Register and nonvolatile EEPROM (3 bytes payload, C2=0, C1=1, C0=1).
        ///
        /// Updates the DAC register and writes the configuration and value to EEPROM.
        /// The device will be busy (`ready == false`) for ~25ms while EEPROM programming completes.
        pub fn setRawDacAndEeprom(self: *Self, value: u12, pd: PowerDownMode) !void {
            const pd_bits: u8 = @backingInt(pd);
            const cmd_byte: u8 = 0x60 | (pd_bits << 1);
            const high_byte: u8 = @intCast((value >> 4) & 0xFF);
            const low_byte: u8 = @intCast((value & 0x0F) << 4);

            const buf = [3]u8{ cmd_byte, high_byte, low_byte };
            try self.bus.write(self.addr, &buf);
        }

        /// Write DAC Register and EEPROM helper for updating target voltage.
        pub fn setVoltageDacAndEeprom(self: *Self, voltage: f32, vdd: f32) !void {
            const raw = voltageToRaw(voltage, vdd);
            try self.setRawDacAndEeprom(raw, .normal);
        }

        /// Put output into power-down mode (using Fast Mode write).
        pub fn powerDown(self: *Self, pd: PowerDownMode) !void {
            if (pd == .normal) return;
            try self.setRawFast(0, pd);
        }

        /// Read device status, active DAC register, and stored EEPROM contents (5-byte read).
        pub fn readStatus(self: *Self) !Status {
            var buf: [5]u8 = undefined;
            try self.bus.read(self.addr, &buf);

            const ready = (buf[0] & 0x80) != 0;
            const por = (buf[0] & 0x40) != 0;
            const pd: PowerDownMode = @fromBackingInt(@as(u2, @truncate((buf[0] >> 1) & 0x03)));
            const dac_val: u12 = (@as(u12, buf[1]) << 4) | @as(u12, buf[2] >> 4);

            const eeprom_pd: PowerDownMode = @fromBackingInt(@as(u2, @truncate((buf[3] >> 5) & 0x03)));
            const eeprom_dac_val: u12 = (@as(u12, buf[3] & 0x0F) << 8) | @as(u12, buf[4]);

            return Status{
                .ready = ready,
                .por = por,
                .power_down = pd,
                .dac_value = dac_val,
                .eeprom_power_down = eeprom_pd,
                .eeprom_dac_value = eeprom_dac_val,
            };
        }

        /// Check whether EEPROM programming has completed.
        pub fn isReady(self: *Self) !bool {
            const st = try self.readStatus();
            return st.ready;
        }
    };
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// Mock I2C Bus for validating driver bit packing and read status parsing.
const MockI2cBus = struct {
    last_write: [4]u8 = @as([4]u8, @splat(0)),
    last_write_len: usize = 0,
    read_buf: [5]u8 = @as([5]u8, @splat(0)),

    pub fn write(self: *MockI2cBus, addr: u7, bytes: []const u8) !void {
        if (addr != i2c_address_default_a0_low and addr != i2c_address_default_a0_high) {
            return error.DeviceNotFound;
        }
        if (bytes.len > self.last_write.len) return error.BufferOverflow;
        @memcpy(self.last_write[0..bytes.len], bytes);
        self.last_write_len = bytes.len;
    }

    pub fn read(self: *MockI2cBus, addr: u7, out: []u8) !void {
        if (addr != i2c_address_default_a0_low and addr != i2c_address_default_a0_high) {
            return error.DeviceNotFound;
        }
        const count = @min(out.len, self.read_buf.len);
        @memcpy(out[0..count], self.read_buf[0..count]);
    }
};

test "voltage to raw conversion and bounds" {
    const vdd: f32 = 3.3;

    try testing.expectEqual(@as(u16, 0), voltageToRaw(0.0, vdd));
    try testing.expectEqual(@as(u16, 4095), voltageToRaw(3.3, vdd));
    try testing.expectEqual(@as(u16, 4095), voltageToRaw(5.0, vdd));
    try testing.expectEqual(@as(u16, 0), voltageToRaw(-1.0, vdd));

    // Midpoint: ~1.65V -> ~2048
    const mid_raw = voltageToRaw(1.65, vdd);
    try testing.expect(mid_raw >= 2047 and mid_raw <= 2048);

    const calc_v = rawToVoltage(mid_raw, vdd);
    try testing.expect(calc_v >= 1.64 and calc_v <= 1.66);
}

test "Fast Mode Write bit packing" {
    var mock_bus = MockI2cBus{};
    var dac = Driver(*MockI2cBus).init(&mock_bus, i2c_address_default_a0_low);

    // Target value 0xA5B = 2651
    try dac.setRawFast(0x0A5B, .normal);

    try testing.expectEqual(@as(usize, 2), mock_bus.last_write_len);
    try testing.expectEqual(@as(u8, 0x0A), mock_bus.last_write[0]); // PD=00, high nibble 0xA
    try testing.expectEqual(@as(u8, 0x5B), mock_bus.last_write[1]); // low byte 0x5B

    // Fast write with 100k power down
    try dac.setRawFast(0x0123, .res_100k);
    try testing.expectEqual(@as(u8, 0x21), mock_bus.last_write[0]); // PD=10 -> 0x20 | 0x01 = 0x21
    try testing.expectEqual(@as(u8, 0x23), mock_bus.last_write[1]);
}

test "Write DAC Register bit packing" {
    var mock_bus = MockI2cBus{};
    var dac = Driver(*MockI2cBus).init(&mock_bus, i2c_address_default_a0_low);

    // Target value 0x0FFF = 4095
    try dac.setRawDac(4095, .normal);

    try testing.expectEqual(@as(usize, 3), mock_bus.last_write_len);
    try testing.expectEqual(@as(u8, 0x40), mock_bus.last_write[0]); // C2=0, C1=1, C0=0
    try testing.expectEqual(@as(u8, 0xFF), mock_bus.last_write[1]); // D11..D4
    try testing.expectEqual(@as(u8, 0xF0), mock_bus.last_write[2]); // D3..D0 << 4
}

test "Write DAC and EEPROM bit packing" {
    var mock_bus = MockI2cBus{};
    var dac = Driver(*MockI2cBus).init(&mock_bus, i2c_address_default_a0_low);

    try dac.setRawDacAndEeprom(0x0800, .res_1k); // 2048, 1k PD

    try testing.expectEqual(@as(usize, 3), mock_bus.last_write_len);
    try testing.expectEqual(@as(u8, 0x62), mock_bus.last_write[0]); // C2=0, C1=1, C0=1 | PD=01 << 1 = 0x62
    try testing.expectEqual(@as(u8, 0x80), mock_bus.last_write[1]);
    try testing.expectEqual(@as(u8, 0x00), mock_bus.last_write[2]);
}

test "Read Status 5-byte parsing" {
    var mock_bus = MockI2cBus{};
    // Construct fake read buffer:
    // Byte 0: RDY/BSY=1, POR=1, PD=00 -> 0xC0
    // Byte 1: DAC D11..D4 = 0x80
    // Byte 2: DAC D3..D0 = 0x00 -> 0x00
    // Byte 3: EEPROM PD=10, EEPROM D11..D8 = 0x0F -> 0x4F
    // Byte 4: EEPROM D7..D0 = 0xFF
    mock_bus.read_buf = [5]u8{ 0xC0, 0x80, 0x00, 0x4F, 0xFF };

    var dac = Driver(*MockI2cBus).init(&mock_bus, i2c_address_default_a0_low);
    const st = try dac.readStatus();

    try testing.expect(st.ready);
    try testing.expect(st.por);
    try testing.expectEqual(PowerDownMode.normal, st.power_down);
    try testing.expectEqual(@as(u12, 2048), st.dac_value);
    try testing.expectEqual(PowerDownMode.res_100k, st.eeprom_power_down);
    try testing.expectEqual(@as(u12, 4095), st.eeprom_dac_value);
}
