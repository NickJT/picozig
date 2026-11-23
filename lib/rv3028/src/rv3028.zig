//! Micro Crystal RV-3028-C7 RTC (Real-Time Clock) Driver.
//!
//! Ultra-low power real-time clock module with I2C bus interface.

const std = @import("std");
const rp2350 = @import("rp2350");

/// Standard 7-bit I2C address for the RV-3028-C7.
pub const i2c_address: u7 = 0x52;

/// Register addresses for RV-3028-C7 per datasheet §3.2.
pub const Register = enum(u8) {
    seconds = 0x00,
    minutes = 0x01,
    hours = 0x02,
    weekday = 0x03,
    date = 0x04,
    month = 0x05,
    year = 0x06,

    minutes_alarm = 0x07,
    hours_alarm = 0x08,
    weekday_date_alarm = 0x09,

    timer_value_0 = 0x0A,
    timer_value_1 = 0x0B,
    timer_mode = 0x0C,

    status = 0x0E,
    ctrl1 = 0x0F,
    ctrl2 = 0x10,
    gp_bits = 0x11,
    clock_int_mask = 0x12,
    event_control = 0x13,

    unix_time_0 = 0x1B,
    unix_time_1 = 0x1C,
    unix_time_2 = 0x1D,
    unix_time_3 = 0x1E,
};

/// Status Register (0x0E) bit flags.
pub const Status = packed struct(u8) {
    /// Periodic Countdown Timer Flag
    tf: bool = false,
    /// Periodic Time Update Flag
    uf: bool = false,
    /// Alarm Flag
    af: bool = false,
    /// External Event Flag
    evf: bool = false,
    /// Power On Reset Flag (1 if VDD dropped below threshold)
    porf: bool = false,
    _reserved: u3 = 0,
};

/// Date and time structure.
pub const DateTime = struct {
    seconds: u8 = 0, // 0..59
    minutes: u8 = 0, // 0..59
    hours: u8 = 0, // 0..23 (24h format)
    weekday: u8 = 0, // 0..6 (0=Sunday, 1=Monday, ... 6=Saturday)
    date: u8 = 1, // 1..31
    month: u8 = 1, // 1..12
    year: u16 = 2026, // Full year (e.g., 2026 -> BCD stores year % 100)

    pub fn isValid(self: DateTime) bool {
        if (self.seconds > 59) return false;
        if (self.minutes > 59) return false;
        if (self.hours > 23) return false;
        if (self.weekday > 6) return false;
        if (self.date < 1 or self.date > 31) return false;
        if (self.month < 1 or self.month > 12) return false;
        return true;
    }
};

/// Convert Binary Coded Decimal (BCD) to Decimal.
pub fn bcdToDec(val: u8) u8 {
    return (val >> 4) * 10 + (val & 0x0F);
}

/// Convert Decimal to Binary Coded Decimal (BCD).
pub fn decToBcd(val: u8) u8 {
    return ((val / 10) << 4) | (val % 10);
}

/// Convenience adapter wrapping `rp2350.i2c` for the freestanding driver.
pub const PicoI2c = struct {
    inst: rp2350.i2c.Instance,
    timeout_polls: u32 = 10000,

    pub fn writeRegs(self: PicoI2c, addr: u7, start_reg: u8, data: []const u8) !void {
        var buf: [16]u8 = undefined;
        if (data.len + 1 > buf.len) return error.BufferOverflow;
        buf[0] = start_reg;
        @memcpy(buf[1 .. data.len + 1], data);
        try rp2350.i2c.write(self.inst, addr, buf[0 .. data.len + 1], true, self.timeout_polls);
    }

    pub fn readRegs(self: PicoI2c, addr: u7, start_reg: u8, out: []u8) !void {
        const reg_buf = [1]u8{start_reg};
        try rp2350.i2c.write(self.inst, addr, &reg_buf, true, self.timeout_polls);
        try rp2350.i2c.read(self.inst, addr, out, true, self.timeout_polls);
    }
};

/// Generic RV-3028 Driver parameterised over an I2C bus interface.
///
/// `I2cBus` must provide:
/// - `writeRegs(addr: u7, start_reg: u8, data: []const u8) !void`
/// - `readRegs(addr: u7, start_reg: u8, out: []u8) !void`
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,

        pub fn init(bus: I2cBus) Self {
            return Self{ .bus = bus };
        }

        /// Read current time from RV-3028 in 24-hour format.
        pub fn getTime(self: *Self) !DateTime {
            var raw: [7]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.seconds), &raw);

            return DateTime{
                .seconds = bcdToDec(raw[0] & 0x7F),
                .minutes = bcdToDec(raw[1] & 0x7F),
                .hours = bcdToDec(raw[2] & 0x3F),
                .weekday = raw[3] & 0x07,
                .date = bcdToDec(raw[4] & 0x3F),
                .month = bcdToDec(raw[5] & 0x1F),
                .year = 2000 + @as(u16, bcdToDec(raw[6])),
            };
        }

        /// Set time on RV-3028 in 24-hour format.
        ///
        /// Writing to Seconds (0x00) resets the internal prescaler for precise alignment.
        pub fn setTime(self: *Self, dt: DateTime) !void {
            if (!dt.isValid()) return error.InvalidDateTime;

            const yr_short: u8 = @intCast(dt.year % 100);
            const raw = [7]u8{
                decToBcd(dt.seconds),
                decToBcd(dt.minutes),
                decToBcd(dt.hours),
                dt.weekday & 0x07,
                decToBcd(dt.date),
                decToBcd(dt.month),
                decToBcd(yr_short),
            };

            try self.bus.writeRegs(i2c_address, @backingInt(Register.seconds), &raw);
        }

        /// Read Status Register (0x0E).
        pub fn getStatus(self: *Self) !Status {
            var raw: [1]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.status), &raw);
            return @bitCast(raw[0]);
        }

        /// Clear status register bits (writing 0 to a bit clears it on RV-3028).
        pub fn clearStatusFlags(self: *Self, mask: u8) !void {
            var current: [1]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.status), &current);
            const new_val = current[0] & ~mask;
            const data = [1]u8{new_val};
            try self.bus.writeRegs(i2c_address, @backingInt(Register.status), &data);
        }
    };
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// Mock I2C bus for testing register read/write logic.
const MockI2cBus = struct {
    registers: [64]u8 = @as([64]u8, @splat(0)),
    last_write_reg: ?u8 = null,
    last_write_len: usize = 0,

    pub fn writeRegs(self: *MockI2cBus, addr: u7, start_reg: u8, data: []const u8) !void {
        if (addr != i2c_address) return error.DeviceNotFound;
        for (data, 0..) |byte, i| {
            const reg = start_reg + @as(u8, @intCast(i));
            if (reg < self.registers.len) {
                self.registers[reg] = byte;
            }
        }
        self.last_write_reg = start_reg;
        self.last_write_len = data.len;
    }

    pub fn readRegs(self: *MockI2cBus, addr: u7, start_reg: u8, out: []u8) !void {
        if (addr != i2c_address) return error.DeviceNotFound;
        for (out, 0..) |*byte, i| {
            const reg = start_reg + @as(u8, @intCast(i));
            if (reg < self.registers.len) {
                byte.* = self.registers[reg];
            } else {
                byte.* = 0;
            }
        }
    }
};

test "BCD conversion round-trip" {
    for (0..100) |i| {
        const dec: u8 = @intCast(i);
        const bcd = decToBcd(dec);
        try testing.expectEqual(dec, bcdToDec(bcd));
    }
}

test "DateTime validation" {
    const valid = DateTime{
        .seconds = 30,
        .minutes = 45,
        .hours = 12,
        .weekday = 2,
        .date = 15,
        .month = 8,
        .year = 2026,
    };
    try testing.expect(valid.isValid());

    const invalid_sec = DateTime{ .seconds = 60 };
    try testing.expect(!invalid_sec.isValid());

    const invalid_month = DateTime{ .month = 13 };
    try testing.expect(!invalid_month.isValid());
}

test "Mock I2C getTime and setTime" {
    var mock_bus = MockI2cBus{};
    var rtc = Driver(*MockI2cBus).init(&mock_bus);

    const target_dt = DateTime{
        .seconds = 42,
        .minutes = 15,
        .hours = 14,
        .weekday = 0, // Sunday
        .date = 9,
        .month = 8,
        .year = 2026,
    };

    try rtc.setTime(target_dt);

    try testing.expectEqual(@as(usize, 7), mock_bus.last_write_len);
    try testing.expectEqual(@as(u8, 0x00), mock_bus.last_write_reg.?);

    const read_dt = try rtc.getTime();

    try testing.expectEqual(target_dt.seconds, read_dt.seconds);
    try testing.expectEqual(target_dt.minutes, read_dt.minutes);
    try testing.expectEqual(target_dt.hours, read_dt.hours);
    try testing.expectEqual(target_dt.weekday, read_dt.weekday);
    try testing.expectEqual(target_dt.date, read_dt.date);
    try testing.expectEqual(target_dt.month, read_dt.month);
    try testing.expectEqual(target_dt.year, read_dt.year);
}
