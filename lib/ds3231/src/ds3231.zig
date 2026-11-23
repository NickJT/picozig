//! Maxim DS3231 RTC — the TCXO one.
//!
//! The accurate half of the pair `lib/rv3028` is the low-power half of. Both sit
//! on one Qwiic chain at different addresses (0x68 here, 0x52 there) so drift
//! can be measured on both at once rather than in two runs that differ by
//! temperature as well as by part.
//!
//! ## Where the field values come from
//!
//! **Every register address, bit position and encoding below is from
//! `reference/DS3231.pdf` Figure 1 and the register sections around it**, read
//! before any of it was written. That is this workspace's standing rule and it
//! earned itself here: two details are not what memory supplies.
//!
//!   * **The day-of-week values are user-defined.** The part counts 1-7 and
//!     rolls over; the datasheet only requires them to be *sequential* ("if 1
//!     equals Sunday, then 2 equals Monday"). Nothing in the chip knows which
//!     day is which, so a mapping is a decision this file has to make and
//!     document, not one it can read out.
//!   * **The century bit lives in the month register**, bit 7 of 05h, and it
//!     *toggles* when the year register overflows 99 to 00. It is not a
//!     high-order year bit and cannot be treated as one.
//!
//! The other trap the map makes obvious: bit 6 of the hours register is the
//! 12/24 flag and bit 5 is either AM/PM or the 20-hour bit depending on it. An
//! unmasked `bcdToDec` on that byte reads a perfectly ordinary afternoon as
//! some hour in the sixties.

const std = @import("std");
const rp2350 = @import("rp2350");

/// Standard 7-bit I2C address. Fixed in silicon — the DS3231 has no address
/// pins, which is why it and the RV-3028 can share a chain without conflict.
pub const i2c_address: u7 = 0x68;

// ---------------------------------------------------------------------------
// BCD
// ---------------------------------------------------------------------------

/// Binary-coded decimal to binary.
///
/// **Masking is the caller's job**, because which bits are data differs per
/// register: the DS3231's hours register carries a 12/24 flag and the month
/// register carries a century bit in the space this would otherwise decode.
/// Handing this an unmasked byte is how a century bit becomes 40 years.
pub fn bcdToDec(val: u8) u8 {
    return (val >> 4) * 10 + (val & 0x0F);
}

/// Binary to binary-coded decimal.
pub fn decToBcd(val: u8) u8 {
    return ((val / 10) << 4) | (val % 10);
}

// ---------------------------------------------------------------------------
// Registers — reference/DS3231.pdf, Figure 1
// ---------------------------------------------------------------------------

pub const Register = enum(u8) {
    seconds = 0x00,
    minutes = 0x01,
    /// Bit 6 is 12/24; bit 5 is AM/PM in 12-hour mode and the 20-hour bit in
    /// 24-hour mode. Mask before decoding.
    hours = 0x02,
    /// 1-7, and **what those values mean is up to the user** — see the header.
    day = 0x03,
    date = 0x04,
    /// Bit 7 is the century bit.
    month = 0x05,
    /// 00-99.
    year = 0x06,

    alarm1_seconds = 0x07,
    alarm1_minutes = 0x08,
    alarm1_hours = 0x09,
    /// Bit 6 `DY/DT` selects day-of-week rather than date.
    alarm1_day_date = 0x0A,
    alarm2_minutes = 0x0B,
    alarm2_hours = 0x0C,
    alarm2_day_date = 0x0D,

    control = 0x0E,
    status = 0x0F,
    /// Two's complement; see `Driver.setAgingOffset`.
    aging_offset = 0x10,
    /// Two's complement integer degrees.
    temp_msb = 0x11,
    /// Fraction in bits 7:6, quarter degrees.
    temp_lsb = 0x12,
};

/// Control register, 0Eh.
///
/// **Every field defaults to 0, and that is not the part's power-on state.**
/// These structs are register *images* for `@bitCast`, so a literal that names
/// one field silently zeroes the rest — which for this register would turn off
/// the square wave and both alarm enables on the way to setting something
/// unrelated. Read, modify, write; never construct one from nothing unless you
/// mean every bit of it.
pub const Control = packed struct(u8) {
    /// Alarm 1 interrupt enable.
    a1ie: bool = false,
    /// Alarm 2 interrupt enable.
    a2ie: bool = false,
    /// 1 = INT/SQW carries alarm interrupts; 0 = it carries a square wave.
    intcn: bool = false,
    rs1: bool = false,
    rs2: bool = false,
    /// Start a temperature conversion. Self-clearing; check `bsy` first.
    conv: bool = false,
    /// Enable the square wave when running from VBAT.
    bbsqw: bool = false,
    /// **Enable Oscillator, and it is active LOW.** 0 means the oscillator
    /// runs. The datasheet names the bit for the thing it disables, so the
    /// obvious reading — set it to start the clock — stops the clock instead.
    eosc: bool = false,
};

/// Control/status register, 0Fh.
pub const Status = packed struct(u8) {
    /// Alarm 1 fired. **Write 0 to clear; writing 1 does nothing.**
    a1f: bool = false,
    /// Alarm 2 fired. Same clearing rule.
    a2f: bool = false,
    /// A temperature conversion is in progress.
    bsy: bool = false,
    /// 32 kHz output enable.
    en32khz: bool = false,
    _reserved: u3 = 0,
    /// **Oscillator Stop Flag — the one field to read before trusting a
    /// reading.** Set whenever the oscillator has stopped, and it is *sticky*:
    /// it stays set through the restart until software clears it. A 1 means the
    /// timekeeping data may be nonsense however plausible it looks, which is
    /// exactly the failure an RTC is bought to avoid.
    osf: bool = false,
};

/// What a day-of-week number means.
///
/// The part enforces nothing here — see the header. This file pins **1 =
/// Sunday** so that `DateTime.weekday` (0 = Sunday, matching `lib/rv3028`) maps
/// to the register by adding one, and both parts on a shared Qwiic chain agree.
pub const sunday_register_value: u8 = 1;

// ---------------------------------------------------------------------------
// The calendar
// ---------------------------------------------------------------------------

/// Days in `month` of `year`, Gregorian.
pub fn daysInMonth(year: u16, month: u8) u8 {
    return switch (month) {
        1, 3, 5, 7, 8, 10, 12 => 31,
        4, 6, 9, 11 => 30,
        2 => if (isLeapYear(year)) 29 else 28,
        else => 0,
    };
}

/// **The full Gregorian rule, not the divisible-by-four shorthand.** 2100 is
/// not a leap year and 2000 was; a driver that gets this wrong is correct for
/// decades and then is not.
pub fn isLeapYear(year: u16) bool {
    if (year % 400 == 0) return true;
    if (year % 100 == 0) return false;
    return year % 4 == 0;
}

/// Day of week for a date, 0 = Sunday, by Sakamoto's method.
///
/// **Worth computing rather than trusting the caller.** The DS3231 keeps the
/// day of week in its own register and does *not* derive it from the date — it
/// simply counts 1-7 and rolls over. So a `setTime` that writes whatever it was
/// handed will happily store a Tuesday against a Sunday's date, and nothing
/// reports it: every read afterwards is self-consistent and wrong.
pub fn dayOfWeek(year: u16, month: u8, date: u8) u8 {
    const t = [_]u8{ 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    var y: u32 = year;
    if (month < 3) y -= 1;
    const idx: usize = @intCast(month - 1);
    const d: u32 = date;
    return @intCast((y + y / 4 - y / 100 + y / 400 + t[idx] + d) % 7);
}

/// Date and time, in the 24-hour form this driver presents regardless of what
/// the part is configured to keep.
pub const DateTime = struct {
    seconds: u8 = 0,
    minutes: u8 = 0,
    /// 0..23. **Always 24-hour here.** The DS3231 can be put in 12-hour mode,
    /// and a driver that leaks that choice to its callers makes every consumer
    /// handle both.
    hours: u8 = 0,
    /// 0 = Sunday. The part stores 1..7 with no fixed meaning; this file pins
    /// 0 = Sunday to match `dayOfWeek` and `lib/rv3028`.
    weekday: u8 = 0,
    date: u8 = 1,
    month: u8 = 1,
    /// Full year, e.g. 2026.
    year: u16 = 2026,

    /// **Month-aware, and leap-year-aware.**
    ///
    /// A range check of `1..31` on the day accepts the 31st of February, and
    /// `lib/rv3028`'s `isValid` does exactly that. It matters more than it
    /// looks: an RTC handed an impossible date does not refuse it — it stores
    /// the BCD and reads it back, so the bad value survives every round trip
    /// and only surfaces as a clock that jumps.
    pub fn isValid(self: DateTime) bool {
        if (self.seconds > 59) return false;
        if (self.minutes > 59) return false;
        if (self.hours > 23) return false;
        if (self.weekday > 6) return false;
        if (self.month < 1 or self.month > 12) return false;
        if (self.date < 1 or self.date > daysInMonth(self.year, self.month)) return false;
        // The part keeps two BCD digits of year plus a century bit; outside
        // this range a value cannot be represented at all.
        if (self.year < 2000 or self.year > 2199) return false;
        return true;
    }

    /// Fill `weekday` from the date, rather than trusting whatever was set.
    pub fn withComputedWeekday(self: DateTime) DateTime {
        var out = self;
        out.weekday = dayOfWeek(self.year, self.month, self.date);
        return out;
    }

    /// Seconds since the Unix epoch, treating the stored time as UTC.
    ///
    /// **UTC, and the caller applies any offset.** Local time with summer time
    /// is a rule about a place rather than about this chip, so it belongs with
    /// the application, which can then hold one implementation of it.
    pub fn toUnix(self: DateTime) i64 {
        var days: i64 = 0;
        var y: u16 = 1970;
        while (y < self.year) : (y += 1) days += if (isLeapYear(y)) 366 else 365;
        var m: u8 = 1;
        while (m < self.month) : (m += 1) days += daysInMonth(self.year, m);
        days += @as(i64, self.date) - 1;
        return ((days * 24 + self.hours) * 60 + self.minutes) * 60 + self.seconds;
    }

    /// The inverse of `toUnix`, for a caller setting the part from NTP.
    pub fn fromUnix(unix: i64) DateTime {
        var days = @divFloor(unix, 86_400);
        var rem = @mod(unix, 86_400);

        var out = DateTime{};
        out.hours = @intCast(@divFloor(rem, 3600));
        rem = @mod(rem, 3600);
        out.minutes = @intCast(@divFloor(rem, 60));
        out.seconds = @intCast(@mod(rem, 60));

        var y: u16 = 1970;
        while (true) {
            const len: i64 = if (isLeapYear(y)) 366 else 365;
            if (days < len) break;
            days -= len;
            y += 1;
        }
        out.year = y;

        var m: u8 = 1;
        while (true) {
            const len: i64 = daysInMonth(y, m);
            if (days < len) break;
            days -= len;
            m += 1;
        }
        out.month = m;
        out.date = @intCast(days + 1);
        out.weekday = dayOfWeek(out.year, out.month, out.date);
        return out;
    }
};

// ---------------------------------------------------------------------------
// Bus
// ---------------------------------------------------------------------------

/// Adapter wrapping `rp2350.i2c` for the freestanding driver. Same shape as
/// `lib/rv3028`'s, so a caller can hold both parts the same way.
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

    /// Register pointer then repeated start. **`false` on the write is the
    /// whole point** — it holds the bus so the read is a combined transaction
    /// rather than two. `rp2350.i2c` supports that through `restart_on_next`,
    /// which arms on a `stop = false` write and makes the next transfer's first
    /// command carry `IC_DATA_CMD.RESTART`. **Few drivers take that path**, so
    /// bringing this part up is also a test of it.
    pub fn readRegs(self: PicoI2c, addr: u7, start_reg: u8, out: []u8) !void {
        const reg_buf = [1]u8{start_reg};
        try rp2350.i2c.write(self.inst, addr, &reg_buf, false, self.timeout_polls);
        try rp2350.i2c.read(self.inst, addr, out, true, self.timeout_polls);
    }
};

/// Generic DS3231 driver over an I2C bus.
///
/// `I2cBus` must provide:
///   * `writeRegs(addr: u7, start_reg: u8, data: []const u8) !void`
///   * `readRegs(addr: u7, start_reg: u8, out: []u8) !void`
///
/// Same shape as `lib/rv3028`'s, so an application can hold both parts
/// identically — which is what makes the side-by-side drift comparison one
/// program rather than two.
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,

        pub fn init(bus: I2cBus) Self {
            return .{ .bus = bus };
        }

        /// Read the time, always as 24-hour.
        ///
        /// **A seven-byte burst, not seven reads.** The datasheet: "On an I2C
        /// START or address pointer incrementing to location 00h, the current
        /// time is transferred to a second set of registers... This eliminates
        /// the need to reread the registers in case the main registers update
        /// during a read." Reading fields one at a time gives up that snapshot
        /// and can straddle a tick — 12:59:59 becomes 12:00:59 once an hour.
        pub fn getTime(self: *Self) !DateTime {
            var raw: [7]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.seconds), &raw);
            return decodeTime(raw);
        }

        /// Set the time. The weekday is computed from the date rather than
        /// taken on trust — see `dayOfWeek`.
        pub fn setTime(self: *Self, dt: DateTime) !void {
            if (!dt.isValid()) return error.InvalidDateTime;
            const raw = encodeTime(dt.withComputedWeekday());
            try self.bus.writeRegs(i2c_address, @backingInt(Register.seconds), &raw);
        }

        pub fn readStatus(self: *Self) !Status {
            var b: [1]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.status), &b);
            return @bitCast(b[0]);
        }

        pub fn readControl(self: *Self) !Control {
            var b: [1]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.control), &b);
            return @bitCast(b[0]);
        }

        /// Has the oscillator stopped since this was last cleared?
        ///
        /// **Check this before believing a reading.** `OSF` is sticky across
        /// the restart, so it answers "has the time been wrong at any point",
        /// which is the question that matters after a battery change.
        pub fn oscillatorStopped(self: *Self) !bool {
            return (try self.readStatus()).osf;
        }

        /// Clear `OSF`, after setting the time.
        ///
        /// Read-modify-write, and **the alarm flags are written back as 0**:
        /// they clear on a 0 and ignore a 1, so preserving a set flag is not
        /// possible and pretending otherwise would be misleading.
        pub fn clearOscillatorStopped(self: *Self) !void {
            var st = try self.readStatus();
            st.osf = false;
            try self.bus.writeRegs(i2c_address, @backingInt(Register.status), &[_]u8{@bitCast(st)});
        }

        /// The on-die temperature, in quarter-degrees Celsius.
        ///
        /// Quarters rather than a float: this target has no FPU enabled, and
        /// the part's resolution is exactly 0.25 C, so an integer of quarters
        /// is lossless and a float would be both slower and a hard fault.
        /// Divide by 4 for degrees; the value is signed.
        pub fn temperatureQuarterC(self: *Self) !i16 {
            var raw: [2]u8 = undefined;
            try self.bus.readRegs(i2c_address, @backingInt(Register.temp_msb), &raw);
            return decodeTemperature(raw);
        }
    };
}

/// Decode a seven-byte timekeeping burst. Split out so it can be host-tested
/// without a bus.
pub fn decodeTime(raw: [7]u8) DateTime {
    const hour_byte = raw[2];
    const hours: u8 = if (hour_byte & 0x40 != 0) blk: {
        // 12-hour mode: bits 3:0 plus the 10-hour bit, with bit 5 as PM.
        const h12 = bcdToDec(hour_byte & 0x1F);
        const pm = hour_byte & 0x20 != 0;
        // 12 AM is 0, 12 PM is 12 — the two the naive `+12` gets wrong.
        break :blk if (pm) (if (h12 == 12) 12 else h12 + 12) else (if (h12 == 12) 0 else h12);
    } else bcdToDec(hour_byte & 0x3F);

    const month_byte = raw[5];
    const century: u16 = if (month_byte & 0x80 != 0) 100 else 0;

    return .{
        .seconds = bcdToDec(raw[0] & 0x7F),
        .minutes = bcdToDec(raw[1] & 0x7F),
        .hours = hours,
        // The register is 1-based; `DateTime` is 0 = Sunday.
        .weekday = (raw[3] & 0x07) -| sunday_register_value,
        .date = bcdToDec(raw[4] & 0x3F),
        .month = bcdToDec(month_byte & 0x1F),
        .year = 2000 + century + bcdToDec(raw[6]),
    };
}

/// Encode a `DateTime` into a seven-byte timekeeping burst, 24-hour.
pub fn encodeTime(dt: DateTime) [7]u8 {
    return .{
        decToBcd(dt.seconds),
        decToBcd(dt.minutes),
        // Bit 6 clear = 24-hour mode, which this driver always writes.
        decToBcd(dt.hours),
        dt.weekday + sunday_register_value,
        decToBcd(dt.date),
        decToBcd(dt.month) | (if (dt.year >= 2100) @as(u8, 0x80) else 0),
        decToBcd(@intCast(dt.year % 100)),
    };
}

/// Temperature from the two registers, in quarter-degrees.
pub fn decodeTemperature(raw: [2]u8) i16 {
    const whole: i16 = @as(i8, @bitCast(raw[0]));
    const quarters: i16 = @intCast(raw[1] >> 6);
    // **Two's complement over the whole 10-bit value**, so the fraction adds
    // upward even when the integer part is negative: 0xFF,0xC0 is -0.25 C, not
    // -1.75 C. Getting this wrong is invisible above freezing.
    return whole * 4 + quarters;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "BCD round-trips every value the part can hold" {
    var v: u8 = 0;
    while (v <= 99) : (v += 1) {
        try testing.expectEqual(v, bcdToDec(decToBcd(v)));
    }
}

test "BCD encodes as two decimal digits, not as hex" {
    // 0x59, not 0x3B. Getting this backwards produces a clock that runs
    // and is wrong in a way that looks like drift.
    try testing.expectEqual(@as(u8, 0x59), decToBcd(59));
    try testing.expectEqual(@as(u8, 59), bcdToDec(0x59));
    try testing.expectEqual(@as(u8, 0x00), decToBcd(0));
    try testing.expectEqual(@as(u8, 0x10), decToBcd(10));
}

test "the leap rule is Gregorian, not divisible-by-four" {
    try testing.expect(isLeapYear(2000)); // /400
    try testing.expect(!isLeapYear(2100)); // /100 but not /400
    try testing.expect(isLeapYear(2024));
    try testing.expect(!isLeapYear(2026));
    try testing.expectEqual(@as(u8, 29), daysInMonth(2024, 2));
    try testing.expectEqual(@as(u8, 28), daysInMonth(2026, 2));
    try testing.expectEqual(@as(u8, 29), daysInMonth(2000, 2));
    try testing.expectEqual(@as(u8, 28), daysInMonth(2100, 2));
}

test "isValid rejects the 31st of February" {
    // **The check `lib/rv3028` does not make.** A plain 1..31 range test passes
    // this, the part stores it, and it reads back unchanged.
    const bad = DateTime{ .year = 2026, .month = 2, .date = 31 };
    try testing.expect(!bad.isValid());

    const leap_ok = DateTime{ .year = 2024, .month = 2, .date = 29 };
    try testing.expect(leap_ok.isValid());

    const leap_bad = DateTime{ .year = 2026, .month = 2, .date = 29 };
    try testing.expect(!leap_bad.isValid());

    const thirty_first_of_april = DateTime{ .year = 2026, .month = 4, .date = 31 };
    try testing.expect(!thirty_first_of_april.isValid());
}

test "isValid bounds every field" {
    try testing.expect(!(DateTime{ .seconds = 60 }).isValid());
    try testing.expect(!(DateTime{ .minutes = 60 }).isValid());
    try testing.expect(!(DateTime{ .hours = 24 }).isValid());
    try testing.expect(!(DateTime{ .weekday = 7 }).isValid());
    try testing.expect(!(DateTime{ .month = 0 }).isValid());
    try testing.expect(!(DateTime{ .month = 13 }).isValid());
    try testing.expect(!(DateTime{ .date = 0 }).isValid());
    // Outside what two BCD digits plus a century bit can represent.
    try testing.expect(!(DateTime{ .year = 1999 }).isValid());
    try testing.expect((DateTime{ .year = 2000 }).isValid());
    try testing.expect((DateTime{ .year = 2199 }).isValid());
    try testing.expect(!(DateTime{ .year = 2200 }).isValid());
}

test "day of week against dates that are known" {
    // 0 = Sunday. **Every expectation here was checked against an independent
    // calendar**, after a first draft asserted 2026-08-25 was a Thursday. It is
    // a Tuesday; the implementation was right and the test was wrong, which is
    // the failure mode a hand-written expectation always has.
    try testing.expectEqual(@as(u8, 2), dayOfWeek(2026, 8, 25)); // a Tuesday
    try testing.expectEqual(@as(u8, 6), dayOfWeek(2000, 1, 1)); // a Saturday
    try testing.expectEqual(@as(u8, 1), dayOfWeek(2024, 2, 26)); // a Monday
    // Either side of a leap day, which is where an off-by-one hides.
    try testing.expectEqual(@as(u8, 4), dayOfWeek(2024, 2, 29));
    try testing.expectEqual(@as(u8, 5), dayOfWeek(2024, 3, 1));
}

test "withComputedWeekday overrides whatever it was handed" {
    // The part counts 1..7 and never checks the day against the date, so a
    // wrong weekday survives every round trip.
    const wrong = DateTime{ .year = 2026, .month = 8, .date = 25, .weekday = 0 };
    try testing.expectEqual(@as(u8, 2), wrong.withComputedWeekday().weekday);
}

test "Unix conversion round-trips, across leap years and month ends" {
    const cases = [_]DateTime{
        .{ .year = 1970, .month = 1, .date = 1 },
        .{ .year = 2000, .month = 2, .date = 29, .hours = 12, .minutes = 34, .seconds = 56 },
        .{ .year = 2024, .month = 2, .date = 29, .hours = 23, .minutes = 59, .seconds = 59 },
        .{ .year = 2026, .month = 8, .date = 25, .hours = 20, .minutes = 15, .seconds = 1 },
        .{ .year = 2026, .month = 12, .date = 31, .hours = 23, .minutes = 59, .seconds = 59 },
        .{ .year = 2100, .month = 3, .date = 1 },
    };
    for (cases) |c| {
        const round = DateTime.fromUnix(c.toUnix());
        try testing.expectEqual(c.year, round.year);
        try testing.expectEqual(c.month, round.month);
        try testing.expectEqual(c.date, round.date);
        try testing.expectEqual(c.hours, round.hours);
        try testing.expectEqual(c.minutes, round.minutes);
        try testing.expectEqual(c.seconds, round.seconds);
        // And the weekday comes out consistent with the date.
        try testing.expectEqual(dayOfWeek(c.year, c.month, c.date), round.weekday);
    }
}

test "the epoch itself is zero, and a known instant matches" {
    try testing.expectEqual(@as(i64, 0), (DateTime{ .year = 1970, .month = 1, .date = 1 }).toUnix());
    // 2000-01-01T00:00:00Z
    try testing.expectEqual(
        @as(i64, 946_684_800),
        (DateTime{ .year = 2000, .month = 1, .date = 1 }).toUnix(),
    );
}

test "a timekeeping burst decodes to the time it encodes" {
    const cases = [_]DateTime{
        .{ .year = 2026, .month = 8, .date = 25, .hours = 20, .minutes = 15, .seconds = 1 },
        .{ .year = 2000, .month = 1, .date = 1, .hours = 0, .minutes = 0, .seconds = 0 },
        .{ .year = 2099, .month = 12, .date = 31, .hours = 23, .minutes = 59, .seconds = 59 },
        // Across the century bit.
        .{ .year = 2100, .month = 3, .date = 1, .hours = 12 },
        .{ .year = 2199, .month = 6, .date = 15, .hours = 13, .minutes = 30 },
    };
    for (cases) |c| {
        const want = c.withComputedWeekday();
        const got = decodeTime(encodeTime(want));
        try testing.expectEqual(want.year, got.year);
        try testing.expectEqual(want.month, got.month);
        try testing.expectEqual(want.date, got.date);
        try testing.expectEqual(want.hours, got.hours);
        try testing.expectEqual(want.minutes, got.minutes);
        try testing.expectEqual(want.seconds, got.seconds);
        try testing.expectEqual(want.weekday, got.weekday);
    }
}

test "the century bit is in the month register and does not corrupt the month" {
    const y2100 = encodeTime((DateTime{ .year = 2100, .month = 12, .date = 1 }).withComputedWeekday());
    try testing.expectEqual(@as(u8, 0x80 | 0x12), y2100[5]); // century set, month 12 BCD
    try testing.expectEqual(@as(u8, 0x00), y2100[6]); // year 00

    const y2099 = encodeTime((DateTime{ .year = 2099, .month = 12, .date = 1 }).withComputedWeekday());
    try testing.expectEqual(@as(u8, 0x12), y2099[5]); // century clear
    try testing.expectEqual(@as(u8, 0x99), y2099[6]);
}

test "12-hour mode decodes, including both noons" {
    // Bit 6 set = 12-hour. Bit 5 = PM.
    // 12 AM is hour 0 and 12 PM is hour 12 -- the two a naive +12 gets wrong.
    var raw = [_]u8{ 0, 0, 0, 1, 0x01, 0x01, 0x26 };

    raw[2] = 0x40 | 0x12; // 12, AM
    try testing.expectEqual(@as(u8, 0), decodeTime(raw).hours);

    raw[2] = 0x40 | 0x20 | 0x12; // 12, PM
    try testing.expectEqual(@as(u8, 12), decodeTime(raw).hours);

    raw[2] = 0x40 | 0x01; // 1 AM
    try testing.expectEqual(@as(u8, 1), decodeTime(raw).hours);

    raw[2] = 0x40 | 0x20 | 0x01; // 1 PM
    try testing.expectEqual(@as(u8, 13), decodeTime(raw).hours);

    raw[2] = 0x40 | 0x20 | 0x11; // 11 PM
    try testing.expectEqual(@as(u8, 23), decodeTime(raw).hours);
}

test "24-hour mode uses bit 5 as the 20-hour bit, not as PM" {
    // 0x20 in 24-hour mode means the twenties, so 0x20|0x03 is 23:00.
    const raw = [_]u8{ 0, 0, 0x23, 1, 0x01, 0x01, 0x26 };
    try testing.expectEqual(@as(u8, 23), decodeTime(raw).hours);
}

test "the day register is 1-based and DateTime is 0-based" {
    const dt = (DateTime{ .year = 2026, .month = 8, .date = 25 }).withComputedWeekday();
    try testing.expectEqual(@as(u8, 2), dt.weekday); // Tuesday
    const raw = encodeTime(dt);
    try testing.expectEqual(@as(u8, 3), raw[3]); // 1 = Sunday, so Tuesday is 3
    try testing.expectEqual(@as(u8, 2), decodeTime(raw).weekday);
}

test "temperature is two's complement over the whole 10 bits" {
    // The datasheet's own example: 00011001 01b = +25.25 C.
    try testing.expectEqual(@as(i16, 101), decodeTemperature(.{ 0b0001_1001, 0b0100_0000 }));
    try testing.expectEqual(@as(i16, 0), decodeTemperature(.{ 0x00, 0x00 }));
    try testing.expectEqual(@as(i16, 1), decodeTemperature(.{ 0x00, 0x40 })); // +0.25
    // **The fraction adds upward even when the integer is negative.**
    // 0xFF,0xC0 is -1 + 0.75 = -0.25 C, not -1.75.
    try testing.expectEqual(@as(i16, -1), decodeTemperature(.{ 0xFF, 0xC0 }));
    try testing.expectEqual(@as(i16, -4), decodeTemperature(.{ 0xFF, 0x00 })); // -1.00
    try testing.expectEqual(@as(i16, -40 * 4), decodeTemperature(.{ 0xD8, 0x00 }));
}

test "control and status pack to the bits the datasheet names" {
    // Figure 1: control 0Eh is EOSC BBSQW CONV RS2 RS1 INTCN A2IE A1IE, MSB first.
    const c = Control{ .eosc = true };
    try testing.expectEqual(@as(u8, 0x80), @as(u8, @bitCast(c)));
    const rs = Control{ .rs1 = true, .rs2 = true, .intcn = false };
    try testing.expectEqual(@as(u8, 0x18), @as(u8, @bitCast(rs)));

    // Status 0Fh is OSF 0 0 0 EN32kHz BSY A2F A1F.
    const st = Status{ .osf = true, .en32khz = false };
    try testing.expectEqual(@as(u8, 0x80), @as(u8, @bitCast(st)));
    const flags = Status{ .osf = false, .en32khz = false, .a1f = true, .a2f = true };
    try testing.expectEqual(@as(u8, 0x03), @as(u8, @bitCast(flags)));
}

/// A bus that answers from a register file, so the driver can be exercised
/// without a part. Mirrors the DS3231's pointer-then-burst behaviour.
const FakeBus = struct {
    regs: [0x13]u8 = @splat(0),
    last_write_reg: u8 = 0xFF,

    fn writeRegs(self: *FakeBus, addr: u7, start_reg: u8, data: []const u8) !void {
        try testing.expectEqual(i2c_address, addr);
        self.last_write_reg = start_reg;
        for (data, 0..) |b, i| self.regs[(start_reg + i) % self.regs.len] = b;
    }

    fn readRegs(self: *FakeBus, addr: u7, start_reg: u8, out: []u8) !void {
        try testing.expectEqual(i2c_address, addr);
        for (out, 0..) |*b, i| b.* = self.regs[(start_reg + i) % self.regs.len];
    }
};

test "the driver round-trips a time through a register file" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus);

    const set = DateTime{ .year = 2026, .month = 8, .date = 25, .hours = 20, .minutes = 15, .seconds = 1 };
    try dev.setTime(set);
    // Written as one burst starting at 00h, not seven single writes.
    try testing.expectEqual(@as(u8, 0x00), bus.last_write_reg);

    const got = try dev.getTime();
    try testing.expectEqual(set.hours, got.hours);
    try testing.expectEqual(set.date, got.date);
    try testing.expectEqual(set.year, got.year);
    try testing.expectEqual(@as(u8, 2), got.weekday); // computed, not supplied
}

test "the driver refuses an impossible date rather than storing it" {
    var bus = FakeBus{};
    var dev = Driver(*FakeBus).init(&bus);
    // The part would accept this and read it back unchanged.
    const bad = DateTime{ .year = 2026, .month = 2, .date = 31 };
    try testing.expectError(error.InvalidDateTime, dev.setTime(bad));
    try testing.expectEqual(@as(u8, 0xFF), bus.last_write_reg); // nothing written
}

test "OSF is reported and cleared without disturbing the rest" {
    var bus = FakeBus{};
    bus.regs[@backingInt(Register.status)] = 0x88; // OSF set, EN32kHz set
    var dev = Driver(*FakeBus).init(&bus);

    try testing.expect(try dev.oscillatorStopped());
    try dev.clearOscillatorStopped();
    try testing.expect(!(try dev.oscillatorStopped()));
    // EN32kHz survived.
    try testing.expect((try dev.readStatus()).en32khz);
}
