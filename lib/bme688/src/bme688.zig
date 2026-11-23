//! Bosch BME688 4-in-1 Environmental Sensor Driver (Temperature, Pressure, Humidity, Gas Resistance).
//!
//! Freestanding, hardware-agnostic Zig driver focused on Forced Mode single-shot environmental sensing.

const std = @import("std");
const rp2350 = @import("rp2350");

/// Standard 7-bit I2C addresses for the BME688.
pub const i2c_address_low: u7 = 0x76;
pub const i2c_address_high: u7 = 0x77;

/// Expected BME688 unique chip identifier.
pub const expected_chip_id: u8 = 0x61;

/// Register addresses for BME688 per Bosch datasheet / C API.
pub const Register = enum(u8) {
    coeff3 = 0x00,
    field0 = 0x1D,
    idac_heat0 = 0x50,
    res_heat0 = 0x5A,
    gas_wait0 = 0x64,
    shd_heatr_dur = 0x6E,
    ctrl_gas_0 = 0x70,
    ctrl_gas_1 = 0x71,
    ctrl_hum = 0x72,
    ctrl_meas = 0x74,
    config = 0x75,
    coeff1 = 0x8A,
    chip_id = 0xD0,
    soft_reset = 0xE0,
    coeff2 = 0xE1,
    variant_id = 0xF0,
};

/// Oversampling settings for Temperature, Pressure, and Humidity.
pub const Oversampling = enum(u3) {
    none = 0,
    x1 = 1,
    x2 = 2,
    x4 = 3,
    x8 = 4,
    x16 = 5,
};

/// IIR Filter coefficient settings.
pub const Filter = enum(u3) {
    off = 0,
    c1 = 1,
    c3 = 2,
    c7 = 3,
    c15 = 4,
    c31 = 5,
    c63 = 6,
    c127 = 7,
};

/// Sensor measurement reading result.
pub const Data = struct {
    temperature_deg_c: f32 = 0.0,
    pressure_hpa: f32 = 0.0,
    humidity_percent: f32 = 0.0,
    gas_resistance_ohms: f32 = 0.0,
    raw_status: u8 = 0,
    new_data: bool = false,
    gas_valid: bool = false,
    heat_stable: bool = false,
};

/// Calibration parameter structure extracted from BME688 NVM.
pub const CalibrationData = struct {
    par_t1: u16 = 0,
    par_t2: i16 = 0,
    par_t3: i8 = 0,

    par_p1: u16 = 0,
    par_p2: i16 = 0,
    par_p3: i8 = 0,
    par_p4: i16 = 0,
    par_p5: i16 = 0,
    par_p6: i8 = 0,
    par_p7: i8 = 0,
    par_p8: i16 = 0,
    par_p9: i16 = 0,
    par_p10: u8 = 0,

    par_h1: u16 = 0,
    par_h2: u16 = 0,
    par_h3: i8 = 0,
    par_h4: i8 = 0,
    par_h5: i8 = 0,
    par_h6: u8 = 0,
    par_h7: i8 = 0,

    par_gh1: i8 = 0,
    par_gh2: i16 = 0,
    par_gh3: i8 = 0,

    res_heat_range: u8 = 0,
    res_heat_val: i8 = 0,
    range_sw_err: i8 = 0,

    t_fine: f32 = 0.0,
};

/// Transport adapter wrapping `rp2350.i2c`.
pub const PicoI2c = struct {
    inst: rp2350.i2c.Instance,
    timeout_polls: u32 = 10000,

    pub fn writeRegs(self: PicoI2c, addr: u7, start_reg: u8, data: []const u8) !void {
        var buf: [32]u8 = undefined;
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

/// Generic BME688 Driver parameterised over an I2C bus interface.
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,
        addr: u7,
        calib: CalibrationData = .{},
        variant_id: u8 = 0,

        pub fn init(bus: I2cBus, addr: u7) !Self {
            var self = Self{ .bus = bus, .addr = addr };
            try self.softReset();
            const chip_val = try self.readChipId();
            if (chip_val != expected_chip_id) {
                return error.InvalidChipId;
            }
            try self.readCalibration();
            return self;
        }

        pub fn softReset(self: *Self) !void {
            const cmd = [1]u8{0xB6};
            try self.bus.writeRegs(self.addr, @backingInt(Register.soft_reset), &cmd);
            if (@import("builtin").target.os.tag == .freestanding) {
                rp2350.timer.delayUs(5000);
            }
        }

        pub fn readChipId(self: *Self) !u8 {
            var buf: [1]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.chip_id), &buf);
            return buf[0];
        }

        pub fn readCalibration(self: *Self) !void {
            var coeff: [42]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.coeff1), coeff[0..23]);
            try self.bus.readRegs(self.addr, @backingInt(Register.coeff2), coeff[23..37]);
            try self.bus.readRegs(self.addr, @backingInt(Register.coeff3), coeff[37..42]);

            var var_id: [1]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.variant_id), &var_id);
            self.variant_id = var_id[0];

            // Temperature calibration coefficients
            self.calib.par_t1 = concatBytes(coeff[32], coeff[31]);
            self.calib.par_t2 = @bitCast(concatBytes(coeff[1], coeff[0]));
            self.calib.par_t3 = @bitCast(coeff[2]);

            // Pressure calibration coefficients
            self.calib.par_p1 = concatBytes(coeff[5], coeff[4]);
            self.calib.par_p2 = @bitCast(concatBytes(coeff[7], coeff[6]));
            self.calib.par_p3 = @bitCast(coeff[8]);
            self.calib.par_p4 = @bitCast(concatBytes(coeff[11], coeff[10]));
            self.calib.par_p5 = @bitCast(concatBytes(coeff[13], coeff[12]));
            self.calib.par_p6 = @bitCast(coeff[15]);
            self.calib.par_p7 = @bitCast(coeff[14]);
            self.calib.par_p8 = @bitCast(concatBytes(coeff[19], coeff[18]));
            self.calib.par_p9 = @bitCast(concatBytes(coeff[21], coeff[20]));
            self.calib.par_p10 = coeff[22];

            // Humidity calibration coefficients
            self.calib.par_h1 = (@as(u16, coeff[25]) << 4) | (coeff[24] & 0x0F);
            self.calib.par_h2 = (@as(u16, coeff[23]) << 4) | (coeff[24] >> 4);
            self.calib.par_h3 = @bitCast(coeff[26]);
            self.calib.par_h4 = @bitCast(coeff[27]);
            self.calib.par_h5 = @bitCast(coeff[28]);
            self.calib.par_h6 = coeff[29];
            self.calib.par_h7 = @bitCast(coeff[30]);

            // Gas heater calibration coefficients
            self.calib.par_gh1 = @bitCast(coeff[35]);
            self.calib.par_gh2 = @bitCast(concatBytes(coeff[34], coeff[33]));
            self.calib.par_gh3 = @bitCast(coeff[36]);

            // Other coefficients
            self.calib.res_heat_range = (coeff[39] & 0x30) >> 4;
            self.calib.res_heat_val = @bitCast(coeff[37]);
            self.calib.range_sw_err = @bitCast(@as(u8, (coeff[41] & 0xF0)) >> 4);
        }

        pub fn setOversampling(self: *Self, temp: Oversampling, pres: Oversampling, hum: Oversampling) !void {
            const ctrl_hum = [1]u8{@backingInt(hum)};
            try self.bus.writeRegs(self.addr, @backingInt(Register.ctrl_hum), &ctrl_hum);

            var current_meas: [1]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.ctrl_meas), &current_meas);

            const new_meas = (current_meas[0] & 0x03) |
                (@as(u8, @backingInt(temp)) << 5) |
                (@as(u8, @backingInt(pres)) << 2);

            const buf = [1]u8{new_meas};
            try self.bus.writeRegs(self.addr, @backingInt(Register.ctrl_meas), &buf);
        }

        pub fn setIirFilter(self: *Self, filter: Filter) !void {
            var config_val: [1]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.config), &config_val);
            const new_cfg = (config_val[0] & ~@as(u8, 0x1C)) | (@as(u8, @backingInt(filter)) << 2);
            const buf = [1]u8{new_cfg};
            try self.bus.writeRegs(self.addr, @backingInt(Register.config), &buf);
        }

        pub fn setHeaterProfile(self: *Self, target_temp_deg_c: u16, duration_ms: u16, ambient_temp_deg_c: i16) !void {
            const res_heat = self.calcResHeat(target_temp_deg_c, ambient_temp_deg_c);
            const gas_wait = self.calcGasWait(duration_ms);

            const res_buf = [1]u8{res_heat};
            try self.bus.writeRegs(self.addr, @backingInt(Register.res_heat0), &res_buf);

            const wait_buf = [1]u8{gas_wait};
            try self.bus.writeRegs(self.addr, @backingInt(Register.gas_wait0), &wait_buf);

            // Enable heater & gas conversion for step 0
            const ctrl_gas0 = [1]u8{0x00}; // turn heater ON (hctrl = 0)
            try self.bus.writeRegs(self.addr, @backingInt(Register.ctrl_gas_0), &ctrl_gas0);

            const run_gas_bit: u8 = if (self.variant_id == 1) 0x20 else 0x10;
            const ctrl_gas1 = [1]u8{run_gas_bit}; // run_gas ON, nb_conv = 0
            try self.bus.writeRegs(self.addr, @backingInt(Register.ctrl_gas_1), &ctrl_gas1);
        }

        pub fn triggerForcedMeasurement(self: *Self) !void {
            var ctrl_meas: [1]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.ctrl_meas), &ctrl_meas);
            const forced_val = (ctrl_meas[0] & 0xFC) | 0x01; // mode = 1 (forced)
            const buf = [1]u8{forced_val};
            try self.bus.writeRegs(self.addr, @backingInt(Register.ctrl_meas), &buf);
        }

        pub fn readSensorData(self: *Self) !Data {
            var raw: [17]u8 = undefined;
            try self.bus.readRegs(self.addr, @backingInt(Register.field0), &raw);

            const status = raw[0];
            const new_data = (status & 0x80) != 0;
            const gas_valid = (status & 0x20) != 0;
            const heat_stable = (status & 0x10) != 0;

            const adc_pres: u32 = (@as(u32, raw[2]) << 12) | (@as(u32, raw[3]) << 4) | (raw[4] >> 4);
            const adc_temp: u32 = (@as(u32, raw[5]) << 12) | (@as(u32, raw[6]) << 4) | (raw[7] >> 4);
            const adc_hum: u16 = (@as(u16, raw[8]) << 8) | raw[9];
            const adc_gas: u16 = (@as(u16, raw[13]) << 2) | (raw[14] >> 6);
            const gas_range: u4 = @truncate(raw[14] & 0x0F);

            const temp_comp = self.calcTemperature(adc_temp);
            const pres_comp = self.calcPressure(adc_pres);
            const hum_comp = self.calcHumidity(adc_hum);
            const gas_comp = self.calcGasResistance(adc_gas, gas_range);

            return Data{
                .temperature_deg_c = temp_comp,
                .pressure_hpa = pres_comp / 100.0,
                .humidity_percent = hum_comp,
                .gas_resistance_ohms = gas_comp,
                .raw_status = status,
                .new_data = new_data,
                .gas_valid = gas_valid,
                .heat_stable = heat_stable,
            };
        }

        fn calcTemperature(self: *Self, temp_adc: u32) f32 {
            const adc_f = @as(f32, @floatFromInt(temp_adc));
            const t1_f = @as(f32, @floatFromInt(self.calib.par_t1));
            const t2_f = @as(f32, @floatFromInt(self.calib.par_t2));
            const t3_f = @as(f32, @floatFromInt(self.calib.par_t3));

            const var1 = ((adc_f / 16384.0) - (t1_f / 1024.0)) * t2_f;
            const term = (adc_f / 131072.0) - (t1_f / 8192.0);
            const var2 = term * term * (t3_f * 16.0);

            self.calib.t_fine = var1 + var2;
            return self.calib.t_fine / 5120.0;
        }

        fn calcPressure(self: *Self, pres_adc: u32) f32 {
            const tf = self.calib.t_fine;
            const pres_f = @as(f32, @floatFromInt(pres_adc));

            var var1 = (tf / 2.0) - 64000.0;
            var var2 = var1 * var1 * (@as(f32, @floatFromInt(self.calib.par_p6)) / 131072.0);
            var2 = var2 + (var1 * @as(f32, @floatFromInt(self.calib.par_p5)) * 2.0);
            var2 = (var2 / 4.0) + (@as(f32, @floatFromInt(self.calib.par_p4)) * 65536.0);

            var1 = (((@as(f32, @floatFromInt(self.calib.par_p3)) * var1 * var1) / 16384.0) +
                (@as(f32, @floatFromInt(self.calib.par_p2)) * var1)) / 524288.0;
            var1 = (1.0 + (var1 / 32768.0)) * @as(f32, @floatFromInt(self.calib.par_p1));

            if (var1 == 0.0) return 0.0;

            var calc_p = 1048576.0 - pres_f;
            calc_p = (((calc_p - (var2 / 4096.0)) * 6250.0) / var1);

            const v1_p9 = (@as(f32, @floatFromInt(self.calib.par_p9)) * calc_p * calc_p) / 2147483648.0;
            const v2_p8 = calc_p * (@as(f32, @floatFromInt(self.calib.par_p8)) / 32768.0);
            const cp_256 = calc_p / 256.0;
            const v3_p10 = cp_256 * cp_256 * cp_256 * (@as(f32, @floatFromInt(self.calib.par_p10)) / 131072.0);

            calc_p = calc_p + (v1_p9 + v2_p8 + v3_p10 + (@as(f32, @floatFromInt(self.calib.par_p7)) * 128.0)) / 16.0;
            return calc_p;
        }

        fn calcHumidity(self: *Self, hum_adc: u16) f32 {
            const temp_comp = self.calib.t_fine / 5120.0;
            const hum_f = @as(f32, @floatFromInt(hum_adc));

            const var1 = hum_f - ((@as(f32, @floatFromInt(self.calib.par_h1)) * 16.0) +
                ((@as(f32, @floatFromInt(self.calib.par_h3)) / 2.0) * temp_comp));

            const var2 = var1 * ((@as(f32, @floatFromInt(self.calib.par_h2)) / 262144.0) *
                (1.0 + ((@as(f32, @floatFromInt(self.calib.par_h4)) / 16384.0) * temp_comp) +
                    ((@as(f32, @floatFromInt(self.calib.par_h5)) / 1048576.0) * temp_comp * temp_comp)));

            const var3 = @as(f32, @floatFromInt(self.calib.par_h6)) / 16384.0;
            const var4 = @as(f32, @floatFromInt(self.calib.par_h7)) / 2097152.0;

            var calc_h = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
            if (calc_h > 100.0) calc_h = 100.0;
            if (calc_h < 0.0) calc_h = 0.0;
            return calc_h;
        }

        fn calcGasResistance(self: *Self, gas_adc: u16, gas_range: u4) f32 {
            const gas_adc_f = @as(f32, @floatFromInt(gas_adc));
            const gas_range_f = @as(f32, @floatFromInt(@as(u32, 1) << gas_range));

            const lookup_k1 = [_]f32{ 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0 };
            const lookup_k2 = [_]f32{ 0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

            const var1 = 1340.0 + (5.0 * @as(f32, @floatFromInt(self.calib.range_sw_err)));
            const var2 = var1 * (1.0 + (lookup_k1[gas_range] / 100.0));
            const var3 = 1.0 + (lookup_k2[gas_range] / 100.0);

            const denom = var3 * 0.000000125 * gas_range_f * (((gas_adc_f - 512.0) / var2) + 1.0);
            if (denom == 0.0) return 0.0;
            return 1.0 / denom;
        }

        fn calcResHeat(self: *Self, temp_deg_c: u16, amb_temp_deg_c: i16) u8 {
            const target_temp: f32 = @floatFromInt(@min(temp_deg_c, 400));
            const amb_temp: f32 = @floatFromInt(amb_temp_deg_c);

            const var1 = (@as(f32, @floatFromInt(self.calib.par_gh1)) / 16.0) + 49.0;
            const var2 = ((@as(f32, @floatFromInt(self.calib.par_gh2)) / 32768.0) * 0.0005) + 0.00235;
            const var3 = @as(f32, @floatFromInt(self.calib.par_gh3)) / 1024.0;
            const var4 = var1 * (1.0 + (var2 * target_temp));
            const var5 = var4 + (var3 * amb_temp);

            const r_heat = 3.4 * ((var5 * (4.0 / (4.0 + @as(f32, @floatFromInt(self.calib.res_heat_range)))) *
                (1.0 / (1.0 + (@as(f32, @floatFromInt(self.calib.res_heat_val)) * 0.002)))) - 25.0);

            if (r_heat < 0.0) return 0;
            if (r_heat > 255.0) return 255;
            return @intFromFloat(r_heat);
        }

        fn calcGasWait(self: *Self, dur_ms: u16) u8 {
            _ = self;
            var dur = dur_ms;
            var factor: u8 = 0;
            if (dur >= 0x0FC0) return 0xFF;

            while (dur > 0x3F) {
                dur /= 4;
                factor += 1;
            }
            return @truncate(@as(u16, @intCast(dur)) + (@as(u16, factor) * 64));
        }
    };
}

fn concatBytes(msb: u8, lsb: u8) u16 {
    return (@as(u16, msb) << 8) | lsb;
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const MockI2cBus = struct {
    registers: [256]u8 = @as([256]u8, @splat(0)),

    pub fn init() MockI2cBus {
        var bus = MockI2cBus{};
        bus.registers[@backingInt(Register.chip_id)] = expected_chip_id;

        // Dummy calibration values in COEFF2 array index 31 & 32 (address 0xE9 & 0xEA)
        // coeff[31] = LSB, coeff[32] = MSB
        const coeff2_base = @backingInt(Register.coeff2);
        bus.registers[coeff2_base + (31 - 23)] = 0x00; // T1 LSB
        bus.registers[coeff2_base + (32 - 23)] = 0x60; // T1 MSB -> 0x6000 = 24576
        return bus;
    }

    pub fn writeRegs(self: *MockI2cBus, addr: u7, start_reg: u8, data: []const u8) !void {
        if (addr != i2c_address_low and addr != i2c_address_high) return error.DeviceNotFound;
        for (data, 0..) |byte, i| {
            const reg = start_reg + @as(u8, @intCast(i));
            self.registers[reg] = byte;
        }
    }

    pub fn readRegs(self: *MockI2cBus, addr: u7, start_reg: u8, out: []u8) !void {
        if (addr != i2c_address_low and addr != i2c_address_high) return error.DeviceNotFound;
        for (out, 0..) |*byte, i| {
            const reg = start_reg + @as(u8, @intCast(i));
            byte.* = self.registers[reg];
        }
    }
};

test "BME688 Mock Initialization & Chip ID Check" {
    var mock_bus = MockI2cBus.init();
    var sensor = try Driver(*MockI2cBus).init(&mock_bus, i2c_address_low);
    try testing.expectEqual(expected_chip_id, sensor.readChipId() catch 0);
    try testing.expectEqual(@as(u16, 24576), sensor.calib.par_t1);
}
