//! ICM-20948 9-Axis MotionTracking Driver
//! Pure Zig protocol and register manipulation, generic over I2C transport.

const std = @import("std");

/// Default I2C slave address for ICM-20948 (AD0 pin LOW).
pub const default_address: u7 = 0x68;
/// Alternate I2C slave address (AD0 pin HIGH).
pub const alt_address: u7 = 0x69;

/// Expected WHO_AM_I register value for ICM-20948.
pub const who_am_i_expected: u8 = 0xEA;

/// Expected WHO_AM_I (WIA2) value for internal AK09916 magnetometer.
pub const ak09916_wia2_expected: u8 = 0x09;
pub const ak09916_address: u8 = 0x0C;

pub const RegisterBank = enum(u2) {
    bank_0 = 0,
    bank_1 = 1,
    bank_2 = 2,
    bank_3 = 3,
};

pub const RegBankSel: u8 = 0x7F;

// Bank 0 Registers
pub const RegBank0 = struct {
    pub const WHO_AM_I: u8 = 0x00;
    pub const USER_CTRL: u8 = 0x03;
    pub const LP_CONFIG: u8 = 0x05;
    pub const PWR_MGMT_1: u8 = 0x06;
    pub const PWR_MGMT_2: u8 = 0x07;
    pub const INT_PIN_CFG: u8 = 0x0F;
    pub const INT_ENABLE: u8 = 0x10;
    pub const INT_ENABLE_1: u8 = 0x11;
    pub const DATA_RDY_STATUS: u8 = 0x1A;

    pub const ACCEL_XOUT_H: u8 = 0x2D;
    pub const ACCEL_XOUT_L: u8 = 0x2E;
    pub const ACCEL_YOUT_H: u8 = 0x2F;
    pub const ACCEL_YOUT_L: u8 = 0x30;
    pub const ACCEL_ZOUT_H: u8 = 0x31;
    pub const ACCEL_ZOUT_L: u8 = 0x32;

    pub const GYRO_XOUT_H: u8 = 0x33;
    pub const GYRO_XOUT_L: u8 = 0x34;
    pub const GYRO_YOUT_H: u8 = 0x35;
    pub const GYRO_YOUT_L: u8 = 0x36;
    pub const GYRO_ZOUT_H: u8 = 0x37;
    pub const GYRO_ZOUT_L: u8 = 0x38;

    pub const TEMP_OUT_H: u8 = 0x39;
    pub const TEMP_OUT_L: u8 = 0x3A;

    pub const EXT_SLV_SENS_DATA_00: u8 = 0x3B;
    pub const FIFO_EN_1: u8 = 0x66;
    pub const FIFO_EN_2: u8 = 0x67;
    pub const FIFO_R_W: u8 = 0x72;
};

// Bank 2 Registers
pub const RegBank2 = struct {
    pub const GYRO_SMPLRT_DIV: u8 = 0x00;
    pub const GYRO_CONFIG_1: u8 = 0x01;
    pub const GYRO_CONFIG_2: u8 = 0x02;
    pub const ACCEL_SMPLRT_DIV_1: u8 = 0x10;
    pub const ACCEL_SMPLRT_DIV_2: u8 = 0x11;
    pub const ACCEL_CONFIG: u8 = 0x14;
    pub const ACCEL_CONFIG_2: u8 = 0x15;
    pub const ODR_ALIGN_EN: u8 = 0x09;
};

// Bank 3 Registers
pub const RegBank3 = struct {
    pub const I2C_MST_ODR_CONFIG: u8 = 0x00;
    pub const I2C_MST_CTRL: u8 = 0x01;
    pub const I2C_MST_DELAY_CTRL: u8 = 0x02;
    pub const I2C_SLV0_ADDR: u8 = 0x03;
    pub const I2C_SLV0_REG: u8 = 0x04;
    pub const I2C_SLV0_CTRL: u8 = 0x05;
    pub const I2C_SLV0_DO: u8 = 0x06;
};

// AK09916 Registers (Internal Aux I2C)
pub const RegAK09916 = struct {
    pub const WIA2: u8 = 0x01;
    pub const ST1: u8 = 0x10;
    pub const HXL: u8 = 0x11;
    pub const HXH: u8 = 0x12;
    pub const HYL: u8 = 0x13;
    pub const HYH: u8 = 0x14;
    pub const HZL: u8 = 0x15;
    pub const HZH: u8 = 0x16;
    pub const ST2: u8 = 0x18;
    pub const CNTL2: u8 = 0x31;
    pub const RSV2: u8 = 0x32;
};

pub const GyroFs = enum(u2) {
    dps_250 = 0,
    dps_500 = 1,
    dps_1000 = 2,
    dps_2000 = 3,

    pub fn sensitivity(self: GyroFs) f32 {
        return switch (self) {
            .dps_250 => 131.0,
            .dps_500 => 65.5,
            .dps_1000 => 32.8,
            .dps_2000 => 16.4,
        };
    }
};

pub const AccelFs = enum(u2) {
    g_2 = 0,
    g_4 = 1,
    g_8 = 2,
    g_16 = 3,

    pub fn sensitivity(self: AccelFs) f32 {
        return switch (self) {
            .g_2 => 16384.0,
            .g_4 => 8192.0,
            .g_8 => 4096.0,
            .g_16 => 2048.0,
        };
    }
};

pub const Vector3 = struct {
    x: f32,
    y: f32,
    z: f32,
};

pub const RawVector3 = struct {
    x: i16,
    y: i16,
    z: i16,
};

pub const SensorData = struct {
    raw_accel: RawVector3,
    raw_gyro: RawVector3,
    raw_temp: i16,

    accel_g: Vector3,
    gyro_dps: Vector3,
    temp_c: f32,
};

pub const MagData = struct {
    raw_mag: RawVector3,
    mag_ut: Vector3,
    st2: u8,
    overflow: bool,
};

pub const Full9DofData = struct {
    sensor: SensorData,
    mag: MagData,
};

pub fn Device(comptime Transport: type) type {
    return struct {
        const Self = @typeInfo(@typeInfo(Transport).fn_type.params[0].type.?).pointer.child;
        transport: *Transport,
        address: u7,
        current_bank: ?RegisterBank = null,
        gyro_fs: GyroFs = .dps_250,
        accel_fs: AccelFs = .g_2,

        pub fn init(transport: *Transport, address: u7) Device(Transport) {
            return .{
                .transport = transport,
                .address = address,
                .current_bank = null,
            };
        }

        pub fn setBank(self: *Device(Transport), bank: RegisterBank) !void {
            if (self.current_bank == null or self.current_bank.? != bank) {
                try self.transport.writeReg(self.address, RegBankSel, @backingInt(bank));
                self.current_bank = bank;
            }
        }

        pub fn readReg(self: *Device(Transport), bank: RegisterBank, reg: u8) !u8 {
            try self.setBank(bank);
            return try self.transport.readReg(self.address, reg);
        }

        pub fn writeReg(self: *Device(Transport), bank: RegisterBank, reg: u8, val: u8) !void {
            try self.setBank(bank);
            try self.transport.writeReg(self.address, reg, val);
        }

        pub fn readBytes(self: *Device(Transport), bank: RegisterBank, reg: u8, dest: []u8) !void {
            try self.setBank(bank);
            try self.transport.readBytes(self.address, reg, dest);
        }

        pub fn writeBytes(self: *Device(Transport), bank: RegisterBank, reg: u8, src: []const u8) !void {
            try self.setBank(bank);
            try self.transport.writeBytes(self.address, reg, src);
        }

        /// Stage 1: Read WHO_AM_I register (Bank 0, Reg 0x00). Should return 0xEA.
        pub fn readWhoAmI(self: *Device(Transport)) !u8 {
            return try self.readReg(.bank_0, RegBank0.WHO_AM_I);
        }

        /// Stage 2: Power management & soft reset
        pub fn resetAndWake(self: *Device(Transport), delay_fn: *const fn (ms: u32) void) !void {
            // Soft reset device
            try self.writeReg(.bank_0, RegBank0.PWR_MGMT_1, 0x80);
            delay_fn(10);

            // Clear sleep (bit 6), auto clock select (clksel = 1)
            try self.writeReg(.bank_0, RegBank0.PWR_MGMT_1, 0x01);

            // Enable Accel & Gyro all axes (0x00 = enable all)
            try self.writeReg(.bank_0, RegBank0.PWR_MGMT_2, 0x00);
        }

        /// Stage 3: Configure Gyro & Accel full scale & DLPF
        pub fn configureSensors(self: *Device(Transport), gyro_fs: GyroFs, accel_fs: AccelFs) !void {
            self.gyro_fs = gyro_fs;
            self.accel_fs = accel_fs;

            // Bank 2 configuration
            const gyro_cfg1: u8 = (@as(u8, @backingInt(gyro_fs)) << 1) | 0x01; // Enable DLPF
            try self.writeReg(.bank_2, RegBank2.GYRO_CONFIG_1, gyro_cfg1);

            const accel_cfg: u8 = (@as(u8, @backingInt(accel_fs)) << 1) | 0x01; // Enable DLPF
            try self.writeReg(.bank_2, RegBank2.ACCEL_CONFIG, accel_cfg);

            // Return to Bank 0
            try self.setBank(.bank_0);
        }

        /// Stage 4: Read raw and converted 6-DOF sensor data (Accel, Gyro, Temp)
        pub fn readSensorData(self: *Device(Transport)) !SensorData {
            var buf: [14]u8 = undefined;
            try self.readBytes(.bank_0, RegBank0.ACCEL_XOUT_H, &buf);

            const raw_accel = RawVector3{
                .x = @as(i16, @bitCast([2]u8{ buf[1], buf[0] })),
                .y = @as(i16, @bitCast([2]u8{ buf[3], buf[2] })),
                .z = @as(i16, @bitCast([2]u8{ buf[5], buf[4] })),
            };

            const raw_gyro = RawVector3{
                .x = @as(i16, @bitCast([2]u8{ buf[7], buf[6] })),
                .y = @as(i16, @bitCast([2]u8{ buf[9], buf[8] })),
                .z = @as(i16, @bitCast([2]u8{ buf[11], buf[10] })),
            };

            const raw_temp = @as(i16, @bitCast([2]u8{ buf[13], buf[12] }));

            const a_sens = self.accel_fs.sensitivity();
            const g_sens = self.gyro_fs.sensitivity();

            const accel_g = Vector3{
                .x = @as(f32, @floatFromInt(raw_accel.x)) / a_sens,
                .y = @as(f32, @floatFromInt(raw_accel.y)) / a_sens,
                .z = @as(f32, @floatFromInt(raw_accel.z)) / a_sens,
            };

            const gyro_dps = Vector3{
                .x = @as(f32, @floatFromInt(raw_gyro.x)) / g_sens,
                .y = @as(f32, @floatFromInt(raw_gyro.y)) / g_sens,
                .z = @as(f32, @floatFromInt(raw_gyro.z)) / g_sens,
            };

            // ICM-20948 Temp formula: Temp_degC = ((raw_temp - 0) / 333.87) + 21.0
            const temp_c = (@as(f32, @floatFromInt(raw_temp)) / 333.87) + 21.0;

            return .{
                .raw_accel = raw_accel,
                .raw_gyro = raw_gyro,
                .raw_temp = raw_temp,
                .accel_g = accel_g,
                .gyro_dps = gyro_dps,
                .temp_c = temp_c,
            };
        }

        // Helper to write a register on the internal AK09916 via ICM-20948 SLV0
        fn writeAk09916Reg(self: *Device(Transport), reg: u8, val: u8, delay_fn: *const fn (ms: u32) void) !void {
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_CTRL, 0x00); // Disable SLV0
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_ADDR, ak09916_address); // Write mode (0x0C)
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_REG, reg);
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_DO, val);
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_CTRL, 0x81); // Enable, 1 byte
            delay_fn(15);
        }

        // Helper to read a register on internal AK09916 via ICM-20948 SLV0
        fn readAk09916Reg(self: *Device(Transport), reg: u8, delay_fn: *const fn (ms: u32) void) !u8 {
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_CTRL, 0x00); // Disable SLV0
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_ADDR, ak09916_address | 0x80); // Read mode (0x8C)
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_REG, reg);
            try self.writeReg(.bank_3, RegBank3.I2C_SLV0_CTRL, 0x81); // Enable, 1 byte
            delay_fn(25);
            return try self.readReg(.bank_0, RegBank0.EXT_SLV_SENS_DATA_00);
        }

        /// Stage 5: Setup I2C Bypass and initialize AK09916 Magnetometer
        pub fn initMagnetometer(self: *Device(Transport), delay_fn: *const fn (ms: u32) void) !u8 {
            // 1. Enable I2C Bypass mode in Bank 0 (INT_PIN_CFG bit 1 = 1)
            try self.writeReg(.bank_0, RegBank0.INT_PIN_CFG, 0x02);
            delay_fn(10);

            // 2. Read WIA2 directly from AK09916 at I2C address 0x0C
            var wia2: u8 = 0;
            if (self.transport.readReg(ak09916_address, RegAK09916.WIA2)) |val| {
                wia2 = val;
            } else |_| {}

            if (wia2 == ak09916_wia2_expected) {
                // Reset AK09916 via direct I2C write
                _ = self.transport.writeReg(ak09916_address, RegAK09916.RSV2, 0x01) catch {};
                delay_fn(10);
                // Set Continuous Measurement Mode 2 (100 Hz) via direct I2C write
                _ = self.transport.writeReg(ak09916_address, RegAK09916.CNTL2, 0x08) catch {};
                delay_fn(10);
            }

            try self.setBank(.bank_0);
            return wia2;
        }

        /// Stage 5: Read 9-DOF sensor data (Accel + Gyro + Temp + Mag)
        pub fn read9DofData(self: *Device(Transport)) !Full9DofData {
            const sensor = try self.readSensorData();

            var mag_buf: [9]u8 = undefined;
            // Attempt direct bypass read from AK09916 at address 0x0C starting at ST1 (0x10)
            if (self.transport.readBytes(ak09916_address, RegAK09916.ST1, &mag_buf)) |_| {} else |_| {
                // Fallback to EXT_SLV_SENS_DATA_00
                try self.readBytes(.bank_0, RegBank0.EXT_SLV_SENS_DATA_00, &mag_buf);
            }

            // AK09916 registers are Little-Endian:
            // mag_buf[0] = ST1
            // mag_buf[1..2] = HXL, HXH
            // mag_buf[3..4] = HYL, HYH
            // mag_buf[5..6] = HZL, HZH
            // mag_buf[7] = TMPS (or RSV)
            // mag_buf[8] = ST2
            const raw_mx = @as(i16, @bitCast([2]u8{ mag_buf[1], mag_buf[2] }));
            const raw_my = @as(i16, @bitCast([2]u8{ mag_buf[3], mag_buf[4] }));
            const raw_mz = @as(i16, @bitCast([2]u8{ mag_buf[5], mag_buf[6] }));
            const st2 = mag_buf[8];

            const mag_scale: f32 = 0.15; // 0.15 uT per LSB for AK09916

            const mag = MagData{
                .raw_mag = .{ .x = raw_mx, .y = raw_my, .z = raw_mz },
                .mag_ut = .{
                    .x = @as(f32, @floatFromInt(raw_mx)) * mag_scale,
                    .y = @as(f32, @floatFromInt(raw_my)) * mag_scale,
                    .z = @as(f32, @floatFromInt(raw_mz)) * mag_scale,
                },
                .st2 = st2,
                .overflow = (st2 & 0x08) != 0,
            };

            return .{
                .sensor = sensor,
                .mag = mag,
            };
        }
    };
}

test "gyro sensitivity calculations" {
    try std.testing.expectEqual(@as(f32, 131.0), GyroFs.dps_250.sensitivity());
    try std.testing.expectEqual(@as(f32, 16.4), GyroFs.dps_2000.sensitivity());
}

test "accel sensitivity calculations" {
    try std.testing.expectEqual(@as(f32, 16384.0), AccelFs.g_2.sensitivity());
    try std.testing.expectEqual(@as(f32, 2048.0), AccelFs.g_16.sensitivity());
}
