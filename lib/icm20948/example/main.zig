//! ICM-20948 Staged Bring-Up Executable (Stage 5 Demo - 9-DOF Telemetry) for Pimoroni Pico LiPo 2

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const icm20948 = @import("icm20948");
const bus_picozig = @import("bus_picozig");

comptime {
    _ = &boot;
}

pub const watch_magic: u32 = 0x1C020948;

/// Diagnostic block pinned to RAM base 0x20000000 (.selftest)
pub const Watch = extern struct {
    magic: u32,
    ticks: u32,
    stage: u32,
    who_am_i: u32,
    i2c_found_count: u32,
    detected_addr: u32,
    pwr1: u32,
    pwr2: u32,
    gyro_cfg: u32,
    accel_cfg: u32,
    who_verify: u32,
    mag_wia2: u32,
    accel_x_mg: i32,
    accel_y_mg: i32,
    accel_z_mg: i32,
    gyro_x_dps: i32,
    gyro_y_dps: i32,
    gyro_z_dps: i32,
    mag_x_ut_x10: i32,
    mag_y_ut_x10: i32,
    mag_z_ut_x10: i32,
    temp_c_x10: i32,
    error_code: u32,
};

export var watch: Watch linksection(".selftest") = undefined;

fn delayMs(ms: u32) void {
    rp2350.timer.delayMs(ms);
}

pub fn main() noreturn {
    // 1. Initialize diagnostic memory at RAM 0x20000000
    watch = .{
        .magic = 0,
        .ticks = 0,
        .stage = 0,
        .who_am_i = 0,
        .i2c_found_count = 0,
        .detected_addr = 0,
        .pwr1 = 0,
        .pwr2 = 0,
        .gyro_cfg = 0,
        .accel_cfg = 0,
        .who_verify = 0,
        .mag_wia2 = 0,
        .accel_x_mg = 0,
        .accel_y_mg = 0,
        .accel_z_mg = 0,
        .gyro_x_dps = 0,
        .gyro_y_dps = 0,
        .gyro_z_dps = 0,
        .mag_x_ut_x10 = 0,
        .mag_y_ut_x10 = 0,
        .mag_z_ut_x10 = 0,
        .temp_c_x10 = 0,
        .error_code = 0,
    };

    // 2. Init system clocks
    rp2350.clocks.init(board.clock_plan);

    // 3. Release GPIO banks from reset
    rp2350.gpio.init();

    // 4. Init UART debug console
    rp2350.uart.init(.uart0, board.uartConfig());

    // 5. Init onboard LED
    if (board.led) |p| rp2350.gpio.initOutput(p);

    rp2350.uart.print(.uart0, "\n=== ICM-20948 Stage 5 Demo (9-DOF Telemetry) ===\n", .{});
    rp2350.uart.print(.uart0, "Board: {s}\n", .{board.name});

    // 6. Init I2C0 on Qw/ST (GP4 / GP5)
    var bus = bus_picozig.Bus.init(.i2c0, 4, 5, board.sys_hz);

    watch.i2c_found_count = 1;
    watch.detected_addr = icm20948.default_address;

    var dev = bus_picozig.Device.init(&bus, icm20948.default_address);

    // ------------------------------------------------------------------
    // STAGE 1: Read WHO_AM_I (Expected: 0xEA)
    // ------------------------------------------------------------------
    const who = dev.readWhoAmI() catch |err| {
        rp2350.uart.print(.uart0, "[FAIL] I2C error reading WHO_AM_I: {s}\n", .{@errorName(err)});
        watch.error_code = switch (err) {
            error.Timeout => 103,
            error.NoAcknowledge => 104,
        };
        watch.magic = watch_magic;
        hang();
    };

    watch.who_am_i = who;
    rp2350.uart.print(.uart0, "[PASS] WHO_AM_I = 0x{X:02}\n", .{who});

    if (who != icm20948.who_am_i_expected) {
        rp2350.uart.print(.uart0, "[FAIL] Unexpected WHO_AM_I value!\n", .{});
        watch.error_code = 102;
        watch.magic = watch_magic;
        hang();
    }
    watch.stage = 1;

    // ------------------------------------------------------------------
    // STAGE 2: Power Management & Soft Reset
    // ------------------------------------------------------------------
    rp2350.uart.print(.uart0, "\n--- STAGE 2: Power Reset & Clock Select ---\n", .{});
    dev.resetAndWake(delayMs) catch |err| {
        rp2350.uart.print(.uart0, "[FAIL] Soft reset failed: {}\n", .{err});
        watch.error_code = 201;
        watch.magic = watch_magic;
        hang();
    };

    const pwr1 = dev.readReg(.bank_0, icm20948.RegBank0.PWR_MGMT_1) catch 0xFF;
    const pwr2 = dev.readReg(.bank_0, icm20948.RegBank0.PWR_MGMT_2) catch 0xFF;

    watch.pwr1 = pwr1;
    watch.pwr2 = pwr2;

    rp2350.uart.print(.uart0, "[PASS] PWR_MGMT_1 = 0x{X:02}, PWR_MGMT_2 = 0x{X:02}\n", .{ pwr1, pwr2 });

    if ((pwr1 & 0x40) != 0 or pwr2 != 0x00) {
        rp2350.uart.print(.uart0, "[FAIL] Unexpected power management state!\n", .{});
        watch.error_code = 202;
        watch.magic = watch_magic;
        hang();
    }
    watch.stage = 2;

    // ------------------------------------------------------------------
    // STAGE 3: Register Bank Switch & Sensor Config
    // ------------------------------------------------------------------
    rp2350.uart.print(.uart0, "\n--- STAGE 3: Register Bank Switch & Config ---\n", .{});
    dev.configureSensors(.dps_2000, .g_4) catch |err| {
        rp2350.uart.print(.uart0, "[FAIL] Sensor config failed: {}\n", .{err});
        watch.error_code = 301;
        watch.magic = watch_magic;
        hang();
    };

    const gyro_cfg = dev.readReg(.bank_2, icm20948.RegBank2.GYRO_CONFIG_1) catch 0xFF;
    const accel_cfg = dev.readReg(.bank_2, icm20948.RegBank2.ACCEL_CONFIG) catch 0xFF;

    watch.gyro_cfg = gyro_cfg;
    watch.accel_cfg = accel_cfg;

    const who_verify = dev.readWhoAmI() catch 0;
    watch.who_verify = who_verify;

    rp2350.uart.print(.uart0, "[PASS] Bank 2 -> GYRO_CFG: 0x{X:02}, ACCEL_CFG: 0x{X:02}\n", .{ gyro_cfg, accel_cfg });
    rp2350.uart.print(.uart0, "[PASS] Return to Bank 0 -> WHO_AM_I: 0x{X:02}\n", .{who_verify});

    if (who_verify != icm20948.who_am_i_expected) {
        rp2350.uart.print(.uart0, "[FAIL] Bank switch back failed!\n", .{});
        watch.error_code = 302;
        watch.magic = watch_magic;
        hang();
    }
    watch.stage = 3;

    // ------------------------------------------------------------------
    // STAGE 5: AK09916 3-Axis Magnetometer Integration
    // ------------------------------------------------------------------
    rp2350.uart.print(.uart0, "\n--- STAGE 5: AK09916 Auxiliary Magnetometer Setup ---\n", .{});
    const mag_wia2 = dev.initMagnetometer(delayMs) catch |err| {
        rp2350.uart.print(.uart0, "[FAIL] Magnetometer setup failed: {}\n", .{err});
        watch.error_code = 501;
        watch.magic = watch_magic;
        hang();
    };

    watch.mag_wia2 = mag_wia2;
    rp2350.uart.print(.uart0, "[DEBUG] AK09916 WIA2 read: 0x{X:02} (Expected: 0x{X:02})\n", .{ mag_wia2, icm20948.ak09916_wia2_expected });

    if (mag_wia2 != icm20948.ak09916_wia2_expected) {
        rp2350.uart.print(.uart0, "[FAIL] Unexpected AK09916 WIA2 ID!\n", .{});
        watch.error_code = 502;
        watch.magic = watch_magic;
        hang();
    }

    watch.stage = 5;
    watch.error_code = 0;
    watch.magic = watch_magic;

    rp2350.uart.print(.uart0, "\n*** STAGE 5 DEMO ACTIVE (Continuous 9-DOF Polling) ***\n", .{});

    var on = false;
    while (true) {
        delayMs(100);
        const data9 = dev.read9DofData() catch |err| {
            rp2350.uart.print(.uart0, "[FAIL] Reading 9-DOF data: {}\n", .{err});
            watch.error_code = 503;
            continue;
        };

        // Telemetry representations
        watch.accel_x_mg = @intFromFloat(data9.sensor.accel_g.x * 1000.0);
        watch.accel_y_mg = @intFromFloat(data9.sensor.accel_g.y * 1000.0);
        watch.accel_z_mg = @intFromFloat(data9.sensor.accel_g.z * 1000.0);

        watch.gyro_x_dps = @intFromFloat(data9.sensor.gyro_dps.x * 10.0);
        watch.gyro_y_dps = @intFromFloat(data9.sensor.gyro_dps.y * 10.0);
        watch.gyro_z_dps = @intFromFloat(data9.sensor.gyro_dps.z * 10.0);

        watch.mag_x_ut_x10 = @intFromFloat(data9.mag.mag_ut.x * 10.0);
        watch.mag_y_ut_x10 = @intFromFloat(data9.mag.mag_ut.y * 10.0);
        watch.mag_z_ut_x10 = @intFromFloat(data9.mag.mag_ut.z * 10.0);

        watch.temp_c_x10 = @intFromFloat(data9.sensor.temp_c * 10.0);

        rp2350.uart.print(.uart0, "[#{d:04}] Acc[g]:({d:5.2},{d:5.2},{d:5.2}) | Gyr[dps]:({d:6.1},{d:6.1},{d:6.1}) | Mag[uT]:({d:6.1},{d:6.1},{d:6.1}) | Temp:{d:4.1}C\n", .{
            watch.ticks,
            data9.sensor.accel_g.x,
            data9.sensor.accel_g.y,
            data9.sensor.accel_g.z,
            data9.sensor.gyro_dps.x,
            data9.sensor.gyro_dps.y,
            data9.sensor.gyro_dps.z,
            data9.mag.mag_ut.x,
            data9.mag.mag_ut.y,
            data9.mag.mag_ut.z,
            data9.sensor.temp_c,
        });

        on = !on;
        if (board.led) |p| {
            if (on) rp2350.gpio.set(p) else rp2350.gpio.clear(p);
        }
        watch.ticks +%= 1;
    }
}

fn hang() noreturn {
    while (true) {
        delayMs(1000);
    }
}
