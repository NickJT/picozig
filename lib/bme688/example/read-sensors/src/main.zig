//! read-sensors — BME688 4-in-1 environmental sensor example for Raspberry Pi Pico 2 W.
//!
//! Measures Temperature, Pressure, Humidity, and MOX Gas Resistance over I2C (GP4/GP5),
//! prints results to UART0 (115200 8N1), and exports RAM block at 0x20000000 for SWD monitoring.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const bme688 = @import("bme688");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    err_code: u32,
    temp_x100: i32, // Temperature in °C * 100 (e.g. 2245 = 22.45 °C)
    pres_x10: u32, // Pressure in hPa * 10 (e.g. 10132 = 1013.2 hPa)
    hum_x100: u32, // Humidity in %RH * 100 (e.g. 4520 = 45.20 %RH)
    gas_ohms: u32, // Gas resistance in Ohms
    raw_status: u32,
};

pub const watch_magic: u32 = 0x6880_c4b0;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== BME688 read-sensors example ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .err_code = 0,
        .temp_x100 = 0,
        .pres_x10 = 0,
        .hum_x100 = 0,
        .gas_ohms = 0,
        .raw_status = 0,
    };

    // GP4 (SDA) / GP5 (SCL) on i2c0 (Qwiic / STEMMA QT port)
    const sda_pin: rp2350.Pin = 4;
    const scl_pin: rp2350.Pin = 5;

    i2c.init(.i2c0, .{
        .sda = sda_pin,
        .scl = scl_pin,
        .speed = .standard_100k,
        .clk_hz = board.sys_hz,
    });

    const bus = bme688.PicoI2c{ .inst = .i2c0 };
    var sensor = bme688.Driver(bme688.PicoI2c).init(bus, bme688.i2c_address_low) catch |err| {
        uart.print(.uart0, "ERROR: Failed to initialize BME688: {s}\n", .{@errorName(err)});
        watch.magic = 0xdead_0001;
        watch.err_code = @intFromError(err);
        while (true) {
            timer.delayMs(1000);
        }
    };

    uart.print(.uart0, "BME688 initialized! Chip ID: 0x{x:0>2}, Variant ID: 0x{x:0>2}\n", .{
        bme688.expected_chip_id,
        sensor.variant_id,
    });

    // Configure oversampling (2x Temp, 16x Pres, 1x Hum) and IIR Filter (c3)
    sensor.setOversampling(.x2, .x16, .x1) catch {};
    sensor.setIirFilter(.c3) catch {};

    // Configure MOX Gas heater profile (300 °C for 100 ms)
    sensor.setHeaterProfile(300, 100, 25) catch {};

    uart.print(.uart0, "Sampling starting (Forced Mode, 300°C / 100ms heater profile)...\n\n", .{});

    var count: u32 = 0;
    while (true) {
        sensor.triggerForcedMeasurement() catch |err| {
            uart.print(.uart0, "ERROR triggering measurement: {s}\n", .{@errorName(err)});
            watch.magic = 0xdead_0002;
            watch.err_code = @intFromError(err);
        };

        // Wait measurement duration (TPH + Gas heater = ~150 ms)
        timer.delayMs(150);

        if (sensor.readSensorData()) |data| {
            count += 1;
            watch.ticks = count;
            watch.temp_x100 = @intFromFloat(data.temperature_deg_c * 100.0);
            watch.pres_x10 = @intFromFloat(data.pressure_hpa * 10.0);
            watch.hum_x100 = @intFromFloat(data.humidity_percent * 100.0);
            watch.gas_ohms = @intFromFloat(data.gas_resistance_ohms);
            watch.raw_status = data.raw_status;

            const temp_whole: i32 = @divTrunc(watch.temp_x100, 100);
            const temp_frac: u32 = @intCast(@abs(@rem(watch.temp_x100, 100)));

            const pres_whole: u32 = watch.pres_x10 / 10;
            const pres_frac: u32 = watch.pres_x10 % 10;

            const hum_whole: u32 = watch.hum_x100 / 100;
            const hum_frac: u32 = watch.hum_x100 % 100;

            uart.print(.uart0, "[BME688] T: {d}.{d:0>2} °C | P: {d}.{d} hPa | H: {d}.{d:0>2} %RH | Gas: {d} Ω (tick {d}s)\n", .{
                temp_whole,
                temp_frac,
                pres_whole,
                pres_frac,
                hum_whole,
                hum_frac,
                watch.gas_ohms,
                count,
            });
        } else |err| {
            uart.print(.uart0, "ERROR reading sensor data: {s}\n", .{@errorName(err)});
            watch.magic = 0xdead_0003;
            watch.err_code = @intFromError(err);
        }

        timer.delayMs(1850); // Loop total ~2s
    }
}
