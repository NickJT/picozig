//! set-time — RV3028 RTC driver example for Raspberry Pi Pico 2 W.
//!
//! Sets the RTC time over I2C (GP4/GP5), then loops printing current time
//! to UART0 (115200 8N1) and publishing a RAM block at 0x20000000 for SWD monitoring.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const rv3028 = @import("rv3028");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    err_code: u32,
    year: u32,
    month: u32,
    date: u32,
    hours: u32,
    minutes: u32,
    seconds: u32,
};

pub const watch_magic: u32 = 0x3028_c700;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== RV3028 set-time example ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .err_code = 0,
        .year = 0,
        .month = 0,
        .date = 0,
        .hours = 0,
        .minutes = 0,
        .seconds = 0,
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

    const bus = rv3028.PicoI2c{ .inst = .i2c0 };
    var rtc = rv3028.Driver(rv3028.PicoI2c).init(bus);

    const new_time = rv3028.DateTime{
        .year = 2026,
        .month = 8,
        .date = 9,
        .weekday = 0, // Sunday
        .hours = 15,
        .minutes = 57,
        .seconds = 0,
    };

    uart.print(.uart0, "Setting RTC time to {d:0>4}-{d:0>2}-{d:0>2} {d:0>2}:{d:0>2}:{d:0>2}...\n", .{
        new_time.year,
        new_time.month,
        new_time.date,
        new_time.hours,
        new_time.minutes,
        new_time.seconds,
    });

    rtc.setTime(new_time) catch |err| {
        uart.print(.uart0, "ERROR: Failed to set RTC time: {s}\n", .{@errorName(err)});
        watch.magic = 0xdead_0001;
        watch.err_code = @intFromError(err);
        while (true) {
            timer.delayMs(1000);
        }
    };

    uart.print(.uart0, "RTC time successfully set!\n\n", .{});

    var count: u32 = 0;
    while (true) {
        timer.delayMs(1000);
        count += 1;

        if (rtc.getTime()) |now| {
            watch.ticks = count;
            watch.year = now.year;
            watch.month = now.month;
            watch.date = now.date;
            watch.hours = now.hours;
            watch.minutes = now.minutes;
            watch.seconds = now.seconds;

            uart.print(.uart0, "[RTC] {d:0>4}-{d:0>2}-{d:0>2} {d:0>2}:{d:0>2}:{d:0>2} (tick {d}s)\n", .{
                now.year,
                now.month,
                now.date,
                now.hours,
                now.minutes,
                now.seconds,
                count,
            });
        } else |err| {
            uart.print(.uart0, "[RTC] Read error: {s}\n", .{@errorName(err)});
            watch.magic = 0xdead_0002;
            watch.err_code = @intFromError(err);
        }
    }
}
