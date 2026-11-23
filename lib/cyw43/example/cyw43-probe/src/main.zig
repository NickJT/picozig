//! cyw43-probe — CYW43439 Hardware Bringup & Probe tool for Pico 2 W.
//!
//! Powers up the CYW43439 wireless chip via WL_REG_ON (GP23), verifies gSPI bus
//! connectivity over GP24/GP25/GP29, prints chip ID diagnostics to UART0 (115200 8N1),
//! and publishes an SWD watch block at RAM address 0x20000000.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    err_code: u32,
    chip_id: u32,
    is_responding: u32,
    _pad1: u32,
    _pad2: u32,
    _pad3: u32,
};

pub const watch_magic: u32 = 0x4343_4339;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== CYW43439 Wireless Probe Tool ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .err_code = 0,
        .chip_id = 0,
        .is_responding = 0,
        ._pad1 = 0,
        ._pad2 = 0,
        ._pad3 = 0,
    };

    const pins = cyw43.default_pins;
    uart.print(.uart0, "Pico 2 W Wireless Pins: WL_REG_ON=GP{d}, WL_DATA=GP{d}, WL_CS=GP{d}, WL_CLK=GP{d}\n", .{
        pins.wl_reg_on,
        pins.wl_data,
        pins.wl_cs,
        pins.wl_clk,
    });

    // Sweep the three gSPI PIO programs and report what each one reads back.
    //
    // Which program is right is an empirical question about this board at this
    // clock — the variants differ only in turnaround length and sample edge,
    // and a wrong choice reads as a chip that is not there. Answering it by
    // measurement takes one flash cycle; answering it by reasoning took longer
    // and was wrong.
    const variants = [_]cyw43.gspi.Variant{ .gap01_sample0, .gap0_sample1, .gap010_sample1 };

    var pass: u32 = 0;
    while (true) : (pass += 1) {
        for (variants, 0..) |v, i| {
            var bus = cyw43.PicogSpiBus.init(pins);
            bus.setVariant(v);
            var dev = cyw43.Device(cyw43.PicogSpiBus).init(bus, pins);

            dev.powerUp() catch |err| {
                uart.print(.uart0, "[{s}] power-up failed: {s}\n", .{ @tagName(v), @errorName(err) });
                continue;
            };

            // Read a few times: a variant that is merely marginal answers
            // sometimes, and that is worth telling apart from one that never does.
            var hits: u32 = 0;
            var mask: u32 = 0;
            var last: u32 = 0;
            for (0..8) |k| {
                last = dev.readTestRegister() catch 0;
                if (last == cyw43.gspi.test_pattern) {
                    hits += 1;
                    mask |= @as(u32, 1) << @truncate(k);
                }
            }

            uart.print(.uart0, "[pass {d}] {s:<16} last=0x{X:0>8} hits={d}/8 {s}\n", .{
                pass,
                @tagName(v),
                last,
                hits,
                if (hits == 8) "<-- GOOD" else "",
            });

            watch.ticks = pass;
            // One slot per variant, so the whole sweep is legible over SWD
            // without a serial console attached.
            switch (i) {
                0 => watch._pad1 = hits,
                1 => watch._pad2 = (mask << 8) | hits,
                2 => watch._pad3 = hits,
                else => {},
            }
            if (hits > 0) {
                watch.is_responding = 1;
                watch.chip_id = last;
                watch.err_code = @intCast(i);
            }
        }
        timer.delayMs(2000);
    }
}
