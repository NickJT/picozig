//! Inventor 2350 W RGB LED Example.
//!
//! Drives the 12 onboard WS2812 RGB LEDs on GP20 using PIO0.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const inventor = @import("inventor");

comptime {
    _ = &boot;
}

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const pio = rp2350.pio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const led_pin: gpio.Pin = 20;
const num_leds: usize = 12;

const Leds = inventor.ws2812.Driver(led_pin, num_leds);

/// Report and hold. Better than a reset loop on a demo: the message stays on
/// the UART where it can be read, instead of scrolling past once a second.
fn stall(comptime why: []const u8) noreturn {
    uart.print(.uart0, "\n[LEDs] {s}\n", .{why});
    while (true) asm volatile ("wfi");
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=======================================\n", .{});
    uart.print(.uart0, "  INVENTOR 2350 W: RGB LED Example     \n", .{});
    uart.print(.uart0, "  Board: {s}                           \n", .{board.name});
    uart.print(.uart0, "=======================================\n\n", .{});

    // A WS2812 wait can only fail one way — the state machine stopped
    // consuming — and on a demo the useful response is to say so and hold,
    // not to carry on driving lights that are not listening.
    var leds = Leds.init(.pio0, 0, board.sys_hz) catch stall("LED driver init timed out");

    uart.print(.uart0, "WS2812 PIO driver initialised on GP{d} for {d} LEDs.\n", .{ led_pin, num_leds });

    var offset: u16 = 0;
    var step: u32 = 0;

    while (true) {
        // Rainbow cycle across the 12 LEDs
        for (0..num_leds) |i| {
            const hue: u16 = @intCast((offset + @as(u16, @intCast(i)) * (360 / num_leds)) % 360);
            leds.setHsv(i, hue, 255, 100);
        }
        leds.show() catch stall("LED refresh timed out");

        offset = (offset + 5) % 360;
        step +%= 1;

        if (step % 50 == 0) {
            uart.print(.uart0, "[LEDs-{d}] Rainbow hue offset = {d}\n", .{ step, offset });
        }

        timer.delayMs(20);
    }
}
