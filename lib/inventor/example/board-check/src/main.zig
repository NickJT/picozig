//! inventor GP2 toggle + UART test (SWD Proof Test).

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

comptime {
    _ = &boot;
}

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const gp2_pin: gpio.Pin = 2;

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    gpio.initOutput(gp2_pin);

    uart.print(.uart0, "\n=======================================\n", .{});
    uart.print(.uart0, "  SWD FLASH PROOF: SWD Flashing Works!  \n", .{});
    uart.print(.uart0, "  Board: {s}\n", .{board.name});
    uart.print(.uart0, "=======================================\n\n", .{});

    var state: bool = false;
    var count: u32 = 0;
    while (true) {
        state = !state;
        gpio.put(gp2_pin, state);
        count +%= 1;
        uart.print(.uart0, "[SWD-{d}] GP2 state = {s}\n", .{ count, if (state) "HIGH" else "LOW" });
        timer.delayMs(1000);
    }
}
