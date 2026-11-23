//! Inventor 2350 W Servo Motor Example.
//!
//! Drives an RC Servo connected to SERVO 1 (GP10) using 50 Hz PWM.

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
const timer = rp2350.timer;
const uart = rp2350.uart;

const servo_pin: gpio.Pin = 10; // SERVO 1 header

const Servo1 = inventor.servo.Driver(servo_pin);

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=======================================\n", .{});
    uart.print(.uart0, "  INVENTOR 2350 W: Servo Motor Example  \n", .{});
    uart.print(.uart0, "  Board: {s}                           \n", .{board.name});
    uart.print(.uart0, "=======================================\n\n", .{});

    var s1 = Servo1.init(board.sys_hz);

    uart.print(.uart0, "PWM Servo driver initialised on SERVO 1 (GP{d}).\n", .{servo_pin});

    var angle: u8 = 0;
    var direction: i8 = 1;
    var step: u32 = 0;

    while (true) {
        s1.setAngleDeg(angle);

        step +%= 1;
        if (step % 10 == 0) {
            uart.print(.uart0, "[Servo-{d}] Angle = {d} deg\n", .{ step, angle });
        }

        if (direction > 0) {
            if (angle >= 180) {
                direction = -1;
                angle = 180;
            } else {
                angle += 5;
            }
        } else {
            if (angle <= 5) {
                direction = 1;
                angle = 0;
            } else {
                angle -= 5;
            }
        }

        timer.delayMs(50);
    }
}
