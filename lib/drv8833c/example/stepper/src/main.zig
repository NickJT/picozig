//! DRV8833 Stepper Motor Bring-up Example.
//!
//! Drives a 4-wire bipolar stepper motor connected to a DRV8833 driver
//! on a Pico 2 W / Pimoroni Inventor 2350 W.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const drv8833c = @import("drv8833c");

comptime {
    _ = &boot;
}

const gpio = rp2350.gpio;
const timer = rp2350.timer;

// Motor 1 (Bridge A) & Motor 2 (Bridge B) GPIO control pins
const AIN1_PIN: gpio.Pin = 2;
const AIN2_PIN: gpio.Pin = 3;
const BIN1_PIN: gpio.Pin = 4;
const BIN2_PIN: gpio.Pin = 5;

const StepperDriver = drv8833c.Driver(AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN, null);

pub fn main() noreturn {
    rp2350.clocks.init(board.clock_plan);
    // IO_BANK0/PADS_BANK0 come out of reset here. Without it every pin write
    // below lands in a peripheral still held in reset and does nothing at all —
    // no fault, no diagnostic, just a motor that never moves.
    gpio.init();

    var driver = StepperDriver.init();

    while (true) {
        // Rotate 200 steps forward in full-step mode (5 ms delay per step)
        driver.stepN(200, .forward, .full_step, 5000);
        timer.delayUs(500_000);

        // Rotate 400 steps reverse in half-step mode (2.5 ms delay per step)
        driver.stepN(400, .reverse, .half_step, 2500);
        timer.delayUs(500_000);

        // Coast output coils during idle pause
        driver.coastAll();
        timer.delayUs(1_000_000);
    }
}
