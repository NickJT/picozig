//! RP2350 chip layer — the hardware, and nothing about any particular board.
//!
//! This is the module root, and its most important property is what it **cannot
//! see**: there is no `board.zig` in this directory, so Zig's module system
//! rejects any attempt to reach for one. The layering is a compile error rather
//! than a convention.
//!
//! That boundary is the thing that decides whether a second board (the Pico
//! LiPo 2 W) is a board file or a fork. It was already leaking once — `spi.zig`
//! and `uart.zig` both used to import `board.zig` for their pins — which is why
//! it is now enforced instead of documented.
//!
//! **Scope: RP2350 only, and RP2350B is the supported target.** No RP2040, no
//! multi-chip abstraction. See ../PLAN.md.
//!
//! Everything here follows three rules learned the hard way on this platform,
//! where essentially every mistake fails silently:
//!
//!   1. Register addresses and bit positions come from the datasheet, and carry
//!      a comment when they are a trap (see `hw.zig`'s header).
//!   2. Pure logic — divisor arithmetic, bank selection — is separated from
//!      register access so it can be host-tested.
//!   3. A peripheral is not "working" until hardware says so.

pub const hw = @import("hw.zig");
pub const bounded = @import("bounded.zig");
pub const silicon = @import("silicon.zig");
pub const pins = @import("pins.zig");
pub const gpio = @import("gpio.zig");
pub const clocks = @import("clocks.zig");
pub const timer = @import("timer.zig");
pub const qmi = @import("qmi.zig");
pub const uart = @import("uart.zig");
pub const spi = @import("spi.zig");
pub const adc = @import("adc.zig");
pub const pwm = @import("pwm.zig");
pub const i2c = @import("i2c.zig");
pub const dma = @import("dma.zig");
pub const pio = @import("pio.zig");
pub const flash = @import("flash.zig");
pub const bootrom = @import("bootrom.zig");
pub const powman = @import("powman.zig");
pub const watchdog = @import("watchdog.zig");
pub const nvic = @import("nvic.zig");
pub const trng = @import("trng.zig");
pub const usb = @import("usb.zig");

/// A GPIO number. `u6` because RP2350B has 48 of them.
pub const Pin = hw.Pin;

test {
    // Pull every submodule's tests into `zig test` on this root.
    _ = hw;
    _ = bounded;
    _ = silicon;
    _ = pins;
    _ = gpio;
    _ = clocks;
    _ = timer;
    _ = uart;
    _ = spi;
    _ = adc;
    _ = pwm;
    _ = i2c;
    _ = dma;
    _ = pio;
    _ = flash;
    _ = bootrom;
    _ = powman;
    _ = watchdog;
    _ = nvic;
    _ = trng;
    _ = usb;
}
