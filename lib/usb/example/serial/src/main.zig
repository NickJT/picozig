//! **The template.** A USB serial port, in as little code as this gets.
//!
//! Flash it, plug the board into a computer, open the port it creates —
//! `/dev/ttyACM*` on Linux, `/dev/cu.usbmodem*` on macOS — and you get a
//! counter printing and an echo of anything you type. No driver to install.
//!
//! **Copy this file and start editing.** Everything below is either something
//! you will change (the product string, what gets printed) or one line you
//! must keep (the `interrupts` namespace).
//!
//! ## The three things that are not obvious
//!
//! **`clk_sys` must be above 48 MHz.** RP2350-E12 requires it for the USB
//! controller, and picozig's default — no clock plan, straight off the 12 MHz
//! crystal — is below it. So this file declares its own 150 MHz plan. Needing a
//! fast clock is a property of doing USB, not of the board; try it at 12 MHz
//! and `clocks.initUsb` refuses to compile rather than failing on the bench.
//!
//! **The `usbctrl` vector must be forwarded.** USB is serviced by its
//! interrupt, and `boot.zig` builds the vector table from *this* module's
//! declarations — so a library cannot claim it and the application does, in the
//! four lines below. Forgetting is not silent: every unclaimed vector traps and
//! records which one fired.
//!
//! **Output produced before a terminal opens the port is discarded.** That is
//! deliberate — see `SerialPort`'s header. If you need the first few
//! milliseconds of boot, print those to a UART.
//!
//! For what is happening underneath, flash `example/usb-trace` instead: it is
//! this same device, narrating every control transfer as it goes.

const std = @import("std");
const rp2350 = @import("rp2350");
const boot = @import("boot");
const usb = @import("usb");

comptime {
    _ = &boot;
}

const plan = rp2350.clocks.Plan.require(150_000_000);

const Port = usb.SerialPort(.{ .product = "picozig serial" });
var port: Port = .{};

/// **This is not really your code — it is the library's handler, given a home.**
///
/// It looks like something you have to write, and that reading is the reason
/// this comment exists. You do not implement anything here; you forward one
/// call. `port.onIrq()` is where the work happens.
///
/// The reason it cannot live inside the library: `boot.zig` builds the
/// interrupt vector table at comptime from the declarations of the **root**
/// module — the one with `main` in it — so only this file can claim a vector.
/// The function's *name* is the wiring: `usbctrl` is matched against
/// `nvic.IrqNum`, so a typo is a compile error rather than a handler that never
/// runs.
///
/// Copy it as it stands. The only thing to change is `port`, if you rename it.
pub const interrupts = struct {
    pub fn usbctrl() callconv(.c) void {
        port.onIrq();
    }
};

pub fn main() noreturn {
    rp2350.clocks.init(plan);
    rp2350.gpio.init();
    port.start(plan.sys_hz);

    var buf: [256]u8 = undefined;
    var out = port.writer(&buf);

    var ticks: u32 = 0;
    var last: u32 = 0;
    var seen: [64]u8 = undefined;

    while (true) {
        // No servicing call here: the interrupt does it. This loop is only the
        // application's own work.

        // Echo anything typed at the port.
        const got = port.read(&seen);
        if (got.len > 0) _ = port.write(got);

        // And print a line a second, to show the writer works.
        const now: u32 = @truncate(rp2350.timer.now64());
        if (now -% last > 1_000_000) {
            last = now;
            ticks += 1;
            out.print("tick {d}, dropped {d}\n", .{ ticks, port.dropped }) catch {};
            out.flush() catch {};
        }
    }
}
