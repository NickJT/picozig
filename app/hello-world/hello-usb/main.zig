//! hello-usb — the first thing to run on a board, over the board's own USB.
//!
//! **There are two of these, and the pair is the point.** `hello-uart` is
//! this same program reporting over UART0 through a Debug Probe. Diff them:
//!
//! ```bash
//! diff app/hello-world/hello-uart/main.zig app/hello-world/hello-usb/main.zig
//! ```
//!
//! Everything that differs is the transport and nothing else — same banner,
//! same tick line, same echo, same clock plan. **Which one you want is a
//! question about your bench, not about your firmware**, so it is two apps
//! rather than one with a `-D` flag: a flag would put the answer somewhere the
//! source does not show it, and the whole reason these exist is to be read.
//!
//! ## This one: USB CDC-ACM
//!
//! One cable, no probe. Flash it, plug the board into a computer, open the port
//! it creates — `/dev/ttyACM*` on Linux, `/dev/cu.usbmodem*` on macOS — and you
//! get a counter printing and an echo of anything you type. No driver to
//! install. Costs three things `hello-uart` does not pay:
//!
//!   **`clk_sys` must be above 48 MHz.** RP2350-E12 requires it for the USB
//!   controller, and picozig's default — no clock plan, straight off the 12 MHz
//!   crystal — is below it. So this file declares a 150 MHz plan. Try it at
//!   12 MHz and `clocks.initUsb` refuses to compile rather than failing on the
//!   bench.
//!
//!   **The `usbctrl` vector must be forwarded.** USB is serviced by its
//!   interrupt, and `boot.zig` builds the vector table from *this* module's
//!   declarations — so a library cannot claim it and the application does, in
//!   the `interrupts` namespace below. Forgetting is not silent: every
//!   unclaimed vector traps and records which one fired.
//!
//!   **The port vanishes when the board resets.** A reflash drops the USB
//!   device, so a terminal watching it disconnects and has to be restarted.
//!   `hello-uart` survives a reflash, because the probe owns that USB device
//!   and the board only drives pins into it. On a bench where the board is
//!   reflashed dozens of times an hour, that is the difference that decides.
//!
//! ## What it says, and what it refuses to say
//!
//! **Every field comes from the chip; none comes from a build flag** — and the
//! missing line is the deliberate part. `board.name` would say "Pimoroni Pico
//! LiPo 2" whenever `-Dboard` said so, whatever board it was actually
//! running on, stated with exactly the confidence of the facts around it. A
//! banner exists to end an argument about which board this is; one that can be
//! wrong about that joins the argument instead. So neither of these files
//! imports `board` — the guarantee is structural rather than a rule to
//! remember.
//!
//! The 64-bit device id is what distinguishes **two boards of the same kind**,
//! which is the case no amount of looking at the bench will settle. Telling a
//! Plus 2 W from a LiPo 2 is easy; telling this Plus 2 W from the other one is
//! not, and that is what gets a flaky behaviour pinned to a board.
//!
//! **The id rides every tick line, not just the banner.** Output produced
//! before a terminal attaches is discarded, so an identity printed once in
//! `main` would be gone by the time anyone connected to read it. Printing once
//! when the port opens fixes that and is still not enough: DTR is asserted by
//! whatever opens the port *first*, which on macOS is not necessarily the
//! terminal you are watching. Measured on the bench — the transition had
//! already happened 21 ticks before the terminal attached, and the block went to
//! whoever probed the port on enumeration.
//!
//! For what is happening underneath, flash `lib/usb/example/usb-trace` instead:
//! it is this same device, narrating every control transfer as it goes.

const std = @import("std");
const rp2350 = @import("rp2350");
const boot = @import("boot");
const usb = @import("usb");

const silicon = rp2350.silicon;

comptime {
    _ = &boot;
}

/// **Above 48 MHz because USB needs it** — see the header. `hello-uart` keeps
/// the same plan so that the two files differ only in their transport, but it
/// would work at the 12 MHz crystal.
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
/// `hello-uart` has no equivalent: its transport is polled, so nothing has to
/// be claimed.
pub const interrupts = struct {
    pub fn usbctrl() callconv(.c) void {
        port.onIrq();
    }
};

pub fn main() noreturn {
    rp2350.clocks.init(plan);
    rp2350.gpio.init();

    // The transport, and the only line in `main` that differs between the two.
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

        // And print a line a second, carrying the id. On every line rather
        // than once at startup: see the header.
        const now: u32 = @truncate(rp2350.timer.now64());
        if (now -% last > 1_000_000) {
            last = now;
            ticks += 1;
            out.print("tick {d}  chip {s}  stepping {s}  package {s}\n", .{
                ticks,                       silicon.idHex(), silicon.stepping().name(),
                // The enum tag, not `package().name()`: this line is read by a
                // person and parsed by a script, and "QFN-80
                // (RP2350B)" is two tokens and a bracket for no gain.
                @tagName(silicon.package()),
            }) catch {};
            out.flush() catch {};
        }
    }
}
