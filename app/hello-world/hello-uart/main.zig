//! hello-uart — the first thing to run on a board, over a Debug Probe.
//!
//! **There are two of these, and the pair is the point.** `hello-usb` is
//! this same program reporting over the board's own USB. Diff them:
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
//! ## This one: UART0 through a Debug Probe
//!
//! Needs a probe and three wires, and gives three things back that
//! `hello-usb` cannot:
//!
//!   **It survives a reflash.** The USB device belongs to the probe, and the
//!   board only drives pins into it, so a terminal stays connected across a
//!   reset. `hello-usb`'s port is the board's own and disappears every time.
//!   On a bench where the board is reflashed dozens of times an hour, that is
//!   the difference that decides.
//!
//!   **It prints before `main` gets far.** There is no host to attach and no
//!   handshake to complete — the first byte goes out as soon as `uart.init`
//!   returns. USB drops everything written before a terminal opens the port,
//!   which is why the id below has to ride every line.
//!
//!   **It still works when USB is what is broken.** A board that enumerates as
//!   nothing at all can still be talked to here.
//!
//! Wire the probe to **GP0 (TX) and GP1 (RX), plus ground**. Those pins are
//! UART0's first function on every RP2350 board, which is why they are written
//! here as literals rather than read from `board.zig` — see below.
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

const silicon = rp2350.silicon;
const uart = rp2350.uart;

comptime {
    _ = &boot;
}

/// **Not required here** — a UART runs happily off the 12 MHz crystal, and only
/// `hello-usb` needs 48 MHz or more. Kept identical so that the two files
/// differ in their transport and nowhere else, and so that the baud divisors
/// are computed against the same clock in both.
const plan = rp2350.clocks.Plan.require(150_000_000);

/// UART0's first-function pins on every RP2350 board. **Literals rather than
/// `board.uart_tx`**: the board is chosen by `-Dboard`, and a wrong
/// flag would silently configure the wrong pads. Nothing in either of these
/// two apps may depend on a build flag — see the header.
const tx_pin: rp2350.Pin = 0;
const rx_pin: rp2350.Pin = 1;

// **No `interrupts` namespace here, and that is the largest single difference
// between these two files.** `hello-usb` must claim the `usbctrl` vector
// because USB is serviced by its interrupt; a UART at this rate is polled, so
// there is nothing to claim and nothing to forget to claim.

/// Drain whatever the receiver holds, without waiting for any of it.
///
/// **The one thing the UART driver has no direct equivalent for.**
/// `usb.SerialPort` buffers in a ring and hands out a slice; a PL011 holds a
/// few bytes in a FIFO and offers them one at a time. `uart.getc` would block,
/// which would stop the tick line, so this takes only what is already there.
fn readAvailable(dest: []u8) []u8 {
    var n: usize = 0;
    while (n < dest.len) : (n += 1) {
        dest[n] = uart.tryGetc(.uart0) orelse break;
    }
    return dest[0..n];
}

pub fn main() noreturn {
    rp2350.clocks.init(plan);
    rp2350.gpio.init();

    // The transport, and the only line in `main` that differs between the two.
    uart.init(.uart0, .{ .tx = tx_pin, .rx = rx_pin, .clk_hz = plan.sys_hz });

    var buf: [256]u8 = undefined;
    var out = uart.writerFor(.uart0, &buf);

    var ticks: u32 = 0;
    var last: u32 = 0;
    var seen: [64]u8 = undefined;

    while (true) {
        // Nothing to service: a polled UART has no interrupt to run behind it.
        // `hello-usb` says the same thing for the opposite reason.

        // Echo anything typed at the port.
        const got = readAvailable(&seen);
        if (got.len > 0) uart.write(.uart0, got);

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
