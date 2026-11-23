//! blink-hello — the smallest complete picozig application.
//!
//! Prints a banner over the debug UART, then blinks the onboard LED once a
//! second and prints an uptime line each time it does. **Start here.** Nothing
//! in this repo is simpler, and everything else begins with the same four calls.
//!
//! ## What it is for
//!
//! Proving the toolchain, the flashing route and the board all work, *before*
//! any of them can be blamed for a real bug. It deliberately reports on two
//! independent channels:
//!
//!   * the **LED**, which needs no cable and proves code is executing;
//!   * the **UART**, which needs the probe's serial cable and proves the clocks
//!     and a peripheral are configured correctly.
//!
//! A blinking LED with no UART output means the console cable or terminal is
//! wrong, not the board. UART output with a dark LED means the LED wiring or
//! pin is wrong, not the image. Either one alone would leave that ambiguous,
//! which on this chip is exactly the trap worth avoiding: essentially every
//! mistake here presents as "nothing happened".
//!
//! ## Build, flash, watch
//!
//! ```
//! zig build swd-blink-hello        # over a Debug Probe — no button dance
//! zig build flash-blink-hello      # UF2 fallback: hold BOOTSEL, replug, copy
//! minicom -D /dev/cu.usbmodem1102 -b115200
//! ```
//!
//! To catch the banner you must be listening *before* the target starts, since
//! it is printed once and never again. Open the terminal first, then flash.
//!
//! This file used to carry a warning that a running `minicom` makes the board
//! invisible over SWD. **It does not** — withdrawn 2026-08-14 after 76 attaches
//! and 5 flashes with `minicom` open, across both boards, with zero failures.
//! Leaving the terminal open while flashing is not just safe, it is the only
//! way to catch this banner.
//! ## Things to change
//!
//! Every one of these should visibly alter the output within one flash cycle:
//!
//!   * `blink_ms` below — the half-period.
//!   * the banner text.
//!   * `test_pin` — GP2, driven in lockstep with the LED so the same blink can
//!     be probed with a scope. Free on both boards; move it and the only thing
//!     to check is that the new pin is not already spoken for in `board.claims`.
//!   * the LED pin: `board.led` is GP25 on the Pico LiPo 2, or GP22 as a fallback
//!     on Pico 2 W (whose onboard LED is on the CYW43 chip). Driving a *different*
//!     pin means adding it to `board.claims` in `src/board.zig` first — the
//!     pin map is validated at compile time, so an undeclared pin is a build
//!     error rather than a dead wire.
//!
//! Adding a user switch:
//!   * On boards with a Bank 0 user switch (e.g. Pimoroni Pico LiPo 2 on GP45):
//!     `board.user_sw` is used. Connects to GND when pressed (active low).
//!   * On standard Pico / Pico 2 W boards: `board.user_sw` is `null`, so GP21 is used
//!     as the switch input (physical pin 27, right next to GND on pin 28).
//!   * Connecting the switch pin to GND increases the flash frequency to 5 Hz (100 ms/state).

const rp2350 = @import("rp2350");
const board = @import("board");
// Imported for its side effects only: `boot` exports the vector table and
// `_reset`, which nothing references by name. Without this import the linker
// has no reason to include the startup code and the image has no entry point.
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const silicon = rp2350.silicon;
const timer = rp2350.timer;
const uart = rp2350.uart;

/// How long the LED stays in each state in normal mode (1 Hz blink rate = 500 ms per state).
const blink_ms: u32 = 500;

/// How long the LED stays in each state in fast mode (5 Hz blink rate = 100 ms per state).
const fast_blink_ms: u32 = 100;

/// A scope/logic-analyser pin toggled in lockstep with the LED, so the blink is
/// visible on a probe as well as by eye. Free and unclaimed on both supported
/// boards, and driven push-pull from the same `gpio.toggle` call as the LED —
/// so its edges are the LED's edges, one instruction apart.
const test_pin: rp2350.Pin = 2;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rx_count: u32,
    last_rx: u32,
    uart_fr: u32,
};

pub const watch_magic: u32 = 0x62_6c_6e_6b; // "blnk"

export var watch: Watch linksection(".selftest") = undefined;

/// `boot.zig` calls this once the chip is up: `.data` copied, `.bss` zeroed,
/// stack set. It never returns — a bare-metal program has nowhere to return to.
pub fn main() noreturn {
    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .rx_count = 0,
        .last_rx = 0,
        .uart_fr = 0,
    };

    // Crystal first. Until `clocks.init()` runs, the chip is on the ROSC, whose
    // rate is a guess — so every delay would be approximate and the UART's baud
    // divisors, computed from `board.sys_hz`, would simply be wrong.
    clocks.init(board.clock_plan);

    // Releases IO_BANK0 and PADS_BANK0 from reset. Peripherals are held in
    // reset at power-on, and a register write to one that is still held reads
    // back as zero with no error at all.
    gpio.init();

    // UART0 on GP0/GP1 under function 2, 115200 8N1 — the wiring the Debug
    // Probe's serial cable expects. `board.uartConfig()` supplies the pins and
    // the clock so the chip layer never has to know which board it is on.
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== blink-hello ===\n", .{});
    uart.print(.uart0, "board      {s}\n", .{board.name});

    const pkg = silicon.package();
    uart.print(.uart0, "silicon    {s} stepping, {s}\n", .{
        silicon.stepping().name(),
        pkg.name(),
    });

    if (pkg != board.expected_package) {
        uart.print(.uart0, "WARNING:   package mismatch — image built for {s}, running on {s}\n", .{
            board.expected_package.name(),
            pkg.name(),
        });
        if (pkg == .qfn60 and board.kind == .pico_lipo2) {
            uart.print(.uart0, "           (rebuild with -Dboard=pico2w for Pico 2 W)\n", .{});
        }
    }

    uart.print(.uart0, "clk_sys    {d} Hz (crystal, no PLL)\n", .{board.sys_hz});
    uart.print(.uart0, "baud       {d}\n", .{uart.default_baud});

    const flash_pin: rp2350.Pin = board.led orelse 22;
    // Push-pull output, driven low first so the pin cannot glitch high on
    // the way up.
    gpio.initOutput(flash_pin);

    if (board.led) |pin| {
        uart.print(.uart0, "led        GP{d}, {d} ms per state\n", .{ pin, blink_ms });
    } else {
        uart.print(.uart0, "led        GP{d}, {d} ms per state (note: Pico 2 W onboard LED is on CYW43; toggling GP{d})\n", .{
            flash_pin,
            blink_ms,
            flash_pin,
        });
    }

    // Same treatment for the scope pin: low first, then output.
    gpio.initOutput(test_pin);
    uart.print(.uart0, "test pin   GP{d}, toggled with the LED\n", .{test_pin});

    const switch_pin: rp2350.Pin = board.user_sw orelse 21;
    // Input with pull-up. Active low — connected to GND when pressed or jumpered.
    gpio.initInput(switch_pin, .up);

    if (board.user_sw) |sw| {
        uart.print(.uart0, "switch     GP{d} (onboard user switch, active low -> 5 Hz fast flash)\n", .{sw});
    } else {
        uart.print(.uart0, "switch     GP{d} (active low -> 5 Hz fast flash; connect GP{d} to GND)\n", .{ switch_pin, switch_pin });
    }

    uart.print(.uart0, "\n", .{});

    var state_ms: u32 = 0;
    var uptime_s: u32 = 0;
    while (true) {
        watch.ticks +%= 1;
        gpio.toggle(flash_pin);
        gpio.toggle(test_pin);

        const pressed = !gpio.get(switch_pin);
        const active_delay = if (pressed) fast_blink_ms else blink_ms;

        var elapsed: u32 = 0;
        while (elapsed < active_delay) {
            const step_ms: u32 = @min(10, active_delay - elapsed);
            timer.delayMs(step_ms);
            elapsed += step_ms;

            while (uart.tryGetc(.uart0)) |c| {
                watch.rx_count +%= 1;
                watch.last_rx = c;
                watch.uart_fr = rp2350.hw.read(rp2350.hw.Uart.fr(rp2350.hw.uart0_base));

                uart.putc(.uart0, '[');
                if (c == '\r' or c == '\n') {
                    uart.putc(.uart0, '\\');
                    uart.putc(.uart0, 'n');
                    uart.putc(.uart0, '\r');
                    uart.putc(.uart0, '\n');
                } else if (c >= 32 and c <= 126) {
                    uart.putc(.uart0, c);
                } else {
                    const hex = "0123456789ABCDEF";
                    uart.putc(.uart0, '0');
                    uart.putc(.uart0, 'x');
                    uart.putc(.uart0, hex[c >> 4]);
                    uart.putc(.uart0, hex[c & 0xF]);
                }
                uart.putc(.uart0, ']');
            }
        }

        state_ms += active_delay;
        if (state_ms >= 1000) {
            state_ms -= 1000;
            uptime_s += 1;
            uart.print(.uart0, "up {d}s{s}\n", .{ uptime_s, if (pressed) " (5 Hz)" else "" });
        }
    }
}

comptime {
    // `boot` is imported for the linker's benefit, not this file's. Referencing
    // it keeps the compiler analysing it and stops anyone "tidying" the import
    // away — which would produce an image with no entry point and no diagnostic
    // beyond a board that does nothing.
    _ = &boot;
}
