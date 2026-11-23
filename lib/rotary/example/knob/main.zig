//! knob — Rungs 1–3: one encoder and its push switch, three decoders at once.
//!
//! Wiring (breadboard, Pico 2 W):
//!
//!   encoder A      GP5   pin 7   through the RC filter below
//!   encoder common GND   pin 8
//!   encoder B      GP6   pin 9   through the RC filter below
//!   switch         GP9   pin 12, other side to GND pin 13
//!   Debug Probe    GP0 TX, GP1 RX, GND pin 3
//!
//! Per encoder channel, the datasheet's fig.4 at 3V3 rather than 5 V:
//!
//!   3V3 (pin 36) ──10k──┬── encoder A or B
//!                       └──10k──┬── GP5 / GP6
//!                              10 nF
//!                               └── GND
//!
//! **The internal pull-ups are off on GP5 and GP6, and must be.** With the
//! filter fitted, an internal pull-up and the 10 k series resistor form a
//! divider whenever the contact closes. At the internal pull-up's strongest
//! (32 k) that leaves the pin at 0.79 V against a VIL maximum of 0.8 V — a read
//! that works on the bench and fails on a warm day. The external 10 k already
//! sets the high level.
//!
//! ## Three decoders over the same two pins
//!
//! | name   | driven by                         | why it is here                     |
//! |--------|-----------------------------------|------------------------------------|
//! | `irq`  | GPIO edge interrupts (Rung 2)     | the thing under test               |
//! | `10k`  | a poll every 100 µs (Rung 1)      | the baseline that already passed   |
//! | `400`  | every 25th of those, 2.5 ms       | the control                        |
//!
//! **The control is what makes `illegal 0` on `irq` mean something.** A spin too
//! slow to trouble anything would also read zero, and prove nothing. The 400 Hz
//! decoder sees exactly the samples the 10 kHz one sees, decimated — the cadence
//! of an application loop that services a knob "on the way round" — so if a spin
//! was fast enough to matter, it shows there first.
//!
//! ## The interrupt side is `rotary.Encoder`
//!
//! This image claims the `io_bank0` vector and calls `knob.onIrq()` from it;
//! the library does the rest — clearing latched edges before reading the pins,
//! publishing through volatile reads, never read-and-clear. The polled
//! decoders beside it use `rotary.Decoder` directly, because they decimate one
//! shared sample and so must not each read the pins for themselves.
//!
//! The handler runs from XIP flash. That is fine here — a knob has no deadline
//! of microseconds — and would not be for a handler that did.
//!
//! ## Why it only speaks when nothing is moving
//!
//! **`uart.write` blocks until the FIFO drains**, so a log line holds the loop
//! for milliseconds. The interrupt decoder would not care, but the two polled
//! ones would miss samples, and the comparison would be of the logging. Reports
//! wait for 50 ms of stillness on every input.
//!
//! ## Reading it
//!
//!   rest A=1 B=1 | steps irq +1 10k +1 400 +1 | illegal irq 0 10k 0 400 0 | edges A 2 B 2
//!
//! Steps and edges are since the previous rest line; `illegal` is cumulative.
//!
//!     zig build swd

const std = @import("std");
const rp2350 = @import("rp2350");
const boot = @import("boot");
const rotary = @import("rotary");

const gpio = rp2350.gpio;
const nvic = rp2350.nvic;
const uart = rp2350.uart;
const timer = rp2350.timer;
const silicon = rp2350.silicon;

comptime {
    _ = &boot;
}

const plan = rp2350.clocks.Plan.require(150_000_000);

const tx_pin: rp2350.Pin = 0;
const rx_pin: rp2350.Pin = 1;
const pin_a: rp2350.Pin = 5;
const pin_b: rp2350.Pin = 6;
const pin_switch: rp2350.Pin = 9;

const sample_us: u64 = 100;
/// 25 × 100 µs = 2.5 ms: an application-loop cadence estimated to start
/// dropping steps at about 2 rev/s.
const slow_every: u32 = 25;
const quiet_us: u64 = 50_000;
const heartbeat_us: u64 = 5_000_000;

fn readCode() rotary.Code {
    return rotary.code(gpio.get(pin_a), gpio.get(pin_b));
}

fn bit(c: rotary.Code, which: u1) u1 {
    return @truncate(c >> which);
}

// ---------------------------------------------------------------------------
// Interrupt side
// ---------------------------------------------------------------------------

const Knob = rotary.Encoder(gpio);

/// Written by the handler, read by `main` — only through its accessors.
var knob: Knob = undefined;

pub const interrupts = struct {
    pub fn io_bank0() callconv(.c) void {
        knob.onIrq();
    }
};

// ---------------------------------------------------------------------------
// Polled side
// ---------------------------------------------------------------------------

pub fn main() noreturn {
    rp2350.clocks.init(plan);
    rp2350.gpio.init();
    uart.init(.uart0, .{ .tx = tx_pin, .rx = rx_pin, .clk_hz = plan.sys_hz });

    // No internal pull-up on the filtered channels. See the header.
    gpio.initInput(pin_a, .none);
    gpio.initInput(pin_b, .none);
    gpio.initInput(pin_switch, .up);
    timer.delayMs(20); // the 10 nF charges through 20 k: 200 µs per time constant

    var buf: [256]u8 = undefined;
    var out = uart.writerFor(.uart0, &buf);

    const start_code = readCode();
    const start_us = timer.now64();

    var fast = rotary.Decoder.init(.{}, start_code);
    var slow = rotary.Decoder.init(.{}, start_code);
    var sw = rotary.Contact.init(.{ .stable_ticks = 20 }, gpio.get(pin_switch), @truncate(start_us / 1000));

    knob = Knob.init(.{ .a = pin_a, .b = pin_b }, .{});
    knob.arm();
    nvic.enable(nvic.IrqNum.io_bank0);

    out.print(
        "\nknob  chip {s}  A=GP5 B=GP6 SW=GP9  decoders: irq, poll 10 kHz, poll 400 Hz\n" ++
            "start A={d} B={d}  switch {s}\n",
        .{ silicon.idHex(), bit(start_code, 0), bit(start_code, 1), if (sw.stable) "open" else "closed" },
    ) catch {};
    out.flush() catch {};

    var last_sample: u64 = start_us;
    var last_change: u64 = start_us;
    var last_heartbeat: u64 = start_us;
    var sample_n: u32 = 0;
    var last_code = start_code;
    var last_sw_raw = sw.stable;

    // Snapshots at the previous report, so lines show what happened since.
    var seen_irq_steps: i32 = 0;
    var seen_edges_a: u32 = 0;
    var seen_edges_b: u32 = 0;
    var watch_a: u32 = 0;
    var watch_b: u32 = 0;
    var fast_steps: i32 = 0;
    var slow_steps: i32 = 0;
    var sw_raw_changes: u32 = 0;
    var sw_edges: u32 = 0;
    var dirty = false;

    var total_fast: i32 = 0;
    var total_slow: i32 = 0;

    while (true) {
        const now = timer.now64();

        // Motion seen by the handler resets the quiet timer too — this is how
        // the polled side learns of edges it may itself have been too slow for.
        //
        // Compared against its own watch values, **not** the report snapshot:
        // that only moves when a line is printed, so testing against it would
        // restart the quiet timer on every pass after the first edge, and the
        // firmware would never report again.
        const moved = knob.transitions();
        const edges_a = moved.a;
        const edges_b = moved.b;
        if (edges_a != watch_a or edges_b != watch_b) {
            watch_a = edges_a;
            watch_b = edges_b;
            last_change = now;
            dirty = true;
        }

        if (now -% last_sample >= sample_us) {
            last_sample = now;
            sample_n +%= 1;

            const c = readCode();
            if (c != last_code) {
                last_code = c;
                last_change = now;
                dirty = true;
            }
            const fs = fast.update(c).delta();
            fast_steps += fs;
            total_fast += fs;
            if (sample_n % slow_every == 0) {
                const ss = slow.update(c).delta();
                slow_steps += ss;
                total_slow += ss;
            }

            const sw_raw = gpio.get(pin_switch);
            if (sw_raw != last_sw_raw) {
                last_sw_raw = sw_raw;
                sw_raw_changes += 1;
                last_change = now;
                dirty = true;
            }
            if (sw.update(sw_raw, @truncate(now / 1000)) != .none) sw_edges += 1;
        }

        if (now -% last_change < quiet_us) continue;

        if (dirty) {
            dirty = false;
            const irq_total = knob.steps();
            const c = readCode();
            out.print(
                "rest A={d} B={d} | steps irq {d:>3} 10k {d:>3} 400 {d:>3} | illegal irq {d} 10k {d} 400 {d} | edges A {d} B {d} | total irq {d} 10k {d} 400 {d}\n",
                .{
                    bit(c, 0),                   bit(c, 1),
                    irq_total -% seen_irq_steps, fast_steps,
                    slow_steps,                  knob.illegal(),
                    fast.illegal,                slow.illegal,
                    edges_a -% seen_edges_a,     edges_b -% seen_edges_b,
                    irq_total,                   total_fast,
                    total_slow,
                },
            ) catch {};
            if (sw_raw_changes > 0) {
                out.print("switch {s} | edges {d} raw {d}\n", .{
                    if (sw.stable) "open" else "closed", sw_edges, sw_raw_changes,
                }) catch {};
            }
            out.flush() catch {};

            seen_irq_steps = irq_total;
            seen_edges_a = edges_a;
            seen_edges_b = edges_b;
            fast_steps = 0;
            slow_steps = 0;
            sw_raw_changes = 0;
            sw_edges = 0;
            last_heartbeat = now;
        } else if (now -% last_heartbeat >= heartbeat_us) {
            last_heartbeat = now;
            out.print("alive chip {s}  total irq {d} 10k {d} 400 {d}  illegal irq {d} 10k {d} 400 {d}  tx-dropped {d}\n", .{
                silicon.idHex(), knob.steps(),       total_fast,
                total_slow,      knob.illegal(),     fast.illegal,
                slow.illegal,    uart.dropped_bytes,
            }) catch {};
            out.flush() catch {};
        }
    }
}
