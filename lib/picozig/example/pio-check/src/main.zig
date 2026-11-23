//! pio-check — does a PIO state machine fetch, execute and move data?
//!
//! ```
//! zig build swd-pio-check      # flash it
//! zig build watch-pio-check                      # read the block
//! ```
//!
//! ## Why it proves itself through the FIFOs rather than a pin
//!
//! The obvious PIO bring-up toggles a pin and you watch it, which needs a free
//! pin and something to watch it with — neither guaranteed, and on a board
//! whose every GPIO is committed, neither available.
//!
//! Driving the FIFOs is the better test anyway. It proves more: a program that
//! reads its TX FIFO, transforms a value and pushes the result to RX has
//! executed real instructions, moved data both ways across the FIFO boundary
//! and kept its state — where a blinking pin proves only that *something* runs.
//! And it exercises the path that matters, since anything driving a peripheral
//! from PIO is DMA feeding a TX FIFO; this runs it in the direction where the
//! answer can be checked.
//!
//! ## The program
//!
//! Four instructions, and the transform is deliberately not the identity:
//!
//! ```
//!   pull block      ; take a word from the TX FIFO into the OSR
//!   out x, 32       ; move it to X
//!   mov isr, ::x    ; bit-REVERSE it into the ISR
//!   push block      ; hand it back through the RX FIFO
//! ```
//!
//! **`mov ::x` is the point.** A loopback that returned its input unchanged
//! would pass just as well if the state machine never ran and the FIFOs were
//! merely wired together — which is not how the hardware works, but is exactly
//! the kind of assumption a bring-up test should not rest on. A bit-reversal is
//! something only the instruction can have done.
//!
//! The check runs three values chosen so a wrong answer is unmistakable rather
//! than plausible: a lone low bit, a lone high bit, and an asymmetric pattern.
//!
//! **Never `-c halt` on this target.** It stops cm0 only, the following `resume`
//! fails with `[rp2350.cm1] not halted`, and the chip is left stopped. Memory
//! reads work on a running core.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const pio = rp2350.pio;
const timer = rp2350.timer;

const period_ms: u32 = 1000;

/// PIO0. Nothing else in this package uses PIO, so any of the three would do.
const block: pio.Block = .pio0;
const sm: u32 = 0;

/// The program, assembled at comptime. No side-set, so the delay field is a
/// plain delay throughout and every instruction here uses 0.
const program = [_]u16{
    pio.pull(false, true, 0), // pull block
    pio.out(.x, 32, 0), // out x, 32
    pio.mov(.isr, .reverse, .x, 0), // mov isr, ::x
    pio.push(false, true, 0), // push block
};

/// Reverse the bits of a word — the answer the state machine must produce,
/// computed here by a completely different route so agreement means something.
fn reverseBits(v: u32) u32 {
    var out: u32 = 0;
    var i: u5 = 0;
    while (true) {
        if (v & (@as(u32, 1) << i) != 0) out |= @as(u32, 1) << (31 - i);
        if (i == 31) break;
        i += 1;
    }
    return out;
}

const probes = [_]u32{ 0x0000_0001, 0x8000_0000, 0x1234_5678 };

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// 1 if all three probes came back bit-reversed as expected.
    all_ok: u32,
    /// The three answers, so a partial failure shows *how* it failed rather
    /// than only that it did.
    got0: u32,
    got1: u32,
    got2: u32,
    /// What `got2` should be. The other two are their own check by inspection:
    /// 0x00000001 must return 0x80000000 and vice versa.
    want2: u32,
    /// 1 if the state machine never answered at all — distinguishes "PIO is
    /// dead" from "PIO ran and got it wrong", which look identical in `all_ok`.
    timed_out: u32,
};

pub const watch_magic: u32 = 0x00b1_0000;

export var watch: Watch linksection(".selftest") = undefined;

// ---------------------------------------------------------------------------

comptime {
    _ = &boot;
}

/// Read a reply, giving up rather than spinning forever.
///
/// **A blocking read would be the wrong call in bring-up code.** If the state
/// machine never runs, `pio.read` never returns, and the symptom is firmware
/// that appears to hang — indistinguishable from a crash, and with the
/// self-test block never written. A bounded wait turns that into a reported
/// fact.
fn readWithTimeout() ?u32 {
    var spins: u32 = 0;
    while (spins < 1_000_000) : (spins += 1) {
        if (pio.readNonBlocking(block, sm)) |v| return v;
    }
    return null;
}

pub fn main() noreturn {
    watch = .{
        .magic = 0,
        .ticks = 0,
        .all_ok = 0,
        .got0 = 0,
        .got1 = 0,
        .got2 = 0,
        .want2 = reverseBits(probes[2]),
        .timed_out = 0,
    };

    clocks.init(board.clock_plan);

    pio.init(block);
    // GPIO 0-31 from this block's point of view. Nothing here drives a pin, but
    // leaving it unset on a board that later wants GP32-47 is a silent failure
    // waiting to happen, so it is stated.
    pio.setGpioBase(block, 0);

    pio.loadProgram(block, 0, &program);
    pio.configure(block, sm, .{
        .wrap_bottom = 0,
        .wrap_top = @intCast(program.len - 1),
        // Shift direction is irrelevant to a 32-bit whole-word move, but the
        // hardware default is right and saying so keeps it from looking
        // accidental.
        .out_shiftdir = .right,
        .in_shiftdir = .right,
    });
    pio.jumpTo(block, sm, 0);
    pio.setEnabled(block, sm, true);

    var results: [probes.len]u32 = .{ 0, 0, 0 };
    var all_ok: u32 = 1;
    for (probes, 0..) |p, i| {
        // The write can time out for the same reason the read can — a machine
        // that never runs — and this example exists to say which.
        pio.write(block, sm, p) catch {
            watch.timed_out = 1;
            all_ok = 0;
            break;
        };
        if (readWithTimeout()) |got| {
            results[i] = got;
            if (got != reverseBits(p)) all_ok = 0;
        } else {
            watch.timed_out = 1;
            all_ok = 0;
        }
    }

    watch.got0 = results[0];
    watch.got1 = results[1];
    watch.got2 = results[2];
    watch.all_ok = all_ok;
    watch.magic = watch_magic;

    const sign_of_life: ?rp2350.Pin = board.led;
    if (sign_of_life) |p| gpio.initOutput(p);

    var on = false;
    while (true) {
        timer.delayMs(period_ms);
        on = !on;
        if (sign_of_life) |p| {
            if (on) gpio.set(p) else gpio.clear(p);
        }
        watch.ticks +%= 1;
    }
}
