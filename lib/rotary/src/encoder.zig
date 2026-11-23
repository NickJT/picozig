//! One encoder on two GPIO pins: the decoder, driven by pin-change interrupts
//! or by polling, with its results published safely to the main loop.
//!
//! ```zig
//! const Knob = rotary.Encoder(rp2350.gpio);
//! var knob: Knob = undefined;
//!
//! pub const interrupts = struct {
//!     pub fn io_bank0() callconv(.c) void {
//!         knob.onIrq();
//!     }
//! };
//!
//! // in main, with GP5 and GP6 already configured as inputs:
//! knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
//! knob.arm();
//! rp2350.nvic.enable(rp2350.nvic.IrqNum.io_bank0);
//! ...
//! const moved = knob.steps() -% last; // never read-and-clear
//! ```
//!
//! ## Generic over a gpio namespace, not over a chip
//!
//! **This file imports nothing but `std` and `decode.zig`.** It is handed the
//! application's gpio module at comptime and uses six names from it: `Pin`,
//! `Trigger` (with `.edge_low` and `.edge_high`), `get`, `irqPending`,
//! `clearIrq` and `setIrqEnabled` — `lib/picozig`'s `rp2350.gpio`, spelled
//! exactly. Importing `rp2350` here instead would give this package a picozig
//! dependency, which would then have to be passed identical options by every
//! consumer or the build instantiates picozig twice. And the host tests hand it
//! a fake namespace, which is the only way the interrupt ordering below is
//! testable without a board.
//!
//! ## What the application still owns
//!
//! **The interrupt vector, and enabling its NVIC line.** `io_bank0` is one
//! vector for every GPIO on the chip, and `boot.zig` builds the table from the
//! application's root module, so a library cannot claim it and must not try.
//! With several encoders, the handler calls `onIrq` on each; each one looks
//! only at its own pins.
//!
//! **Pin configuration.** Call `gpio.initInput` before `init`. Which pull to ask
//! for depends on the board: `.up` for a bare encoder, **`.none` with the
//! datasheet's RC filter fitted** — an internal pull-up against its series
//! resistor leaves a closed contact within millivolts of the input threshold.
//!
//! **Which input path.** Interrupts with the filter, or a fast poll without
//! it: measured, those are the two clean combinations, and the other two are
//! not. Use one path per encoder, never both.

const std = @import("std");
const decode = @import("decode.zig");

/// How many times each pin has moved, since `init`. With interrupts these are
/// latched edges; when polling, changes seen between polls. Free-running.
///
/// **The diagnostic for a dead channel.** One pin not connected makes every
/// click decode to nothing, with no error — but its count stays still while
/// the other's climbs.
pub const Transitions = struct { a: u32, b: u32 };

pub fn Encoder(comptime gpio: type) type {
    return struct {
        const Self = @This();

        pub const Pins = struct { a: gpio.Pin, b: gpio.Pin };

        const triggers = [_]gpio.Trigger{ .edge_low, .edge_high };

        pins: Pins,

        /// Touched only by whichever context services the encoder — the
        /// interrupt handler, or the poll.
        decoder: decode.Decoder,

        // **Published. Read these through the accessors, never directly.**
        // They are written in interrupt context, and to the optimiser a plain
        // read in a loop that never writes them is loop-invariant: it hoists
        // the load, and the loop reads one value forever, with a green build.
        // The accessors read through a volatile pointer, which it may not do.
        //
        // All four are free-running and wrap. Nothing here is ever reset by
        // being read, so a reader never needs an atomic read-and-clear: it
        // subtracts its own previous reading instead.
        published_steps: i32 = 0,
        published_illegal: u32 = 0,
        published_a: u32 = 0,
        published_b: u32 = 0,

        /// Read the pins as they are now and start from there. The pins must
        /// already be inputs.
        pub fn init(pins: Pins, config: decode.Config) Self {
            return .{
                .pins = pins,
                .decoder = decode.Decoder.init(config, readPins(pins)),
            };
        }

        /// Enable both edges on both pins, discarding anything latched during
        /// setup. **Call before enabling the NVIC line**, and not from the
        /// handler: enabling is a read-modify-write on a register other pins
        /// share.
        pub fn arm(self: *Self) void {
            for ([_]gpio.Pin{ self.pins.a, self.pins.b }) |pin| {
                for (triggers) |t| {
                    gpio.clearIrq(pin, t);
                    gpio.setIrqEnabled(pin, t, true);
                }
            }
        }

        pub fn disarm(self: *Self) void {
            for ([_]gpio.Pin{ self.pins.a, self.pins.b }) |pin| {
                for (triggers) |t| gpio.setIrqEnabled(pin, t, false);
            }
        }

        /// Service a pin-change interrupt. Call from the `io_bank0` handler.
        ///
        /// **Clears the latched edges before reading the pins, never after.**
        /// An edge that lands after the read stays latched and brings the
        /// handler straight back, to read the new level. Clearing after the
        /// read would erase an edge that landed between the two, with nothing
        /// left to report it: the decoder would keep a stale code, and the next
        /// transition would arrive as a two-bit jump.
        pub fn onIrq(self: *Self) void {
            var a: u32 = 0;
            var b: u32 = 0;
            for (triggers) |t| {
                if (gpio.irqPending(self.pins.a, t)) {
                    gpio.clearIrq(self.pins.a, t);
                    a += 1;
                }
                if (gpio.irqPending(self.pins.b, t)) {
                    gpio.clearIrq(self.pins.b, t);
                    b += 1;
                }
            }
            // Not ours. Returning is safe: an edge on our pins after this
            // check is latched and brings the handler back.
            if (a == 0 and b == 0) return;

            self.published_a +%= a;
            self.published_b +%= b;
            self.apply(readPins(self.pins));
        }

        /// Service the encoder by polling. Call at a fixed, fast rate — 100 µs
        /// was clean on the bench without a filter; 2.5 ms was not.
        pub fn poll(self: *Self) void {
            const c = readPins(self.pins);
            const was = self.decoder.prev;
            if ((c ^ was) & 0b01 != 0) self.published_a +%= 1;
            if ((c ^ was) & 0b10 != 0) self.published_b +%= 1;
            self.apply(c);
        }

        fn apply(self: *Self, c: decode.Code) void {
            self.published_steps +%= self.decoder.update(c).delta();
            self.published_illegal = self.decoder.illegal;
        }

        /// Detent steps since `init`. Free-running; subtract a previous reading.
        pub fn steps(self: *const Self) i32 {
            const p: *const volatile i32 = &self.published_steps;
            return p.*;
        }

        /// Transitions where both pins had changed. See `decode.Decoder.illegal`.
        pub fn illegal(self: *const Self) u32 {
            const p: *const volatile u32 = &self.published_illegal;
            return p.*;
        }

        pub fn transitions(self: *const Self) Transitions {
            const a: *const volatile u32 = &self.published_a;
            const b: *const volatile u32 = &self.published_b;
            return .{ .a = a.*, .b = b.* };
        }

        fn readPins(pins: Pins) decode.Code {
            return decode.code(gpio.get(pins.a), gpio.get(pins.b));
        }
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// A gpio namespace with `rp2350.gpio`'s shape: levels, latched edges, and a
/// hook that can move a pin at the worst possible moment.
const FakeGpio = struct {
    pub const Pin = u6;
    pub const Trigger = enum(u2) { level_low, level_high, edge_low, edge_high };

    var level: [64]bool = @splat(true);
    var latched: [64][4]bool = @splat(@splat(false));
    var enabled: [64][4]bool = @splat(@splat(false));

    /// Applied once, immediately after the next `get` of `after_read_pin` —
    /// an edge landing just after the pins were read.
    var after_read_pin: ?Pin = null;
    var after_read_move: ?struct { pin: Pin, to: bool } = null;

    fn reset() void {
        level = @splat(true);
        latched = @splat(@splat(false));
        enabled = @splat(@splat(false));
        after_read_pin = null;
        after_read_move = null;
    }

    /// Move a pin, latching the edge as the hardware would.
    fn set(pin: Pin, to: bool) void {
        if (level[pin] == to) return;
        level[pin] = to;
        latched[pin][@backingInt(if (to) Trigger.edge_high else Trigger.edge_low)] = true;
    }

    pub fn get(pin: Pin) bool {
        const v = level[pin];
        if (after_read_pin == pin) {
            after_read_pin = null;
            if (after_read_move) |m| set(m.pin, m.to);
            after_read_move = null;
        }
        return v;
    }

    pub fn irqPending(pin: Pin, t: Trigger) bool {
        return latched[pin][@backingInt(t)] and enabled[pin][@backingInt(t)];
    }

    pub fn clearIrq(pin: Pin, t: Trigger) void {
        latched[pin][@backingInt(t)] = false;
    }

    pub fn setIrqEnabled(pin: Pin, t: Trigger, on: bool) void {
        enabled[pin][@backingInt(t)] = on;
    }

    /// Run the "handler" until nothing is pending on these pins, as the NVIC
    /// would by re-entering it.
    fn service(enc: anytype) void {
        var guard: usize = 0;
        while (guard < 16) : (guard += 1) {
            var any = false;
            for ([_]Pin{ enc.pins.a, enc.pins.b }) |p| {
                for (latched[p], enabled[p]) |l, e| any = any or (l and e);
            }
            if (!any) return;
            enc.onIrq();
        }
    }
};

const Knob = Encoder(FakeGpio);

/// One forward cycle from rest, as pin moves: A falls, B falls, A rises, B rises.
fn forwardClick(enc: *Knob) void {
    FakeGpio.set(enc.pins.a, false);
    FakeGpio.service(enc);
    FakeGpio.set(enc.pins.b, false);
    FakeGpio.service(enc);
    FakeGpio.set(enc.pins.a, true);
    FakeGpio.service(enc);
    FakeGpio.set(enc.pins.b, true);
    FakeGpio.service(enc);
}

test "a click through interrupts is one step and four edges" {
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();

    forwardClick(&knob);
    try testing.expectEqual(@as(i32, 1), knob.steps());
    try testing.expectEqual(Transitions{ .a = 2, .b = 2 }, knob.transitions());
    try testing.expectEqual(@as(u32, 0), knob.illegal());
}

test "an edge landing just after the read is not lost" {
    // The ordering rule, made testable. B falls in the instant after the
    // handler has read the pins for A's edge. Because the handler cleared its
    // latches *before* reading, B's edge is still latched and is serviced next.
    // Clearing after the read would erase it, leave the decoder a quarter-step
    // behind, and turn B's rise into a two-bit jump.
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();

    FakeGpio.set(5, false);
    FakeGpio.after_read_pin = 6; // readPins reads A, then B
    FakeGpio.after_read_move = .{ .pin = 6, .to = false };
    FakeGpio.service(&knob);

    FakeGpio.set(5, true);
    FakeGpio.service(&knob);
    FakeGpio.set(6, true);
    FakeGpio.service(&knob);

    try testing.expectEqual(@as(i32, 1), knob.steps());
    try testing.expectEqual(@as(u32, 0), knob.illegal());
}

test "stale latches from setup do not count" {
    FakeGpio.reset();
    FakeGpio.set(5, false); // chatter while the board powered up
    FakeGpio.set(5, true);
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();
    FakeGpio.service(&knob);
    try testing.expectEqual(Transitions{ .a = 0, .b = 0 }, knob.transitions());
}

test "an interrupt for someone else's pins changes nothing" {
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    var other = Knob.init(.{ .a = 10, .b = 11 }, .{});
    knob.arm();
    other.arm();

    FakeGpio.set(10, false); // the other knob moves
    knob.onIrq(); // shared vector: every encoder is asked
    try testing.expectEqual(Transitions{ .a = 0, .b = 0 }, knob.transitions());
    other.onIrq();
    try testing.expectEqual(Transitions{ .a = 1, .b = 0 }, other.transitions());
}

test "bounce through interrupts is counted as edges and rejected as motion" {
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();

    for (0..10) |_| {
        FakeGpio.set(5, false);
        FakeGpio.service(&knob);
        FakeGpio.set(5, true);
        FakeGpio.service(&knob);
    }
    try testing.expectEqual(@as(i32, 0), knob.steps());
    try testing.expectEqual(@as(u32, 20), knob.transitions().a);
}

test "two encoders on one vector keep their own counts" {
    FakeGpio.reset();
    var one = Knob.init(.{ .a = 5, .b = 6 }, .{});
    var two = Knob.init(.{ .a = 7, .b = 8 }, .{ .invert = true });
    one.arm();
    two.arm();

    forwardClick(&one);
    forwardClick(&two);
    forwardClick(&two);
    try testing.expectEqual(@as(i32, 1), one.steps());
    try testing.expectEqual(@as(i32, -2), two.steps());
}

test "polling decodes the same click and counts changes per pin" {
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    // No `arm`: a polled encoder never enables its interrupts.
    FakeGpio.set(5, false);
    knob.poll();
    FakeGpio.set(6, false);
    knob.poll();
    FakeGpio.set(5, true);
    knob.poll();
    FakeGpio.set(6, true);
    knob.poll();
    knob.poll(); // no change: nothing counted

    try testing.expectEqual(@as(i32, 1), knob.steps());
    try testing.expectEqual(Transitions{ .a = 2, .b = 2 }, knob.transitions());
}

test "a dead channel shows as one pin that never moves" {
    // Trap 7, as the counts see it: A clicks, B was never wired.
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();
    for (0..5) |_| {
        FakeGpio.set(5, false);
        FakeGpio.service(&knob);
        FakeGpio.set(5, true);
        FakeGpio.service(&knob);
    }
    try testing.expectEqual(@as(i32, 0), knob.steps());
    try testing.expectEqual(@as(u32, 0), knob.illegal());
    try testing.expectEqual(Transitions{ .a = 10, .b = 0 }, knob.transitions());
}

test "disarm stops interrupts from being seen" {
    FakeGpio.reset();
    var knob = Knob.init(.{ .a = 5, .b = 6 }, .{});
    knob.arm();
    knob.disarm();
    FakeGpio.set(5, false);
    knob.onIrq();
    try testing.expectEqual(Transitions{ .a = 0, .b = 0 }, knob.transitions());
}
