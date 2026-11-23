//! Quadrature decoding — a switched contact pair in, detent steps out.
//!
//! **This file imports nothing but `std`.** No pin, no timer, no allocator, no
//! chip. Everything an encoder actually gets wrong — contact bounce, a reversal
//! part-way through a cycle, a sample missed at speed — is reachable from a
//! host test holding a literal array of codes, which is the only reason any of
//! it is trustworthy before there is hardware to try it on.
//!
//! ## The sign convention, and why it does not matter
//!
//! `forward` is the cycle `11 → 10 → 00 → 01 → 11`. Whether that is clockwise
//! depends on which contact is wired to which pin and which side of the panel
//! the knob is on, so it is not worth arguing about: `Config.invert` flips the
//! reported direction and the wiring is then never a reason to edit this file.
//!
//! ## Bounce is rejected structurally, not by a timer
//!
//! A bouncing contact rattles between two *adjacent* codes. Each rattle adds
//! and immediately subtracts a quarter-step, so the accumulator returns to
//! where it started and no step is ever emitted. That is the whole mechanism —
//! there is no debounce interval here, and there must not be one: this part's
//! chatter reaches 5 ms at end of life while its guaranteed phase separation
//! falls to 2.5 ms, so any mask wide enough to cover chatter on one contact is
//! wide enough to swallow a real edge on the other.

const std = @import("std");

/// A sampled contact pair: **bit 0 is A, bit 1 is B**, and a set bit is an
/// *open* contact — the idle state of a switch-to-common encoder read through a
/// pull-up. So the detent code of a part whose detents sit at both-open is
/// `0b11`, and `Config.detent_code` says so rather than this assuming it.
pub const Code = u2;

/// Assemble a `Code` from two pin reads, so a caller never has to remember
/// which bit is which.
pub fn code(a: bool, b: bool) Code {
    return (@as(Code, @intFromBool(b)) << 1) | @intFromBool(a);
}

/// What one `Decoder.update` produced.
pub const Step = enum(i2) {
    reverse = -1,
    none = 0,
    forward = 1,

    /// `-1`, `0` or `+1`, for a caller accumulating rather than switching.
    pub fn delta(self: Step) i2 {
        return @backingInt(self);
    }
};

/// One quarter-step of the Gray code, or `null` where **both bits changed**.
///
/// Both bits changing is not a direction, it is a missing sample, and it is
/// never guessed at — see `Decoder.illegal`.
pub fn quarterStep(prev: Code, cur: Code) ?i2 {
    return switch ((@as(u4, prev) << 2) | cur) {
        0b00_00, 0b01_01, 0b11_11, 0b10_10 => 0,
        0b11_10, 0b10_00, 0b00_01, 0b01_11 => 1,
        0b11_01, 0b01_00, 0b00_10, 0b10_11 => -1,
        else => null,
    };
}

/// What the knob is, rather than what this file assumes it is.
pub const Config = struct {
    /// Quarter-steps in one emitted step. Four for a part specified as
    /// "1 click = 1 pulse"; two for the half-detent variants; **one** to emit
    /// every quarter-step, which is what a detentless knob wants.
    quarters_per_detent: u8 = 4,

    /// The code the contacts rest in at a detent, or `null` for a knob with no
    /// detents. When set, it does two jobs: no step is emitted anywhere but at
    /// a rest position, and arriving there resets the accumulator, so a cycle
    /// that lost a sample cannot carry its error into the next one.
    detent_code: ?Code = 0b11,

    /// Report the opposite direction. Swapping the A and B wires does this in
    /// hardware; this exists so that doing it is not a reason to edit code.
    invert: bool = false,
};

/// The accumulator is bounded so that systematic under-sampling which never
/// lands on `detent_code` cannot wrap it. Legal input cannot reach this — a
/// four-step cycle visits every code, which `decode: a legal cycle visits every
/// code` proves — so it is a guard, not a mechanism.
const quarter_limit: i16 = 1024;

/// Decodes one encoder. Holds no pin and no clock: feed it codes from wherever
/// they come from — an interrupt, a poll, or a test array.
pub const Decoder = struct {
    config: Config,

    /// The last code seen.
    prev: Code,

    /// Quarter-steps banked since the last emitted step. Zeroed at a detent,
    /// which is what makes a detent a position fix rather than just a gate.
    quarters: i16,

    /// Transitions where both bits changed. **Not an error count** — it is the
    /// sample rate telling on itself, and the number Rung 2 exists to measure.
    /// It saturates rather than wrapping, so a long run cannot make a bad
    /// reading look like a good one.
    illegal: u32,

    pub fn init(config: Config, at_rest: Code) Decoder {
        std.debug.assert(config.quarters_per_detent >= 1);
        return .{ .config = config, .prev = at_rest, .quarters = 0, .illegal = 0 };
    }

    /// Resynchronise on a freshly read code, discarding any partial cycle.
    pub fn reset(self: *Decoder, at_rest: Code) void {
        self.prev = at_rest;
        self.quarters = 0;
    }

    /// Feed one sample. Returns at most one step — a single transition can
    /// never complete more than one cycle.
    pub fn update(self: *Decoder, cur: Code) Step {
        const q = quarterStep(self.prev, cur) orelse {
            self.illegal +|= 1;
            self.quarters = 0;
            self.prev = cur;
            return .none;
        };
        self.prev = cur;
        if (q == 0) return .none;

        self.quarters = std.math.clamp(
            self.quarters + @as(i16, q),
            -quarter_limit,
            quarter_limit,
        );

        const needed: i16 = self.config.quarters_per_detent;
        if (self.quarters > -needed and self.quarters < needed) return .none;
        if (self.config.detent_code) |detent| {
            if (cur != detent) return .none;
        }

        const forward = (self.quarters > 0) != self.config.invert;
        self.quarters = 0;
        return if (forward) .forward else .reverse;
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// The forward cycle away from a both-open detent, as fig.1 of the EC12 sheet
/// draws it: the four codes a knob passes through between one click and the
/// next, ending back at the detent.
const forward_cycle = [_]Code{ 0b10, 0b00, 0b01, 0b11 };
const reverse_cycle = [_]Code{ 0b01, 0b00, 0b10, 0b11 };

/// Feed a sequence and return the net step count.
fn run(dec: *Decoder, codes: []const Code) i32 {
    var net: i32 = 0;
    for (codes) |c| net += dec.update(c).delta();
    return net;
}

fn nextForward(c: Code) Code {
    return switch (c) {
        0b11 => 0b10,
        0b10 => 0b00,
        0b00 => 0b01,
        0b01 => 0b11,
    };
}

test "quarterStep: the table is antisymmetric" {
    for (0..4) |p| {
        for (0..4) |c| {
            const prev: Code = @intCast(p);
            const cur: Code = @intCast(c);
            if (quarterStep(prev, cur)) |q| {
                try testing.expectEqual(-q, quarterStep(cur, prev).?);
            } else {
                try testing.expect(quarterStep(cur, prev) == null);
            }
        }
    }
}

test "quarterStep: exactly the four both-bits-changed pairs are illegal" {
    var illegal: usize = 0;
    for (0..4) |p| {
        for (0..4) |c| {
            const prev: Code = @intCast(p);
            const cur: Code = @intCast(c);
            if (quarterStep(prev, cur) == null) {
                try testing.expectEqual(@as(u2, 0b11), prev ^ cur);
                illegal += 1;
            }
        }
    }
    try testing.expectEqual(@as(usize, 4), illegal);
}

test "decode: a legal cycle visits every code" {
    // This is what makes `quarter_limit` unreachable from legal input: motion
    // in one direction cannot avoid the detent code, so the accumulator is
    // always zeroed within four transitions.
    for (0..4) |s| {
        const start: Code = @intCast(s);
        var seen = @as([4]bool, @splat(false));
        var c = start;
        for (0..4) |_| {
            seen[c] = true;
            c = nextForward(c);
        }
        try testing.expectEqual(start, c);
        for (seen) |v| try testing.expect(v);
    }
}

test "decode: one forward cycle is one step, emitted at the detent" {
    var dec = Decoder.init(.{}, 0b11);

    // Nothing until the cycle closes.
    try testing.expectEqual(Step.none, dec.update(0b10));
    try testing.expectEqual(Step.none, dec.update(0b00));
    try testing.expectEqual(Step.none, dec.update(0b01));
    try testing.expectEqual(Step.forward, dec.update(0b11));

    try testing.expectEqual(@as(i16, 0), dec.quarters);
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: one reverse cycle is one reverse step" {
    var dec = Decoder.init(.{}, 0b11);
    try testing.expectEqual(@as(i32, -1), run(&dec, &reverse_cycle));
}

test "decode: a full revolution is 24 steps" {
    var dec = Decoder.init(.{}, 0b11);
    var net: i32 = 0;
    for (0..24) |_| net += run(&dec, &forward_cycle);
    try testing.expectEqual(@as(i32, 24), net);
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: bounce at the detent emits nothing" {
    // The contact that just closed rattles: 11-10-11-10-11. Each rattle banks a
    // quarter-step and gives it straight back.
    var dec = Decoder.init(.{}, 0b11);
    const chatter = [_]Code{ 0b10, 0b11, 0b10, 0b11, 0b10, 0b11 };
    try testing.expectEqual(@as(i32, 0), run(&dec, &chatter));
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: bounce part-way through a cycle emits nothing, and the cycle still closes" {
    var dec = Decoder.init(.{}, 0b11);
    const noisy = [_]Code{
        0b10, // a genuine quarter-step
        0b00, 0b10, 0b00, 0b10, 0b00, // the next contact chattering
        0b01,
        0b11, // and the cycle closes
    };
    try testing.expectEqual(@as(i32, 1), run(&dec, &noisy));
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: a half turn and back emits nothing" {
    var dec = Decoder.init(.{}, 0b11);
    const there_and_back = [_]Code{ 0b10, 0b00, 0b10, 0b11 };
    try testing.expectEqual(@as(i32, 0), run(&dec, &there_and_back));
}

test "decode: a reversal after a completed step is reported" {
    var dec = Decoder.init(.{}, 0b11);
    try testing.expectEqual(@as(i32, 1), run(&dec, &forward_cycle));
    try testing.expectEqual(@as(i32, -1), run(&dec, &reverse_cycle));
}

test "decode: invert flips the direction and nothing else" {
    var dec = Decoder.init(.{ .invert = true }, 0b11);
    try testing.expectEqual(@as(i32, -1), run(&dec, &forward_cycle));
    try testing.expectEqual(@as(i32, 1), run(&dec, &reverse_cycle));
}

test "decode: quarters_per_detent = 1 emits every quarter-step" {
    // A detentless knob: no rest position to gate on, so the gate is off too.
    var dec = Decoder.init(.{ .quarters_per_detent = 1, .detent_code = null }, 0b11);
    try testing.expectEqual(@as(i32, 4), run(&dec, &forward_cycle));
    try testing.expectEqual(@as(i32, -4), run(&dec, &reverse_cycle));
}

test "decode: quarters_per_detent = 2 is two steps per cycle" {
    var dec = Decoder.init(.{ .quarters_per_detent = 2, .detent_code = null }, 0b11);
    try testing.expectEqual(@as(i32, 2), run(&dec, &forward_cycle));
}

test "decode: both bits changing is counted and discards the partial cycle" {
    var dec = Decoder.init(.{}, 0b11);
    try testing.expectEqual(Step.none, dec.update(0b10)); // one good quarter-step
    try testing.expectEqual(@as(i16, 1), dec.quarters);

    try testing.expectEqual(Step.none, dec.update(0b01)); // both bits: impossible
    try testing.expectEqual(@as(u32, 1), dec.illegal);
    try testing.expectEqual(@as(i16, 0), dec.quarters);

    // ...and it resynchronises from there rather than staying wrong.
    try testing.expectEqual(@as(i32, 1), run(&dec, &.{ 0b11, 0b10, 0b00, 0b01, 0b11 }));
}

test "decode: a repeated sample is not motion" {
    var dec = Decoder.init(.{}, 0b11);
    try testing.expectEqual(@as(i32, 0), run(&dec, &.{ 0b11, 0b11, 0b11 }));
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: two missed samples read as motion the wrong way" {
    // **A known limitation, pinned deliberately.** One missed sample changes
    // both bits and is caught. Two in a row do not: the jump 11 -> 01 is a
    // perfectly legal reverse quarter-step, so under-sampling does not merely
    // lose steps, it can invent one in the opposite direction. The detent gate
    // bounds the damage to a single detent; nothing here can detect it.
    try testing.expectEqual(@as(i2, -1), quarterStep(0b11, 0b01).?);

    var dec = Decoder.init(.{}, 0b11);
    _ = dec.update(0b01); // sampled two quarter-steps late, travelling forward
    try testing.expectEqual(@as(i16, -1), dec.quarters);
    try testing.expectEqual(@as(u32, 0), dec.illegal);
}

test "decode: reset discards a partial cycle" {
    var dec = Decoder.init(.{}, 0b11);
    _ = dec.update(0b10);
    try testing.expectEqual(@as(i16, 1), dec.quarters);
    dec.reset(0b10);
    try testing.expectEqual(@as(i16, 0), dec.quarters);
    try testing.expectEqual(@as(i32, 0), run(&dec, &.{ 0b00, 0b01, 0b11 }));
}

test "decode: the illegal counter saturates rather than wrapping" {
    var dec = Decoder.init(.{}, 0b11);
    dec.illegal = std.math.maxInt(u32);
    _ = dec.update(0b00); // both bits from 0b11
    try testing.expectEqual(std.math.maxInt(u32), dec.illegal);
}

test "code: bit 0 is A and bit 1 is B" {
    try testing.expectEqual(@as(Code, 0b00), code(false, false));
    try testing.expectEqual(@as(Code, 0b01), code(true, false));
    try testing.expectEqual(@as(Code, 0b10), code(false, true));
    try testing.expectEqual(@as(Code, 0b11), code(true, true));
}

test "Step: delta is the accumulating form of the enum" {
    try testing.expectEqual(@as(i2, 1), Step.forward.delta());
    try testing.expectEqual(@as(i2, 0), Step.none.delta());
    try testing.expectEqual(@as(i2, -1), Step.reverse.delta());
}
