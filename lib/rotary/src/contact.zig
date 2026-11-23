//! Debouncing a mechanical contact to a stable level.
//!
//! **The level is the mechanism; edges are derived from it.** A two-position
//! toggle has no press or release, only a position, and a momentary switch is
//! that same level with someone watching for changes. Building this the other
//! way round — press/release events, with the level reconstructed by the caller
//! — makes the toggle the awkward case, and a toggle is the commoner part.
//!
//! ## Why this one *is* timed, when the quadrature decoder is not
//!
//! A lone contact carries no ordering to exploit. The decoder can reject bounce
//! structurally because a rattle walks between two adjacent Gray codes and
//! never completes a cycle; there is no equivalent here, because there is
//! nothing for a bounce to be inconsistent *with*. So this waits for the level
//! to hold, and the wait is the whole algorithm.
//!
//! ## No clock of its own
//!
//! `update` takes the time rather than reading it, for the same reason the
//! decoder takes a code rather than a pin: the caller already has a timestamp,
//! knows whether it is monotonic, and can drive this from a test array. Ticks
//! are unsigned and compared with wrapping subtraction, so any width behaves
//! correctly across its own rollover.

const std = @import("std");

/// What `update` observed. `.none` is by far the common case — a debouncer is
/// mostly a thing that says nothing.
pub const Edge = enum { none, rising, falling };

pub const Config = struct {
    /// How long a level must hold before it is believed.
    ///
    /// The default is twice the 10 ms the EC12 sheet allows its push switch
    /// (§6-2), which is the right kind of margin for a part whose bounce is
    /// specified as a maximum rather than a distribution. **Zero is legal** and
    /// means "believe every sample" — useful for a debounced input, or a test.
    stable_ticks: u32 = 20,
};

/// One contact. Hold one per switch; they share nothing.
pub const Contact = struct {
    config: Config,

    /// The debounced level — **this is the output**, and a toggle needs
    /// nothing else.
    stable: bool,

    /// The level currently being timed. Equal to `stable` when settled.
    candidate: bool,

    /// When `candidate` last changed.
    since: u32,

    pub fn init(config: Config, level: bool, now: u32) Contact {
        return .{
            .config = config,
            .stable = level,
            .candidate = level,
            .since = now,
        };
    }

    /// Feed one raw sample. Returns the edge, if this sample settled one.
    pub fn update(self: *Contact, raw: bool, now: u32) Edge {
        if (raw != self.candidate) {
            // Any movement restarts the clock, which is what makes a burst of
            // chatter cost one settling time rather than one per transition.
            //
            // **Deliberately not an early return.** Falling through means a
            // `stable_ticks` of zero settles on the sample that showed the
            // change rather than the one after it, which is what "believe every
            // sample" has to mean if it is to mean anything.
            self.candidate = raw;
            self.since = now;
        }
        if (self.candidate == self.stable) return .none;
        if (now -% self.since < self.config.stable_ticks) return .none;

        self.stable = self.candidate;
        return if (self.stable) .rising else .falling;
    }

    /// Accept a level immediately, discarding anything in flight. For a
    /// power-on read, where there is nothing to debounce against yet.
    pub fn reset(self: *Contact, level: bool, now: u32) void {
        self.stable = level;
        self.candidate = level;
        self.since = now;
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "contact: a clean change settles after the hold time and not before" {
    var c = Contact.init(.{ .stable_ticks = 20 }, false, 0);

    try testing.expectEqual(Edge.none, c.update(true, 0));
    try testing.expectEqual(false, c.stable); // not yet believed

    try testing.expectEqual(Edge.none, c.update(true, 19));
    try testing.expectEqual(false, c.stable); // one tick short

    try testing.expectEqual(Edge.rising, c.update(true, 20));
    try testing.expectEqual(true, c.stable);
}

test "contact: the edge is reported exactly once" {
    var c = Contact.init(.{ .stable_ticks = 5 }, false, 0);
    _ = c.update(true, 0);
    try testing.expectEqual(Edge.rising, c.update(true, 5));
    try testing.expectEqual(Edge.none, c.update(true, 6));
    try testing.expectEqual(Edge.none, c.update(true, 1000));
}

test "contact: chatter costs one settling time, not one per transition" {
    var c = Contact.init(.{ .stable_ticks = 10 }, false, 0);

    // 8 ms of rattle, then it holds high.
    var t: u32 = 0;
    while (t < 8) : (t += 1) {
        try testing.expectEqual(Edge.none, c.update(t % 2 == 0, t));
    }
    try testing.expectEqual(false, c.stable);

    // Held from t = 8. Settles at 18, not at 8 + 10 per transition.
    try testing.expectEqual(Edge.none, c.update(true, 8));
    try testing.expectEqual(Edge.none, c.update(true, 17));
    try testing.expectEqual(Edge.rising, c.update(true, 18));
}

test "contact: a level that returns before settling produces nothing" {
    var c = Contact.init(.{ .stable_ticks = 20 }, false, 0);
    try testing.expectEqual(Edge.none, c.update(true, 0));
    try testing.expectEqual(Edge.none, c.update(false, 10)); // gave up
    try testing.expectEqual(Edge.none, c.update(false, 100));
    try testing.expectEqual(false, c.stable);
}

test "contact: falling is reported too" {
    var c = Contact.init(.{ .stable_ticks = 3 }, true, 0);
    _ = c.update(false, 0);
    try testing.expectEqual(Edge.falling, c.update(false, 3));
    try testing.expectEqual(false, c.stable);
}

test "contact: stable_ticks = 0 believes every sample" {
    var c = Contact.init(.{ .stable_ticks = 0 }, false, 0);
    try testing.expectEqual(Edge.rising, c.update(true, 0));
    try testing.expectEqual(Edge.falling, c.update(false, 0));
}

test "contact: the tick counter may wrap" {
    // Settling must not depend on `now` being larger than `since`, or one
    // switch press every 49 days hangs until the counter comes round again.
    const near_end: u32 = std.math.maxInt(u32) - 5;
    var c = Contact.init(.{ .stable_ticks = 20 }, false, near_end);

    try testing.expectEqual(Edge.none, c.update(true, near_end));
    try testing.expectEqual(Edge.none, c.update(true, near_end +% 19));
    try testing.expectEqual(Edge.rising, c.update(true, near_end +% 20));
}

test "contact: reset accepts a level with no settling" {
    var c = Contact.init(.{ .stable_ticks = 20 }, false, 0);
    _ = c.update(true, 0);
    c.reset(true, 50);
    try testing.expectEqual(true, c.stable);
    try testing.expectEqual(Edge.none, c.update(true, 50));
    try testing.expectEqual(Edge.none, c.update(true, 100));
}

test "contact: a toggle only ever reads the level" {
    // The shape the three toggle switches on an I/O board actually use: no
    // edge is consulted, and nothing about the API forces one to be.
    var c = Contact.init(.{}, false, 0);
    const raw = [_]bool{ true, false, true, true, true, true };
    for (raw, 0..) |r, i| _ = c.update(r, @intCast(i * 10));
    try testing.expectEqual(true, c.stable);
}
