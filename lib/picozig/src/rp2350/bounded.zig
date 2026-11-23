//! Bounded spins — the house idiom for waiting on hardware.
//!
//! Every wait in this layer is a `while` around a register read, and **an
//! unbounded one turns any stall below it into a powered board doing nothing**:
//! no error code, no clue which wait it is sitting in, and nothing on the UART.
//! That failure has been paid for twice — a cyw43 bring-up hang on 2026-08-22
//! whose PC resolved only as far as `hw.read` inside `GspiBus.transfer`, and
//! `lib/pcm5100`'s buffer-swap wait, which presents a poisoned DMA as a silent
//! hang. In both cases the *cause* was unknown but the *defect* was certain:
//! the spin had no bound.
//!
//! So the rule for this layer: **a wait on hardware is bounded and reports.**
//!
//! ## Why polls and not microseconds
//!
//! Two reasons, and the first is decisive.
//!
//! **`timer.zig` is not available to most of these waits.** TIMER0 counts the
//! TICKS block's 1 us tick, which is disabled at reset and started by
//! `clocks.init()` — and `clocks.init` is itself built out of waits, on XOSC
//! stability, PLL lock and the TICKS generators. A microsecond deadline in
//! those would be a wait on a timer that the wait is trying to start. Reaching
//! for `timer.now()` there does not fail loudly; it hangs, which is the exact
//! failure being removed.
//!
//! **A poll count costs one increment and one compare.** Some of these spins
//! are the innermost loop of a bus running at 25-50 MHz with the hot paths
//! deliberately placed in RAM (`lib/cyw43`), so a second APB read per iteration
//! to consult a timer is not free. Counting is.
//!
//! The cost is that a limit is in polls, not time, so it scales with clock
//! speed and with how many register reads the predicate does. That is fine for
//! what this is for: the limits below are three to six orders of magnitude
//! above any legitimate wait, because **the job is to tell "wedged" from
//! "working", not "slow" from "fast".** A bound that tries to be a deadline
//! invites tuning; a bound that is obviously absurd never fires by accident.
//!
//! ## Using it
//!
//! ```zig
//! var s = bounded.Spin.init(bounded.fifo_polls);
//! while (rxEmpty(block, sm)) try s.tick();
//! return hw.read(block.base() + hw.Pio.rxf(sm));
//! ```
//!
//! `tick` is the loop body, not the condition, so the predicate stays readable
//! and is evaluated exactly as often as it was before.
//!
//! Named `bounded` rather than `wait` because `pio.zig` already exports a
//! `wait` — the PIO assembler's wait instruction — and one module unable to
//! import another by its obvious name is worse than a slightly duller name
//! everywhere.

const std = @import("std");

/// The only thing a bounded wait can add to a caller's error set.
///
/// Deliberately one error and not one per peripheral: a caller that wants to
/// know *which* wait gave up learns it from where the `try` is, and a shared
/// error means `bounded.Error || Foo.Error` stays small.
pub const Error = error{
    /// A hardware wait exceeded its poll limit. The condition it was waiting
    /// for is orders of magnitude overdue — this is a wedge, not a slow part.
    Timeout,
};

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------
//
// Three tiers, so a call site picks by the *shape* of what it is waiting for
// rather than by inventing a number. All three are far beyond any legitimate
// wait; see the header on why that is the point.

/// A FIFO slot, a status flag, a peripheral that answers within a few bus
/// cycles. At ~150 MHz an iteration is on the order of 100 ns, so this is
/// roughly a tenth of a second — against a legitimate wait of single-digit
/// polls.
pub const fifo_polls: u32 = 1_000_000;

/// A block starting up, or settling: XOSC stability, PLL lock, a reset
/// releasing, an ADC conversion. Slower and more variable than a FIFO, and
/// some of these run on a clock that is itself still being brought up.
pub const startup_polls: u32 = 10_000_000;

/// A whole transfer draining — a DMA channel finishing its count, a UART
/// emptying a FIFO at 115200 baud. Bounded by wire time rather than by the
/// peripheral, so it needs the most room.
pub const transfer_polls: u32 = 100_000_000;

// ---------------------------------------------------------------------------
// Spin
// ---------------------------------------------------------------------------

/// A poll counter with a limit. One increment and one compare per iteration.
///
/// Not a closure-taking helper on purpose: the predicates here read registers
/// through `hw`, several are `inline` in hot paths, and a `comptime fn`
/// parameter would either fight inlining or force every call site into the same
/// shape. A counter the loop ticks composes with any `while` that already
/// exists, which is what made this a mechanical pass over the layer rather than
/// a rewrite of it.
pub const Spin = struct {
    polls: u32 = 0,
    limit: u32,

    pub inline fn init(limit: u32) Spin {
        return .{ .limit = limit };
    }

    /// Count one poll; fail once the limit is passed.
    ///
    /// Call this **in the loop body**, so the predicate is evaluated exactly as
    /// often as it was before the bound was added:
    ///
    /// ```zig
    /// while (notReady()) try s.tick();
    /// ```
    pub inline fn tick(self: *Spin) Error!void {
        self.polls += 1;
        if (self.polls > self.limit) return error.Timeout;
    }

    /// How many polls this spin has taken. For diagnostics that want to report
    /// a wait that succeeded but only just — a margin is worth more than a
    /// pass/fail, and a wait that is drifting towards its limit is a fault
    /// arriving slowly.
    pub inline fn count(self: *const Spin) u32 {
        return self.polls;
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "a spin inside its limit never fails" {
    var s = Spin.init(4);
    var n: usize = 0;
    while (n < 4) : (n += 1) try s.tick();
    try testing.expectEqual(@as(u32, 4), s.count());
}

test "a spin fails one poll past its limit, not at it" {
    // The boundary matters: a limit of N must permit N polls, or every call
    // site is quietly one iteration short of what it asked for.
    var s = Spin.init(2);
    try s.tick();
    try s.tick();
    try testing.expectError(error.Timeout, s.tick());
}

test "a zero limit fails on the first poll" {
    // Degenerate but reachable — a caller computing a limit from a length can
    // land on zero, and it must fail rather than spin forever.
    var s = Spin.init(0);
    try testing.expectError(error.Timeout, s.tick());
}

test "the limit tiers are ordered, and none is zero" {
    // A tier accidentally set to zero would turn every wait using it into an
    // immediate failure — the opposite defect, and just as silent.
    try testing.expect(fifo_polls > 0);
    try testing.expect(startup_polls > fifo_polls);
    try testing.expect(transfer_polls > startup_polls);
}
