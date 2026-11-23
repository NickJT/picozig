//! dma-check — does the DMA block actually move bytes?
//!
//! ```
//! zig build swd-dma-check      # flash it
//! zig build watch-dma-check                      # read the block
//! ```
//!
//! DMA is the sort of block whose failures surface a long way from their cause:
//! a transfer that silently did not happen looks like whatever was supposed to
//! consume the data being broken. So it gets an example that checks it directly
//! rather than being trusted because something above it works.
//!
//! ## What it proves, in order
//!
//! Each check is a superset of the last, so the **first failure names the
//! fault** rather than leaving the whole block under suspicion:
//!
//!   1. **word copy** — the block is out of reset and a plain incrementing
//!      copy lands. If this fails nothing else means anything.
//!   2. **halfword copy** — `DATA_SIZE` is honoured and `count` is a count of
//!      *transfers*, not bytes. This is the check that catches the most likely
//!      arithmetic error in the layer, and it is deliberately given a source
//!      pattern where a wrong width is visible rather than plausible.
//!   3. **fixed-destination write** — `INCR_WRITE = 0`, which is what feeding a
//!      PIO FIFO needs. Every transfer lands on the same address, so the
//!      destination ends up holding the *last* source word.
//!   4. **no error flags** — a channel pointed somewhere invalid does not fault
//!      the CPU, it sets a flag and stops. Silence is not success.
//!
//! ## Why the result is a checksum and not a flag
//!
//! `copied_sum` is the sum of the destination buffer after check 1. A boolean
//! "it worked" can be produced by a comparison loop that itself is wrong; a sum
//! that matches an independently known value cannot, and it also distinguishes
//! "copied nothing" from "copied half" from "copied the wrong thing". The
//! expected value is computed by the same closed form the host test uses.
//!
//! **Never `-c halt` on this target.** It stops cm0 only, the following `resume`
//! fails with `[rp2350.cm1] not halted`, and the chip is left stopped. Memory
//! reads work on a running core.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const dma = rp2350.dma;
const gpio = rp2350.gpio;
const timer = rp2350.timer;

/// How long between heartbeats.
const period_ms: u32 = 1000;

/// Transfers per check. Small enough to stay clear of the framebuffer-sized
/// allocations this board will want later, large enough that an off-by-one in
/// the transfer count changes the checksum.
const n: u32 = 256;

/// Source pattern. `i * 7 + 1` rather than plain `i`, so that a transfer which
/// lands at the wrong offset produces a visibly wrong sum instead of one that
/// is merely slightly off.
fn pattern(i: u32) u32 {
    return i *% 7 +% 1;
}

/// What `copied_sum` must be if every word arrived. Computed the long way here
/// on purpose: a closed form would share its algebra with whatever produced the
/// data, and then both could be wrong together.
fn expectedSum() u32 {
    var sum: u32 = 0;
    var i: u32 = 0;
    while (i < n) : (i += 1) sum +%= pattern(i);
    return sum;
}

var src: [n]u32 = undefined;
var dst: [n]u32 = undefined;
var src16: [n]u16 = undefined;
var dst16: [n]u16 = undefined;
var fixed_dst: u32 = 0;

/// All-`u32`, so every field is one `mdw` word.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// Sum of `dst` after the word copy. Compare against `expected_sum`.
    copied_sum: u32,
    expected_sum: u32,
    /// 1 if every halfword arrived — the DATA_SIZE / transfer-count check.
    halfword_ok: u32,
    /// The value left in a fixed destination: the last source word.
    fixed_value: u32,
    fixed_expected: u32,
    /// 0 = clean, 1 = read error, 2 = write error, on any channel used.
    error_flag: u32,
    /// Channel number, plus one, of the first `dma.wait` that timed out; 0 if
    /// none did.
    ///
    /// **A channel that never reports BUSY-clear is a distinct failure from a
    /// channel that copies the wrong bytes**, and before the wait was bounded
    /// it was the one this example could not report — it simply stopped here.
    wait_timeout_ch: u32,
};

pub const watch_magic: u32 = 0x00d3_a11c;

export var watch: Watch linksection(".selftest") = undefined;

// ---------------------------------------------------------------------------

comptime {
    // File scope, not inside `main` — see clock-check for what happens
    // otherwise.
    _ = &boot;
}

pub fn main() noreturn {
    // `.selftest` is NOLOAD, so a fresh image inherits the previous one's
    // values field for field and they read as current data. Initialise all of
    // it before anything else runs.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .copied_sum = 0,
        .expected_sum = expectedSum(),
        .halfword_ok = 0,
        .fixed_value = 0,
        .fixed_expected = pattern(n - 1),
        .error_flag = 0,
        .wait_timeout_ch = 0,
    };

    clocks.init(board.clock_plan);
    dma.init();

    // --- 1. word copy -------------------------------------------------------
    for (&src, 0..) |*w, i| w.* = pattern(@intCast(i));
    @memset(&dst, 0);

    dma.start(0, .{ .size = .word }, @intFromPtr(&src), @intFromPtr(&dst), n);
    dma.wait(0) catch {
        if (watch.wait_timeout_ch == 0) watch.wait_timeout_ch = 1;
    };

    var sum: u32 = 0;
    for (dst) |w| sum +%= w;
    watch.copied_sum = sum;

    // --- 2. halfword copy ---------------------------------------------------
    // `count` is transfers, not bytes: n halfwords is n transfers and 2n bytes.
    for (&src16, 0..) |*h, i| h.* = @truncate(pattern(@intCast(i)));
    @memset(&dst16, 0);

    dma.start(1, .{ .size = .halfword }, @intFromPtr(&src16), @intFromPtr(&dst16), n);
    dma.wait(1) catch {
        if (watch.wait_timeout_ch == 0) watch.wait_timeout_ch = 2;
    };

    var halfword_ok: u32 = 1;
    for (dst16, 0..) |h, i| {
        if (h != @as(u16, @truncate(pattern(@intCast(i))))) halfword_ok = 0;
    }
    watch.halfword_ok = halfword_ok;

    // --- 3. fixed destination ----------------------------------------------
    // What feeding a peripheral FIFO looks like. Every transfer lands on the
    // same address, so what survives is the last source word.
    fixed_dst = 0;
    dma.start(
        2,
        .{ .size = .word, .incr_write = false },
        @intFromPtr(&src),
        @intFromPtr(&fixed_dst),
        n,
    );
    dma.wait(2) catch {
        if (watch.wait_timeout_ch == 0) watch.wait_timeout_ch = 3;
    };
    watch.fixed_value = fixed_dst;

    // --- 4. error flags -----------------------------------------------------
    // Checked last, across every channel used: a channel pointed at an invalid
    // address sets a flag and stops rather than faulting the CPU, so a quiet
    // failure above would otherwise read as success.
    for ([_]u32{ 0, 1, 2 }) |ch| {
        if (dma.errorFlag(ch)) |e| {
            watch.error_flag = switch (e) {
                .read => 1,
                .write => 2,
            };
        }
    }

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
