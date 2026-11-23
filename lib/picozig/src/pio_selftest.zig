//! Five rungs between "a state machine runs" and "a scan-out runs".
//!
//! ## Why this is a module and not an example
//!
//! It started as `example/pio-tick`, which passed on a Pico LiPo 2 and then
//! could not be built for the board it existed to bisect — a display board
//! that is deliberately not a `picozig` board, and the example imported `board`.
//! The logic is worth testing, so it lives in a library rather than in a `main`,
//! and the two `pio-tick` examples are thin shells
//! that bring their own clock plan and their own `.selftest` block.
//!
//! **It depends on `rp2350` and nothing else.** No board, no pins, no LED, no
//! allocator. Every answer is a register read, so it runs identically on a
//! board with an LED and on one whose every GPIO is committed.
//!
//! ## What the rungs are for
//!
//! `pio-check` proves the simplest possible state machine: an explicit
//! `pull`/`push` handshake, no divisor, no side-set, a four-deep FIFO.
//! A display panel's timing machine adds a clock divider, a side-set
//! generating a clock, autopull with a left shift, both FIFOs joined, `out
//! exec`, and three muxed pins — **all at once**. When it did not run there was
//! nothing in between to bisect against, and a fortnight went into reasoning
//! about the top of the ladder.
//!
//! | Stage | Adds | Answers |
//! |---|---|---|
//! | A | nothing — a two-instruction loop | does a state machine tick at all? |
//! | B | a clock divider and a side-set | does it still tick when clocked down? |
//! | C | autopull, left shift, joined TX FIFO | does it consume a joined FIFO? |
//! | D | `out exec` | does it run an instruction from the FIFO? |
//! | E | a stopwatch | does it run at the speed we think? |
//!
//! **E was added after the others, and it is the one that would have saved a
//! day.** C and D ask "did it finish before I looked", which a debugger cannot
//! answer honestly — an SWD round-trip is milliseconds and a state machine is
//! nanoseconds, so "drained" and "drained ten thousand times too slowly" are
//! the same observation. E times a known number of instructions against TIMER0
//! and reports the number, so a throughput error shows up as a number rather
//! than as a hunch.

const std = @import("std");
const rp2350 = @import("rp2350");

const hw = rp2350.hw;
const pio = rp2350.pio;
const timer = rp2350.timer;

// ---------------------------------------------------------------------------
// Knobs
// ---------------------------------------------------------------------------

/// How many times to sample the program counter in stages A and B.
///
/// Generous, because the sampling is aliased against the state machine's own
/// clock: any single pair of reads can legitimately return the same address
/// twice on a perfectly healthy machine. It is the *union* over many samples
/// that carries the answer.
const pc_samples: u32 = 512;

/// Long enough that a working machine has finished many times over, short
/// enough that every stage failing still reports inside a second.
const drain_timeout_us: u64 = 50_000;

/// The divisor stage E times at, and it is deliberately enormous.
///
/// Eight words at the panel's divisor of 6 drain in **240 ns** at 200 MHz,
/// which TIMER0's microsecond tick cannot resolve at all — so a stage that
/// timed the real divisor would be measuring its own quantisation. At 10,000
/// the same eight words take about 400 us at 200 MHz and 6.7 ms at 12 MHz.
/// Both are comfortably measurable, and both are far enough from zero that an
/// order-of-magnitude error is unmissable rather than arguable.
pub const timed_divisor: u32 = 10_000;

/// Words pushed through in stage E — the full depth of a joined TX FIFO.
const timed_words: u32 = 8;

/// Divisors the sweep tries, one bit of `Result.div_ticks` each.
///
/// **The sweep exists because stage B changed two things at once.** It adds a
/// divisor *and* a side-set, and on the display board it fails while stage A passes —
/// which does not say which of the two is responsible. This isolates the
/// divisor: same two-instruction program as stage A, no side-set, nothing but
/// the clock changing.
///
/// The values bracket the panel's 6 closely on both sides, because if there is
/// a threshold it matters exactly where it is.
pub const sweep_divisors = [_]u32{ 1, 2, 3, 4, 6, 8, 16, 64 };

// ---------------------------------------------------------------------------
// Programs
// ---------------------------------------------------------------------------

/// Stages A and B: a two-instruction loop that does nothing but advance.
///
/// Two instructions rather than one because a single `jmp 0` leaves the program
/// counter at 0 forever, and a program counter that never changes is exactly
/// what a dead machine looks like. **The test has to be able to fail.**
const loop_program = [_]u16{
    pio.nop(0),
    pio.jmp(.always, 0, 0),
};

/// Stage B, with a side-set bit on every instruction. Same two instructions;
/// the difference is entirely in the delay/side-set field and in `SIDESET_COUNT`
/// being 1 — the pairing this chip makes so easy to get wrong.
const loop_program_sideset = [_]u16{
    pio.nop(pio.sideSet(1, 0, 0)),
    pio.jmp(.always, 0, pio.sideSet(1, 1, 0)),
};

/// Stages C and E: consume the FIFO as fast as it arrives, and do nothing with
/// it. `out x, 32` under autopull at a threshold of 32 takes exactly one word
/// per instruction. X is written and never read — the destination is
/// irrelevant, the *consumption* is the test.
const drain_program = [_]u16{
    pio.out(.x, 32, 0),
};

/// Stage D: execute whatever the FIFO says. Two `out exec, 16` per word.
const exec_program = [_]u16{
    pio.out(.exec, 16, 0),
};

/// The word stage D feeds. `out_shiftdir` is **left**, so the *top* halfword is
/// consumed first: `irq 1`, then a nop. Both are encoded for a machine with no
/// side-set, which is what stage D configures — the same instruction on a
/// side-setting machine would mean something else entirely.
const exec_word: u32 = (@as(u32, pio.irqSet(1, false, 0)) << 16) | pio.nop(0);

/// Which `irq` flag stage D watches for.
const irq_index: u5 = 1;

// ---------------------------------------------------------------------------

pub const Config = struct {
    /// Which PIO block to use. Nothing here depends on the choice; matching the
    /// block the real application uses removes one difference for free.
    block: pio.Block,
    /// The state machine under test.
    sm: u32 = 0,
    /// A state machine that is loaded but never enabled. `SMx_INSTR` reads the
    /// instruction at the program counter for an idle machine, which makes this
    /// the only way to confirm `loadProgram` wrote what it meant to —
    /// `INSTR_MEM` is write-only and reads back as zero.
    control_sm: u32 = 3,
    /// clk_sys in Hz. Stage E's expected duration divides by it, so it must be
    /// the *measured* value rather than the requested one where they can differ.
    sys_hz: u32,
    /// The divisor stages B, C and D run at. Defaults to the display panel's.
    divisor: u32 = 6,
};

/// Seventeen words, laid out for `mdw`. Every field is a u32 so the block can be
/// read without symbols and without GDB.
pub const Result = extern struct {
    /// 1 only if every stage passed. The per-stage words say which did not.
    all_ok: u32,

    /// Stage A: a bitmask of program counter values seen. **Both bits 0 and 1
    /// must be set.** `0x1` alone means the machine is parked at the top of the
    /// program; `0x0` means `SMx_ADDR` never read as an address in the program
    /// at all, which is a different and worse failure.
    pc_seen_a: u32,
    /// Stage B, at `Config.divisor` with a side-set. Same expectation.
    pc_seen_b: u32,
    /// The divisor sweep: **bit i set means `sweep_divisors[i]` produced a
    /// moving program counter**, with the same program stage A uses and no
    /// side-set anywhere. `0xff` is a pass.
    ///
    /// This is the field that separates "the clock divider" from "the side-set"
    /// when stage A passes and stage B does not — which is exactly what the
    /// display board does, and what a Pico LiPo 2 does not.
    div_ticks: u32,

    /// Stage C: the FIFO level with the queue primed, read **before** the
    /// machine is enabled. **8 proves `FifoJoin.tx` took effect**; 4 means the
    /// join was configured and did not, which no other read reports.
    tx_level_c: u32,
    /// 1 if the FIFO drained to empty within the timeout.
    tx_drained_c: u32,
    /// 1 if TXSTALL ever set — the machine asked for a word and there was none.
    /// **On a drained FIFO this is the confirmation, not a failure**: it means
    /// the machine outran us, which is what consuming looks like. Drained
    /// *without* a stall would mean something else emptied it.
    tx_stalled_c: u32,

    /// Stage D: 1 if `irq 1` was observed — the state machine executed an
    /// instruction that was in the data stream and not in instruction memory.
    /// **The mechanism no host test can reach.**
    irq_seen_d: u32,
    /// 1 if that stage's FIFO also drained.
    tx_drained_d: u32,

    /// `EXECCTRL.EXEC_STALLED` at the end of each stage. Expected 0 everywhere.
    /// Read alongside `pc_seen_*`: set with a moving program counter is an
    /// `out exec` executee blocking, set with a parked one is a forced
    /// instruction that will not retire, and the flag alone cannot tell them
    /// apart. See `rp2350.hw.Pio.execctrl_exec_stalled`.
    stalled_a: u32,
    stalled_b: u32,
    stalled_c: u32,
    stalled_d: u32,

    /// `SMx_INSTR` read from a state machine that was loaded and never enabled.
    /// Must equal `control_want`. If instruction memory did not take the write,
    /// every other result here describes a program that is not there.
    control_instr: u32,
    control_want: u32,

    /// Stage E: microseconds for `timed_words` to leave the FIFO at
    /// `timed_divisor`, by TIMER0.
    drain_us: u32,
    /// What that should be, computed on-chip from `Config.sys_hz`, so the block
    /// carries its own expectation rather than needing arithmetic done beside it.
    drain_us_expected: u32,
    /// 1 if measured and expected agree within a factor of three.
    ///
    /// **Deliberately a wide band.** The exact instruction count around the
    /// initial autopull is not worth pinning down, and the failure this exists
    /// to catch is not 20% — it is the four orders of magnitude that a
    /// debugger-driven experiment implied on the display board and could not confirm.
    rate_ok: u32,
};

// ---------------------------------------------------------------------------

/// Run all five stages. Resets the block between each, so a stage cannot pass
/// on state its predecessor left behind.
pub fn run(cfg: Config) Result {
    var r = std.mem.zeroes(Result);
    r.control_want = exec_program[0];

    stageA(cfg, &r);
    sweep(cfg, &r);
    stageB(cfg, &r);
    stageC(cfg, &r);
    stageD(cfg, &r);
    stageE(cfg, &r);

    const both_pcs: u32 = 0b11;
    const all_divisors: u32 = (@as(u32, 1) << sweep_divisors.len) - 1;
    r.all_ok = @intFromBool(r.pc_seen_a == both_pcs and
        r.pc_seen_b == both_pcs and
        r.div_ticks == all_divisors and
        r.tx_level_c == 8 and
        r.tx_drained_c == 1 and
        r.tx_stalled_c == 1 and
        r.tx_drained_d == 1 and
        r.irq_seen_d == 1 and
        r.control_instr == r.control_want and
        r.rate_ok == 1 and
        r.stalled_a == 0 and r.stalled_b == 0 and
        r.stalled_c == 0 and r.stalled_d == 0);
    return r;
}

/// A: does a state machine tick at all?
fn stageA(cfg: Config, r: *Result) void {
    prepareBlock(cfg, &loop_program);
    pio.configure(cfg.block, cfg.sm, .{
        .wrap_bottom = 0,
        .wrap_top = @intCast(loop_program.len - 1),
    });
    pio.prepare(cfg.block, cfg.sm, 0);
    pio.setEnabled(cfg.block, cfg.sm, true);

    r.pc_seen_a = pcSeen(cfg.block, cfg.sm);
    r.stalled_a = @intFromBool(pio.execStalled(cfg.block, cfg.sm));
}

/// The divisor sweep: stage A's exact program and configuration, run once per
/// divisor, with nothing else varying.
///
/// **One variable at a time, which stage B does not manage.** Stage B is kept
/// as it is because a side-set is genuinely part of what the panel needs, but a
/// stage that changes two things cannot attribute its own failure.
fn sweep(cfg: Config, r: *Result) void {
    const both_pcs: u32 = 0b11;
    for (sweep_divisors, 0..) |divisor, i| {
        prepareBlock(cfg, &loop_program);
        pio.configure(cfg.block, cfg.sm, .{
            .wrap_bottom = 0,
            .wrap_top = @intCast(loop_program.len - 1),
            .clkdiv = pio.clkdivInt(divisor),
        });
        pio.prepare(cfg.block, cfg.sm, 0);
        pio.setEnabled(cfg.block, cfg.sm, true);
        if (pcSeen(cfg.block, cfg.sm) == both_pcs) {
            r.div_ticks |= @as(u32, 1) << @truncate(i);
        }
    }
}

/// B: and does it still tick with a divisor and a side-set?
///
/// No pad is muxed to the side-set pin. A state machine drives its side-set
/// whether or not a pad is listening, and the question here is whether the
/// machine runs — not whether a pin wiggles.
fn stageB(cfg: Config, r: *Result) void {
    prepareBlock(cfg, &loop_program_sideset);
    pio.configure(cfg.block, cfg.sm, .{
        .wrap_bottom = 0,
        .wrap_top = @intCast(loop_program_sideset.len - 1),
        .sideset_base = 0,
        .sideset_count = 1,
        .clkdiv = pio.clkdivInt(cfg.divisor),
    });
    pio.prepare(cfg.block, cfg.sm, 0);
    pio.setEnabled(cfg.block, cfg.sm, true);

    r.pc_seen_b = pcSeen(cfg.block, cfg.sm);
    r.stalled_b = @intFromBool(pio.execStalled(cfg.block, cfg.sm));
}

/// C: does it consume a joined TX FIFO under autopull, shifting left?
fn stageC(cfg: Config, r: *Result) void {
    prepareBlock(cfg, &drain_program);
    pio.configure(cfg.block, cfg.sm, drainConfig(cfg.divisor));
    pio.prepare(cfg.block, cfg.sm, 0);

    // **Prime after `prepare` and before `setEnabled`**, in both directions:
    // `prepare` clears the FIFOs, so priming ahead of it is discarded, and
    // priming after the machine is running races it and makes the level
    // meaningless.
    fill(cfg.block, cfg.sm, 0xdead_beef);
    r.tx_level_c = pio.txLevel(cfg.block, cfg.sm);

    pio.setEnabled(cfg.block, cfg.sm, true);
    r.tx_drained_c = @intFromBool(drained(cfg.block, cfg.sm));
    r.tx_stalled_c = @intFromBool(pio.txStalled(cfg.block, cfg.sm));
    r.stalled_c = @intFromBool(pio.execStalled(cfg.block, cfg.sm));
}

/// D: does `out exec` run an instruction that arrived through the FIFO?
fn stageD(cfg: Config, r: *Result) void {
    prepareBlock(cfg, &exec_program);
    pio.configure(cfg.block, cfg.sm, drainConfig(cfg.divisor));
    pio.prepare(cfg.block, cfg.sm, 0);

    // The never-enabled control machine reads back instruction memory. Done in
    // this stage because its program is the one `control_want` names.
    r.control_instr = pio.instruction(cfg.block, cfg.control_sm);

    pio.clearIrq(cfg.block, 0xff);
    fill(cfg.block, cfg.sm, exec_word);
    pio.setEnabled(cfg.block, cfg.sm, true);

    r.tx_drained_d = @intFromBool(drained(cfg.block, cfg.sm));
    r.irq_seen_d = @intFromBool(pio.irqFlags(cfg.block) & (@as(u32, 1) << irq_index) != 0);
    r.stalled_d = @intFromBool(pio.execStalled(cfg.block, cfg.sm));
}

/// E: does it run at the speed we think?
///
/// The stage the other four cannot be trusted to imply. "The FIFO was empty
/// when I looked" is not a throughput measurement when looking takes a
/// millisecond and the work takes nanoseconds.
fn stageE(cfg: Config, r: *Result) void {
    prepareBlock(cfg, &drain_program);
    pio.configure(cfg.block, cfg.sm, drainConfig(timed_divisor));
    pio.prepare(cfg.block, cfg.sm, 0);
    fill(cfg.block, cfg.sm, 0xa5a5_5a5a);

    r.drain_us_expected = expectedDrainUs(cfg.sys_hz);

    const started = timer.now64();
    pio.setEnabled(cfg.block, cfg.sm, true);
    const ok = drained(cfg.block, cfg.sm);
    const elapsed = timer.now64() - started;

    r.drain_us = if (ok) @intCast(@min(elapsed, std.math.maxInt(u32))) else 0;
    r.rate_ok = @intFromBool(ok and withinFactorOfThree(r.drain_us, r.drain_us_expected));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// The configuration stages C, D and E share: the panel's FIFO arrangement,
/// with only the divisor varying.
fn drainConfig(divisor: u32) pio.SmConfig {
    return .{
        .wrap_bottom = 0,
        .wrap_top = 0,
        .out_shiftdir = .left,
        .autopull = true,
        .pull_threshold = 32,
        .fifo_join = .tx,
        .clkdiv = pio.clkdivInt(divisor),
    };
}

/// Reset the block, zero its instruction memory and load `program` at 0.
fn prepareBlock(cfg: Config, program: []const u16) void {
    pio.init(cfg.block);
    // Every pin any stage names is below 32, so the block stays in its low
    // window. A block left at 0 and asked for GP32-47 drives nothing and says
    // nothing.
    pio.setGpioBase(cfg.block, 0);
    pio.loadProgram(cfg.block, 0, program);
}

/// The set of program counter values seen, as a bitmask.
fn pcSeen(block: pio.Block, sm: u32) u32 {
    var seen: u32 = 0;
    var i: u32 = 0;
    while (i < pc_samples) : (i += 1) {
        const pc = pio.programCounter(block, sm);
        if (pc < 32) seen |= @as(u32, 1) << @truncate(pc);
    }
    return seen;
}

/// Fill the TX FIFO with a repeated word.
///
/// Bounded rather than `while (!txFull)`: a status register that never reports
/// full would otherwise spin here forever, which is the one failure a bring-up
/// image must not have.
fn fill(block: pio.Block, sm: u32, word: u32) void {
    var n: u32 = 0;
    while (!pio.txFull(block, sm) and n < 16) : (n += 1) {
        hw.write(block.base() + hw.Pio.txf(sm), word);
    }
}

/// Did the TX FIFO empty within the timeout? Bounded, not blocking — a machine
/// that never consumes is the failure this module exists to report, and
/// reporting it requires surviving it.
fn drained(block: pio.Block, sm: u32) bool {
    const deadline = timer.now64() + drain_timeout_us;
    while (timer.now64() < deadline) {
        if (pio.txLevel(block, sm) == 0) return true;
    }
    return false;
}

/// Microseconds `timed_words` should take to leave the FIFO at `timed_divisor`.
fn expectedDrainUs(sys_hz: u32) u32 {
    if (sys_hz == 0) return 0;
    const cycles: u64 = @as(u64, timed_words) * timed_divisor;
    return @intCast(cycles * 1_000_000 / sys_hz);
}

fn withinFactorOfThree(measured: u32, expected: u32) bool {
    if (expected == 0 or measured == 0) return false;
    return measured <= @as(u64, expected) * 3 and @as(u64, measured) * 3 >= expected;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the programs are what they claim to be" {
    // A two-instruction loop, so the program counter has somewhere to go.
    try testing.expectEqual(@as(u16, 0xa042), loop_program[0]); // nop
    try testing.expectEqual(@as(u16, 0x0000), loop_program[1]); // jmp 0
    // Same pair with side-set, alternating — bit 12 is the side-set value with
    // SIDESET_COUNT 1, not a delay of 16.
    try testing.expectEqual(@as(u16, 0), (loop_program_sideset[0] >> 12) & 1);
    try testing.expectEqual(@as(u16, 1), (loop_program_sideset[1] >> 12) & 1);
    // `out x, 32` and `out exec, 16`, counts encoded as 0 and 16.
    try testing.expectEqual(@as(u16, 0x6020), drain_program[0]);
    try testing.expectEqual(@as(u16, 0x60f0), exec_program[0]);
}

test "stage D's word supplies irq 1 first, then a nop" {
    // OUT shifts left, so the top halfword is consumed first. Getting this the
    // wrong way round would execute the nop first and the irq second — which
    // still passes, and would hide a shift-direction error completely.
    try testing.expectEqual(@as(u16, 0xc001), @as(u16, @truncate(exec_word >> 16)));
    try testing.expectEqual(@as(u16, 0xa042), @as(u16, @truncate(exec_word)));
    // And the irq index the stage watches must be the one the word raises.
    try testing.expectEqual(pio.irqSet(irq_index, false, 0), @as(u16, @truncate(exec_word >> 16)));
}

test "the shared drain config is the panel's FIFO arrangement" {
    const cfg = drainConfig(6);
    const sc = cfg.shiftctrl();
    try testing.expect(sc & hw.Pio.shift_fjoin_tx != 0);
    try testing.expect(sc & hw.Pio.shift_autopull != 0);
    // Left is not the hardware default, and a machine left on the default would
    // consume the wrong half of every word.
    try testing.expect(sc & hw.Pio.shift_out_shiftdir_right == 0);
    try testing.expectEqual(pio.clkdivInt(6), cfg.clkdiv);
    try testing.expectEqual(@as(u32, 0), cfg.clkdiv & 0xffff);
}

test "stage E's expected duration is resolvable by a microsecond timer" {
    // The whole point of `timed_divisor`: at the panel's divisor of 6 this
    // would be 0 us at 200 MHz, i.e. unmeasurable, and a stage that cannot
    // resolve its own answer is worse than no stage.
    const at_200mhz = expectedDrainUs(200_000_000);
    const at_12mhz = expectedDrainUs(12_000_000);
    try testing.expectEqual(@as(u32, 400), at_200mhz);
    try testing.expectEqual(@as(u32, 6666), at_12mhz);
    // Both comfortably above TIMER0's 1 us tick.
    try testing.expect(at_200mhz > 100);
    try testing.expect(at_12mhz > 100);
    // A divisor of 6 would not be, which is why it is not used here.
    const naive: u64 = @as(u64, timed_words) * 6 * 1_000_000 / 200_000_000;
    try testing.expectEqual(@as(u64, 0), naive);
}

test "the rate band catches orders of magnitude, not percentages" {
    try testing.expect(withinFactorOfThree(400, 400));
    try testing.expect(withinFactorOfThree(500, 400));
    try testing.expect(withinFactorOfThree(1200, 400));
    // The failure this exists to catch.
    try testing.expect(!withinFactorOfThree(400_000, 400));
    try testing.expect(!withinFactorOfThree(1, 400));
    // A stage that never finished reports 0, which must never read as a pass.
    try testing.expect(!withinFactorOfThree(0, 400));
    try testing.expect(!withinFactorOfThree(400, 0));
}

test "Result is eighteen words, and a shell can lay it out by hand" {
    // The `watch-*` build steps take a word count that nothing checks against
    // this struct, so it is asserted here rather than discovered on hardware.
    try testing.expectEqual(@as(usize, 18 * 4), @sizeOf(Result));
    try testing.expectEqual(@as(usize, 0), @offsetOf(Result, "all_ok"));
    try testing.expectEqual(@as(usize, 3 * 4), @offsetOf(Result, "div_ticks"));
    try testing.expectEqual(@as(usize, 15 * 4), @offsetOf(Result, "drain_us"));
}

test "the divisor sweep brackets the panel's divisor on both sides" {
    // The sweep exists to locate a threshold, so it is worthless if the value
    // under suspicion sits at one end of it.
    const panel = 6;
    var below = false;
    var above = false;
    for (sweep_divisors) |d| {
        if (d < panel) below = true;
        if (d > panel) above = true;
    }
    try testing.expect(below);
    try testing.expect(above);
    // Divisor 1 must be first: it is the case stage A already proves, so a
    // sweep whose first bit is clear indicts the sweep rather than the chip.
    try testing.expectEqual(@as(u32, 1), sweep_divisors[0]);
    // And the mask must fit the field it is packed into.
    try testing.expect(sweep_divisors.len <= 32);
}
