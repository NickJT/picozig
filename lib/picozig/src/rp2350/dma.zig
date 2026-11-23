//! DMA — moving bytes without the CPU in the loop.
//!
//! Added 2026-07-30 for a display driver whose RGB panel is scanned continuously
//! and forever. Feeding one from the CPU is not a performance question but a
//! structural one: the core would never be free to do anything else.
//!
//! ## What this layer is, and is not
//!
//! It is address arithmetic and a control word. Everything here is pure — no
//! MMIO is executed by the divisor maths or by `Config.word()` — so the part
//! most likely to be silently wrong is host-tested rather than eyeballed on
//! hardware. That is the same bet `spi.Divisors` made and it paid off there.
//!
//! It is not a channel allocator. Channels are named by the caller, because on
//! this chip a display scan-out owns particular channels for the life of the
//! program and a general-purpose allocator would only obscure that.
//!
//! ## The two things that fail silently
//!
//! **CHAIN_TO defaults to 0, and 0 is a real channel.** A channel that should
//! not chain must name *itself* — the datasheet's own way of saying "no
//! chaining". Leave the field at its reset value on channel 5 and finishing a
//! transfer quietly triggers channel 0, which may well be configured for
//! something else entirely. `Config` therefore has no "no chaining" option: it
//! takes `chain_to` and defaults it to the channel's own number.
//!
//! **The field layout is not RP2040's.** `INCR_READ_REV` and `INCR_WRITE_REV`
//! are new at bits 5 and 7, so everything above them shifted up. RP2040
//! constants produce a channel that configures cleanly and transfers to the
//! wrong address, or not at all. See `hw.Dma`.

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// How wide each individual bus transfer is.
///
/// **This is not the total size** — `Config.count` is a count of transfers, not
/// of bytes. Getting that backwards moves four times too much data with a
/// `.word` size, which on a framebuffer is the difference between a picture and
/// a corrupted heap.
pub const Size = enum(u2) {
    byte = 0,
    halfword = 1,
    word = 2,
};

/// A channel's configuration, before it becomes a control word.
pub const Config = struct {
    /// Bus transfer width. `.halfword` is what an RGB565 framebuffer wants.
    size: Size = .word,
    /// Advance the read pointer after each transfer. False for a fixed source,
    /// e.g. draining one peripheral register.
    incr_read: bool = true,
    /// Advance the write pointer. False for a fixed destination, e.g. filling a
    /// PIO FIFO.
    incr_write: bool = true,
    /// What paces the channel. The default runs flat out; a peripheral's DREQ
    /// number here makes the peripheral set the rate, which is what feeding a
    /// PIO state machine needs.
    treq: u32 = hw.Dma.treq_permanent,
    /// Channel to trigger on completion. **Null means "none", which is encoded
    /// as this channel's own number** — see the module note above.
    chain_to: ?u32 = null,
    /// Suppress the completion interrupt. True is right for anything polled.
    irq_quiet: bool = true,
    /// Preferential scheduling. Worth setting for a scan-out that must not be
    /// starved by a bulk copy.
    high_priority: bool = false,

    /// **Address wrapping**: log2 of a block size *in bytes* that one address
    /// stays inside, so the channel walks a circular buffer without anyone
    /// re-pointing it. 0 is off; 4 is a 16-byte block, i.e. four words.
    ///
    /// The buffer must be **aligned to that block size** — the hardware wraps
    /// by masking address bits, not by comparing against a base, so a
    /// misaligned buffer wraps somewhere in the middle of itself and the
    /// symptom is a channel reading a neighbour's memory. `align(16)` on the
    /// declaration, and nothing checks it for you.
    ring_bits: u4 = 0,
    /// Which address `ring_bits` applies to: false wraps the read address,
    /// true the write address. Only one can wrap — the hardware has a single
    /// selector — which is why this is a bool and not two fields.
    ring_on_write: bool = false,

    /// The CTRL word for `ch`. Pure — no MMIO, host-tested below.
    pub fn word(self: Config, ch: u32) u32 {
        var w: u32 = hw.Dma.ctrl_en;
        w |= @as(u32, @backingInt(self.size)) << hw.Dma.ctrl_data_size_shift;
        if (self.incr_read) w |= hw.Dma.ctrl_incr_read;
        if (self.incr_write) w |= hw.Dma.ctrl_incr_write;
        if (self.irq_quiet) w |= hw.Dma.ctrl_irq_quiet;
        if (self.high_priority) w |= hw.Dma.ctrl_high_priority;
        w |= @as(u32, self.ring_bits) << hw.Dma.ctrl_ring_size_shift;
        if (self.ring_on_write) w |= hw.Dma.ctrl_ring_sel;
        w |= (self.treq & 0x3f) << hw.Dma.ctrl_treq_sel_shift;
        w |= ((self.chain_to orelse ch) & 0xf) << hw.Dma.ctrl_chain_to_shift;
        return w;
    }
};

/// Release the DMA block from reset. Idempotent; safe to call more than once.
///
/// Like every peripheral on this chip it answers nothing at all until this
/// runs, and `unreset` spins on RESET_DONE so a write cannot land in the window
/// between "released" and "ready".
/// Bring the DMA block to a known state.
///
/// **A reset cycle, not merely a release — for exactly the reason `pio.init`
/// gives, which had never been applied here.** `unreset` on a block already out
/// of reset is a no-op, and the DMA block is out of reset for the whole life of
/// a firmware image. So every one of sixteen channels arrived at `init` holding
/// whatever the *previous* firmware left in it: `READ_ADDR`, `TRANS_COUNT`,
/// `CTRL` with `EN` and `BUSY` still set, and a half-finished DREQ handshake
/// against a PIO block that `pio.init` had meanwhile reset out from under it.
///
/// That is the "whichever of `tone-check` and `bt-speaker` runs second gets no
/// audio" bug, and it is why **only removing power cleared it**: a warm reset
/// and a reflash both leave `RESETS` untouched, so nothing ever reset DMA.
/// `unreset`'s own doc comment already said to prefer `resetCycle` for a block
/// that carries state; sixteen channels of it qualify.
///
/// **This resets the whole block, so it must be called before any channel is
/// configured, and once per binary.** Every caller does — `pcm5100.start` and
/// `example/dma-check` each call it ahead of their first channel write. A second driver calling it later would stamp on the first,
/// which is the same caveat `pio.init` carries per block.
pub fn init() void {
    hw.Resets.resetCycle(hw.Resets.dma);
}

/// A `count` for `start` that never runs out: TRANS_COUNT's MODE field (bits
/// 31:28) at 0xF, ENDLESS, so the count never decrements — for a channel
/// filling a ring forever with no CPU in the loop.
///
/// **The count field must not be zero.** RP2350 12.6.2.2: a channel triggered
/// in ENDLESS mode with a count of zero halts immediately. `0xF000_0000` looks
/// like "endless" and transfers nothing — measured, on an audio capture driver's first
/// run: enabled, correctly configured, never busy.
pub const count_endless: u32 = 0xF000_0001;

/// Configure `ch` and start it. `count` is a number of **transfers**, not bytes.
pub fn start(ch: u32, cfg: Config, src: u32, dst: u32, count: u32) void {
    hw.write(hw.Dma.readAddr(ch), src);
    hw.write(hw.Dma.writeAddr(ch), dst);
    hw.write(hw.Dma.transCount(ch), count);
    // CTRL_TRIG last: writing it is what starts the channel, so every other
    // register must already be right.
    hw.write(hw.Dma.ctrlTrig(ch), cfg.word(ch));
}

/// True while the channel is transferring.
pub fn busy(ch: u32) bool {
    return hw.read(hw.Dma.ctrlTrig(ch)) & hw.Dma.ctrl_busy != 0;
}

/// Spin until the channel finishes — bounded, and it reports.
///
/// **A poisoned channel is exactly what this has to survive.** `lib/pcm5100`
/// has a reproducible state in which the DMA feeding the I2S machine is left
/// with `BUSY` clear and `TRANS_COUNT` frozen, and which no warm reset or
/// reflash clears. An unbounded wait on a channel in that state is a firmware
/// that stops with nothing to show for it; `error.Timeout` at least names the
/// channel that did not finish.
///
/// The limit is the transfer tier: a legitimate wait here is bounded by how
/// long the *transfer* takes, not by how fast the block responds.
pub fn wait(ch: u32) bounded.Error!void {
    var s = bounded.Spin.init(bounded.transfer_polls);
    while (busy(ch)) try s.tick();
}

/// Any bus error the channel latched, or null if it completed cleanly.
///
/// **Worth checking rather than assuming.** A DMA channel pointed at an invalid
/// address does not fault the CPU — it sets a flag here and stops, so the
/// symptom is a transfer that silently did not happen.
pub const Error = enum { read, write };

pub fn errorFlag(ch: u32) ?Error {
    const ctrl = hw.read(hw.Dma.ctrlTrig(ch));
    if (ctrl & hw.Dma.ctrl_read_error != 0) return .read;
    if (ctrl & hw.Dma.ctrl_write_error != 0) return .write;
    return null;
}

/// Clear latched error flags — both are write-one-to-clear.
pub fn clearErrors(ch: u32) void {
    hw.write(hw.Dma.ctrlTrig(ch), hw.Dma.ctrl_read_error | hw.Dma.ctrl_write_error);
}

/// Force a set of channels to stop, and wait until they are safe to re-arm.
///
/// `mask` is a bitmap of channels — **and it must name every channel in a
/// chain, not just the one being stopped.** Datasheet §12.6.8.3 recommends it,
/// and erratum **RP2350-E5** (A2, A3 and A4, so every part this repo has) makes
/// it mandatory:
///
/// > aborting a channel will cause its `CHAIN_TO` to fire, if and only if the
/// > aborted channel is the last channel to have completed a write transfer
///
/// …and separately, a channel being aborted can be re-triggered on the final
/// cycle before `CHAN_ABORT` clears. Either behaviour restarts a chained pair
/// the instant it is stopped, which presents as an abort that did nothing.
///
/// The erratum's workaround is the first step below: **clear `EN` on every
/// channel involved before writing `CHAN_ABORT`**, so none of them is in a
/// state where a trigger can take.
///
/// `BUSY` staying high for a few cycles afterwards is normal — in-flight bus
/// transfers cannot be revoked — and re-arming before it drops is documented as
/// unpredictable, which is why this polls rather than returning immediately.
pub fn abort(mask: u16) bounded.Error!void {
    // 1. Clear EN (and with it CHAIN_TO's effect) on every channel named.
    var ch: u32 = 0;
    while (ch < hw.Dma.channel_count) : (ch += 1) {
        if (mask & (@as(u16, 1) << @truncate(ch)) == 0) continue;
        // **`al1Ctrl`, not `ctrlTrig`.** The aliased CTRL at +0x10 writes the
        // same register without triggering; clearing EN through the trigger
        // alias would start the channel we are trying to stop.
        const ctrl = hw.read(hw.Dma.al1Ctrl(ch));
        hw.write(hw.Dma.al1Ctrl(ch), ctrl & ~hw.Dma.ctrl_en);
    }

    // 2. Command the abort.
    hw.write(hw.Dma.chan_abort, mask);

    // 3. The register self-clears as each channel comes to rest.
    //
    // **Bounded 2026-08-25.** A channel that will not come to rest is the
    // documented shape of the I2S poisoning, and this spin is where that
    // presents as a hang rather than as a fault.
    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(hw.Dma.chan_abort) & mask != 0) try s.tick();

    // And BUSY can outlast the abort bit by a few cycles.
    ch = 0;
    while (ch < hw.Dma.channel_count) : (ch += 1) {
        if (mask & (@as(u16, 1) << @truncate(ch)) == 0) continue;
        try wait(ch);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "channel register addresses stride correctly across all 16 channels" {
    try testing.expectEqual(@as(u32, 0x5000_0000), hw.Dma.readAddr(0));
    try testing.expectEqual(@as(u32, 0x5000_000c), hw.Dma.ctrlTrig(0));
    // Channel 1 starts one 0x40 block later — datasheet Table 1147.
    try testing.expectEqual(@as(u32, 0x5000_0040), hw.Dma.readAddr(1));
    try testing.expectEqual(@as(u32, 0x5000_004c), hw.Dma.ctrlTrig(1));
    // The last channel's CTRL_TRIG is documented as 0x3cc.
    try testing.expectEqual(@as(u32, 0x5000_03cc), hw.Dma.ctrlTrig(15));
    // And the re-arm alias, documented at 0x03c within the block.
    try testing.expectEqual(@as(u32, 0x5000_003c), hw.Dma.al3ReadAddrTrig(0));
}

test "a config with no chaining names its own channel" {
    // The trap this module exists to prevent: leaving CHAIN_TO at 0 on channel
    // 5 triggers channel 0 on completion, which is a real channel doing
    // something else.
    const w = (Config{}).word(5);
    try testing.expectEqual(@as(u32, 5), (w >> hw.Dma.ctrl_chain_to_shift) & 0xf);
    // Channel 0's own no-chain encoding is 0, which is correct and harmless.
    try testing.expectEqual(@as(u32, 0), ((Config{}).word(0) >> hw.Dma.ctrl_chain_to_shift) & 0xf);
}

test "field positions are RP2350's, not RP2040's" {
    // RP2040 has no INCR_*_REV bits, so its CHAIN_TO sits at 11 and TREQ_SEL at
    // 15. If someone "corrects" hw.Dma from an RP2040 header this fails.
    const w = (Config{ .size = .halfword, .treq = 0x3f, .chain_to = 3 }).word(7);
    try testing.expectEqual(@as(u32, 1), (w >> hw.Dma.ctrl_data_size_shift) & 0x3);
    try testing.expectEqual(@as(u32, 0x3f), (w >> hw.Dma.ctrl_treq_sel_shift) & 0x3f);
    try testing.expectEqual(@as(u32, 3), (w >> hw.Dma.ctrl_chain_to_shift) & 0xf);
    try testing.expectEqual(@as(u5, 13), hw.Dma.ctrl_chain_to_shift);
    try testing.expectEqual(@as(u5, 17), hw.Dma.ctrl_treq_sel_shift);
}

test "address wrapping is off by default, and selects one side when asked" {
    // Off is the default, and "off" is ring_size 0 rather than a flag.
    const plain = (Config{}).word(0);
    try testing.expectEqual(@as(u32, 0), (plain >> hw.Dma.ctrl_ring_size_shift) & 0xf);
    try testing.expect(plain & hw.Dma.ctrl_ring_sel == 0);

    // Four words of read-side ring: 16 bytes, so 4 bits.
    const read_ring = (Config{ .ring_bits = 4 }).word(0);
    try testing.expectEqual(@as(u32, 4), (read_ring >> hw.Dma.ctrl_ring_size_shift) & 0xf);
    try testing.expect(read_ring & hw.Dma.ctrl_ring_sel == 0);

    const write_ring = (Config{ .ring_bits = 4, .ring_on_write = true }).word(0);
    try testing.expect(write_ring & hw.Dma.ctrl_ring_sel != 0);

    // **RING_SIZE must not collide with CHAIN_TO**, which is the failure that
    // would put a ring channel's size into another channel's trigger number.
    const both = (Config{ .ring_bits = 0xf, .chain_to = 5 }).word(0);
    try testing.expectEqual(@as(u32, 0xf), (both >> hw.Dma.ctrl_ring_size_shift) & 0xf);
    try testing.expectEqual(@as(u32, 5), (both >> hw.Dma.ctrl_chain_to_shift) & 0xf);
}

test "increment flags are independent and both default on" {
    const both = (Config{}).word(0);
    try testing.expect(both & hw.Dma.ctrl_incr_read != 0);
    try testing.expect(both & hw.Dma.ctrl_incr_write != 0);

    // Feeding a FIFO: source advances, destination does not.
    const to_fifo = (Config{ .incr_write = false }).word(0);
    try testing.expect(to_fifo & hw.Dma.ctrl_incr_read != 0);
    try testing.expect(to_fifo & hw.Dma.ctrl_incr_write == 0);

    // Neither REV bit is ever set by this layer — reverse addressing exists but
    // nothing here has asked for it, and a stray bit would silently run a
    // transfer backwards.
    try testing.expect(both & hw.Dma.ctrl_incr_read_rev == 0);
    try testing.expect(both & hw.Dma.ctrl_incr_write_rev == 0);
}

test "the channel is always enabled and the size enum matches the encoding" {
    try testing.expect((Config{}).word(0) & hw.Dma.ctrl_en != 0);
    try testing.expectEqual(@as(u2, 0), @backingInt(Size.byte));
    try testing.expectEqual(@as(u2, 1), @backingInt(Size.halfword));
    try testing.expectEqual(@as(u2, 2), @backingInt(Size.word));
}
