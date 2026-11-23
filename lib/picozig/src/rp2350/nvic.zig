//! Cortex-M33 Nested Vectored Interrupt Controller.
//!
//! **This is the piece everything asynchronous is shaped by**, which is why it
//! lands before I2C and DMA rather than after: a driver written to poll and a
//! driver written to be interrupt-driven have different APIs, and retrofitting
//! means designing them twice.
//!
//! RP2350 gives each core **52 interrupt inputs**. `boot.zig`'s vector table is
//! sized to match, with every entry defaulting to a handler that traps rather
//! than to whatever happened to be at that address.
//!
//! Nothing here enables anything by itself. An interrupt only fires when its
//! peripheral is configured to raise it *and* it is enabled here — two separate
//! steps, and forgetting the second is the classic "my handler never runs".

const std = @import("std");
const hw = @import("hw.zig");

/// Number of external interrupts on RP2350.
pub const irq_count = 62;

pub const Irq = u6;

// Standard Cortex-M NVIC block inside the PPB.
const nvic_base: u32 = 0xe000_e100;
pub const iser: u32 = nvic_base + 0x000; // interrupt set-enable
pub const icer: u32 = nvic_base + 0x080; // interrupt clear-enable
pub const ispr: u32 = nvic_base + 0x100; // set-pending
pub const icpr: u32 = nvic_base + 0x180; // clear-pending
pub const ipr: u32 = nvic_base + 0x300; // priority, one BYTE per interrupt

/// RP2350 interrupt numbers, from datasheet §3.2.
///
/// **Two of these were wrong until 2026-08-01, and the comment that used to sit
/// here explained why they could not be** — it said numbers are "added as
/// drivers start using them rather than transcribed speculatively, since an
/// unused wrong number is worse than an absent one". They had been transcribed
/// speculatively anyway, and two were RP2040 values or near misses:
///
/// | Name | Was | §3.2 |
/// |---|---|---|
/// | `io_bank0` | 15 | **21** — 15 is `PIO0_IRQ_0` |
/// | `adc_fifo` | 39 | **35** — 39 is `TRNG_IRQ` |
///
/// Neither had ever been enabled, so nothing misbehaved; the first driver to
/// use either would have enabled somebody else's interrupt and taken the
/// unclaimed-vector trap. **The in-range test below passed the whole time**,
/// which is why there is now a second test pinning the values themselves.
pub const IrqNum = struct {
    pub const timer0_0: Irq = 0;
    pub const timer0_1: Irq = 1;
    pub const timer0_2: Irq = 2;
    pub const timer0_3: Irq = 3;
    pub const pwm_wrap_0: Irq = 8;
    pub const dma_0: Irq = 10;
    pub const dma_1: Irq = 11;
    /// The USB controller. §3.2's table, transcribed on the day `lib/usb`
    /// first needed it rather than speculatively — see the note above about the
    /// two entries here that were wrong for two months.
    pub const usbctrl: Irq = 14;
    pub const pio0_irq_0: Irq = 15;
    pub const pio0_irq_1: Irq = 16;
    pub const pio1_irq_0: Irq = 17;
    pub const pio1_irq_1: Irq = 18;
    pub const pio2_irq_0: Irq = 19;
    pub const pio2_irq_1: Irq = 20;
    pub const io_bank0: Irq = 21;
    pub const spi0: Irq = 31;
    pub const uart0: Irq = 33;
    pub const adc_fifo: Irq = 35;
};

/// The NVIC number for one of a PIO block's two interrupt request lines.
///
/// The six PIO entries above are consecutive — block 0 line 0 at 15, then two
/// per block — so a driver parameterised over `pio.Block` can name its
/// interrupt without a switch, which is the whole reason this exists: the
/// panel's block is a build option, and a `-Dpanel-block` that moved the state
/// machines without moving the interrupt would be silent.
pub fn pioIrq(block_index: u2, line: u1) Irq {
    return IrqNum.pio0_irq_0 + @as(Irq, block_index) * 2 + line;
}

pub fn enable(irq: Irq) void {
    const n = @as(u32, irq);
    hw.write(iser + (n / 32) * 4, @as(u32, 1) << @truncate(n % 32));
}

pub fn disable(irq: Irq) void {
    const n = @as(u32, irq);
    hw.write(icer + (n / 32) * 4, @as(u32, 1) << @truncate(n % 32));
    // Architecturally required after a clear-enable: without these the
    // pipeline can still take an interrupt that was already in flight, which
    // shows up as a handler running once after being "disabled".
    asm volatile ("dsb");
    asm volatile ("isb");
}

pub fn setPending(irq: Irq) void {
    const n = @as(u32, irq);
    hw.write(ispr + (n / 32) * 4, @as(u32, 1) << @truncate(n % 32));
}

pub fn clearPending(irq: Irq) void {
    const n = @as(u32, irq);
    hw.write(icpr + (n / 32) * 4, @as(u32, 1) << @truncate(n % 32));
}

/// Whether the interrupt is pending — either its system IRQ line is currently
/// asserted, or `setPending` has forced it high.
///
/// **Pending is not the same as "about to run".** A pending interrupt that is
/// disabled, or masked by PRIMASK or by a higher-priority handler already in
/// progress, stays pending indefinitely. This answers "has the peripheral
/// raised its line", which is what makes it useful for polling a peripheral
/// without installing a handler at all.
pub fn isPending(irq: Irq) bool {
    const n = @as(u32, irq);
    return hw.read(ispr + (n / 32) * 4) & (@as(u32, 1) << @truncate(n % 32)) != 0;
}

/// Set an interrupt's priority. **Only the top bits are implemented** — RP2350
/// gives 2 priority bits, so the hardware sees `priority & 0xc0` and the low
/// bits are discarded. A value of 0 is the *highest* priority, which is the
/// opposite of most people's intuition and worth stating rather than assuming.
pub fn setPriority(irq: Irq, priority: u8) void {
    const addr = ipr + @as(u32, irq);
    // IPR is byte-addressable on Cortex-M33.
    const p: *volatile u8 = @ptrFromInt(addr);
    p.* = priority;
}

pub inline fn enableInterrupts() void {
    asm volatile ("cpsie i" ::: .{ .memory = true });
}

pub inline fn disableInterrupts() void {
    asm volatile ("cpsid i" ::: .{ .memory = true });
}

/// Run `body` with interrupts masked, restoring the previous state afterwards.
///
/// Restores rather than blindly re-enabling, so nesting is safe — a critical
/// section inside another one must not turn interrupts back on early.
pub inline fn critical(body: anytype) void {
    const primask = asm volatile ("mrs %[out], primask"
        : [out] "=r" (-> u32),
    );
    disableInterrupts();
    defer if (primask & 1 == 0) enableInterrupts();
    body();
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "register addresses and bit maths for the 32-interrupt boundary" {
    try testing.expectEqual(@as(u32, 0xe000_e100), iser);
    try testing.expectEqual(@as(u32, 0xe000_e180), icer);
    try testing.expectEqual(@as(u32, 0xe000_e400), ipr);

    // RP2350's 52 interrupts span two ISER words, so the /32 and %32 maths is
    // load-bearing rather than decorative: IRQ 35 (ADC FIFO) lives in word 1,
    // bit 3.
    //
    // **This test caught the `adc_fifo` correction rather than the other way
    // round**, because it asserted the bit position of a specific number. That
    // is the argument for writing the value into a test at all — the sibling
    // test above now does it for every number here.
    const n: u32 = IrqNum.adc_fifo;
    try testing.expectEqual(@as(u32, 1), n / 32);
    try testing.expectEqual(@as(u32, 3), n % 32);
    try testing.expectEqual(iser + 4, iser + (n / 32) * 4);

    // ...and IRQ 21 stays in word 0.
    const io: u32 = IrqNum.io_bank0;
    try testing.expectEqual(@as(u32, 0), io / 32);
    try testing.expectEqual(iser, iser + (io / 32) * 4);
}

test "every declared IRQ number is within range" {
    inline for (@typeInfo(IrqNum).@"struct".decl_names) |d_name| {
        const v = @field(IrqNum, d_name);
        try testing.expect(v < irq_count);
    }
}

test "the numbers themselves, against datasheet 3.2" {
    // **The range test above is not enough, and this is the evidence.** It
    // passed for `io_bank0 = 15` and `adc_fifo = 39`, both wrong, for as long
    // as they existed. A number in range is not a number that is right, and the
    // only check that can tell the difference is one that names the value.
    try testing.expectEqual(@as(Irq, 10), IrqNum.dma_0);
    try testing.expectEqual(@as(Irq, 15), IrqNum.pio0_irq_0);
    try testing.expectEqual(@as(Irq, 21), IrqNum.io_bank0);
    try testing.expectEqual(@as(Irq, 31), IrqNum.spi0);
    try testing.expectEqual(@as(Irq, 33), IrqNum.uart0);
    try testing.expectEqual(@as(Irq, 35), IrqNum.adc_fifo);
}

test "pioIrq walks the six consecutive PIO vectors" {
    try testing.expectEqual(IrqNum.pio0_irq_0, pioIrq(0, 0));
    try testing.expectEqual(IrqNum.pio0_irq_1, pioIrq(0, 1));
    try testing.expectEqual(IrqNum.pio1_irq_0, pioIrq(1, 0));
    try testing.expectEqual(IrqNum.pio1_irq_1, pioIrq(1, 1));
    try testing.expectEqual(IrqNum.pio2_irq_0, pioIrq(2, 0));
    try testing.expectEqual(IrqNum.pio2_irq_1, pioIrq(2, 1));
}

test "USBCTRL is interrupt 14" {
    // §3.2. Pinned by value because an unused wrong number is worse than an
    // absent one: it enables somebody else's interrupt and takes the
    // unclaimed-vector trap, which reads as a fault in whatever ran last.
    try std.testing.expectEqual(@as(Irq, 14), IrqNum.usbctrl);
    try std.testing.expect(IrqNum.usbctrl < irq_count);
    // And it is not one of the numbers already claimed here.
    try std.testing.expect(IrqNum.usbctrl != IrqNum.pio0_irq_0);
    try std.testing.expect(IrqNum.usbctrl != IrqNum.dma_1);
}
