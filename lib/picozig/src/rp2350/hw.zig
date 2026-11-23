//! RP2350 register map — only the peripherals the alert display actually
//! needs. This is the "just enough" layer: no SVD import, no generated code,
//! no pico-sdk. Every address here is copied from the RP2350 hardware_regs
//! headers and carries a comment saying so, because the one thing that makes
//! this approach painful is a wrong address failing silently.
//!
//! IMPORTANT — RP2350 is NOT RP2040 with a bigger part number. Several
//! peripherals were relocated and, worse, SIO's register offsets were
//! *reshuffled* (the HI_* registers for GPIO32-47 are interleaved with the
//! low ones, so GPIO_OUT_SET moved from 0x14 to 0x18 and GPIO_OE from 0x20
//! to 0x30). Copying RP2040 offsets from memory or from an RP2040 tutorial
//! produces code that compiles, runs, and does nothing. Verified against
//! pico-sdk/src/rp2350/hardware_regs/include/hardware/regs/{addressmap,sio,
//! resets}.h.
//!
//! TARGET: RP2350**B** — the 80-pin part with 48 GPIOs and 8 ADC channels.
//! That is why `Pin` is a `u6` and why every SIO access has to choose a
//! register bank: GPIO 32-47 are invisible to the low registers, so `1 << 43`
//! written to GPIO_OUT is a no-op with no diagnostic whatsoever.
//!
//! **Board-specific facts do not live here** — pin assignments, the VBAT
//! divider and the charger belong to ../board.zig. This file is the chip.

// ---------------------------------------------------------------------------
// Peripheral base addresses (addressmap.h)
// ---------------------------------------------------------------------------

pub const rom_base: u32 = 0x0000_0000;
pub const xip_base: u32 = 0x1000_0000;
pub const sram_base: u32 = 0x2000_0000;

pub const sysinfo_base: u32 = 0x4000_0000;
pub const clocks_base: u32 = 0x4001_0000;
pub const psm_base: u32 = 0x4001_8000;
pub const resets_base: u32 = 0x4002_0000;
pub const io_bank0_base: u32 = 0x4002_8000;
pub const pads_bank0_base: u32 = 0x4003_8000;
pub const xosc_base: u32 = 0x4004_8000;
pub const pll_sys_base: u32 = 0x4005_0000;
pub const pll_usb_base: u32 = 0x4005_8000;
pub const uart0_base: u32 = 0x4007_0000;
pub const uart1_base: u32 = 0x4007_8000;
pub const spi0_base: u32 = 0x4008_0000;
pub const spi1_base: u32 = 0x4008_8000;
pub const i2c0_base: u32 = 0x4009_0000;
pub const i2c1_base: u32 = 0x4009_8000;
pub const adc_base: u32 = 0x400a_0000;
pub const pwm_base: u32 = 0x400a_8000;
pub const timer0_base: u32 = 0x400b_0000;
pub const watchdog_base: u32 = 0x400d_8000;
pub const trng_base: u32 = 0x400f_8000;
pub const powman_base: u32 = 0x4010_0000;
pub const ticks_base: u32 = 0x4010_8000;
pub const sio_base: u32 = 0xd000_0000;

/// **On the AHB, not the APB** — which is why it is nowhere near the other
/// peripheral bases, and why it is fast enough to feed a display.
pub const dma_base: u32 = 0x5000_0000;

/// The three PIO blocks. **RP2350 has three; RP2040 had two** — PIO2 is new,
/// and a vendor's display-board firmware uses all three at once (PIO1 for the
/// panel, PIO2 for the RM2, PIO0 for the LED chain).
pub const pio0_base: u32 = 0x5020_0000;
pub const pio1_base: u32 = 0x5030_0000;
pub const pio2_base: u32 = 0x5040_0000;

/// Cortex-M33 System Control Block — vector table offset register.
pub const scb_vtor: u32 = 0xe000_ed08;

/// Configurable Fault Status Register: MemManage / BusFault / UsageFault
/// causes, one byte-or-halfword field each. Meaningless on its own; read
/// alongside HFSR, which says whether the fault escalated.
pub const scb_cfsr: u32 = 0xe000_ed28;

/// HardFault Status Register. `FORCED` (bit 30) is the common case — a
/// configurable fault that was not enabled, so it escalated to HardFault and
/// the real cause is in CFSR. `VECTTBL` (bit 1) means the fault happened
/// *reading the vector table*, which on this board is a flash read.
pub const scb_hfsr: u32 = 0xe000_ed2c;

/// MemManage Fault Address, valid when CFSR's `MMARVALID` is set.
pub const scb_mmfar: u32 = 0xe000_ed34;

/// BusFault Address, valid when CFSR's `BFARVALID` is set. **This is the one
/// that matters for a bad XIP fetch** — it names the address that would not
/// read.
pub const scb_bfar: u32 = 0xe000_ed38;

// ---------------------------------------------------------------------------
// Atomic register aliases
// ---------------------------------------------------------------------------
// Every APB peripheral is mirrored three more times at +0x1000/+0x2000/+0x3000,
// where a write XORs / sets / clears the addressed bits instead of replacing
// the register. This is how you touch one field without a read-modify-write
// (and without a race against the other core or an interrupt).
//
// NOTE: SIO is deliberately excluded — it sits on a single-cycle bus, not the
// APB, and has NO alias regions. Writing sio_base + set_alias lands in the
// middle of the interpolator registers. Use SIO's own dedicated GPIO_OUT_SET /
// _CLR / _XOR registers instead; that is exactly what they are for.

pub const alias_xor: u32 = 0x1000;
pub const alias_set: u32 = 0x2000;
pub const alias_clr: u32 = 0x3000;

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

/// A memory-mapped 32-bit register.
pub inline fn reg(addr: u32) *volatile u32 {
    return @ptrFromInt(addr);
}

pub inline fn read(addr: u32) u32 {
    return reg(addr).*;
}

pub inline fn write(addr: u32, value: u32) void {
    reg(addr).* = value;
}

/// Atomically set the given bits of an **APB** register. Not valid for SIO.
pub inline fn setBits(addr: u32, mask: u32) void {
    reg(addr + alias_set).* = mask;
}

/// Atomically clear the given bits of an **APB** register. Not valid for SIO.
pub inline fn clrBits(addr: u32, mask: u32) void {
    reg(addr + alias_clr).* = mask;
}

/// Atomically toggle the given bits of an **APB** register. Not valid for SIO.
pub inline fn xorBits(addr: u32, mask: u32) void {
    reg(addr + alias_xor).* = mask;
}

// ---------------------------------------------------------------------------
// RESETS (resets.h)
// ---------------------------------------------------------------------------
// Every peripheral comes out of power-on reset *held in reset* and produces no
// response at all until released. Forgetting this is the second-most-common
// reason bare-metal RP2 code does nothing.

pub const Resets = struct {
    pub const reset: u32 = resets_base + 0x00;
    pub const wdsel: u32 = resets_base + 0x04;
    pub const reset_done: u32 = resets_base + 0x08;

    // Bit positions within RESET / RESET_DONE.
    pub const adc: u32 = 1 << 0;
    pub const busctrl: u32 = 1 << 1;
    pub const dma: u32 = 1 << 2;
    pub const hstx: u32 = 1 << 3;
    pub const i2c0: u32 = 1 << 4;
    pub const i2c1: u32 = 1 << 5;
    pub const io_bank0: u32 = 1 << 6;
    pub const io_qspi: u32 = 1 << 7;
    pub const jtag: u32 = 1 << 8;
    pub const pads_bank0: u32 = 1 << 9;
    pub const pads_qspi: u32 = 1 << 10;
    pub const pio0: u32 = 1 << 11;
    pub const pio1: u32 = 1 << 12;
    pub const pio2: u32 = 1 << 13;
    pub const pll_sys: u32 = 1 << 14;
    pub const pll_usb: u32 = 1 << 15;
    pub const pwm: u32 = 1 << 16;
    pub const sha256: u32 = 1 << 17;
    pub const spi0: u32 = 1 << 18;
    pub const spi1: u32 = 1 << 19;
    pub const syscfg: u32 = 1 << 20;
    pub const sysinfo: u32 = 1 << 21;
    pub const tbman: u32 = 1 << 22;
    pub const timer0: u32 = 1 << 23;
    pub const timer1: u32 = 1 << 24;
    pub const trng: u32 = 1 << 25;
    pub const uart0: u32 = 1 << 26;
    pub const uart1: u32 = 1 << 27;
    pub const usbctrl: u32 = 1 << 28;

    /// Put the given peripherals **back** into reset, then release them — a
    /// known state rather than whatever the last program left behind.
    ///
    /// `unreset` alone is not enough for anything with state worth clearing.
    /// It clears a reset bit that is usually already clear, and then every
    /// register a previous program wrote survives. That is fine for a UART
    /// whose every field gets configured anyway, and wrong for a block like PIO
    /// where "configured" means four state machines, a shared instruction
    /// memory, per-pin directions and eight interrupt flags, of which a driver
    /// naturally sets only the ones it uses.
    ///
    /// Found on a display board, 2026-08-01. `reset run` over SWD does not reset
    /// peripherals, so PIO1 arrived at our firmware still holding the vendor
    /// MicroPython's panel setup: SM1 configured and *enabled*, and PIO pin
    /// directions asserted for GP1-16, a bus our image drives from SIO and
    /// never hands to PIO at all. Every register read off that block was a
    /// blend of two programs, and it took three wrong theories to notice.
    pub fn resetCycle(mask: u32) void {
        setBits(reset, mask);
        unreset(mask);
    }

    /// Release the given peripherals from reset and spin until the hardware
    /// confirms each one is up. The confirm half matters: a peripheral written
    /// to in the window between "reset released" and "reset done" ignores the
    /// write.
    ///
    /// **Prefer `resetCycle` for any block that carries state**, per its note.
    pub fn unreset(mask: u32) void {
        clrBits(reset, mask);
        while (read(reset_done) & mask != mask) {}
    }
};

// ---------------------------------------------------------------------------
// SIO — single-cycle IO, the GPIO fast path (sio.h)
// ---------------------------------------------------------------------------
// Offsets verified against the RP2350 header; see the warning at the top of
// this file about how these differ from RP2040's.

// The low registers cover GPIO 0-31; the HI_ ones cover GPIO 32-47 and are
// interleaved between them (+4 from each low register). Rather than expose
// both sets and invite picking the wrong one, `bank()` below selects for a
// given pin — every caller goes through it.

pub const Sio = struct {
    pub const cpuid: u32 = sio_base + 0x000;

    pub const gpio_in: u32 = sio_base + 0x004;
    pub const gpio_hi_in: u32 = sio_base + 0x008;
    pub const gpio_out: u32 = sio_base + 0x010;
    pub const gpio_hi_out: u32 = sio_base + 0x014;
    pub const gpio_out_set: u32 = sio_base + 0x018;
    pub const gpio_hi_out_set: u32 = sio_base + 0x01c;
    pub const gpio_out_clr: u32 = sio_base + 0x020;
    pub const gpio_hi_out_clr: u32 = sio_base + 0x024;
    pub const gpio_out_xor: u32 = sio_base + 0x028;
    pub const gpio_hi_out_xor: u32 = sio_base + 0x02c;
    pub const gpio_oe: u32 = sio_base + 0x030;
    pub const gpio_hi_oe: u32 = sio_base + 0x034;
    pub const gpio_oe_set: u32 = sio_base + 0x038;
    pub const gpio_hi_oe_set: u32 = sio_base + 0x03c;
    pub const gpio_oe_clr: u32 = sio_base + 0x040;
    pub const gpio_hi_oe_clr: u32 = sio_base + 0x044;
    pub const gpio_oe_xor: u32 = sio_base + 0x048;
    pub const gpio_hi_oe_xor: u32 = sio_base + 0x04c;

    /// Which SIO register bank a pin lives in, and its bit within that bank.
    /// `low` is the GPIO 0-31 register address; the GPIO 32-47 sibling is
    /// always `low + 4`.
    pub inline fn bank(pin: Pin, low: u32) struct { addr: u32, mask: u32 } {
        return .{
            .addr = if (pin >= 32) low + 4 else low,
            .mask = @as(u32, 1) << @truncate(pin),
        };
    }
};

// ---------------------------------------------------------------------------
// IO_BANK0 / PADS_BANK0
// ---------------------------------------------------------------------------

/// A GPIO number. u6 because this is an RP2350B with 48 of them — see the
/// board note at the top of this file.
pub const Pin = u6;

pub const IoBank0 = struct {
    /// Each GPIO has a STATUS/CTRL pair, 8 bytes apart; CTRL is the second.
    pub fn ctrl(gpio: Pin) u32 {
        return io_bank0_base + @as(u32, gpio) * 8 + 4;
    }

    /// GPIO_CTRL.FUNCSEL values we care about. SIO is the plain
    /// software-driven-pin function.
    pub const func_spi: u32 = 1;
    pub const func_uart: u32 = 2;
    pub const func_sio: u32 = 5;
    /// The three PIO blocks are consecutive FUNCSELs above SIO. **PIO2 exists
    /// only on RP2350** — RP2040 has no function 8.
    pub const func_pio0: u32 = 6;
    pub const func_pio1: u32 = 7;
    pub const func_pio2: u32 = 8;
    pub const func_null: u32 = 31;

    // -- interrupts ---------------------------------------------------------
    //
    // **Four bits per GPIO, eight GPIOs per word**, in the order LEVEL_LOW,
    // LEVEL_HIGH, EDGE_LOW, EDGE_HIGH — datasheet §9.4, and the register
    // listing at 0x230 onwards. Six words covers 48 GPIOs, which is the whole
    // of an RP2350B.

    /// Raw interrupts, before enable or force. **Edge bits are latched here
    /// and cleared by writing 1; level bits are not latched at all** and
    /// simply follow the pin.
    pub fn intr(word: u3) u32 {
        return io_bank0_base + 0x230 + @as(u32, word) * 4;
    }

    /// Interrupt enable for proc0. Writing 0 here is the only way to silence a
    /// level-triggered source, because there is nothing to clear.
    pub fn proc0_inte(word: u3) u32 {
        return io_bank0_base + 0x248 + @as(u32, word) * 4;
    }

    /// Interrupt status for proc0, after masking and forcing — this is what an
    /// ISR reads to find out which pin called it.
    pub fn proc0_ints(word: u3) u32 {
        return io_bank0_base + 0x278 + @as(u32, word) * 4;
    }

    /// Which of the six words holds `gpio`.
    pub fn irqWord(gpio: Pin) u3 {
        return @intCast(gpio / 8);
    }

    /// The bit within that word for `gpio` and one trigger condition.
    pub fn irqBit(gpio: Pin, trigger: u2) u5 {
        return @intCast((@as(u32, gpio) % 8) * 4 + trigger);
    }
};

pub const PadsBank0 = struct {
    /// Offset 0x00 is VOLTAGE_SELECT; the per-GPIO pad controls start at 0x04.
    /// Unlike SIO, this array is flat across all 48 GPIOs — no bank split.
    pub fn pad(gpio: Pin) u32 {
        return pads_bank0_base + 0x04 + @as(u32, gpio) * 4;
    }

    pub const slewfast: u32 = 1 << 0;
    pub const schmitt: u32 = 1 << 1;
    pub const pde: u32 = 1 << 2; // pull-down enable
    pub const pue: u32 = 1 << 3; // pull-up enable
    pub const drive_2ma: u32 = 0 << 4;
    pub const drive_4ma: u32 = 1 << 4;
    pub const drive_8ma: u32 = 2 << 4;
    pub const drive_12ma: u32 = 3 << 4;
    pub const ie: u32 = 1 << 6; // input enable
    pub const od: u32 = 1 << 7; // output DISABLE (active high)

    /// Pad isolation latch. **Resets to 1**, and while set the pad is frozen —
    /// nothing software does to OE, output level or pulls reaches the outside
    /// world. This does not exist on RP2040, and the datasheet calls out our
    /// exact situation (§9.7): "Non-SDK applications ported from RP2040 must
    /// clear the ISO bit before using a GPIO... The SDK automatically clears
    /// the ISO bit when gpio_set_function() is called."
    ///
    /// We have no SDK doing it for us, so gpio.zig clears it explicitly — and
    /// *last*, after the muxing is set up, which is the order §9.7 prescribes
    /// so the pad never briefly presents a half-configured state.
    ///
    /// Also relevant to the sleep work: powering the switched core domain back
    /// up resets every GPIO's ISO bit to 1, deliberately holding the pre-sleep
    /// pad state until software re-engages each pad. Waking from a POWMAN
    /// dormant state that drops that domain therefore has to re-run pad setup.
    pub const iso: u32 = 1 << 8;
};

// ---------------------------------------------------------------------------
// XOSC — crystal oscillator
// ---------------------------------------------------------------------------

pub const Xosc = struct {
    pub const ctrl: u32 = xosc_base + 0x00;
    pub const status: u32 = xosc_base + 0x04;
    pub const dormant: u32 = xosc_base + 0x08;
    pub const startup: u32 = xosc_base + 0x0c;
    pub const count: u32 = xosc_base + 0x10;

    /// CTRL fields are 12-bit magic codes rather than plain bits, deliberately:
    /// "The 12-bit code is intended to give some protection against accidental
    /// writes. An invalid setting will retain the previous value." So a wrong
    /// value here does not fault — it is silently ignored, and the chip carries
    /// on running from the ROSC. Read STATUS back rather than trusting a write.
    pub const freq_range_1_15mhz: u32 = 0xaa0;
    pub const freq_range_10_30mhz: u32 = 0xaa1;
    pub const enable: u32 = 0xfab << 12;
    pub const disable: u32 = 0xd1e << 12;

    pub const status_stable: u32 = 1 << 31;
    pub const status_badwrite: u32 = 1 << 24;
    pub const status_enabled: u32 = 1 << 12;

    pub const dormant_wake: u32 = 0x7761_6b65; // "wake"
    pub const dormant_dormant: u32 = 0x636f_6d61; // "coma"
};

// ---------------------------------------------------------------------------
// DMA
// ---------------------------------------------------------------------------

/// 16 channels, each a 16-word block: four live registers followed by three
/// sets of aliases.
///
/// **The alias sets are the interesting part and the reason the block is this
/// shape.** Every set ends in a *trigger* register whose write both stores the
/// value and starts the channel, so a channel can be re-armed with a single
/// store — write the new read address to `AL3_READ_ADDR_TRIG` and it goes.
/// That is what makes a chained, free-running scan-out possible without the CPU
/// in the loop, which is exactly what a DPI panel needs.
///
/// Field positions are the RP2350 datasheet's (Table 1151) and **differ from
/// RP2040's**: `INCR_READ_REV` and `INCR_WRITE_REV` are new at bits 5 and 7,
/// which pushes RING_SIZE, RING_SEL, CHAIN_TO and TREQ_SEL up from where an
/// RP2040 header or tutorial puts them. Getting that wrong gives a channel that
/// configures cleanly and transfers to the wrong place, or not at all.
pub const Dma = struct {
    pub const channel_count: u32 = 16;
    /// Bytes between one channel's registers and the next.
    pub const channel_stride: u32 = 0x40;

    /// One bit per channel: command it to stop issuing transfers and clear
    /// `BUSY`. Datasheet §12.6.8.3, and the register listing puts it at
    /// **0x464 on RP2350** — it was 0x444 on RP2040.
    pub const chan_abort: u32 = dma_base + 0x464;

    pub inline fn readAddr(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x00;
    }
    pub inline fn writeAddr(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x04;
    }
    pub inline fn transCount(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x08;
    }
    /// Writing this both configures **and starts** the channel.
    pub inline fn ctrlTrig(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x0c;
    }
    /// CTRL without the trigger side effect, for configuring a channel that
    /// something else will start.
    pub inline fn al1Ctrl(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x10;
    }
    /// Write a read address and start, in one store. The re-arm register.
    pub inline fn al3ReadAddrTrig(ch: u32) u32 {
        return dma_base + ch * channel_stride + 0x3c;
    }

    // CTRL_TRIG fields — datasheet Table 1151.
    pub const ctrl_en: u32 = 1 << 0;
    pub const ctrl_high_priority: u32 = 1 << 1;
    pub const ctrl_data_size_shift: u5 = 2;
    pub const ctrl_incr_read: u32 = 1 << 4;
    pub const ctrl_incr_read_rev: u32 = 1 << 5;
    pub const ctrl_incr_write: u32 = 1 << 6;
    pub const ctrl_incr_write_rev: u32 = 1 << 7;
    pub const ctrl_ring_size_shift: u5 = 8;
    pub const ctrl_ring_sel: u32 = 1 << 12;
    pub const ctrl_chain_to_shift: u5 = 13;
    pub const ctrl_treq_sel_shift: u5 = 17;
    pub const ctrl_irq_quiet: u32 = 1 << 23;
    pub const ctrl_bswap: u32 = 1 << 24;
    pub const ctrl_sniff_en: u32 = 1 << 25;
    pub const ctrl_busy: u32 = 1 << 26;
    pub const ctrl_write_error: u32 = 1 << 29;
    pub const ctrl_read_error: u32 = 1 << 30;
    pub const ctrl_ahb_error: u32 = 1 << 31;

    /// TREQ_SEL = 0x3f: unpaced, go as fast as the bus allows. Anything else
    /// paces the channel off a peripheral's request line.
    pub const treq_permanent: u32 = 0x3f;
    /// Datasheet §12.6.4.1, the system DREQ table.
    pub const treq_adc: u32 = 48;

    /// **A channel that chains to itself does not chain.** The datasheet spells
    /// this out, and it is how you say "no chaining": CHAIN_TO must be set to
    /// the channel's own number, *not* left at 0 — leaving it at 0 on any
    /// channel other than 0 silently triggers channel 0 when the transfer ends.
    pub inline fn chainToSelf(ch: u32) u32 {
        return ch << ctrl_chain_to_shift;
    }
};

// ---------------------------------------------------------------------------
// PIO
// ---------------------------------------------------------------------------

/// Programmable IO: four state machines per block, 32 shared instruction words.
///
/// Offsets are datasheet Table 981. The layout is RP2040's for everything named
/// here, but **two RP2350 differences matter**:
///
///   * There is a third block, PIO2.
///   * `GPIOBASE` (0x168) selects which 32 GPIOs the block can reach — 0 or 16.
///     A block left at 0 cannot touch GP32-47 at all, and says nothing about
///     it — a state machine told to drive GP43 simply drives nothing. Any
///     caller working above GP31 must set it.
pub const Pio = struct {
    pub const sm_count: u32 = 4;
    /// Words of shared instruction memory, per block.
    pub const instr_mem_words: u32 = 32;

    pub const ctrl: u32 = 0x000;
    pub const fstat: u32 = 0x004;
    pub const fdebug: u32 = 0x008;
    pub const flevel: u32 = 0x00c;
    pub const irq: u32 = 0x030;
    pub const irq_force: u32 = 0x034;

    /// One bit per GPIO: set it and that pin's input skips the two-flop
    /// synchroniser on the way into the state machine.
    ///
    /// The two cycles it removes are `clk_sys` cycles, not state-machine
    /// cycles, so at a small clock divisor they are a large fraction of a bit
    /// period — which is why a bus that samples its own input near the clock
    /// edge needs this and a bus that samples mid-bit does not. Bypassing is
    /// only safe for a signal already synchronous to `clk_sys`; on an
    /// asynchronous input it trades setup violations for metastability.
    ///
    /// **The bits are relative to `gpiobase`**, like every other pin field in
    /// this block — bit 0 is GP0 only while `gpiobase` is 0.
    pub const input_sync_bypass: u32 = 0x038;

    pub const dbg_padout: u32 = 0x03c;
    pub const gpiobase: u32 = 0x168;

    /// Which PIO conditions are masked into interrupt request **line 0**, the
    /// one `nvic.pioIrq(block, 0)` carries to the core. `irq1_inte` is the same
    /// for line 1.
    ///
    /// **0x170 on RP2350; it was 0x12c on RP2040.** Taken from datasheet §11.7
    /// rather than from an RP2040 header, which is the mistake this whole file
    /// exists to make impossible — an RP2040 offset here lands in `gpiobase`.
    pub const irq0_inte: u32 = 0x170;
    pub const irq0_intf: u32 = 0x174;
    pub const irq0_ints: u32 = 0x178;
    pub const irq1_inte: u32 = 0x17c;

    /// Bit in `IRQ0_INTE`/`IRQ1_INTE` for PIO irq flag `n`.
    ///
    /// **The eight state-machine IRQ flags start at bit 8**, above the four
    /// `RXNEMPTY` and four `TXNFULL` FIFO bits — so enabling "irq 0" means
    /// setting bit 8, not bit 0. Setting bit 0 enables SM0's RX-not-empty
    /// instead, which on a TX-only scan-out never fires and therefore looks
    /// exactly like an interrupt that was never enabled.
    ///
    /// RP2350 exposes all eight; RP2040 exposed only the lower four.
    pub inline fn inteSmIrq(n: u3) u32 {
        return @as(u32, 1) << (8 + @as(u5, n));
    }

    pub inline fn txf(sm: u32) u32 {
        return 0x010 + sm * 4;
    }
    pub inline fn rxf(sm: u32) u32 {
        return 0x020 + sm * 4;
    }
    pub inline fn instrMem(i: u32) u32 {
        return 0x048 + i * 4;
    }

    /// Per-state-machine block: six registers, 0x18 apart.
    pub inline fn smClkdiv(sm: u32) u32 {
        return 0x0c8 + sm * 0x18;
    }
    pub inline fn smExecctrl(sm: u32) u32 {
        return 0x0cc + sm * 0x18;
    }
    pub inline fn smShiftctrl(sm: u32) u32 {
        return 0x0d0 + sm * 0x18;
    }
    pub inline fn smAddr(sm: u32) u32 {
        return 0x0d4 + sm * 0x18;
    }
    pub inline fn smInstr(sm: u32) u32 {
        return 0x0d8 + sm * 0x18;
    }
    pub inline fn smPinctrl(sm: u32) u32 {
        return 0x0dc + sm * 0x18;
    }

    /// `EXECCTRL.EXEC_STALLED` — read-only.
    ///
    /// **Read its definition narrowly.** §11.7 defines it as "an instruction
    /// *written to SMx_INSTR* is stalled", while `CTRL.SM_RESTART` is documented
    /// as clearing "any stalled instruction written to SMx_INSTR **or run by
    /// OUT/MOV EXEC**". The two sentences imply one latch shared by both
    /// mechanisms, but only one of them is named in the flag's own description.
    ///
    /// So a set flag does **not** on its own distinguish "the CPU forced in an
    /// instruction that will not complete" from "the program executed `out exec`
    /// and the executee is blocking". Those are very different faults, and
    /// telling them apart needs `SMx_ADDR` as well — a machine parked at one
    /// address with the flag set is the first; a machine whose PC moves is the
    /// second. A whole investigation was run on the assumption that it meant
    /// only the first.
    pub const execctrl_exec_stalled: u32 = 1 << 31;

    /// CTRL.SM_ENABLE, bits 3:0.
    pub inline fn ctrlSmEnable(sm: u32) u32 {
        return @as(u32, 1) << @truncate(sm);
    }
    /// CTRL.SM_RESTART, bits **7:4** — self-clearing. Clears the shift counters,
    /// the ISR, the delay counter, the waiting-on-IRQ state and any latched
    /// instruction. **It does not clear the OSR contents**, only its shift
    /// counter (to 32, i.e. empty), which is why an OSR read back from a machine
    /// that never pulled tells you nothing about the FIFO.
    pub inline fn ctrlSmRestart(sm: u32) u32 {
        return @as(u32, 1) << @truncate(4 + sm);
    }
    /// CTRL.CLKDIV_RESTART, bits **11:8** — self-clearing.
    ///
    /// These two were transposed here until 2026-08-01, and the only reason it
    /// never showed is that the sole caller sets both at once for the same
    /// state machine, so the union of the two masks was right. A caller wanting
    /// one of them would have got the other, silently.
    pub inline fn ctrlClkdivRestart(sm: u32) u32 {
        return @as(u32, 1) << @truncate(8 + sm);
    }

    /// FSTAT, one bit per SM in each byte-wide field.
    pub inline fn fstatRxEmpty(sm: u32) u32 {
        return @as(u32, 1) << @truncate(8 + sm);
    }
    pub inline fn fstatTxFull(sm: u32) u32 {
        return @as(u32, 1) << @truncate(16 + sm);
    }

    /// FDEBUG, write-1-to-clear, one **nibble** per field rather than a byte:
    /// RXSTALL 3:0, RXUNDER 11:8, TXOVER 19:16, TXSTALL 27:24.
    ///
    /// `TXSTALL` is the one worth reading on an output program: it is the
    /// difference between "the machine is not consuming" and "the machine is
    /// consuming faster than we feed it". Sticky, so it answers for the whole
    /// run rather than the instant it is sampled.
    pub inline fn fdebugTxstall(sm: u32) u32 {
        return @as(u32, 1) << @truncate(24 + sm);
    }
    pub inline fn fdebugTxover(sm: u32) u32 {
        return @as(u32, 1) << @truncate(16 + sm);
    }
    pub inline fn fdebugRxunder(sm: u32) u32 {
        return @as(u32, 1) << @truncate(8 + sm);
    }
    pub inline fn fdebugRxstall(sm: u32) u32 {
        return @as(u32, 1) << @truncate(sm);
    }

    /// FLEVEL is four bytes, one per SM: TX in the low nibble, RX in the high.
    /// A TX-joined machine reports 0-8 here and its RX as permanently empty.
    pub inline fn flevelTx(flevel_value: u32, sm: u32) u32 {
        return (flevel_value >> @truncate(sm * 8)) & 0xf;
    }
    pub inline fn flevelRx(flevel_value: u32, sm: u32) u32 {
        return (flevel_value >> @truncate(sm * 8 + 4)) & 0xf;
    }

    /// CLKDIV is 16.8 fixed point: INT in 31:16, FRAC in 15:8.
    /// INT=1, FRAC=0 is one instruction per clk_sys cycle.
    pub const clkdiv_by_1: u32 = 1 << 16;

    // SM_SHIFTCTRL — datasheet field table.
    pub const shift_in_count_shift: u5 = 0;
    pub const shift_fjoin_rx_get: u32 = 1 << 14;
    pub const shift_fjoin_rx_put: u32 = 1 << 15;
    pub const shift_autopush: u32 = 1 << 16;
    pub const shift_autopull: u32 = 1 << 17;
    /// 1 = shift right. **Both default to 1**, which is the opposite of what a
    /// parallel-output program usually wants.
    pub const shift_in_shiftdir_right: u32 = 1 << 18;
    pub const shift_out_shiftdir_right: u32 = 1 << 19;
    pub const shift_push_thresh_shift: u5 = 20;
    pub const shift_pull_thresh_shift: u5 = 25;
    pub const shift_fjoin_tx: u32 = 1 << 30;
    pub const shift_fjoin_rx: u32 = 1 << 31;

    // SM_PINCTRL.
    pub const pin_out_base_shift: u5 = 0;
    pub const pin_set_base_shift: u5 = 5;
    pub const pin_sideset_base_shift: u5 = 10;
    pub const pin_in_base_shift: u5 = 15;
    /// 6 bits, so an OUT can drive up to 32 pins — which is what an 18-bit
    /// RGB666 bus needs in one instruction.
    pub const pin_out_count_shift: u5 = 20;
    pub const pin_set_count_shift: u5 = 26;
    pub const pin_sideset_count_shift: u5 = 29;

    // SM_EXECCTRL — only the wrap fields are needed so far.
    pub const exec_wrap_bottom_shift: u5 = 7;
    pub const exec_wrap_top_shift: u5 = 12;
    pub const exec_side_pindir: u32 = 1 << 29;
    pub const exec_side_en: u32 = 1 << 30;
};

// ---------------------------------------------------------------------------
// PLL_SYS
// ---------------------------------------------------------------------------

/// The two PLLs. **`pll_sys` and `pll_usb` are the same block at different
/// bases**, so one set of field definitions serves both and `Regs` below names
/// the two instances. `pll_usb` was added 2026-08-29 for `clk_usb`, which USB
/// requires to be exactly 48 MHz.
///
/// Unlike XOSC's CTRL (above) there are **no magic codes and no BADWRITE flag**
/// here: a nonsensical divider is written, accepted, and produces a nonsensical
/// clock. LOCK is the only feedback, and it reports "the loop closed", not "the
/// frequency you meant". That is why `clocks.Plan` does its arithmetic at
/// comptime and asserts the VCO range rather than trusting the hardware to
/// object.
pub const Pll = struct {
    pub const sys_cs: u32 = pll_sys_base + 0x00;
    pub const sys_pwr: u32 = pll_sys_base + 0x04;
    pub const sys_fbdiv_int: u32 = pll_sys_base + 0x08;
    pub const sys_prim: u32 = pll_sys_base + 0x0c;

    pub const usb_cs: u32 = pll_usb_base + 0x00;
    pub const usb_pwr: u32 = pll_usb_base + 0x04;
    pub const usb_fbdiv_int: u32 = pll_usb_base + 0x08;
    pub const usb_prim: u32 = pll_usb_base + 0x0c;

    /// CS.LOCK is read-only and set once the loop has settled.
    pub const cs_lock: u32 = 1 << 31;
    /// CS.BYPASS passes the reference straight through, ignoring the loop.
    pub const cs_bypass: u32 = 1 << 8;
    /// CS.REFDIV, bits 5:0.
    pub const cs_refdiv_mask: u32 = 0x3f;

    /// PWR bits are **active-low power-downs**: a set bit powers that part off.
    /// Reset value is all of them set, so bringing the PLL up is mostly a
    /// matter of clearing bits in the right order.
    ///
    /// DSMPD stays set: it powers down the delta-sigma modulator, which is what
    /// fractional feedback needs. We only ever use integer FBDIV, and a live
    /// modulator on an integer ratio adds jitter for nothing. A vendor's stock
    /// firmware agrees — its PWR reads 0x4, DSMPD alone.
    pub const pwr_pd: u32 = 1 << 0;
    pub const pwr_dsmpd: u32 = 1 << 2;
    pub const pwr_postdivpd: u32 = 1 << 3;
    pub const pwr_vcopd: u32 = 1 << 5;

    /// PRIM.POSTDIV1 (bits 18:16) and POSTDIV2 (bits 14:12).
    pub const prim_postdiv1_shift: u5 = 16;
    pub const prim_postdiv2_shift: u5 = 12;

    pub inline fn prim(postdiv1: u32, postdiv2: u32) u32 {
        return (postdiv1 << prim_postdiv1_shift) | (postdiv2 << prim_postdiv2_shift);
    }

    /// One PLL's four registers and its RESETS bit, so the bring-up sequence in
    /// `clocks.startPll` can be written once and applied to either.
    ///
    /// **The sequence is the reason this exists rather than a second copy.**
    /// Its ordering is load-bearing in ways the hardware does not report — see
    /// the comment on `startPll` — and a duplicate would be a second place for
    /// that order to rot.
    pub const Regs = struct {
        cs: u32,
        pwr: u32,
        fbdiv_int: u32,
        prim: u32,
        reset_bit: u32,
    };

    pub const sys = Regs{
        .cs = sys_cs,
        .pwr = sys_pwr,
        .fbdiv_int = sys_fbdiv_int,
        .prim = sys_prim,
        .reset_bit = Resets.pll_sys,
    };

    pub const usb = Regs{
        .cs = usb_cs,
        .pwr = usb_pwr,
        .fbdiv_int = usb_fbdiv_int,
        .prim = usb_prim,
        .reset_bit = Resets.pll_usb,
    };
};

// ---------------------------------------------------------------------------
// CLOCKS
// ---------------------------------------------------------------------------

pub const Clocks = struct {
    pub const clk_ref_ctrl: u32 = clocks_base + 0x30;
    pub const clk_ref_div: u32 = clocks_base + 0x34;
    pub const clk_ref_selected: u32 = clocks_base + 0x38;
    pub const clk_sys_ctrl: u32 = clocks_base + 0x3c;
    pub const clk_sys_div: u32 = clocks_base + 0x40;
    pub const clk_sys_selected: u32 = clocks_base + 0x44;
    pub const clk_peri_ctrl: u32 = clocks_base + 0x48;
    pub const clk_hstx_ctrl: u32 = clocks_base + 0x54;
    pub const clk_usb_ctrl: u32 = clocks_base + 0x60;
    pub const clk_usb_div: u32 = clocks_base + 0x64;
    pub const clk_usb_selected: u32 = clocks_base + 0x68;
    pub const clk_peri_div: u32 = clocks_base + 0x4c;
    pub const clk_peri_selected: u32 = clocks_base + 0x50;
    pub const clk_adc_ctrl: u32 = clocks_base + 0x6c;
    pub const clk_adc_div: u32 = clocks_base + 0x70;

    /// CLK_REF_CTRL.SRC (bits 1:0), glitchless.
    pub const ref_src_rosc: u32 = 0x0;
    pub const ref_src_aux: u32 = 0x1;
    pub const ref_src_xosc: u32 = 0x2;
    pub const ref_src_lposc: u32 = 0x3;

    /// CLK_SYS_CTRL.SRC (bit 0), glitchless. Resets to 1 (aux = ROSC).
    pub const sys_src_ref: u32 = 0x0;
    pub const sys_src_aux: u32 = 0x1;

    /// The two SRC fields as masks, for the read-modify-write that changing SRC
    /// requires. **`sys_src_ref` is 0, so storing it bare also zeroes AUXSRC**
    /// — which on a cold boot swaps the running ROSC for a stopped PLL beneath
    /// a glitchless mux that has not finished handing over, and stops clk_sys
    /// permanently. See `clocks.parkOnCrystal`.
    pub const sys_src_mask: u32 = 0x1;
    pub const ref_src_mask: u32 = 0x3;

    /// CLK_SYS_CTRL.AUXSRC (bits 7:5). Only the PLL matters here; the rest of
    /// the mux (pll_usb, ROSC, XOSC, the two GPINs) is reachable but unused.
    ///
    /// **SRC and AUXSRC are not interchangeable.** SRC is the glitchless mux
    /// and may be switched live; AUXSRC selects what SRC=1 *means* and must be
    /// set while SRC is 0, because it is not glitchless. Switching AUXSRC under
    /// a running clk_sys is how you stop the core mid-instruction.
    pub const sys_auxsrc_pll_sys: u32 = 0x0 << 5;

    /// CLK_PERI_CTRL.AUXSRC (bits 7:5) + ENABLE (bit 11).
    pub const peri_auxsrc_clk_sys: u32 = 0x0 << 5;
    pub const peri_auxsrc_xosc: u32 = 0x4 << 5;
    pub const peri_enable: u32 = 1 << 11;

    /// CLK_USB_CTRL.AUXSRC (bits 7:5) + ENABLE (bit 11).
    ///
    /// **clk_usb has no glitchless SRC mux at all** — only AUXSRC, like clk_peri
    /// and unlike clk_sys. The datasheet says AUXSRC "will glitch when
    /// switching", so ENABLE must be cleared before it is changed. There is a
    /// CLK_USB_SELECTED, but with no SRC field it is hardwired to 0x1 and says
    /// nothing; ENABLED (bit 28, read-only) is the bit that reports reality.
    pub const usb_auxsrc_pll_usb: u32 = 0x0 << 5;
    pub const usb_auxsrc_pll_sys: u32 = 0x1 << 5;
    pub const usb_auxsrc_xosc: u32 = 0x3 << 5;
    pub const usb_auxsrc_mask: u32 = 0x7 << 5;
    pub const usb_enable: u32 = 1 << 11;
    pub const usb_kill: u32 = 1 << 10;
    /// ENABLED is read-only and reports whether the generator is actually running.
    pub const usb_enabled: u32 = 1 << 28;

    /// CLK_ADC_CTRL.AUXSRC (bits 7:5) + ENABLE (bit 11). XOSC is source 3 —
    /// the reason this project needs no PLL for the ADC; see adc.zig.
    pub const adc_auxsrc_pll_usb: u32 = 0x0 << 5;
    pub const adc_auxsrc_pll_sys: u32 = 0x1 << 5;
    pub const adc_auxsrc_xosc: u32 = 0x3 << 5;
    pub const adc_auxsrc_mask: u32 = 0x7 << 5;

    /// **`CLK_ADC_DIV.INT` is four bits wide and there is no fractional part**
    /// (bits 19:16). That is the whole reason a 150 MHz clk_sys cannot feed the
    /// ADC and 144 MHz can: 144/3 is exactly 48 MHz, and no integer divides 150
    /// to 48 at all.
    pub inline fn adcDiv(int: u4) u32 {
        return @as(u32, int) << 16;
    }
    pub const adc_enable: u32 = 1 << 11;

    /// The *_SELECTED registers are **one-hot on the SRC value**, not a copy of
    /// it — SRC=2 reads back as 0b100. Comparing them to the SRC value instead
    /// of `1 << src` spins forever on a mux that switched perfectly.
    pub inline fn selectedMask(src: u32) u32 {
        return @as(u32, 1) << @truncate(src);
    }

    /// DIV is 16.16 fixed point: INT in 31:16, FRAC in 15:0. INT=1, FRAC=0 is
    /// divide-by-one. Note INT=0 means max+1, not zero.
    ///
    /// **CLK_USB_DIV is not 16.16** — its INT is only bits 19:16 and there is no
    /// fractional part at all. The bit pattern for divide-by-one is the same
    /// `1 << 16`, which is why one constant serves both, but a *non-unity* USB
    /// divisor would need its own encoder. Nothing needs one: clk_usb must be
    /// exactly 48 MHz and the PLL delivers that directly.
    pub const div_by_1: u32 = 1 << 16;

    // -----------------------------------------------------------------------
    // FC0 — the frequency counter
    // -----------------------------------------------------------------------

    /// Counts a selected clock against clk_ref and reports the result in kHz.
    ///
    /// **This is the only way to find out what a clock is actually doing.**
    /// Everything else in this block reports what was *requested*: a divider
    /// reads back the value written to it whether or not the source behind it
    /// is running. FC0 measures, which makes it the right instrument for
    /// answering "did the PLL land on the frequency we planned" — a question
    /// LOCK does not answer.
    ///
    /// Offsets differ from RP2040's, like most of this chip. Taken from the
    /// RP2350 datasheet directly, not from an RP2040 header.
    pub const fc0_ref_khz: u32 = clocks_base + 0x8c;
    pub const fc0_min_khz: u32 = clocks_base + 0x90;
    pub const fc0_max_khz: u32 = clocks_base + 0x94;
    pub const fc0_delay: u32 = clocks_base + 0x98;
    pub const fc0_interval: u32 = clocks_base + 0x9c;
    /// Writing SRC is what starts a measurement. Writing 0 stops it.
    pub const fc0_src: u32 = clocks_base + 0xa0;
    pub const fc0_status: u32 = clocks_base + 0xa4;
    pub const fc0_result: u32 = clocks_base + 0xa8;

    pub const fc0_src_null: u32 = 0x00;
    pub const fc0_src_pll_sys: u32 = 0x01;
    pub const fc0_src_xosc: u32 = 0x05;
    pub const fc0_src_clk_ref: u32 = 0x08;
    pub const fc0_src_clk_sys: u32 = 0x09;
    pub const fc0_src_clk_peri: u32 = 0x0a;
    pub const fc0_src_clk_usb: u32 = 0x0b;
    pub const fc0_src_clk_adc: u32 = 0x0c;
    /// **The PLL's own output, before the clk_usb mux and divider.** Measuring
    /// this and `clk_usb` separately is how a PLL that locked on the wrong
    /// frequency is told from a mux pointed at the wrong source — two faults
    /// that look identical from `clk_usb` alone.
    pub const fc0_src_pll_usb: u32 = 0x02;

    pub const fc0_status_pass: u32 = 1 << 0;
    pub const fc0_status_done: u32 = 1 << 4;
    pub const fc0_status_running: u32 = 1 << 8;
    pub const fc0_status_waiting: u32 = 1 << 12;
    pub const fc0_status_fail: u32 = 1 << 16;
    pub const fc0_status_slow: u32 = 1 << 20;
    pub const fc0_status_fast: u32 = 1 << 24;
    pub const fc0_status_died: u32 = 1 << 28;

    /// RESULT is 25.5 fixed point: KHZ in 29:5, FRAC in 4:0.
    pub const fc0_result_khz_shift: u5 = 5;
};

// ---------------------------------------------------------------------------
// TICKS — tick generators (RP2350 only; RP2040 had no such block)
// ---------------------------------------------------------------------------
// A trap for anyone carrying over RP2040 habits: on RP2040 the system timer got
// its 1 us tick from the watchdog, and worked as soon as the watchdog was set
// up. On RP2350 there is a separate TICKS block feeding TIMER0/TIMER1, SysTick,
// the watchdog and the RISC-V timer, and **it is disabled at reset**. Until its
// generator is enabled, TIMER0 simply never counts — a hang in any delay loop
// built on it, with no other symptom.

pub const Ticks = struct {
    pub const proc0_ctrl: u32 = ticks_base + 0x00;
    pub const proc0_cycles: u32 = ticks_base + 0x04;
    pub const timer0_ctrl: u32 = ticks_base + 0x18;
    pub const timer0_cycles: u32 = ticks_base + 0x1c;
    pub const timer0_count: u32 = ticks_base + 0x20;
    pub const watchdog_ctrl: u32 = ticks_base + 0x30;
    pub const watchdog_cycles: u32 = ticks_base + 0x34;

    pub const ctrl_enable: u32 = 1 << 0;
    pub const ctrl_running: u32 = 1 << 1;
};

// ---------------------------------------------------------------------------
// TIMER0 — 64-bit microsecond system timer
// ---------------------------------------------------------------------------

pub const Timer0 = struct {
    pub const timehw: u32 = timer0_base + 0x00;
    pub const timelw: u32 = timer0_base + 0x04;
    /// Reading TIMELR latches TIMEHR, so a coherent 64-bit read is
    /// TIMELR-then-TIMEHR. The RAW pair has no such side effect and needs a
    /// retry loop instead.
    pub const timehr: u32 = timer0_base + 0x08;
    pub const timelr: u32 = timer0_base + 0x0c;
    pub const timerawh: u32 = timer0_base + 0x24;
    pub const timerawl: u32 = timer0_base + 0x28;

    /// The four alarms. **The compare is `ALARMx == TIMELR`, an equality test on
    /// the low 32 bits only** — §12.8. That is the trap in this block: arm an
    /// alarm for a time that has *just* passed and it does not fire late, it
    /// fires 71.6 minutes later when the counter next equals it. Every caller
    /// has to check whether the target went by while it was being written.
    pub fn alarm(index: u2) u32 {
        return timer0_base + 0x10 + @as(u32, index) * 4;
    }

    /// Writing an ALARM arms it; alarms disarm themselves on firing. Writing a
    /// 1 here disarms early without waiting.
    pub const armed: u32 = timer0_base + 0x20;

    pub const dbgpause: u32 = timer0_base + 0x2c;
    pub const pause: u32 = timer0_base + 0x30;
    pub const locked: u32 = timer0_base + 0x34;
    pub const source: u32 = timer0_base + 0x38;

    /// Raw interrupts — one bit per alarm. **Write 1 to clear**, which is how a
    /// fired alarm's latched interrupt is acknowledged.
    pub const intr: u32 = timer0_base + 0x3c;
    pub const inte: u32 = timer0_base + 0x40;
    pub const intf: u32 = timer0_base + 0x44;
    pub const ints: u32 = timer0_base + 0x48;
};

// ---------------------------------------------------------------------------
// ADC (12-bit SAR)
// ---------------------------------------------------------------------------

pub const Adc = struct {
    pub const cs: u32 = adc_base + 0x00;
    pub const result: u32 = adc_base + 0x04;
    pub const fcs: u32 = adc_base + 0x08;
    pub const fifo: u32 = adc_base + 0x0c;
    pub const div: u32 = adc_base + 0x10;

    pub const cs_en: u32 = 1 << 0;
    pub const cs_ts_en: u32 = 1 << 1; // temperature sensor power
    pub const cs_start_once: u32 = 1 << 2;
    pub const cs_start_many: u32 = 1 << 3;
    pub const cs_ready: u32 = 1 << 8;
    pub const cs_err: u32 = 1 << 9;
    pub const cs_err_sticky: u32 = 1 << 10;

    /// AINSEL occupies bits 15:12.
    pub inline fn csAinsel(ch: u4) u32 {
        return @as(u32, ch) << 12;
    }
    pub const cs_ainsel_mask: u32 = 0xf << 12;

    /// Round-robin channel mask, CS bits 24:16 — one bit per AINSEL value.
    ///
    /// **The channel AINSEL names is converted once even if its own bit is
    /// clear.** Datasheet §12.4.3.3 spells this out: with AINSEL 0 and RROBIN
    /// 0x06 the order is 0, 1, 2, 1, 2, 1, 2… So a stream's first sample can
    /// belong to a channel the caller never asked to be in the rotation.
    pub inline fn csRrobin(mask: u9) u32 {
        return @as(u32, mask) << 16;
    }
    pub const cs_rrobin_mask: u32 = 0x1ff << 16;

    // FIFO control and status, ADC_FCS.
    pub const fcs_en: u32 = 1 << 0;
    /// Right-shift results to 8 bits, for byte-wide DMA into a deeper buffer.
    pub const fcs_shift: u32 = 1 << 1;
    /// Put the per-sample conversion-error flag in `fifo_err`.
    pub const fcs_err: u32 = 1 << 2;
    pub const fcs_dreq_en: u32 = 1 << 3;
    pub const fcs_empty: u32 = 1 << 8;
    pub const fcs_full: u32 = 1 << 9;
    /// Sticky. Write 1 to clear. Set when a conversion completed with the FIFO
    /// full — the FIFO keeps its contents and the conversion is lost.
    pub const fcs_under: u32 = 1 << 10;
    pub const fcs_over: u32 = 1 << 11;

    /// Entries waiting, FCS bits 19:16.
    pub inline fn fcsLevel(v: u32) u4 {
        return @intCast((v >> 16) & 0xf);
    }
    /// DREQ and IRQ assert at level >= this. FCS bits 27:24.
    pub inline fn fcsThresh(n: u4) u32 {
        return @as(u32, n) << 24;
    }
    pub const fcs_thresh_mask: u32 = 0xf << 24;

    /// Set on a sample whose SAR failed to converge. **Discard that sample** —
    /// the datasheet puts this in a Caution box because the result is undefined
    /// rather than merely inaccurate.
    pub const fifo_err: u32 = 1 << 15;
    pub const fifo_val_mask: u32 = 0xfff;

    /// Pacing. `int` is cycles per conversion minus one, DIV bits 23:8, with a
    /// first-order delta-sigma fraction in bits 7:0. Zero free-runs, which is a
    /// conversion every 96 cycles.
    ///
    /// **The hardware ignores a trigger while a conversion is in progress**, so
    /// an `int` below 95 paces nothing and silently means free-running.
    pub inline fn divWord(int: u16, frac: u8) u32 {
        return (@as(u32, int) << 8) | frac;
    }
};

// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------

/// The power-on state machine.
///
/// **Only `wdsel` is here, and it is the register that decides whether the
/// watchdog does anything at all.** Datasheet §12.9: "The rst_wdsel register
/// determines which subsystems are reset when the watchdog is triggered", and
/// §7.4.3: "The watchdog can trigger a full or partial sequence by writing to
/// the WDSEL register." Left at its reset value of zero, a watchdog timeout
/// restarts *no* stage of the PSM — the counter expires, `reason` records it,
/// and the chip carries on running.
pub const Psm = struct {
    /// Force a PSM stage off. **This is how core 1 gets parked.**
    ///
    /// The bootrom leaves core 1 wherever it happened to stop, clocked and
    /// drawing current, and nothing in a single-core firmware ever tells it
    /// otherwise — a battery firmware's current budget found it unaccounted
    /// for. Holding `PROC1` off is one register write.
    pub const frce_off: u32 = psm_base + 0x04;
    pub const frce_off_proc1: u32 = 1 << 24;

    pub const wdsel: u32 = psm_base + 0x08;

    /// Every stage the SDK selects, which is all of them **except the two
    /// oscillators**. Resetting ROSC or XOSC would stop the clock the reset
    /// sequence itself needs; the SDK's comment is "Reset everything apart
    /// from ROSC and XOSC" and this is that expression, spelled out.
    pub const wdsel_all: u32 = 0x01ff_ffff;
    pub const wdsel_rosc: u32 = 0x0000_0004;
    pub const wdsel_xosc: u32 = 0x0000_0008;
    pub const wdsel_reboot: u32 = wdsel_all & ~(wdsel_rosc | wdsel_xosc);
};

pub const Watchdog = struct {
    pub const ctrl: u32 = watchdog_base + 0x00;
    pub const load: u32 = watchdog_base + 0x04;
    pub const reason: u32 = watchdog_base + 0x08;
    /// Eight scratch words that survive a watchdog reset — the bootrom uses
    /// some of them, so only the upper ones are safely ours.
    pub const scratch0: u32 = watchdog_base + 0x0c;

    /// Bits 23:0, and **read-only**: "TIME: Indicates the time in usec before a
    /// watchdog reset will be triggered" (§12.9). So `ctrl` read back gives the
    /// live countdown, which is the only way to see the watchdog actually
    /// counting — `load` is write-only in effect.
    pub const ctrl_time_mask: u32 = 0x00ff_ffff;

    // **These three were rotated by one place until 2026-08-25.** Bit 24 was
    // named `pause_dbg0`, 25 `pause_dbg1` and 26 `pause_jtag`; the datasheet's
    // CTRL table has 24 PAUSE_JTAG, 25 PAUSE_DBG0, 26 PAUSE_DBG1. It changed no
    // behaviour, because the only caller sets all three together — which is
    // exactly why a wrong name in a register map can sit for months.
    pub const ctrl_pause_jtag: u32 = 1 << 24;
    pub const ctrl_pause_dbg0: u32 = 1 << 25;
    pub const ctrl_pause_dbg1: u32 = 1 << 26;
    pub const ctrl_enable: u32 = 1 << 30;
    pub const ctrl_trigger: u32 = 1 << 31;

    pub const reason_timer: u32 = 1 << 0;
    pub const reason_force: u32 = 1 << 1;
};

// ---------------------------------------------------------------------------
// UART (ARM PL011)
// ---------------------------------------------------------------------------

/// The PL011 UART, at whichever base (uart0_base or uart1_base).
pub const Uart = struct {
    pub inline fn dr(b: u32) u32 {
        return b + 0x00;
    }
    pub inline fn fr(b: u32) u32 {
        return b + 0x18;
    }
    pub inline fn ibrd(b: u32) u32 {
        return b + 0x24;
    }
    pub inline fn fbrd(b: u32) u32 {
        return b + 0x28;
    }
    pub inline fn lcr_h(b: u32) u32 {
        return b + 0x2c;
    }
    pub inline fn cr(b: u32) u32 {
        return b + 0x30;
    }

    // Flag register
    pub const fr_cts: u32 = 1 << 0;
    pub const fr_busy: u32 = 1 << 3;
    pub const fr_rxfe: u32 = 1 << 4; // receive FIFO empty
    pub const fr_txff: u32 = 1 << 5; // transmit FIFO full
    pub const fr_rxff: u32 = 1 << 6;
    pub const fr_txfe: u32 = 1 << 7;

    // Line control
    pub const lcr_brk: u32 = 1 << 0;
    pub const lcr_pen: u32 = 1 << 1;
    pub const lcr_eps: u32 = 1 << 2;
    pub const lcr_stp2: u32 = 1 << 3;
    pub const lcr_fen: u32 = 1 << 4; // FIFO enable
    pub const lcr_wlen_8: u32 = 0b11 << 5;

    // Control
    pub const cr_uarten: u32 = 1 << 0;
    pub const cr_txe: u32 = 1 << 8;
    pub const cr_rxe: u32 = 1 << 9;
};

// ---------------------------------------------------------------------------
// TRNG (True Random Number Generator)
// ---------------------------------------------------------------------------

pub const Trng = struct {
    pub const imr: u32 = trng_base + 0x00;
    pub const isr: u32 = trng_base + 0x04;
    pub const icr: u32 = trng_base + 0x08;
    pub const config: u32 = trng_base + 0x0c;
    pub const valid: u32 = trng_base + 0x10;
    pub const ehr_data0: u32 = trng_base + 0x14;
    pub const ehr_data1: u32 = trng_base + 0x18;
    pub const ehr_data2: u32 = trng_base + 0x1c;
    pub const ehr_data3: u32 = trng_base + 0x20;
    pub const ehr_data4: u32 = trng_base + 0x24;
    pub const ehr_data5: u32 = trng_base + 0x28;
    pub const rnd_source_enable: u32 = trng_base + 0x2c;

    pub const valid_ehr_valid: u32 = 1 << 0;
    pub const rnd_source_enable_bit: u32 = 1 << 0;
};

// ---------------------------------------------------------------------------
// SPI (ARM PL022 SSP)
// ---------------------------------------------------------------------------

/// The PL022, at whichever base. **Offsets only** — pass a base in, so that the
/// same block describes SPI0 and SPI1 rather than existing twice.
///
/// It was `Spi0` with absolute addresses baked in until 2026-07-30, which was
/// right while only one instance was wired to anything. A display panel's
/// config port is on SPI1, so the second instance stopped being hypothetical.
pub const Spi = struct {
    pub inline fn cr0(b: u32) u32 {
        return b + 0x00;
    }
    pub inline fn cr1(b: u32) u32 {
        return b + 0x04;
    }
    pub inline fn dr(b: u32) u32 {
        return b + 0x08;
    }
    pub inline fn sr(b: u32) u32 {
        return b + 0x0c;
    }
    pub inline fn cpsr(b: u32) u32 {
        return b + 0x10;
    }
    pub inline fn imsc(b: u32) u32 {
        return b + 0x14;
    }
    pub inline fn icr(b: u32) u32 {
        return b + 0x20;
    }
    pub inline fn dmacr(b: u32) u32 {
        return b + 0x24;
    }

    // CR0
    /// DSS is bits 3:0 and holds **frame size minus one**, so 8-bit is 0b0111
    /// and 9-bit is 0b1000. Values below 3 (i.e. under 4 bits) are reserved.
    ///
    /// The off-by-one is the trap: writing 8 here gives 9-bit frames, and a
    /// device fed 9-bit frames when it expected 8 sees every byte shifted and
    /// answers nothing — with no error anywhere.
    ///
    /// `bits` is `u5` rather than `u4` precisely because of that off-by-one:
    /// the *field* is four bits, but the frame size it encodes runs to 16.
    pub inline fn cr0Dss(bits: u5) u32 {
        return @as(u32, bits) - 1;
    }
    pub const cr0_dss_8bit: u32 = 0b0111; // bits 3:0
    pub const cr0_frf_motorola: u32 = 0b00 << 4;
    pub const cr0_spo: u32 = 1 << 6; // clock polarity  (CPOL)
    pub const cr0_sph: u32 = 1 << 7; // clock phase     (CPHA)
    pub inline fn cr0Scr(scr: u32) u32 {
        return (scr & 0xff) << 8;
    }

    // CR1
    pub const cr1_lbm: u32 = 1 << 0; // internal loopback: TX tied to RX
    pub const cr1_sse: u32 = 1 << 1; // enable
    pub const cr1_ms: u32 = 1 << 2; // 0 = master
    pub const cr1_sod: u32 = 1 << 3;

    // SR
    pub const sr_tfe: u32 = 1 << 0; // transmit FIFO empty
    pub const sr_tnf: u32 = 1 << 1; // transmit FIFO not full
    pub const sr_rne: u32 = 1 << 2; // receive FIFO not empty
    pub const sr_rff: u32 = 1 << 3;
    pub const sr_bsy: u32 = 1 << 4;
};

// ---------------------------------------------------------------------------
// USB (device mode)
// ---------------------------------------------------------------------------

/// The USB 2.0 Full Speed controller, device side. Datasheet §12.7.
///
/// **Only the device half is described here.** The block does host mode too and
/// deliberately none of it is transcribed: `lib/usb` is device-only, and host
/// mode is not planned. A register named `Only valid for HOST mode` in the
/// datasheet is absent below rather than merely unused.
///
/// Two things about this block are unlike anything else in this file.
///
/// **It has a 4 kB dual-port RAM, and the layout of that RAM is the API.**
/// Endpoint configuration is not a register bank — it is a structure the
/// programmer writes into DPRAM at fixed offsets, with the data buffers carved
/// out of whatever is left. `Dpram` below is that layout.
///
/// **Instruction fetch from DPRAM unconditionally bus-faults**, whatever the
/// MPU/SAU/ACCESSCTRL say (§12.7.1.1.3). Nothing here would want to, but a
/// stray function pointer into this region faults rather than misbehaving,
/// which is at least a loud failure.
pub const Usb = struct {
    pub const regs_base: u32 = 0x5011_0000;
    pub const dpram_base: u32 = 0x5010_0000;
    pub const dpram_size: u32 = 4096;

    // -- Registers, §12.7.5 -------------------------------------------------

    pub const addr_endp: u32 = regs_base + 0x00;
    pub const main_ctrl: u32 = regs_base + 0x40;
    pub const sie_ctrl: u32 = regs_base + 0x4c;
    pub const sie_status: u32 = regs_base + 0x50;
    pub const buff_status: u32 = regs_base + 0x58;
    pub const ep_abort: u32 = regs_base + 0x60;
    pub const ep_abort_done: u32 = regs_base + 0x64;
    pub const ep_stall_arm: u32 = regs_base + 0x68;
    pub const ep_status_stall_nak: u32 = regs_base + 0x70;
    pub const muxing: u32 = regs_base + 0x74;
    pub const pwr: u32 = regs_base + 0x78;
    pub const intr: u32 = regs_base + 0x8c;
    pub const inte: u32 = regs_base + 0x90;
    pub const ints: u32 = regs_base + 0x98;

    /// ADDR_ENDP: the address the device answers to, once the host has assigned
    /// one. **Zero after a bus reset, and that is not a detail** — a device
    /// answers on address 0 until `SET_ADDRESS`, and the assignment must be
    /// written only *after* the status stage of that request completes.
    pub const addr_mask: u32 = 0x7f;
    pub const addr_endp_shift: u5 = 16;

    /// MAIN_CTRL.
    ///
    /// **`phy_iso` resets to 1 and must be cleared before the PHY will work.**
    /// This is new on RP2350 and §12.7.2 names it as the single exception to
    /// RP2040 software compatibility — so every RP2040 example, and every
    /// snippet derived from one, omits it. Forgetting it presents as no
    /// enumeration, no error and no clue.
    pub const main_controller_en: u32 = 1 << 0;
    pub const main_host_ndevice: u32 = 1 << 1;
    pub const main_phy_iso: u32 = 1 << 2;

    /// SIE_CTRL, device-relevant bits only.
    ///
    /// **`pulldown_en` resets to 1** on RP2350 (it matches the PHY's isolation
    /// latches) and is a *host* mode control. A device wants the pull-*up*.
    pub const sie_ep0_int_1buf: u32 = 1 << 29;
    pub const sie_ep0_int_stall: u32 = 1 << 31;
    pub const sie_ep0_double_buf: u32 = 1 << 30;
    pub const sie_ep0_stop_on_short_packet: u32 = 1 << 19;
    pub const sie_transceiver_pd: u32 = 1 << 18;
    pub const sie_pullup_en: u32 = 1 << 16;
    pub const sie_pulldown_en: u32 = 1 << 15;
    pub const sie_resume: u32 = 1 << 12;

    /// SIE_STATUS. **These are write-1-to-clear**, so clearing one means writing
    /// exactly that bit, never a read-modify-write — which would clear every
    /// other set bit as a side effect.
    pub const status_data_seq_error: u32 = 1 << 31;
    pub const status_ack_rec: u32 = 1 << 30;
    pub const status_rx_timeout: u32 = 1 << 27;
    pub const status_rx_overflow: u32 = 1 << 26;
    pub const status_bit_stuff_error: u32 = 1 << 25;
    pub const status_crc_error: u32 = 1 << 24;
    pub const status_endpoint_error: u32 = 1 << 23;
    pub const status_bus_reset: u32 = 1 << 19;
    pub const status_trans_complete: u32 = 1 << 18;
    pub const status_setup_rec: u32 = 1 << 17;
    pub const status_connected: u32 = 1 << 16;
    pub const status_rx_short_packet: u32 = 1 << 12;
    pub const status_suspended: u32 = 1 << 4;
    pub const status_vbus_detected: u32 = 1 << 0;

    /// INTE / INTS / INTR share a layout. Only what a device stack arms.
    pub const int_ep_stall_nak: u32 = 1 << 19;
    pub const int_setup_req: u32 = 1 << 16;
    pub const int_dev_resume_from_host: u32 = 1 << 15;
    pub const int_dev_suspend: u32 = 1 << 14;
    pub const int_dev_conn_dis: u32 = 1 << 13;
    pub const int_bus_reset: u32 = 1 << 12;
    pub const int_error_data_seq: u32 = 1 << 5;
    pub const int_buff_status: u32 = 1 << 4;
    pub const int_trans_complete: u32 = 1 << 3;

    /// EP_STALL_ARM. **EP0 only** — bit 0 arms IN, bit 1 arms OUT. Every other
    /// endpoint stalls through its buffer control register alone. The
    /// controller clears this register itself when a SETUP arrives, because the
    /// specification requires a STALL to be cleared by the next setup packet.
    pub const ep_stall_arm_ep0_in: u32 = 1 << 0;
    pub const ep_stall_arm_ep0_out: u32 = 1 << 1;

    /// USB_MUXING. `to_phy` resets to 1 on RP2350; `softcon` is what the SDK's
    /// own example also sets, and both are written explicitly rather than
    /// relied upon.
    pub const muxing_to_phy: u32 = 1 << 0;
    pub const muxing_softcon: u32 = 1 << 3;
    pub const muxing_usbphy_as_gpio: u32 = 1 << 4;

    /// USB_PWR. A board whose VBUS is not wired to a GPIO has to *tell* the
    /// controller it is plugged in, by overriding the detect signal. Both bits
    /// are needed: the value, and the enable that makes the value count.
    pub const pwr_vbus_detect: u32 = 1 << 2;
    pub const pwr_vbus_detect_override_en: u32 = 1 << 3;

    // -- DPRAM, §12.7.1.1.3 -------------------------------------------------

    /// Which way a transfer goes, named from the **host's** point of view,
    /// because that is how USB names it and disagreeing would be worse.
    ///
    ///  * `in`  — device to host. What the device *sends*.
    ///  * `out` — host to device. What the device *receives*.
    pub const Dir = enum { in, out };

    /// The DPRAM layout. Offsets are from `dpram_base`.
    ///
    /// ```
    /// 0x000  setup packet, 8 bytes — written by the controller, read by us
    /// 0x008  endpoint control, EP1..EP15, {in, out} — 8 bytes per endpoint
    /// 0x080  buffer control,  EP0..EP15, {in, out} — 8 bytes per endpoint
    /// 0x100  EP0 IN buffer,  64 bytes  (fixed; single-buffered EP0 assumed)
    /// 0x140  EP0 OUT buffer, 64 bytes  (fixed)
    /// 0x180  free — 3712 bytes to carve into endpoint buffers
    /// ```
    ///
    /// **The two control arrays do not start at the same endpoint, and that is
    /// the single easiest mistake to make here.** Endpoint control begins at
    /// **EP1**, because EP0's configuration is implicit and it has no such
    /// register; buffer control begins at **EP0**, because EP0 very much has
    /// buffers. An index shared between the two is wrong for one of them.
    pub const Dpram = struct {
        pub const setup_packet: u32 = dpram_base + 0x00;
        pub const setup_packet_len: u32 = 8;

        pub const ep_ctrl_base: u32 = dpram_base + 0x08;
        pub const buf_ctrl_base: u32 = dpram_base + 0x80;
        pub const ep0_buf_in: u32 = dpram_base + 0x100;
        pub const ep0_buf_out: u32 = dpram_base + 0x140;
        pub const ep0_buf_size: u32 = 64;

        /// First offset a driver may allocate buffers from, and the space left.
        pub const free_offset: u32 = 0x180;
        pub const free_bytes: u32 = dpram_size - free_offset;

        /// The endpoint control register for `ep`, which **must not be 0**:
        /// EP0 has no endpoint control register at all.
        pub fn epControl(ep: u4, dir: Dir) u32 {
            std.debug.assert(ep != 0);
            const slot: u32 = @as(u32, ep) - 1;
            return ep_ctrl_base + slot * 8 + @as(u32, if (dir == .in) 0 else 4);
        }

        /// The buffer control register for `ep`. Valid for EP0.
        pub fn bufControl(ep: u4, dir: Dir) u32 {
            return buf_ctrl_base + @as(u32, ep) * 8 + @as(u32, if (dir == .in) 0 else 4);
        }
    };

    /// Endpoint control register fields. The low 16 bits are the buffer's
    /// **offset within DPRAM**, not its address — see `bufferOffset`.
    pub const ep_enable: u32 = 1 << 31;
    pub const ep_double_buffered: u32 = 1 << 30;
    pub const ep_interrupt_per_buffer: u32 = 1 << 29;
    pub const ep_interrupt_per_double_buffer: u32 = 1 << 28;
    pub const ep_interrupt_on_stall: u32 = 1 << 17;
    pub const ep_interrupt_on_nak: u32 = 1 << 16;
    pub const ep_buffer_type_shift: u5 = 26;

    /// Transfer type, in the USB specification's own encoding — which is also
    /// what an endpoint descriptor's `bmAttributes` carries, so the two need no
    /// translation between them.
    pub const TransferType = enum(u2) { control = 0, isochronous = 1, bulk = 2, interrupt = 3 };

    /// A DPRAM address as the offset an endpoint control register wants.
    ///
    /// **Buffers must be 64-byte aligned**, and an unaligned one is accepted
    /// silently by hardware that then reads and writes somewhere else.
    pub fn bufferOffset(addr: u32) u32 {
        std.debug.assert(addr >= dpram_base and addr < dpram_base + dpram_size);
        std.debug.assert((addr - dpram_base) % 64 == 0);
        return addr - dpram_base;
    }

    pub inline fn epControlWord(
        buf: u32,
        transfer: TransferType,
        extra: u32,
    ) u32 {
        return ep_enable | ep_interrupt_per_buffer |
            (@as(u32, @backingInt(transfer)) << ep_buffer_type_shift) |
            bufferOffset(buf) | extra;
    }

    /// Buffer control fields.
    ///
    /// **One 32-bit register holds two buffers**: buffer 0 in bits 15:0 and
    /// buffer 1 in 31:16, for double buffering. Everything below describes
    /// buffer 0; shift by `buf_ctrl_buf1_shift` for the other. A single-buffered
    /// endpoint uses only the low half, and writing a bare 32-bit value is
    /// therefore also writing buffer 1's control — as zero, which is harmless
    /// only because AVAILABLE is then clear.
    pub const buf_ctrl_full: u32 = 1 << 15;
    pub const buf_ctrl_last: u32 = 1 << 14;
    pub const buf_ctrl_data1_pid: u32 = 1 << 13;
    pub const buf_ctrl_sel: u32 = 1 << 12;
    pub const buf_ctrl_stall: u32 = 1 << 11;
    pub const buf_ctrl_avail: u32 = 1 << 10;
    pub const buf_ctrl_len_mask: u32 = 0x3ff;
    pub const buf_ctrl_buf1_shift: u5 = 16;

    /// The BUFF_STATUS bit for an endpoint. Two bits per endpoint, IN then OUT,
    /// so EP0 IN is bit 0, EP0 OUT bit 1, EP1 IN bit 2, and so on.
    pub fn buffStatusBit(ep: u4, dir: Dir) u32 {
        const n = @as(u5, ep) * 2 + @as(u5, if (dir == .in) 0 else 1);
        return @as(u32, 1) << n;
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
// Address arithmetic only — no MMIO — so these run on the host
// (see build.zig's `pico-hw` test module). Worth having precisely because
// every bug this file can contain is silent: a wrong bank or a wrong
// offset produces a write that lands somewhere harmless and a pin that never
// moves.

const std = @import("std");

test "Sio.bank: pins 0-31 use the low register, 32-47 the HI sibling at +4" {
    const low = Sio.gpio_out_set;

    const led = Sio.bank(25, low); // GP25, user LED
    try std.testing.expectEqual(low, led.addr);
    try std.testing.expectEqual(@as(u32, 1) << 25, led.mask);

    const vbat = Sio.bank(43, low); // GP43, VBAT sense
    try std.testing.expectEqual(low + 4, vbat.addr);
    try std.testing.expectEqual(@as(u32, 1) << 11, vbat.mask); // 43 - 32

    const boot = Sio.bank(45, low); // GP45, BOOT button
    try std.testing.expectEqual(low + 4, boot.addr);
    try std.testing.expectEqual(@as(u32, 1) << 13, boot.mask);

    // The boundary, where an off-by-one would be easiest to write.
    try std.testing.expectEqual(low, Sio.bank(31, low).addr);
    try std.testing.expectEqual(@as(u32, 1) << 31, Sio.bank(31, low).mask);
    try std.testing.expectEqual(low + 4, Sio.bank(32, low).addr);
    try std.testing.expectEqual(@as(u32, 1), Sio.bank(32, low).mask);
}

test "Sio: every HI register sits exactly 4 bytes above its low counterpart" {
    // `bank()` hardcodes the +4 relationship, so a mistyped constant here would
    // silently break it for one register only. Assert the layout it assumes.
    const pairs = [_][2]u32{
        .{ Sio.gpio_in, Sio.gpio_hi_in },
        .{ Sio.gpio_out, Sio.gpio_hi_out },
        .{ Sio.gpio_out_set, Sio.gpio_hi_out_set },
        .{ Sio.gpio_out_clr, Sio.gpio_hi_out_clr },
        .{ Sio.gpio_out_xor, Sio.gpio_hi_out_xor },
        .{ Sio.gpio_oe, Sio.gpio_hi_oe },
        .{ Sio.gpio_oe_set, Sio.gpio_hi_oe_set },
        .{ Sio.gpio_oe_clr, Sio.gpio_hi_oe_clr },
        .{ Sio.gpio_oe_xor, Sio.gpio_hi_oe_xor },
    };
    for (pairs) |p| try std.testing.expectEqual(p[0] + 4, p[1]);
}

test "Sio: offsets are RP2350's, not RP2040's" {
    // The trap this file exists to document. On RP2040 these were 0x14 / 0x20;
    // using those values compiles and runs and moves no pins.
    try std.testing.expectEqual(sio_base + 0x18, Sio.gpio_out_set);
    try std.testing.expectEqual(sio_base + 0x30, Sio.gpio_oe);
    try std.testing.expectEqual(sio_base + 0x38, Sio.gpio_oe_set);
}

test "IoBank0 interrupt registers: addresses and the four-bits-per-GPIO layout" {
    // Base 0x40028000; the register listing puts INTR0 at 0x230, PROC0_INTE0
    // at 0x248 and PROC0_INTS0 at 0x278, each a six-word array.
    try std.testing.expectEqual(@as(u32, 0x4002_8230), IoBank0.intr(0));
    try std.testing.expectEqual(@as(u32, 0x4002_8244), IoBank0.intr(5));
    try std.testing.expectEqual(@as(u32, 0x4002_8248), IoBank0.proc0_inte(0));
    try std.testing.expectEqual(@as(u32, 0x4002_825c), IoBank0.proc0_inte(5));
    try std.testing.expectEqual(@as(u32, 0x4002_8278), IoBank0.proc0_ints(0));
    try std.testing.expectEqual(@as(u32, 0x4002_828c), IoBank0.proc0_ints(5));

    // **The bit order is LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH**, which is
    // not the order anyone guesses — the datasheet's own comment block spells
    // GPIO0 out as bits 0..3 in exactly that sequence.
    try std.testing.expectEqual(@as(u5, 0), IoBank0.irqBit(0, 0)); // GPIO0_LEVEL_LOW
    try std.testing.expectEqual(@as(u5, 1), IoBank0.irqBit(0, 1)); // GPIO0_LEVEL_HIGH
    try std.testing.expectEqual(@as(u5, 2), IoBank0.irqBit(0, 2)); // GPIO0_EDGE_LOW
    try std.testing.expectEqual(@as(u5, 3), IoBank0.irqBit(0, 3)); // GPIO0_EDGE_HIGH
    try std.testing.expectEqual(@as(u5, 4), IoBank0.irqBit(1, 0)); // GPIO1_LEVEL_LOW

    // Eight GPIOs per word, so the bit index restarts at every multiple of 8
    // while the word advances.
    try std.testing.expectEqual(@as(u3, 0), IoBank0.irqWord(7));
    try std.testing.expectEqual(@as(u5, 28), IoBank0.irqBit(7, 0));
    try std.testing.expectEqual(@as(u3, 1), IoBank0.irqWord(8));
    try std.testing.expectEqual(@as(u5, 0), IoBank0.irqBit(8, 0));

    // **The one this was written for.** GP24 is the CYW43/RM2 shared data and
    // host-wake pin, and the interrupt it needs is level-high: word 3, bit 1.
    try std.testing.expectEqual(@as(u3, 3), IoBank0.irqWord(24));
    try std.testing.expectEqual(@as(u32, 0x4002_8254), IoBank0.proc0_inte(IoBank0.irqWord(24)));
    try std.testing.expectEqual(@as(u5, 1), IoBank0.irqBit(24, 1));

    // The last GPIO on an RP2350B lands in the last word.
    try std.testing.expectEqual(@as(u3, 5), IoBank0.irqWord(47));
    try std.testing.expectEqual(@as(u5, 31), IoBank0.irqBit(47, 3));
}

test "IoBank0.ctrl / PadsBank0.pad: flat arrays across all 48 GPIOs" {
    // IO_BANK0 is STATUS/CTRL pairs; CTRL is the second word of each.
    try std.testing.expectEqual(@as(u32, 0x4002_8004), IoBank0.ctrl(0));
    try std.testing.expectEqual(@as(u32, 0x4002_80cc), IoBank0.ctrl(25));
    try std.testing.expectEqual(@as(u32, 0x4002_815c), IoBank0.ctrl(43));

    // PADS_BANK0 starts at +4 because offset 0 is VOLTAGE_SELECT. Unlike SIO
    // this array does NOT split into banks — pin 43 is just the 44th entry.
    try std.testing.expectEqual(@as(u32, 0x4003_8004), PadsBank0.pad(0));
    try std.testing.expectEqual(@as(u32, 0x4003_8068), PadsBank0.pad(25));
    try std.testing.expectEqual(@as(u32, 0x4003_80b0), PadsBank0.pad(43));
}

test "PadsBank0: bit layout matches datasheet, including RP2350-only ISO" {
    // Verified against the PADS_BANK0 GPIO0 register table in the RP2350
    // datasheet. ISO is the one with no RP2040 counterpart: it resets to 1 and
    // freezes the pad until cleared, so omitting it produces a pin that
    // configures cleanly and never moves.
    try std.testing.expectEqual(@as(u32, 1) << 0, PadsBank0.slewfast);
    try std.testing.expectEqual(@as(u32, 1) << 1, PadsBank0.schmitt);
    try std.testing.expectEqual(@as(u32, 1) << 2, PadsBank0.pde);
    try std.testing.expectEqual(@as(u32, 1) << 3, PadsBank0.pue);
    try std.testing.expectEqual(@as(u32, 1) << 4, PadsBank0.drive_4ma);
    try std.testing.expectEqual(@as(u32, 3) << 4, PadsBank0.drive_12ma);
    try std.testing.expectEqual(@as(u32, 1) << 6, PadsBank0.ie);
    try std.testing.expectEqual(@as(u32, 1) << 7, PadsBank0.od);
    try std.testing.expectEqual(@as(u32, 1) << 8, PadsBank0.iso);
}

test "Clocks.selectedMask: the SELECTED registers are one-hot, not a copy of SRC" {
    // The trap: CLK_REF_SELECTED reads 0b100 when SRC is 2. Waiting for it to
    // equal 2 spins forever on a mux that switched perfectly. Confirmed on
    // hardware: clk_ref on XOSC (SRC=2) reads 4, clk_sys on clk_ref (SRC=0)
    // reads 1.
    try std.testing.expectEqual(@as(u32, 0b100), Clocks.selectedMask(Clocks.ref_src_xosc));
    try std.testing.expectEqual(@as(u32, 0b001), Clocks.selectedMask(Clocks.sys_src_ref));
    try std.testing.expectEqual(@as(u32, 0b010), Clocks.selectedMask(Clocks.sys_src_aux));
}

test "Clocks / Xosc / Ticks / Timer0: offsets and codes match the datasheet" {
    try std.testing.expectEqual(@as(u32, 0x4001_0030), Clocks.clk_ref_ctrl);
    try std.testing.expectEqual(@as(u32, 0x4001_003c), Clocks.clk_sys_ctrl);
    try std.testing.expectEqual(@as(u32, 0x4001_0048), Clocks.clk_peri_ctrl);
    // 16.16 fixed point: INT in the high half.
    try std.testing.expectEqual(@as(u32, 0x0001_0000), Clocks.div_by_1);

    // CTRL is 12-bit magic codes, not bit flags — an invalid code is silently
    // ignored and only shows up as STATUS.BADWRITE.
    try std.testing.expectEqual(@as(u32, 0x00fa_b000), Xosc.enable);
    try std.testing.expectEqual(@as(u32, 0x0000_0aa0), Xosc.freq_range_1_15mhz);
    try std.testing.expectEqual(@as(u32, 1) << 31, Xosc.status_stable);
    try std.testing.expectEqual(@as(u32, 1) << 24, Xosc.status_badwrite);

    // TICKS is RP2350-only and disabled at reset; TIMER0 never counts without it.
    try std.testing.expectEqual(@as(u32, 0x4010_8018), Ticks.timer0_ctrl);
    try std.testing.expectEqual(@as(u32, 0x4010_801c), Ticks.timer0_cycles);

    try std.testing.expectEqual(@as(u32, 0x400b_0028), Timer0.timerawl);
    try std.testing.expectEqual(@as(u32, 0x400b_000c), Timer0.timelr);
}

test "Resets: bit positions match the RP2350 header" {
    try std.testing.expectEqual(@as(u32, 1) << 6, Resets.io_bank0);
    try std.testing.expectEqual(@as(u32, 1) << 9, Resets.pads_bank0);
    try std.testing.expectEqual(@as(u32, 1) << 18, Resets.spi0);
    try std.testing.expectEqual(@as(u32, 1) << 14, Resets.pll_sys);
}

test "USB register addresses are where the datasheet puts them" {
    // §12.7.5. Base 0x50110000, and the offsets read off the register list.
    // Every one of these is silent when wrong: a write lands on a neighbouring
    // register or on nothing, and the controller simply does not start.
    try std.testing.expectEqual(@as(u32, 0x5011_0000), Usb.addr_endp);
    try std.testing.expectEqual(@as(u32, 0x5011_0040), Usb.main_ctrl);
    try std.testing.expectEqual(@as(u32, 0x5011_004c), Usb.sie_ctrl);
    try std.testing.expectEqual(@as(u32, 0x5011_0050), Usb.sie_status);
    try std.testing.expectEqual(@as(u32, 0x5011_0058), Usb.buff_status);
    try std.testing.expectEqual(@as(u32, 0x5011_0060), Usb.ep_abort);
    try std.testing.expectEqual(@as(u32, 0x5011_0068), Usb.ep_stall_arm);
    try std.testing.expectEqual(@as(u32, 0x5011_0074), Usb.muxing);
    try std.testing.expectEqual(@as(u32, 0x5011_0078), Usb.pwr);
    try std.testing.expectEqual(@as(u32, 0x5011_008c), Usb.intr);
    try std.testing.expectEqual(@as(u32, 0x5011_0090), Usb.inte);
    try std.testing.expectEqual(@as(u32, 0x5011_0098), Usb.ints);
}

test "the DPRAM layout matches the SDK's structure exactly" {
    // The C SDK asserts `offsetof(usb_device_dpram_t, epx_data) == 0x180` and
    // `sizeof(...) == 4096`. Those two are reproduced here, plus the interior
    // offsets that get you there, so this layout is pinned against an
    // independent implementation rather than against a reading of the prose.
    try std.testing.expectEqual(@as(u32, 0x5010_0000), Usb.Dpram.setup_packet);
    try std.testing.expectEqual(@as(u32, 0x5010_0008), Usb.Dpram.ep_ctrl_base);
    try std.testing.expectEqual(@as(u32, 0x5010_0080), Usb.Dpram.buf_ctrl_base);
    try std.testing.expectEqual(@as(u32, 0x5010_0100), Usb.Dpram.ep0_buf_in);
    try std.testing.expectEqual(@as(u32, 0x5010_0140), Usb.Dpram.ep0_buf_out);
    try std.testing.expectEqual(@as(u32, 0x180), Usb.Dpram.free_offset);
    try std.testing.expectEqual(@as(u32, 3712), Usb.Dpram.free_bytes);
    try std.testing.expectEqual(@as(u32, 4096), Usb.dpram_size);
}

test "endpoint control starts at EP1 and buffer control starts at EP0" {
    // The trap this file's comment is about, asserted rather than described.
    // EP1's control register is the *first* slot, so it sits at the base — an
    // implementation that indexed by endpoint number would put it 8 bytes late
    // and configure EP2 while believing it configured EP1.
    try std.testing.expectEqual(Usb.Dpram.ep_ctrl_base, Usb.Dpram.epControl(1, .in));
    try std.testing.expectEqual(Usb.Dpram.ep_ctrl_base + 4, Usb.Dpram.epControl(1, .out));
    try std.testing.expectEqual(Usb.Dpram.ep_ctrl_base + 8, Usb.Dpram.epControl(2, .in));

    // Buffer control is indexed straight by endpoint number, EP0 included.
    try std.testing.expectEqual(Usb.Dpram.buf_ctrl_base, Usb.Dpram.bufControl(0, .in));
    try std.testing.expectEqual(Usb.Dpram.buf_ctrl_base + 4, Usb.Dpram.bufControl(0, .out));
    try std.testing.expectEqual(Usb.Dpram.buf_ctrl_base + 8, Usb.Dpram.bufControl(1, .in));

    // **The same endpoint lands at different offsets in the two arrays**, which
    // is the whole point: EP1 is slot 0 of one and slot 1 of the other.
    try std.testing.expect(
        Usb.Dpram.epControl(1, .in) - Usb.Dpram.ep_ctrl_base !=
            Usb.Dpram.bufControl(1, .in) - Usb.Dpram.buf_ctrl_base,
    );
}

test "neither control array runs into what follows it" {
    // EP15 OUT is the last entry of each. Endpoint control must stop before
    // buffer control begins, and buffer control before the EP0 buffers.
    const last_ep_ctrl = Usb.Dpram.epControl(15, .out);
    try std.testing.expect(last_ep_ctrl + 4 <= Usb.Dpram.buf_ctrl_base);
    try std.testing.expectEqual(Usb.Dpram.buf_ctrl_base, last_ep_ctrl + 4);

    const last_buf_ctrl = Usb.Dpram.bufControl(15, .out);
    try std.testing.expect(last_buf_ctrl + 4 <= Usb.Dpram.ep0_buf_in);
    try std.testing.expectEqual(Usb.Dpram.ep0_buf_in, last_buf_ctrl + 4);

    // And the fixed EP0 buffers end exactly where allocatable space starts.
    try std.testing.expectEqual(
        Usb.dpram_base + Usb.Dpram.free_offset,
        Usb.Dpram.ep0_buf_out + Usb.Dpram.ep0_buf_size,
    );
}

test "BUFF_STATUS is two bits per endpoint, IN before OUT" {
    try std.testing.expectEqual(@as(u32, 1 << 0), Usb.buffStatusBit(0, .in));
    try std.testing.expectEqual(@as(u32, 1 << 1), Usb.buffStatusBit(0, .out));
    try std.testing.expectEqual(@as(u32, 1 << 2), Usb.buffStatusBit(1, .in));
    try std.testing.expectEqual(@as(u32, 1 << 3), Usb.buffStatusBit(1, .out));
    try std.testing.expectEqual(@as(u32, 1 << 31), Usb.buffStatusBit(15, .out));

    // Every one of the 32 is distinct and accounted for — a sweep, because an
    // overlapping pair would mean one endpoint's completion clearing another's.
    var seen: u32 = 0;
    for (0..16) |i| {
        const ep: u4 = @intCast(i);
        for ([_]Usb.Dir{ .in, .out }) |d| {
            const bit = Usb.buffStatusBit(ep, d);
            try std.testing.expectEqual(@as(u32, 0), seen & bit);
            seen |= bit;
        }
    }
    try std.testing.expectEqual(~@as(u32, 0), seen);
}

test "an endpoint control word carries the buffer's DPRAM offset, not its address" {
    // The low 16 bits are an offset from dpram_base. Writing an absolute
    // address here would set a nonsensical offset and a scattering of the
    // flag bits above it, which is worse than either mistake alone.
    const buf = Usb.dpram_base + Usb.Dpram.free_offset;
    try std.testing.expectEqual(@as(u32, 0x180), Usb.bufferOffset(buf));

    const word = Usb.epControlWord(buf, .bulk, 0);
    try std.testing.expectEqual(@as(u32, 0x180), word & 0xffff);
    try std.testing.expect(word & Usb.ep_enable != 0);
    try std.testing.expect(word & Usb.ep_interrupt_per_buffer != 0);
    try std.testing.expectEqual(
        @as(u32, @backingInt(Usb.TransferType.bulk)),
        (word >> Usb.ep_buffer_type_shift) & 0x3,
    );
}

test "transfer types use the USB specification's own encoding" {
    // Which is also an endpoint descriptor's bmAttributes, so a descriptor's
    // field can be dropped straight into an endpoint control register with no
    // translation table in between. Pinned because a private encoding here
    // would work until the first descriptor was written from the same value.
    try std.testing.expectEqual(@as(u2, 0), @backingInt(Usb.TransferType.control));
    try std.testing.expectEqual(@as(u2, 1), @backingInt(Usb.TransferType.isochronous));
    try std.testing.expectEqual(@as(u2, 2), @backingInt(Usb.TransferType.bulk));
    try std.testing.expectEqual(@as(u2, 3), @backingInt(Usb.TransferType.interrupt));
}

test "the buffer control halves do not overlap" {
    // One 32-bit register, two buffers. Buffer 1's fields are buffer 0's
    // shifted up 16, so every flag must fit inside 16 bits or the two collide.
    const all = Usb.buf_ctrl_full | Usb.buf_ctrl_last | Usb.buf_ctrl_data1_pid |
        Usb.buf_ctrl_sel | Usb.buf_ctrl_stall | Usb.buf_ctrl_avail | Usb.buf_ctrl_len_mask;
    try std.testing.expectEqual(@as(u32, 0), all >> 16);
    try std.testing.expectEqual(@as(u5, 16), Usb.buf_ctrl_buf1_shift);
    // A 64-byte packet fits the length field with room to spare; the field is
    // 10 bits because an isochronous packet can be 1023.
    try std.testing.expect(64 <= Usb.buf_ctrl_len_mask);
    try std.testing.expectEqual(@as(u32, 1023), Usb.buf_ctrl_len_mask);
}

test "PHY_ISO is bit 2, and the RP2350-only bit that must be cleared" {
    // §12.7.2: the one documented break in RP2040 software compatibility.
    // Pinned by value because the symptom of getting it wrong is silence.
    try std.testing.expectEqual(@as(u32, 1 << 2), Usb.main_phy_iso);
    try std.testing.expectEqual(@as(u32, 1 << 0), Usb.main_controller_en);
    try std.testing.expectEqual(@as(u32, 1 << 1), Usb.main_host_ndevice);
    // Device mode is HOST_NDEVICE clear, so a device's MAIN_CTRL never has it.
    try std.testing.expectEqual(@as(u32, 0), Usb.main_controller_en & Usb.main_host_ndevice);
}
