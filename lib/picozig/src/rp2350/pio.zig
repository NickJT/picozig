//! PIO — programmable IO, and a comptime assembler for it.
//!
//! Added 2026-07-30 for a display driver whose RGB panel needs a pixel clock, sync
//! signals and an 18-bit data bus — none of which are peripherals on this chip.
//! They are a PIO program. Nothing here knows anything about that panel; this
//! is the chip's programmable IO and nothing more.
//!
//! ## Why there is an assembler in here
//!
//! Because the alternative is a table of hex literals, and a wrong bit in one
//! of those is a state machine that runs perfectly and does something else.
//! `pioasm` is a C++ tool from the SDK that emits a C header, which is not a
//! dependency this project is going to take on (see the workspace rule about
//! bare metal and Zig over the pico-sdk).
//!
//! So instructions are built by named functions returning `u16`, evaluated at
//! **comptime**, with the encoding tested against the datasheet's own examples.
//! A program is then a plain `[]const u16` any board file can hold. This is the
//! same bet as `spi.Divisors` and `clocks.Plan`: the part that fails silently
//! is arithmetic, so do it where it can be tested on a laptop.
//!
//! ## The encoding, in one place
//!
//! Every instruction is 16 bits: `[15:13]` opcode, `[12:8]` delay/side-set,
//! `[7:0]` operands. **The delay field is shared with side-set**, and how it
//! splits is set per state machine by `PINCTRL.SIDESET_COUNT`, not by the
//! instruction. That is the single most confusing thing about PIO: the same
//! 16-bit word means different things depending on a register elsewhere.
//!
//! ## RP2350 specifics
//!
//! Three blocks rather than two, and `GPIOBASE` selects which 32 GPIOs a block
//! can see. A block left at 0 cannot drive GP32-47 at all and will not say so
//! — see `hw.Pio`.

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// Which PIO block. Three on RP2350.
pub const Block = enum(u2) {
    pio0,
    pio1,
    pio2,

    pub fn base(self: Block) u32 {
        return switch (self) {
            .pio0 => hw.pio0_base,
            .pio1 => hw.pio1_base,
            .pio2 => hw.pio2_base,
        };
    }

    /// 0, 1 or 2 — for the places a block has to be counted rather than
    /// switched on, `nvic.pioIrq` being the one that matters: the six PIO
    /// vectors are consecutive, two per block.
    pub fn index(self: Block) u2 {
        return @backingInt(self);
    }

    /// The `IO_BANK0` FUNCSEL that hands a pin to this block. **A pin left on
    /// SIO is not driven by PIO at all**, and a state machine writing to it
    /// succeeds silently — the usual shape of a fault on this chip.
    pub fn funcsel(self: Block) u32 {
        return switch (self) {
            .pio0 => hw.IoBank0.func_pio0,
            .pio1 => hw.IoBank0.func_pio1,
            .pio2 => hw.IoBank0.func_pio2,
        };
    }

    pub fn resetBit(self: Block) u32 {
        return switch (self) {
            .pio0 => hw.Resets.pio0,
            .pio1 => hw.Resets.pio1,
            .pio2 => hw.Resets.pio2,
        };
    }
};

// ---------------------------------------------------------------------------
// Instruction encoding
// ---------------------------------------------------------------------------

/// Sources and destinations, as the datasheet numbers them.
pub const Src = enum(u3) { pins = 0, x = 1, y = 2, null_ = 3, status = 5, isr = 6, osr = 7 };
/// `out` destinations, datasheet §11.4.5. **`null_` discards the bits**, which
/// is the only way to move the output shift counter without touching anything
/// else — `mov osr, null` then `out null, 32` is how a machine's shift state is
/// reset to "empty and armed" between frames.
pub const Dest = enum(u3) { pins = 0, x = 1, y = 2, null_ = 3, pindirs = 4, pc = 5, isr = 6, exec = 7 };
/// `mov` destinations differ from `out`'s — a separate enum on purpose, because
/// the numbers overlap and mixing them compiles fine.
pub const MovDest = enum(u3) { pins = 0, x = 1, y = 2, pindirs = 3, exec = 4, pc = 5, isr = 6, osr = 7 };
pub const MovSrc = enum(u3) { pins = 0, x = 1, y = 2, null_ = 3, status = 5, isr = 6, osr = 7 };
/// The operation applied to a `mov` source. **`invert` and `reverse` are the
/// reason `mov` exists** for display work: `mov pins, ::isr` bit-reverses a
/// word on its way to the pins, which is exactly what a reversed panel
/// wiring needs.
pub const MovOp = enum(u2) { none = 0, invert = 1, reverse = 2 };
pub const JmpCond = enum(u3) {
    always = 0,
    x_zero = 1,
    x_dec = 2,
    y_zero = 3,
    y_dec = 4,
    x_ne_y = 5,
    pin = 6,
    osr_not_empty = 7,
};
pub const WaitSrc = enum(u2) { gpio = 0, pin = 1, irq = 2 };

/// `delay_or_sideset` occupies bits 12:8 and is **split between delay and
/// side-set according to `PINCTRL.SIDESET_COUNT`**. Callers building a
/// side-setting program pass the already-combined value; `sideSet` below does
/// the combining so the split is written down once.
fn encode(op: u3, delay_or_sideset: u5, operands: u8) u16 {
    return (@as(u16, op) << 13) | (@as(u16, delay_or_sideset) << 8) | operands;
}

/// Combine a side-set value and a delay into the shared 5-bit field.
///
/// `sideset_bits` must match `PINCTRL.SIDESET_COUNT` for the state machine this
/// program runs on, **including the extra bit if `SIDE_EN` is set**. Getting it
/// wrong silently reinterprets every delay in the program as a side-set value
/// and vice versa.
pub fn sideSet(comptime sideset_bits: u3, side: u32, delay: u32) u5 {
    const delay_bits: u3 = 5 - sideset_bits;
    std.debug.assert(side < (@as(u32, 1) << sideset_bits));
    std.debug.assert(delay < (@as(u32, 1) << delay_bits));
    return @truncate((side << delay_bits) | delay);
}

pub fn jmp(cond: JmpCond, addr: u5, ds: u5) u16 {
    return encode(0b000, ds, (@as(u8, @backingInt(cond)) << 5) | addr);
}

pub fn wait(polarity: u1, src: WaitSrc, index: u5, ds: u5) u16 {
    return encode(0b001, ds, (@as(u8, polarity) << 7) | (@as(u8, @backingInt(src)) << 5) | index);
}

/// `count` is 1..32, encoded as 0..31 with 32 written as 0.
pub fn in(src: Src, count: u6, ds: u5) u16 {
    return encode(0b010, ds, (@as(u8, @backingInt(src)) << 5) | @as(u8, @truncate(count & 0x1f)));
}

pub fn out(dest: Dest, count: u6, ds: u5) u16 {
    return encode(0b011, ds, (@as(u8, @backingInt(dest)) << 5) | @as(u8, @truncate(count & 0x1f)));
}

pub fn push(if_full: bool, block: bool, ds: u5) u16 {
    const operands: u8 = (@as(u8, @intFromBool(if_full)) << 6) | (@as(u8, @intFromBool(block)) << 5);
    return encode(0b100, ds, operands);
}

pub fn pull(if_empty: bool, block: bool, ds: u5) u16 {
    const operands: u8 = 0x80 | (@as(u8, @intFromBool(if_empty)) << 6) | (@as(u8, @intFromBool(block)) << 5);
    return encode(0b100, ds, operands);
}

pub fn mov(dest: MovDest, op: MovOp, src: MovSrc, ds: u5) u16 {
    const operands: u8 = (@as(u8, @backingInt(dest)) << 5) |
        (@as(u8, @backingInt(op)) << 3) |
        @as(u8, @backingInt(src));
    return encode(0b101, ds, operands);
}

pub fn irqSet(index: u3, wait_for_clear: bool, ds: u5) u16 {
    const operands: u8 = (@as(u8, @intFromBool(wait_for_clear)) << 5) | index;
    return encode(0b110, ds, operands);
}

pub fn set(dest: Dest, value: u5, ds: u5) u16 {
    return encode(0b111, ds, (@as(u8, @backingInt(dest)) << 5) | value);
}

/// `nop` is `mov y, y` — it has no encoding of its own. Worth a named function
/// because a program full of `mov y, y` reads as a mistake.
pub fn nop(ds: u5) u16 {
    return mov(.y, .none, .y, ds);
}

// ---------------------------------------------------------------------------
// State machine configuration
// ---------------------------------------------------------------------------

pub const ShiftDir = enum { left, right };

/// Which FIFO the other one's four entries are donated to.
///
/// A state machine that only transmits has a useless 4-entry RX FIFO, and
/// joining it to TX doubles the depth to 8. **That is not a micro-optimisation
/// for a scan-out program** — it halves how often the CPU or DMA must top the
/// FIFO up, and an underrun there is a visibly torn line rather than an error.
pub const FifoJoin = enum { none, tx, rx };

/// A whole-number clock divisor, in the 16.8 fixed-point form `SM_CLKDIV`
/// wants. Fractional division adds jitter to every instruction, so anything
/// generating a pixel clock wants an integer and should say so.
pub fn clkdivInt(div: u32) u32 {
    return div << 16;
}

pub const SmConfig = struct {
    /// Where in instruction memory the program was loaded.
    wrap_bottom: u5 = 0,
    wrap_top: u5 = 31,

    out_base: u5 = 0,
    out_count: u6 = 0,
    set_base: u5 = 0,
    set_count: u3 = 0,
    sideset_base: u5 = 0,
    sideset_count: u3 = 0,
    in_base: u5 = 0,

    /// **Both shift directions default to right in hardware**, which is rarely
    /// what a parallel-output program wants. Stated explicitly here so the
    /// default is visible rather than inherited.
    out_shiftdir: ShiftDir = .right,
    in_shiftdir: ShiftDir = .right,
    autopull: bool = false,
    autopush: bool = false,
    /// Bits shifted before auto-pull/push fires. 32 is encoded as 0.
    pull_threshold: u6 = 32,
    push_threshold: u6 = 32,
    fifo_join: FifoJoin = .none,

    /// clk_sys cycles per instruction, 16.8 fixed point. `hw.Pio.clkdiv_by_1`
    /// runs one instruction per cycle.
    clkdiv: u32 = hw.Pio.clkdiv_by_1,

    pub fn pinctrl(self: SmConfig) u32 {
        return (@as(u32, self.out_base) << hw.Pio.pin_out_base_shift) |
            (@as(u32, self.set_base) << hw.Pio.pin_set_base_shift) |
            (@as(u32, self.sideset_base) << hw.Pio.pin_sideset_base_shift) |
            (@as(u32, self.in_base) << hw.Pio.pin_in_base_shift) |
            (@as(u32, self.out_count & 0x3f) << hw.Pio.pin_out_count_shift) |
            (@as(u32, self.set_count) << hw.Pio.pin_set_count_shift) |
            (@as(u32, self.sideset_count) << hw.Pio.pin_sideset_count_shift);
    }

    pub fn shiftctrl(self: SmConfig) u32 {
        var w: u32 = 0;
        if (self.out_shiftdir == .right) w |= hw.Pio.shift_out_shiftdir_right;
        if (self.in_shiftdir == .right) w |= hw.Pio.shift_in_shiftdir_right;
        if (self.autopull) w |= hw.Pio.shift_autopull;
        if (self.autopush) w |= hw.Pio.shift_autopush;
        switch (self.fifo_join) {
            .none => {},
            .tx => w |= hw.Pio.shift_fjoin_tx,
            .rx => w |= hw.Pio.shift_fjoin_rx,
        }
        // A threshold of 32 is written as 0 — the field is 5 bits.
        w |= (@as(u32, self.pull_threshold & 0x1f)) << hw.Pio.shift_pull_thresh_shift;
        w |= (@as(u32, self.push_threshold & 0x1f)) << hw.Pio.shift_push_thresh_shift;
        return w;
    }

    pub fn execctrl(self: SmConfig) u32 {
        return (@as(u32, self.wrap_bottom) << hw.Pio.exec_wrap_bottom_shift) |
            (@as(u32, self.wrap_top) << hw.Pio.exec_wrap_top_shift);
    }
};

// ---------------------------------------------------------------------------
// Driving the hardware
// ---------------------------------------------------------------------------

/// Release a PIO block from reset.
pub fn init(block: Block) void {
    // **A reset cycle, not merely a release.** A PIO block carries four state
    // machine configurations, 32 words of shared instruction memory, per-pin
    // directions and eight interrupt flags — and a driver configures only the
    // parts it uses, so anything it does not touch is inherited from whatever
    // ran before. `unreset` on a block already out of reset is a no-op and
    // leaves all of it.
    //
    // That is not hypothetical: see `Resets.resetCycle`, which this cost a
    // session to find.
    hw.Resets.resetCycle(block.resetBit());

    // **And the reset does not clear instruction memory.** It is SRAM, it
    // survives, and a program that loads five instructions leaves the other
    // twenty-seven holding whatever ran last. On a display board that was the
    // vendor's parallel program, and the timing machine wandered into its
    // `wait 1 irq 4` at address 25 and parked there forever — waiting on an
    // interrupt from a state machine that does not exist in our image.
    //
    // Zero is `jmp 0`, which makes the safe default free: any stray jump into
    // unwritten memory returns to the start of the program instead of
    // executing a stranger's code. That is worth having whether or not the
    // stray jump is itself a bug.
    const base = block.base();
    for (0..hw.Pio.instr_mem_words) |i| {
        hw.write(base + hw.Pio.instrMem(@intCast(i)), 0);
    }
}

/// Load a program at `offset` in the block's shared instruction memory.
///
/// **A `jmp` inside the program is absolute**, so a program containing one can
/// only run at the offset it was assembled for. Everything here so far is
/// loaded at 0, which is also what `.origin 0` in Pimoroni's timing program
/// requires.
pub fn loadProgram(block: Block, offset: u5, program: []const u16) void {
    const base = block.base();
    for (program, 0..) |instr, i| {
        hw.write(base + hw.Pio.instrMem(offset + @as(u32, @intCast(i))), instr);
    }
}

/// Add `offset` to an instruction's jump target, if it has one.
///
/// **`jmp` is the only instruction with an address operand, and its address is
/// absolute.** A program containing one therefore only runs at the offset it
/// was assembled for — which is fine until a second program has to share the
/// same 32 words of instruction memory, at which point exactly one of them can
/// be at zero.
pub fn relocate(instr: u16, offset: u5) u16 {
    if (instr >> 13 != 0b000) return instr;
    const target: u16 = instr & 0x1f;
    // A program relocated past the end of instruction memory would wrap and
    // jump somewhere arbitrary, which is the silent version of this failure.
    std.debug.assert(target + offset < hw.Pio.instr_mem_words);
    return (instr & ~@as(u16, 0x1f)) | (target + offset);
}

/// Load a program at `offset`, relocating its `jmp` targets to match.
///
/// This is what the SDK's `pio_add_program` does and what `loadProgram` does
/// not. Use `loadProgram` for a program pinned to zero — one that declares
/// `.origin 0` — and this for anything sharing the block with
/// it.
pub fn loadProgramRelocated(block: Block, offset: u5, program: []const u16) void {
    const base = block.base();
    for (program, 0..) |instr, i| {
        hw.write(base + hw.Pio.instrMem(offset + @as(u32, @intCast(i))), relocate(instr, offset));
    }
}

/// The DMA request number that paces a channel feeding this state machine's TX
/// FIFO.
///
/// **A wrong DREQ does not fail, it just runs at the wrong rate** — usually
/// flat out, which overruns the FIFO and tears every line. The numbering is
/// eight per block with TX first: PIO0 TX0-3 are 0-3, its RX 4-7, PIO1 TX from
/// 8, PIO2 TX from 16.
pub inline fn txDreq(block: Block, sm: u32) u32 {
    return @as(u32, @backingInt(block)) * 8 + sm;
}

/// The DMA request number that paces a channel draining this state machine's
/// RX FIFO: the block's TX numbers, then its RX, four each — PIO0 RX0-3 are
/// 4-7, PIO1 RX from 12, PIO2 RX from 20.
pub inline fn rxDreq(block: Block, sm: u32) u32 {
    return @as(u32, @backingInt(block)) * 8 + 4 + sm;
}

/// Apply a configuration to a state machine. Does not start it.
pub fn configure(block: Block, sm: u32, cfg: SmConfig) void {
    const base = block.base();
    hw.write(base + hw.Pio.smClkdiv(sm), cfg.clkdiv);
    hw.write(base + hw.Pio.smExecctrl(sm), cfg.execctrl());
    hw.write(base + hw.Pio.smShiftctrl(sm), cfg.shiftctrl());
    hw.write(base + hw.Pio.smPinctrl(sm), cfg.pinctrl());
}

/// Set the state machine's program counter, by executing a `jmp` out of band.
///
/// `SM_INSTR` runs an instruction immediately without disturbing the program
/// counter's normal progress — which is how you seed a starting address, and
/// how the SDK does it too.
pub inline fn jumpTo(block: Block, sm: u32, addr: u5) void {
    hw.write(block.base() + hw.Pio.smInstr(sm), jmp(.always, addr, 0));
}

/// Run one instruction on a state machine, out of band.
///
/// `SM_INSTR` executes immediately without disturbing the program counter's
/// normal progress. This is how a machine is seeded before it starts — with an
/// `out` to load `y`, or a `set` to establish pin directions.
///
/// **It stalls if the instruction does**, so an `out` issued to a machine with
/// an empty FIFO and no autopull hangs here with no diagnostic. Push first.
pub inline fn exec(block: Block, sm: u32, instr: u16) void {
    hw.write(block.base() + hw.Pio.smInstr(sm), instr);
}

/// Set the direction of a run of consecutive pins, as the state machine sees
/// them.
///
/// **PIO owns the output enable of any pin muxed to it**, so a data bus left at
/// the reset direction is an input, and a program driving it drives nothing.
/// There is no register for this: directions are set by *executing* `set
/// pindirs` on the machine, which reaches only the 5 pins the SET mapping
/// currently names. So `PINCTRL` is repointed in chunks of at most five and
/// restored afterwards — the same dance the SDK does, and the reason a 16-pin
/// bus takes four passes.
pub fn setConsecutivePindirs(block: Block, sm: u32, base: u5, count: u6, as_output: bool) void {
    const pinctrl_addr = block.base() + hw.Pio.smPinctrl(sm);
    const saved = hw.read(pinctrl_addr);

    var done: u6 = 0;
    while (done < count) {
        const chunk: u6 = @min(@as(u6, 5), count - done);
        hw.write(pinctrl_addr, (@as(u32, chunk) << hw.Pio.pin_set_count_shift) |
            (@as(u32, base + done) << hw.Pio.pin_set_base_shift));
        exec(block, sm, set(.pindirs, if (as_output) 0x1f else 0, 0));
        done += chunk;
    }

    hw.write(pinctrl_addr, saved);
}

/// Clear a state machine's internal state: shift counters, the ISR, the delay
/// counter, the waiting-on-IRQ state and any latched instruction. The clock
/// divider counter is restarted alongside it, so a machine about to be enabled
/// begins its first cycle from a known phase.
///
/// **It does not clear the OSR contents.** §11.7 is explicit: `SM_RESTART`
/// sets the *output shift counter* to 32 — empty, nothing to shift — and leaves
/// the register itself holding whatever was last shifted in. So an OSR read
/// back from a machine that has not yet auto-pulled is stale data, unrelated to
/// anything in the FIFO, and cannot be used as evidence about the FIFO path.
pub fn restart(block: Block, sm: u32) void {
    const addr = block.base() + hw.Pio.ctrl;
    hw.setBits(addr, hw.Pio.ctrlSmRestart(sm) | hw.Pio.ctrlClkdivRestart(sm));
}

/// Empty both FIFOs, by toggling the join bit twice.
///
/// There is no "clear FIFO" register. Changing `SHIFTCTRL`'s join state
/// discards the contents, so writing it and writing it back is the documented
/// way to do this, and is what `pio_sm_clear_fifos` does. Ending on the
/// original value means it does not matter which join state the caller chose.
pub fn clearFifos(block: Block, sm: u32) void {
    const addr = block.base() + hw.Pio.smShiftctrl(sm);
    hw.xorBits(addr, hw.Pio.shift_fjoin_rx);
    hw.xorBits(addr, hw.Pio.shift_fjoin_rx);
}

/// Clear this machine's four sticky FIFO error flags. They are write-1-to-clear
/// and survive a block reset, so a run that does not clear them first is
/// reporting some earlier program's stalls as its own.
pub fn clearFifoDebug(block: Block, sm: u32) void {
    hw.write(block.base() + hw.Pio.fdebug, hw.Pio.fdebugTxstall(sm) |
        hw.Pio.fdebugTxover(sm) | hw.Pio.fdebugRxunder(sm) | hw.Pio.fdebugRxstall(sm));
}

/// Everything between `configure` and `setEnabled`: a machine emptied, reset
/// and parked at `initial_pc`, ready to run the instant it is enabled.
///
/// **This is `pio_sm_init` less the config write, in the SDK's order**, and the
/// ordering is deliberately not our own invention. It was, briefly, and is
/// worth recording as a caution: a version of this enabled the machine first and jumped second, on the theory
/// that an instruction written to `SMx_INSTR` on a *disabled* machine latches
/// because there is no clock to retire it on. The datasheet says the opposite,
/// twice:
///
///  - §11.7, `CTRL.SM_ENABLE`: "When disabled, a state machine will cease
///    executing instructions, **except those written directly to `SMx_INSTR` by
///    the system**."
///  - §11.5: "Delay cycles are ignored on instructions written to the `INSTR`
///    register, and execute immediately, **ignoring the state machine clock
///    divider**."
///
/// A `jmp` cannot stall in any case, so it cannot latch. `pio-check` had been
/// passing on hardware with this exact ordering the whole time the theory was
/// live, which is the cheapest disproof available and was not looked at.
///
/// **Prime the TX FIFO after this and before `setEnabled`**, not before — the
/// FIFO clear here would throw the priming away.
pub fn prepare(block: Block, sm: u32, initial_pc: u5) void {
    setEnabled(block, sm, false);
    clearFifos(block, sm);
    clearFifoDebug(block, sm);
    // Retires anything `setConsecutivePindirs` left behind, and puts the shift
    // counters and the divider phase in a known state.
    restart(block, sm);
    jumpTo(block, sm, initial_pc);
}

/// Enable the state machines in `mask` (bit n is SM n) on one clk_sys edge,
/// their fractional dividers restarted together.
///
/// **For machines whose clocks must stay in a fixed ratio** — a converter's
/// system clock from one machine and its frame clocks from another. Enabled
/// one at a time they start a few cycles apart, and the phase between them is
/// then whatever those cycles were.
pub inline fn startTogether(block: Block, mask: u4) void {
    var bits: u32 = 0;
    for (0..4) |sm| {
        if ((mask >> @intCast(sm)) & 1 != 0) bits |= hw.Pio.ctrlSmEnable(sm) | hw.Pio.ctrlClkdivRestart(sm);
    }
    hw.setBits(block.base() + hw.Pio.ctrl, bits);
}

pub inline fn setEnabled(block: Block, sm: u32, enabled: bool) void {
    const addr = block.base() + hw.Pio.ctrl;
    if (enabled) hw.setBits(addr, hw.Pio.ctrlSmEnable(sm)) else hw.clrBits(addr, hw.Pio.ctrlSmEnable(sm));
}

// ---------------------------------------------------------------------------
// Looking at a running state machine
// ---------------------------------------------------------------------------
//
// A PIO fault has no error path — the machine simply does the wrong thing, or
// nothing, in silence. These five reads are the whole instrument, and they are
// named here rather than open-coded so that a diagnostic in an example reads as
// a question about the hardware and not as address arithmetic.

/// The program counter. **The single most useful PIO register**: sampled twice
/// and unchanged, the machine is parked; sampled twice and different, it is
/// executing. No other read distinguishes those two.
pub inline fn programCounter(block: Block, sm: u32) u32 {
    return hw.read(block.base() + hw.Pio.smAddr(sm));
}

/// The instruction at the program counter for an idle machine, and the
/// *latched* instruction for a stalled one — so it must be read alongside
/// `execStalled` to know which of the two it is.
///
/// A state machine that was never enabled is therefore a free control: parked
/// at 0, it reads back `program[0]`, which is the only way to confirm that
/// `loadProgram` wrote what it meant to. `INSTR_MEM` itself is write-only and
/// reads as zero.
pub inline fn instruction(block: Block, sm: u32) u32 {
    return hw.read(block.base() + hw.Pio.smInstr(sm));
}

/// See `hw.Pio.execctrl_exec_stalled` for why this is narrower evidence than it
/// looks — on its own it does not separate a forced instruction that will not
/// complete from an `out exec` executee that is blocking.
pub inline fn execStalled(block: Block, sm: u32) bool {
    return hw.read(block.base() + hw.Pio.smExecctrl(sm)) & hw.Pio.execctrl_exec_stalled != 0;
}

/// Words currently in the TX FIFO: 0-4 normally, 0-8 with `FifoJoin.tx`.
pub inline fn txLevel(block: Block, sm: u32) u32 {
    return hw.Pio.flevelTx(hw.read(block.base() + hw.Pio.flevel), sm);
}

/// Whether the machine has *ever* stalled on an empty TX FIFO since the flag
/// was last cleared. Sticky, and the difference between "not consuming" and
/// "consuming faster than we feed it" — two failures that look identical in a
/// FIFO level sampled at one instant.
pub inline fn txStalled(block: Block, sm: u32) bool {
    return hw.read(block.base() + hw.Pio.fdebug) & hw.Pio.fdebugTxstall(sm) != 0;
}

/// The two register addresses a polled receive loop touches, resolved once.
///
/// **This exists because `read` resolves them every call, and at `ReleaseSmall`
/// the optimiser will not hoist that out of a loop.** Disassembled from
/// `lib/cyw43`'s gSPI receive path on 2026-08-24, the per-word body was ~25
/// instructions: a table lookup for the block base, a reload of the state
/// machine index from memory, a shift to rebuild the FIFO-empty mask, and two
/// stack spills — *and the spin loop repeated the lookup on every poll*. The
/// two APB reads that are the actual work were a minority of it.
///
/// That is not the optimiser being wrong. `ReleaseSmall` is asked to trade
/// speed for size and it did. The fix is to do the arithmetic once, in a value
/// the loop carries in registers, rather than to argue with it — and it is a
/// PIO concern rather than a driver one, which is why it lives here.
///
/// A polled loop like this is a bet that the body is shorter than a FIFO word
/// period. Check the bet with `rxOverflowed` rather than assuming it.
pub const RxPort = struct {
    fstat: u32,
    rxf: u32,
    empty_mask: u32,

    pub inline fn of(block: Block, sm: u32) RxPort {
        const b = block.base();
        return .{
            .fstat = b + hw.Pio.fstat,
            .rxf = b + hw.Pio.rxf(sm),
            .empty_mask = hw.Pio.fstatRxEmpty(sm),
        };
    }

    /// Blocking read, with nothing recomputed — bounded, and it reports.
    ///
    /// This is the innermost loop of `lib/cyw43`'s gSPI bus, and it is the wait
    /// a 2026-08-22 bring-up hang was sitting in: the PC resolved to `hw.read`
    /// and no further, because an unbounded spin leaves nothing to resolve.
    /// The bound costs an increment and a compare against a legitimate wait of
    /// single-digit polls; see `bounded.zig` on why it counts polls rather than
    /// microseconds.
    pub inline fn read(self: RxPort) bounded.Error!u32 {
        var s = bounded.Spin.init(bounded.fifo_polls);
        while (hw.read(self.fstat) & self.empty_mask != 0) try s.tick();
        return hw.read(self.rxf);
    }
};

/// Has this machine stalled on a full RX FIFO since `clearFifoDebug`?
///
/// **This is a stall, not an overflow, and the difference matters.** §11.7:
/// `RXSTALL` is set "during a blocking PUSH, or an IN with autopush enabled" —
/// in which case **the state machine waits and no data is lost** — and *also*
/// on a nonblocking PUSH to a full FIFO, which is the case where data is
/// dropped. A program using autopush, as every SPI-shaped one here does, only
/// ever sees the harmless meaning: the machine pauses mid-transfer, side-set
/// holds the clock where it is, and a synchronous bus simply gets a stretched
/// cycle.
///
/// So this is a **performance** signal, not a fault: it says the CPU was
/// briefly slower than the wire, so the transfer took longer than its nominal
/// rate. Treating it as data loss is a mistake this file made for exactly one
/// afternoon — it turned a benign condition into a fatal error and stopped an
/// application that was working.
pub inline fn rxStalled(block: Block, sm: u32) bool {
    return hw.read(block.base() + hw.Pio.fdebug) & hw.Pio.fdebugRxstall(sm) != 0;
}

pub inline fn rxEmpty(block: Block, sm: u32) bool {
    return hw.read(block.base() + hw.Pio.fstat) & hw.Pio.fstatRxEmpty(sm) != 0;
}

pub inline fn txFull(block: Block, sm: u32) bool {
    return hw.read(block.base() + hw.Pio.fstat) & hw.Pio.fstatTxFull(sm) != 0;
}

/// Blocking read from a state machine's RX FIFO, bounded.
///
/// **There is no unbounded variant, on purpose.** Either you wait with a bound,
/// or you have already established there is data and use `readNonBlocking`.
/// A machine that never runs is the normal way for PIO work to fail, and it
/// used to present as a board that had simply stopped.
pub inline fn read(block: Block, sm: u32) bounded.Error!u32 {
    var s = bounded.Spin.init(bounded.fifo_polls);
    while (rxEmpty(block, sm)) try s.tick();
    return hw.read(block.base() + hw.Pio.rxf(sm));
}

/// Non-blocking read; null if the FIFO is empty. Preferable in bring-up code,
/// where a state machine that never runs would otherwise hang the firmware with
/// no diagnostic at all.
pub fn readNonBlocking(block: Block, sm: u32) ?u32 {
    if (rxEmpty(block, sm)) return null;
    return hw.read(block.base() + hw.Pio.rxf(sm));
}

/// Blocking write to a state machine's TX FIFO, bounded.
///
/// A machine that has stopped consuming fills its FIFO and then this never
/// returns — which is how a stalled panel or a wedged bus used to present.
/// Where the caller has *already* checked `txFull` and knows there is room,
/// use `writeUnchecked` and skip the wait entirely rather than spinning on a
/// condition that cannot be true.
pub inline fn write(block: Block, sm: u32, value: u32) bounded.Error!void {
    var s = bounded.Spin.init(bounded.fifo_polls);
    while (txFull(block, sm)) try s.tick();
    hw.write(block.base() + hw.Pio.txf(sm), value);
}

/// Write without waiting, for a caller that has just tested `txFull` itself.
///
/// The `while (!pio.txFull(b, sm)) pio.writeUnchecked(b, sm, next())` shape is
/// how the panel and the LED strip feed their machines: the loop condition is
/// already the space check, so a second one inside the write is dead code with
/// a spin attached. **Writing to a full FIFO silently discards the word** —
/// that is why this is a separate name rather than the default.
pub inline fn writeUnchecked(block: Block, sm: u32, value: u32) void {
    hw.write(block.base() + hw.Pio.txf(sm), value);
}

/// The block's eight `irq` flags, as a bitmask.
///
/// These are how two state machines in the same block synchronise — and it is
/// **only** within a block: a `wait irq` in another PIO waits forever, silently.
pub inline fn irqFlags(block: Block) u32 {
    return hw.read(block.base() + hw.Pio.irq);
}

/// Clear the given `irq` flags. The register is write-1-to-clear, so this is a
/// plain write and not a read-modify-write.
pub inline fn clearIrq(block: Block, mask: u32) void {
    hw.write(block.base() + hw.Pio.irq, mask);
}

/// Route a set of the block's eight `irq` flags to interrupt request **line 0**,
/// the one `nvic.pioIrq(block_index, 0)` carries to the core.
///
/// `flags` is a mask over the flags themselves — bit 0 is `irq 0` — so it takes
/// the same value shape as `irqFlags` and `clearIrq` and cannot be confused
/// with them. **The register's own layout is not that**: the eight SM IRQ bits
/// sit at 8..15, above four `RXNEMPTY` and four `TXNFULL` bits, so passing a
/// raw flag mask straight through would enable FIFO conditions instead. The
/// shift lives in `hw.Pio.inteSmIrq`.
///
/// **Two separate steps make an interrupt fire**: this, and `nvic.enable`.
/// Doing only one is the classic handler that never runs.
pub inline fn routeIrqToLine0(block: Block, flags: u8) void {
    var inte: u32 = 0;
    var n: u8 = 0;
    while (n < 8) : (n += 1) {
        if (flags & (@as(u8, 1) << @truncate(n)) != 0) inte |= hw.Pio.inteSmIrq(@truncate(n));
    }
    // Read-modify-write: the FIFO-condition bits in the low byte belong to
    // whoever set them, and a plain write would silently clear them.
    hw.write(block.base() + hw.Pio.irq0_inte, hw.read(block.base() + hw.Pio.irq0_inte) | inte);
}

/// Raise interrupt request **line 1** whenever `sm`'s TX FIFO is not full.
///
/// The condition is level-sensitive and has no flag to clear: the handler
/// silences it by filling the FIFO, and it re-asserts as soon as the machine
/// consumes a word. That is what makes it the right way to feed a state machine
/// that must never starve — a CPU loop doing the same job cannot also do
/// anything that takes longer than the FIFO holds.
///
/// `TXNFULL` lives at bits 4..7, one per state machine, below the eight SM IRQ
/// flags `routeIrqToLine0` uses.
pub inline fn routeTxNotFullToLine1(block: Block, sm: u32) void {
    const bit = @as(u32, 1) << @truncate(4 + sm);
    hw.write(block.base() + hw.Pio.irq1_inte, hw.read(block.base() + hw.Pio.irq1_inte) | bit);
}

/// Which 32 GPIOs this block can reach: 0 or 16.
///
/// New on RP2350 and **silent when wrong** — a block left at 0 cannot drive
/// GP32-47, and a state machine told to do so simply drives nothing.
pub fn setGpioBase(block: Block, first: u32) void {
    hw.write(block.base() + hw.Pio.gpiobase, first);
}

/// Let the pins in `mask` skip the input synchroniser, additively.
///
/// Costs two `clk_sys` cycles of input latency to leave on, which only matters
/// where the state machine samples close to its own clock edge — a half-duplex
/// bus reading back on the same pin it drove is the case that needs it, and a
/// bus that samples mid-bit is the case that does not. See
/// `hw.Pio.input_sync_bypass` for the constraint that makes it unsafe on a
/// genuinely asynchronous input.
///
/// Reads and ORs rather than writing the mask outright, because the register is
/// shared by all four state machines in the block: a plain write would silently
/// re-synchronise a pin some other machine had already bypassed. Bits are
/// relative to `gpiobase`.
pub fn setInputSyncBypass(block: Block, mask: u32) void {
    const addr = block.base() + hw.Pio.input_sync_bypass;
    hw.write(addr, hw.read(addr) | mask);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "register offsets stride correctly across state machines" {
    // SM blocks are 0x18 apart, starting at 0xc8 — datasheet Table 981.
    try testing.expectEqual(@as(u32, 0x0c8), hw.Pio.smClkdiv(0));
    try testing.expectEqual(@as(u32, 0x0dc), hw.Pio.smPinctrl(0));
    try testing.expectEqual(@as(u32, 0x0e0), hw.Pio.smClkdiv(1));
    try testing.expectEqual(@as(u32, 0x0f8 + 0x18), hw.Pio.smClkdiv(3));
    // FIFOs and instruction memory.
    try testing.expectEqual(@as(u32, 0x010), hw.Pio.txf(0));
    try testing.expectEqual(@as(u32, 0x01c), hw.Pio.txf(3));
    try testing.expectEqual(@as(u32, 0x020), hw.Pio.rxf(0));
    try testing.expectEqual(@as(u32, 0x048), hw.Pio.instrMem(0));
    try testing.expectEqual(@as(u32, 0x0c4), hw.Pio.instrMem(31));
}

test "INPUT_SYNC_BYPASS sits below DBG_PADOUT, not where RP2040 put IRQ0_INTE" {
    // 0x038 per RP2350 datasheet §11.7; DBG_PADOUT immediately follows at 0x03c.
    // Asserted because this block's whole hazard is an RP2040 offset landing in
    // a register RP2350 moved — see `hw.Pio.irq0_inte`.
    try testing.expectEqual(@as(u32, 0x038), hw.Pio.input_sync_bypass);
    try testing.expectEqual(@as(u32, 0x03c), hw.Pio.dbg_padout);
}

test "three blocks, at the RP2350 bases" {
    try testing.expectEqual(@as(u32, 0x5020_0000), Block.pio0.base());
    try testing.expectEqual(@as(u32, 0x5030_0000), Block.pio1.base());
    // PIO2 does not exist on RP2040; its absence there is why this is asserted.
    try testing.expectEqual(@as(u32, 0x5040_0000), Block.pio2.base());
}

test "instruction encoding matches the datasheet's own examples" {
    // `nop` is documented as `mov y, y`, encoding 0xa042.
    try testing.expectEqual(@as(u16, 0xa042), nop(0));
    // `jmp 0` unconditional.
    try testing.expectEqual(@as(u16, 0x0000), jmp(.always, 0, 0));
    // Pimoroni's `st7701_timing.pio` writes two instructions as raw hex, to be
    // fed to `out exec` at runtime — which makes them an independent source for
    // this encoding, arrived at by a different toolchain.
    //
    // **They also caught a mistake in the first draft of this test**, and it is
    // exactly the mistake this module's header warns about. Both literals have
    // bit 12 set, which reads naturally as a delay of 16 and is nothing of the
    // sort: that program declares `.side_set 1`, so bit 12 is the side-set
    // value, and both instructions are "... side 1" with no delay at all. The
    // same 16-bit word means different things depending on SIDESET_COUNT, and
    // there is no way to tell from the word alone.
    //
    //   irq 4 side 1  -> 0xd004
    //   nop   side 1  -> 0xb042
    try testing.expectEqual(@as(u16, 0xd004), irqSet(4, false, sideSet(1, 1, 0)));
    try testing.expectEqual(@as(u16, 0xb042), nop(sideSet(1, 1, 0)));
    // For contrast, the same two with a delay of 1 and no side-set — one bit
    // apart in intent, four bits apart in encoding.
    try testing.expectEqual(@as(u16, 0xc104), irqSet(4, false, sideSet(0, 0, 1)));
    try testing.expectEqual(@as(u16, 0xa142), nop(sideSet(0, 0, 1)));
}

test "out and in encode a count of 32 as zero" {
    // The field is 5 bits and 32 does not fit; the hardware reads 0 as 32.
    // Getting this wrong shifts one bit instead of a whole word.
    try testing.expectEqual(out(.pins, 32, 0) & 0x1f, 0);
    try testing.expectEqual(out(.pins, 16, 0) & 0x1f, 16);
    try testing.expectEqual(in(.null_, 32, 0) & 0x1f, 0);
}

test "mov with reverse is what the panel's bit order needs" {
    // `mov pins, ::isr` — the instruction a vendor uses to undo a panel's
    // reversed channel wiring. Opcode 101, dest pins (0), op reverse (2),
    // src isr (6).
    const w = mov(.pins, .reverse, .isr, 0);
    try testing.expectEqual(@as(u16, 0b101), w >> 13);
    try testing.expectEqual(@as(u16, 0), (w >> 5) & 0x7); // dest = pins
    try testing.expectEqual(@as(u16, 2), (w >> 3) & 0x3); // op = reverse
    try testing.expectEqual(@as(u16, 6), w & 0x7); // src = isr
}

test "side-set and delay share one field, split by the state machine" {
    // With 1 side-set bit there are 4 delay bits: side=1, delay=3 packs as
    // 0b1_0011.
    try testing.expectEqual(@as(u5, 0b10011), sideSet(1, 1, 3));
    // With 2 side-set bits there are 3: side=2, delay=1 packs as 0b10_001.
    try testing.expectEqual(@as(u5, 0b10001), sideSet(2, 2, 1));
    // No side-set at all is a plain delay.
    try testing.expectEqual(@as(u5, 7), sideSet(0, 0, 7));
}

test "PINCTRL packs counts and bases where the datasheet says" {
    const cfg = SmConfig{
        .out_base = 1,
        .out_count = 16,
        .sideset_base = 22,
        .sideset_count = 1,
    };
    const w = cfg.pinctrl();
    try testing.expectEqual(@as(u32, 1), (w >> 0) & 0x1f);
    try testing.expectEqual(@as(u32, 16), (w >> 20) & 0x3f);
    try testing.expectEqual(@as(u32, 22), (w >> 10) & 0x1f);
    try testing.expectEqual(@as(u32, 1), (w >> 29) & 0x7);
    // OUT_COUNT is 6 bits on this chip: 32 pins must be expressible, since an
    // 18-bit RGB666 bus is driven by one OUT.
    const wide = (SmConfig{ .out_count = 32 }).pinctrl();
    try testing.expectEqual(@as(u32, 32), (wide >> 20) & 0x3f);
}

test "SHIFTCTRL thresholds of 32 encode as zero, and directions default right" {
    const dflt = (SmConfig{}).shiftctrl();
    try testing.expect(dflt & hw.Pio.shift_out_shiftdir_right != 0);
    try testing.expect(dflt & hw.Pio.shift_in_shiftdir_right != 0);
    try testing.expectEqual(@as(u32, 0), (dflt >> 25) & 0x1f); // pull thresh 32 -> 0
    try testing.expectEqual(@as(u32, 0), (dflt >> 20) & 0x1f); // push thresh 32 -> 0

    const left = (SmConfig{ .out_shiftdir = .left, .push_threshold = 16, .autopush = true }).shiftctrl();
    try testing.expect(left & hw.Pio.shift_out_shiftdir_right == 0);
    try testing.expectEqual(@as(u32, 16), (left >> 20) & 0x1f);
    try testing.expect(left & hw.Pio.shift_autopush != 0);
}

test "joining the FIFOs sets one bit and never both" {
    // FJOIN_TX and FJOIN_RX are bits 30 and 31; setting both is a documented
    // "reserved" state, which the enum makes unrepresentable.
    try testing.expectEqual(@as(u32, 0), (SmConfig{}).shiftctrl() & (hw.Pio.shift_fjoin_tx | hw.Pio.shift_fjoin_rx));
    const tx = (SmConfig{ .fifo_join = .tx }).shiftctrl();
    try testing.expect(tx & hw.Pio.shift_fjoin_tx != 0);
    try testing.expect(tx & hw.Pio.shift_fjoin_rx == 0);
    const rx = (SmConfig{ .fifo_join = .rx }).shiftctrl();
    try testing.expect(rx & hw.Pio.shift_fjoin_rx != 0);
    try testing.expect(rx & hw.Pio.shift_fjoin_tx == 0);
}

test "relocation moves jmp targets and leaves everything else alone" {
    // `jmp x--, 2 side 1` from the panel's timing program. Relocated by 5 it
    // must target 7, and its condition, side-set and opcode must be untouched.
    const jmp_x_dec = jmp(.x_dec, 2, sideSet(1, 1, 0));
    try testing.expectEqual(@as(u16, 0x1042), jmp_x_dec);
    try testing.expectEqual(@as(u16, 0x1047), relocate(jmp_x_dec, 5));
    try testing.expectEqual(jmp_x_dec, relocate(jmp_x_dec, 0));

    // Nothing else has an address operand, and the low five bits of these mean
    // something entirely different — relocating an `out` would corrupt its bit
    // count, and an `irq` its index.
    for ([_]u16{
        out(.pins, 2, 0),
        out(.exec, 16, 0),
        in(.null_, 16, 0),
        mov(.pins, .reverse, .isr, 0),
        irqSet(4, false, 0),
        set(.pindirs, 0x1f, 0),
        wait(1, .irq, 4, 0),
        nop(0),
    }) |instr| {
        try testing.expectEqual(instr, relocate(instr, 5));
    }
}

test "TX DREQs are eight apart, TX before RX" {
    // Getting this wrong paces a scan-out from the wrong peripheral, which does
    // not error — it tears.
    // RP2350 datasheet 12.6.4.1, the system DREQ table: DREQ_PIO0_RX0 is 4, DREQ_PIO1_RX2 is 14.
    try testing.expectEqual(@as(u32, 4), rxDreq(.pio0, 0));
    try testing.expectEqual(@as(u32, 14), rxDreq(.pio1, 2));
    try testing.expectEqual(@as(u32, 0), txDreq(.pio0, 0));
    try testing.expectEqual(@as(u32, 1), txDreq(.pio0, 1));
    try testing.expectEqual(@as(u32, 3), txDreq(.pio0, 3));
    try testing.expectEqual(@as(u32, 8), txDreq(.pio1, 0));
    try testing.expectEqual(@as(u32, 16), txDreq(.pio2, 0));
}

test "CTRL puts SM_RESTART at 7:4 and CLKDIV_RESTART at 11:8" {
    // These were transposed until 2026-08-01 and nothing caught it, because the
    // only caller sets both at once and the union of the two wrong masks is the
    // union of the two right ones. Asserted separately so the next caller —
    // `CLKDIV_RESTART` alone is the documented way to resynchronise divisors
    // on the fly — cannot inherit the mistake.
    try testing.expectEqual(@as(u32, 1 << 0), hw.Pio.ctrlSmEnable(0));
    try testing.expectEqual(@as(u32, 1 << 3), hw.Pio.ctrlSmEnable(3));
    try testing.expectEqual(@as(u32, 1 << 4), hw.Pio.ctrlSmRestart(0));
    try testing.expectEqual(@as(u32, 1 << 7), hw.Pio.ctrlSmRestart(3));
    try testing.expectEqual(@as(u32, 1 << 8), hw.Pio.ctrlClkdivRestart(0));
    try testing.expectEqual(@as(u32, 1 << 11), hw.Pio.ctrlClkdivRestart(3));
    // The three fields must not overlap — the property the transposition broke.
    const all = hw.Pio.ctrlSmEnable(1) | hw.Pio.ctrlSmRestart(1) | hw.Pio.ctrlClkdivRestart(1);
    try testing.expectEqual(@as(u32, 0x222), all);
}

test "FDEBUG is four nibbles, not four bytes" {
    // RXSTALL 3:0, RXUNDER 11:8, TXOVER 19:16, TXSTALL 27:24 — the fields are a
    // nibble apart because there are four state machines, and reading TXSTALL
    // at bit 24+sm rather than 24 is what makes it per-machine.
    try testing.expectEqual(@as(u32, 1 << 24), hw.Pio.fdebugTxstall(0));
    try testing.expectEqual(@as(u32, 1 << 26), hw.Pio.fdebugTxstall(2));
    try testing.expectEqual(@as(u32, 1 << 16), hw.Pio.fdebugTxover(0));
    try testing.expectEqual(@as(u32, 1 << 8), hw.Pio.fdebugRxunder(0));
    try testing.expectEqual(@as(u32, 1 << 0), hw.Pio.fdebugRxstall(0));
}

test "FLEVEL splits each byte into TX low, RX high" {
    // A joined TX FIFO reports up to 8 in the low nibble, which is how the
    // join is confirmed on hardware rather than assumed from the config write.
    const sample: u32 = 0x0000_0038; // SM0: TX 8, RX 3
    try testing.expectEqual(@as(u32, 8), hw.Pio.flevelTx(sample, 0));
    try testing.expectEqual(@as(u32, 3), hw.Pio.flevelRx(sample, 0));
    const sm2: u32 = 0x0004_0000; // SM2: TX 4, RX 0
    try testing.expectEqual(@as(u32, 4), hw.Pio.flevelTx(sm2, 2));
    try testing.expectEqual(@as(u32, 0), hw.Pio.flevelRx(sm2, 2));
}

test "clkdivInt puts a whole number in the integer half" {
    // 16.8 fixed point: the integer part is bits 31:16, and a fraction of zero
    // is what "no jitter" means. A divisor of 1 is the reset value.
    try testing.expectEqual(hw.Pio.clkdiv_by_1, clkdivInt(1));
    try testing.expectEqual(@as(u32, 6 << 16), clkdivInt(6));
    try testing.expectEqual(@as(u32, 0), clkdivInt(3) & 0xffff);
}
