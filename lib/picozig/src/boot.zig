//! RP2350 boot: the IMAGE_DEF metadata block, the Cortex-M33 vector table, and
//! the reset handler that establishes a usable runtime before any Zig code with
//! globals runs.
//!
//! Everything the pico-sdk's crt0 does that we actually need, in about 60 lines
//! — and nothing it does that we don't.

const hw = @import("rp2350").hw;
const nvic = @import("rp2350").nvic;
const qmi = @import("rp2350").qmi;
const root = @import("root");

// ---------------------------------------------------------------------------
// IMAGE_DEF — the RP2350 boot metadata block
// ---------------------------------------------------------------------------
// The bootrom scans the first 4 KB of flash for a "picobin" block. An image
// with no valid IMAGE_DEF is not executed at all: the chip simply falls back
// to BOOTSEL, which looks exactly like "my UF2 didn't flash" and is the single
// most common reason a hand-built RP2350 image appears to do nothing.
//
// This is the minimum valid block per RP2350 datasheet §5.9.5.1 ("Minimum Arm
// IMAGE_DEF"): one IMAGE_TYPE item plus the mandatory LAST terminator, 20
// bytes. The fields are spelled out rather than pasted as five magic words
// precisely because getting one byte wrong here has no diagnostic.

const ImageDef = packed struct(u160) {
    marker_start: u32 = 0xffff_ded3, // PICOBIN_BLOCK_MARKER_START

    // --- Item 0: IMAGE_TYPE ---
    item_image_type: u8 = 0x42, // PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE
    item_size_words: u8 = 1,
    /// 0b0001_0000_0010_0001, read from the top down:
    ///   try-before-you-buy : 0      (not a TBYB image)
    ///   chip               : 0b001  (RP2350)
    ///   cpu                : 0b000  (Arm)
    ///   exe security       : 0b10   (Secure)
    ///   image type         : 0b0001 (EXE)
    image_type_flags: u16 = 0b0001_0000_0010_0001,

    // --- Item 1: LAST (terminator) ---
    item_last: u8 = 0xff, // PICOBIN_BLOCK_ITEM_2BS_LAST
    item_last_size: u16 = 1,
    item_last_pad: u8 = 0,

    /// Relative pointer to the next block in the loop. Zero means "links to
    /// itself", i.e. this is the only block.
    next_block_rel: u32 = 0,

    marker_end: u32 = 0xab12_3579, // PICOBIN_BLOCK_MARKER_END
};

/// Emitted as raw words so the layout is beyond argument, but derived from the
/// documented struct above so the values can't drift from their meaning.
export const picobin_image_def: [5]u32 linksection(".picobin_block") = @bitCast(ImageDef{});

comptime {
    // Cheap insurance: if a field type or the packing rules ever change under
    // us, fail the build rather than the boot.
    const words: [5]u32 = @bitCast(ImageDef{});
    if (words[0] != 0xffff_ded3) @compileError("IMAGE_DEF start marker misplaced");
    if (words[1] != 0x1021_0142) @compileError("IMAGE_DEF IMAGE_TYPE item misencoded");
    if (words[2] != 0x0000_01ff) @compileError("IMAGE_DEF LAST item misencoded");
    if (words[4] != 0xab12_3579) @compileError("IMAGE_DEF end marker misplaced");
}

// ---------------------------------------------------------------------------
// Linker-provided symbols (see rp2350.ld)
// ---------------------------------------------------------------------------

extern const __stack_top: anyopaque;
extern var __data_start: u8;
extern var __data_end: u8;
extern var __data_load: u8;
extern var __bss_start: u8;
extern var __bss_end: u8;
extern var __ramfunc_start: u8;
extern var __ramfunc_end: u8;
extern var __ramfunc_load: u8;

// ---------------------------------------------------------------------------
// Vector table
// ---------------------------------------------------------------------------
// The full Cortex-M33 table: 16 system entries followed by RP2350's 52
// external interrupts.
//
// Every unused slot points at `unhandled` rather than being left as whatever
// bytes follow. That matters more than it looks: an unpopulated vector table
// entry is not "no interrupt", it is a jump to an arbitrary address the moment
// something raises that interrupt — a fault whose stack trace bears no relation
// to the cause. Trapping instead keeps the failure where it happened.

const Handler = *const fn () callconv(.c) noreturn;

/// An external interrupt's handler. Unlike the system vectors above it
/// **returns** — that is what an interrupt does — so it is a distinct type
/// rather than a `Handler`, and the distinction is load-bearing: a handler
/// declared `noreturn` that returns anyway resumes into whatever the exception
/// return sequence left, which is not a failure mode worth having.
pub const IrqHandler = *const fn () callconv(.c) void;

pub const irq_count = 62;

const VectorTable = extern struct {
    initial_sp: *const anyopaque,
    reset: Handler,
    nmi: Handler,
    hard_fault: Handler,
    mem_manage: Handler,
    bus_fault: Handler,
    usage_fault: Handler,
    secure_fault: Handler,
    reserved0: [3]?Handler,
    sv_call: Handler,
    debug_monitor: Handler,
    reserved1: ?Handler,
    pend_sv: Handler,
    sys_tick: Handler,
    irq: [irq_count]IrqHandler,
};

export const vector_table: VectorTable linksection(".vectors") = .{
    .initial_sp = &__stack_top,
    .reset = &_reset,
    .nmi = unhandled,
    .hard_fault = unhandled,
    .mem_manage = unhandled,
    .bus_fault = unhandled,
    .usage_fault = unhandled,
    .secure_fault = unhandled,
    .reserved0 = @splat(null),
    .sv_call = unhandled,
    .debug_monitor = unhandled,
    .reserved1 = null,
    .pend_sv = unhandled,
    .sys_tick = unhandled,
    .irq = irqTable(),
};

/// The external-interrupt vectors, built at comptime from the application.
///
/// An application claims one by declaring a `pub const interrupts` namespace
/// whose declaration **names** are `nvic.IrqNum` field names:
///
/// ```zig
/// pub const interrupts = struct {
///     pub fn pio0_irq_0() callconv(.c) void { ... }
/// };
/// ```
///
/// Everything else keeps trapping. Three properties are the reason it is shaped
/// this way rather than as a RAM vector table with a `setHandler` call:
///
///  * **A misspelled name is a compile error, not silence.** Installing by
///    number at runtime makes a typo into a handler that never runs, which is
///    the single most common interrupt bug and gives no diagnostic at all.
///  * **The table stays `const` in flash**, so there is no RAM copy, no VTOR
///    juggling, and nothing that can be corrupted by a wild write.
///  * It matches the contracts already here — `root.main`, `root.earlyInit`,
///    `root.onFault` — rather than inventing a fourth mechanism.
fn irqTable() [irq_count]IrqHandler {
    var table: [irq_count]IrqHandler = @splat(unhandled_irq);
    if (!@hasDecl(root, "interrupts")) return table;

    for (@typeInfo(root.interrupts).@"struct".decl_names) |d_name| {
        if (!@hasDecl(nvic.IrqNum, d_name)) @compileError(
            "root.interrupts." ++ d_name ++ " does not name an interrupt. " ++
                "Declaration names must match a field of nvic.IrqNum (e.g. pio0_irq_0).",
        );
        const num: usize = @field(nvic.IrqNum, d_name);
        if (table[num] != unhandled_irq) @compileError(
            "two handlers claim interrupt " ++ d_name,
        );
        table[num] = &@field(root.interrupts, d_name);
    }
    return table;
}

/// Default for every vector nothing has claimed.
///
/// The cast is the price of `faultEntry` being naked: it has to be, because a
/// compiler-generated prologue would move SP before the exception frame could
/// be found. The table's element type stays a real function pointer so that
/// `_reset` and any future claimed vector are still typechecked.
const unhandled: Handler = @ptrCast(&faultEntry);

/// The same trap, typed for the external-interrupt vectors. Reaching it means
/// something raised an interrupt nothing claimed, and `fault_info.exception`
/// (IPSR, so `16 + n`) names which.
const unhandled_irq: IrqHandler = @ptrCast(&faultEntry);

// ---------------------------------------------------------------------------
// Faults
// ---------------------------------------------------------------------------
// **A fault used to be indistinguishable from a hang, and that cost a week on
// a display board.** Every vector landed in `while (true) wfi`, so a chip that
// faulted before `main` presented as: an undriven LED pin, an all-zero
// `.selftest` block, peripherals still in reset, and — because the core was
// asleep — AP accesses returning WAIT, i.e. `Examination failed`. Every one of
// those was read as evidence about something else, and "core sleeping" was
// explicitly ruled out on the grounds that `timer.delayUs` busy-spins and there
// was "no WFI anywhere". There was one, here, in the place a fault goes.
//
// So two things changed, and both are about being able to see:
//
//   * **The trap busy-spins rather than sleeping.** It costs current, which is
//     a real trade on a battery device — and it is the right trade, because a
//     faulted chip that cannot be attached to cannot be diagnosed at all.
//   * **The cause is recorded** at a fixed address a debugger can read with no
//     symbols, and an application can add its own visible signal.

/// Written into `fault_info.magic`. Distinct from any example's `watch_magic`,
/// because `.faultinfo` is NOLOAD like `.selftest` and therefore inherits
/// whatever the previous image left there — a stale block that looked current
/// has already caused one wrong diagnosis in this project.
pub const fault_magic: u32 = 0xfa17_0001;

/// Read it with, from the workspace root and with no symbols needed:
///
/// ```
/// openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
///         -c init -c "mdw 0x20000080 8" -c shutdown
/// ```
pub const FaultInfo = extern struct {
    magic: u32,
    /// IPSR: which vector fired. 3 is HardFault; 16 + n is external IRQ n.
    exception: u32,
    /// The stacked PC — the instruction that faulted, and the single most
    /// useful word here. Resolve it with `llvm-symbolizer` or `nm` against the
    /// ELF, which `.strip = false` keeps usable.
    pc: u32,
    /// The stacked LR: who called the faulting code.
    lr: u32,
    /// The stacked xPSR.
    psr: u32,
    /// `SCB->CFSR` — the configurable-fault cause.
    cfsr: u32,
    /// `SCB->HFSR`. Bit 30 `FORCED` means "see CFSR"; bit 1 `VECTTBL` means the
    /// vector table itself would not read.
    hfsr: u32,
    /// `SCB->BFAR` — the address that would not read, when CFSR says it is
    /// valid. On a board executing from XIP this is the interesting one.
    bfar: u32,
};

/// Pinned by the linker script at `ORIGIN(RAM) + 0x80`, immediately above the
/// `.selftest` block every bring-up image already uses. NOLOAD and outside
/// `.bss`, so `_reset` does not zero it and it survives a warm reset — which is
/// the entire point, since the interesting fault is the one you have to reset
/// the board to ask about.
/// **`export`, not merely `pub`** — nothing in a working image ever reads this,
/// so as a plain global the optimiser is entitled to delete every store to it,
/// and did: `.faultinfo` linked at size zero and the recording function
/// compiled down to sixteen bytes. Exporting makes the writes observable. The
/// `.selftest` blocks have always been `export var` for the same reason.
pub export var fault_info: FaultInfo linksection(".faultinfo") = undefined;

/// Entry stub for every vector nothing has claimed.
///
/// Naked, and it has to be: the exception frame sits at the stack pointer as
/// the handler is entered, so any compiler-generated prologue would push
/// registers over the top of the address we are trying to report. Six
/// instructions, then hand off to Zig with the frame in a register.
///
/// EXC_RETURN bit 2 selects which stack the frame is on. Nothing here ever runs
/// on PSP, but reading it costs two instructions and removes a footgun from
/// whatever adds an RTOS later.
fn faultEntry() callconv(.naked) noreturn {
    asm volatile (
        \\ mrs r0, ipsr
        \\ mrs r1, msp
        \\ tst lr, #4
        \\ beq 1f
        \\ mrs r1, psp
        \\ 1:
        \\ b picozig_record_fault
    );
}

export fn picozig_record_fault(exception: u32, frame: [*]const u32) callconv(.c) noreturn {
    // Exception frame layout, no FP context: r0 r1 r2 r3 r12 LR PC xPSR.
    fault_info = .{
        .magic = fault_magic,
        .exception = exception,
        .pc = frame[6],
        .lr = frame[5],
        .psr = frame[7],
        .cfsr = hw.read(hw.scb_cfsr),
        .hfsr = hw.read(hw.scb_hfsr),
        .bfar = hw.read(hw.scb_bfar),
    };

    // An application's chance to say so where a human can see it — on this
    // project's boards that means an LED or, on a display board, the backlight. It
    // may return; it may equally blink forever and never come back.
    if (@hasDecl(root, "onFault")) root.onFault(&fault_info);

    // **Never `wfi`.** See above: a sleeping core is one a debugger may not be
    // able to reach, and this is precisely the moment a debugger is wanted.
    while (true) asm volatile ("" ::: .{ .memory = true });
}

// ---------------------------------------------------------------------------
// Reset handler
// ---------------------------------------------------------------------------

export fn _reset() callconv(.c) noreturn {
    // The bootrom enters us with VTOR wherever it left it. Point it at our own
    // table explicitly rather than relying on the default — this is one line
    // now and prevents an extremely confusing class of bug once interrupts
    // exist.
    hw.write(hw.scb_vtor, @intFromPtr(&vector_table));

    // Copy .ramfunc into RAM before calling any RAM-resident routines
    const ramfunc_len = @intFromPtr(&__ramfunc_end) - @intFromPtr(&__ramfunc_start);
    if (ramfunc_len > 0) {
        const dst: [*]u8 = @ptrCast(&__ramfunc_start);
        const src: [*]const u8 = @ptrCast(&__ramfunc_load);
        @memcpy(dst[0..ramfunc_len], src[0..ramfunc_len]);
    }

    // **Deliberately not called, and the history matters.** `qmi.initQuad()`
    // switches the QSPI flash to Quad-IO (0xEB) continuous-read, matching the
    // pico-sdk CRT0. It was added enabled in 80bb217 (2026-08-04) and commented
    // out the same day inside 5c508c6 — an *ICM-20948 driver* commit — with no
    // note in the message and nothing written down anywhere. Nobody recorded why.
    //
    // It stays off because nothing here needs the speed and it is not free:
    // continuous-read mode lives in the flash die, which an RP2350 reset does
    // not reset, so an image using it depends on the bootrom's XIP exit
    // sequence to break the flash back out on every boot. That is legitimate —
    // it is what the SDK does — but it is a dependency this project has no
    // reason to take on. Turning it on means re-testing cold boot, not just
    // observing that a warm one works.
    //
    // If it is ever re-enabled it must stay above the copies below and remain
    // `.ramfunc`: it reconfigures the interface it would otherwise be running
    // from.
    // qmi.initQuad();

    // An optional board hook, for work that must happen before *anything* else
    // — including the copies below, which are themselves bulk reads from flash.
    if (@hasDecl(root, "earlyInit")) root.earlyInit();

    // .data — copy initialised globals from their flash load address into RAM.
    const data_len = @intFromPtr(&__data_end) - @intFromPtr(&__data_start);
    if (data_len > 0) {
        const dst: [*]u8 = @ptrCast(&__data_start);
        const src: [*]const u8 = @ptrCast(&__data_load);
        @memcpy(dst[0..data_len], src[0..data_len]);
    }

    // .bss — zero.
    const bss_len = @intFromPtr(&__bss_end) - @intFromPtr(&__bss_start);
    if (bss_len > 0) {
        const bss: [*]u8 = @ptrCast(&__bss_start);
        @memset(bss[0..bss_len], 0);
    }

    root.main();
}
