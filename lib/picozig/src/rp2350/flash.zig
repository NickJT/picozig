//! Runtime flash erase and programming, via the bootrom's own routines.
//!
//! Gives the device persistent storage — a pairing key, a channel choice, a
//! commissioning record — without writing a QMI driver, because the bootrom
//! already contains a correct one.
//!
//! ## Why this is harder than "call the ROM function"
//!
//! An earlier estimate in PLAN.md called this the best capability-per-line on
//! the list, on the assumption that the ROM doing the work made the rest
//! trivial. It does not, for one reason:
//!
//! **Corrected 2026-08-14.** This header used to say "the sequence starts with
//! `flash_exit_xip()`, after which flash is not executable". That is wrong, and
//! it pointed at the wrong hazard. The datasheet: *"The QSPI device should be
//! accessible for XIP reads after calling this function"* — `exit_xip` returns
//! the flash *device* from continuous-read mode to a serial-command state, and
//! XIP keeps working.
//!
//! **The window that matters is inside the ROM erase call**, and the datasheet
//! is explicit: "For the duration of the erase operation, QMI is in direct mode
//! and attempting to access XIP from DMA, the debugger or the other core will
//! return a bus fault. XIP becomes accessible again once the function returns."
//!
//! During that window the CPU is executing ROM and needs no flash, so a
//! single-threaded caller is safe *even from flash* — verified by running this
//! whole sequence from flash without a fault. What is not safe is anything
//! **else** reaching XIP meanwhile: DMA, the other core, or an interrupt whose
//! handler lives in flash.
//!
//! `.ramfunc` stays, for two reasons that survive the correction: it removes
//! the caller from the hazard entirely rather than relying on the XIP cache to
//! hide it, and it is what the SDK does. **Do not rely on the cache.** With the
//! cache on, 85,509 flash-resident interrupt handlers ran during erases without
//! a fault; with it off, the first one killed the core. A hazard the cache
//! usually hides is a hazard that fires unpredictably — `example/flash-wedge`
//! reproduces it.
//!
//! ## Consequences worth knowing before using this
//!
//! * **The second core must not be running flash code**, and no DMA may touch
//!   XIP. This project runs one core and no DMA, so it is safe by construction
//!   — do not assume that survives if either changes.
//! * **Interrupts must be off.** An interrupt handler in flash, taken mid-erase,
//!   is the same fault. `eraseAndProgram` masks them.
//! * **A debugger attached during an erase can fault**, so an unexplained halt
//!   while single-stepping this code is expected rather than a bug.
//! * Erase granularity is a **4096-byte sector**; program granularity is 256
//!   bytes. Both addresses are offsets from the start of flash, not XIP
//!   addresses.

const std = @import("std");
const hw = @import("hw.zig");
const nvic = @import("nvic.zig");

pub const sector_size = 4096;
pub const page_size = 256;

/// Bootrom fixed addresses (datasheet Table 453).
const bootrom_magic_addr: u32 = 0x0000_0010;
const bootrom_table_lookup_addr: u32 = 0x0000_0016;

/// Flag selecting the Arm secure entry in the ROM table. From the SDK's
/// `RT_FLAG_FUNC_ARM_SEC` — the datasheet documents the mechanism but not this
/// constant, so `lookup` verifies the result rather than trusting it.
const rt_flag_func_arm_sec: u32 = 0x0004;

/// Two-character bootrom function codes, packed little-endian.
fn code(comptime a: u8, comptime b: u8) u32 {
    return @as(u32, a) | (@as(u32, b) << 8);
}

const RomTableLookupFn = *const fn (code: u32, mask: u32) callconv(.c) ?*align(1) anyopaque;

/// Is the bootrom laid out the way we expect? Checks the 'M','u' magic and the
/// version byte's presence. Cheap, and the alternative to failing this check is
/// calling a function pointer read out of an unrecognised ROM.
pub fn bootromRecognised() bool {
    const p: [*]const u8 = @ptrFromInt(bootrom_magic_addr);
    return p[0] == 'M' and p[1] == 'u';
}

/// Resolve a bootrom function by its two-character code, or null.
pub fn lookup(fn_code: u32) ?*align(1) anyopaque {
    if (!bootromRecognised()) return null;
    const lookup_ptr: *const u16 = @ptrFromInt(bootrom_table_lookup_addr);
    const lookup_fn: RomTableLookupFn = @ptrFromInt(@as(u32, lookup_ptr.*));
    const result = lookup_fn(fn_code, rt_flag_func_arm_sec) orelse return null;
    // Sanity: a real ROM function lives in the bottom 32 KB. Rejecting anything
    // else turns a misunderstood lookup convention into a clean failure instead
    // of a jump into nowhere.
    if (@intFromPtr(result) >= 0x8000) return null;
    return result;
}

const ConnectFn = *const fn () callconv(.c) void;
const ExitXipFn = *const fn () callconv(.c) void;
const FlushCacheFn = *const fn () callconv(.c) void;
const EraseFn = *const fn (addr: u32, count: u32, block_size: u32, block_cmd: u8) callconv(.c) void;
const ProgramFn = *const fn (addr: u32, data: [*]const u8, count: u32) callconv(.c) void;
const EnterXipFn = *const fn () callconv(.c) void;

/// The ROM entry points this module needs, resolved once.
pub const Funcs = struct {
    connect: ConnectFn,
    exit_xip: ExitXipFn,
    erase: EraseFn,
    program: ProgramFn,
    flush_cache: FlushCacheFn,
    enter_xip: EnterXipFn,

    pub fn resolve() ?Funcs {
        return .{
            .connect = @ptrCast(@alignCast(lookup(code('I', 'F')) orelse return null)),
            .exit_xip = @ptrCast(@alignCast(lookup(code('E', 'X')) orelse return null)),
            .erase = @ptrCast(@alignCast(lookup(code('R', 'E')) orelse return null)),
            .program = @ptrCast(@alignCast(lookup(code('R', 'P')) orelse return null)),
            .flush_cache = @ptrCast(@alignCast(lookup(code('F', 'C')) orelse return null)),
            .enter_xip = @ptrCast(@alignCast(lookup(code('C', 'X')) orelse return null)),
        };
    }
};

pub const Error = error{
    /// The bootrom is not laid out as expected, or a function was not found.
    BootromUnavailable,
    /// Address or length violates the sector/page granularity.
    Misaligned,
};

/// Erase `len` bytes at `offset`, then program `data` there.
///
/// **This function runs from RAM** (`.ramfunc`) and must stay that way — see the
/// header. `callconv(.c)` keeps it a real, callable function rather than
/// something the optimiser may fold back into a flash-resident caller, which
/// would reintroduce the exact fault the section exists to avoid.
///
/// An earlier attempt used `@export(..., .section = ".ramfunc")` and **silently
/// produced no .ramfunc section at all** — the image had the routine in flash
/// while the docs claimed RAM. `verifyRamResident` exists so that can never be
/// a matter of belief again.
///
/// Interrupts are masked for the duration: a handler living in flash, taken
/// mid-erase, faults exactly like anything else touching XIP.
fn eraseAndProgramRam(
    f: *const Funcs,
    offset: u32,
    data: [*]const u8,
    len: u32,
    erase_len: u32,
) linksection(".ramfunc") callconv(.c) void {
    f.connect();
    f.exit_xip();
    f.erase(offset, erase_len, 0, 0);
    if (len > 0) f.program(offset, data, len);
    f.flush_cache();
    f.enter_xip();
}

/// Erase the sector(s) covering `offset`, then write `data`.
///
/// `offset` is a flash offset (0 = start of flash), **not** an XIP address.
/// Must be sector-aligned, and `data.len` must be a whole number of 256-byte
/// pages — the hardware has no finer granularity, and silently rounding would
/// erase more than the caller asked for.
/// **Nothing may touch XIP while this runs — including the debugger.**
///
/// For the duration of an erase or program the QMI is in direct mode, and the
/// datasheet names all three masters that will take a bus fault for trying
/// (§5.4.8.10): "attempting to access XIP from **DMA, the debugger or the other
/// core** will return a bus fault. XIP becomes accessible again once the
/// function returns."
///
/// This module already handles the two on-chip masters — the sequence runs from
/// RAM (`verifyRamResident`) and masks interrupts so no flash-resident handler
/// can be entered. **The third is not ours to prevent**, and it is the one that
/// surprises people: a host `mdw 0x10000000`, an IDE memory window left open on
/// flash, or any polling loop that reads XIP will fault, and can leave QMI
/// mid-sequence with the XIP window unreadable.
///
/// Reading **SRAM** is unaffected, which is why `zig build watch-*` works during
/// a flash write: `.selftest` is at `ORIGIN(RAM)` and direct mode only closes
/// the window at `0x10000000`.
///
/// Confirmed against the datasheet 2026-08-25, closing an open question about a
/// debugger-induced hang.
pub fn eraseAndProgram(offset: u32, data: []const u8) Error!void {
    if (offset % sector_size != 0) return error.Misaligned;
    if (data.len % page_size != 0) return error.Misaligned;

    const funcs = Funcs.resolve() orelse return error.BootromUnavailable;
    const erase_len = std.mem.alignForward(u32, @intCast(@max(data.len, 1)), sector_size);

    nvic.disableInterrupts();
    defer nvic.enableInterrupts();

    eraseAndProgramRam(&funcs, offset, data.ptr, @intCast(data.len), erase_len);
}

/// Is the RAM-resident routine actually in RAM?
///
/// Not paranoia: the first implementation of this module claimed `.ramfunc`
/// placement and did not get it, and nothing about the build or the tests
/// noticed. Calling it in that state would have faulted mid-erase. Cheap enough
/// to assert at startup.
pub fn verifyRamResident() bool {
    const addr = @intFromPtr(&eraseAndProgramRam);
    return addr >= hw.sram_base and addr < hw.sram_base + 520 * 1024;
}

/// Read flash back through the XIP window, which is the normal way to see it.
pub fn readXip(offset: u32, out: []u8) void {
    const src: [*]const u8 = @ptrFromInt(hw.xip_base + offset);
    @memcpy(out, src[0..out.len]);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "function codes pack little-endian, matching the datasheet's two-char codes" {
    // 'R','E' = flash_range_erase.
    try testing.expectEqual(@as(u32, 0x4552), code('R', 'E'));
    try testing.expectEqual(@as(u32, 0x5052), code('R', 'P'));
    try testing.expectEqual(@as(u32, 0x4649), code('I', 'F'));
    try testing.expectEqual(@as(u32, 0x5845), code('E', 'X'));
    try testing.expectEqual(@as(u32, 0x4346), code('F', 'C'));
    try testing.expectEqual(@as(u32, 0x5843), code('C', 'X'));
}

test "granularity constants match the hardware" {
    try testing.expectEqual(@as(u32, 4096), sector_size);
    try testing.expectEqual(@as(u32, 256), page_size);
    try testing.expect(sector_size % page_size == 0);
}

test "erase length rounds up to whole sectors" {
    // One byte still erases a full sector — the hardware cannot do less, and
    // the caller must not be able to believe otherwise.
    try testing.expectEqual(@as(u32, 4096), std.mem.alignForward(u32, 1, sector_size));
    try testing.expectEqual(@as(u32, 4096), std.mem.alignForward(u32, 4096, sector_size));
    try testing.expectEqual(@as(u32, 8192), std.mem.alignForward(u32, 4097, sector_size));
}
