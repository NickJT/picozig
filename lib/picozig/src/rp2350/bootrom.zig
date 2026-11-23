//! The bootrom's information APIs — who this chip is, and how its flash is
//! partitioned.
//!
//! Three questions the C-SDK answers with three separate examples
//! (`system/unique_board_id`, `system/boot_info`, `flash/partition_info`), and
//! two ROM entry points answer all of them:
//!
//!   * `get_sys_info` ('G','S') — chip id, CPU architecture, flash device info,
//!     the per-boot random number, and how the last boot went.
//!   * `get_partition_table_info` ('G','P') — the resident partition table.
//!
//! **Everything here was taken from the RP2350 datasheet §5.4.8.16 and
//! §5.4.8.17, and the field layouts from §5.9.4.2**, cross-checked against the
//! SDK's `bootrom_constants.h` and `picobin.h`. The datasheet's *prose* has been
//! wrong in this project before, but its register and API
//! tables have not, and the SDK headers have been accurate throughout.
//!
//! ## The first word is a contract, not data
//!
//! Both calls take a flags word and fill a buffer. **The first word written
//! back is the subset of those flags the ROM actually supports**, and the
//! datasheet is explicit: "You should always check this value before
//! interpreting the buffer." A caller that assumes it asked for three things and
//! therefore got three things will silently misread the buffer on any ROM
//! revision that drops one. `SysInfo` and `PartitionTable` below check it.
//!
//! ## Why the lookup lives in `flash.zig`
//!
//! Historical: `flash.zig` needed the ROM table first and grew `lookup` for its
//! own use. It is re-exported here rather than moved, because moving it would
//! touch the one module in this layer whose failure mode is an unbootable
//! board, for a tidiness that no caller can see.

const std = @import("std");
const flash = @import("flash.zig");

/// Resolve a bootrom function by its two-character code, or null.
pub const lookup = flash.lookup;

/// True if the bootrom is laid out as expected. Everything here returns null or
/// an error when it is not.
pub const recognised = flash.bootromRecognised;

fn romCode(comptime a: u8, comptime b: u8) u32 {
    return @as(u32, a) | (@as(u32, b) << 8);
}

pub const Error = error{
    /// The bootrom is not laid out as expected, or the entry point is absent.
    Unavailable,
    /// The ROM returned a negative status.
    RomError,
    /// The ROM did not report supporting a flag that was asked for. **Not the
    /// same as an error**: it means this ROM revision does not offer that
    /// information, and the buffer must not be read as though it does.
    Unsupported,
};

// ---------------------------------------------------------------------------
// get_sys_info
// ---------------------------------------------------------------------------

/// `int get_sys_info(uint32_t *out, uint32_t out_words, uint32_t flags)`
const GetSysInfoFn = *const fn (out: [*]u32, out_words: u32, flags: u32) callconv(.c) i32;

/// Flags for `get_sys_info`, datasheet §5.4.8.17.
pub const sys_info = struct {
    /// 3 words: PACKAGE_SEL, device id low, device id high.
    pub const chip_info: u32 = 0x0001;
    /// 1 word: the OTP CRITICAL register as read at the last OTP reset.
    pub const critical: u32 = 0x0002;
    /// 1 word: 0 = Arm, 1 = RISC-V.
    pub const cpu_info: u32 = 0x0004;
    /// 1 word, in the format of OTP FLASH_DEVINFO.
    pub const flash_dev_info: u32 = 0x0008;
    /// 4 words, regenerated every boot.
    pub const boot_random: u32 = 0x0010;
    /// **Not supported** — the datasheet says so outright. Present for
    /// completeness so nobody re-derives it from the numbering.
    pub const nonce: u32 = 0x0020;
    /// 4 words: how the most recent boot went. Does not survive a RUN-pin or
    /// POWMAN reset — only a watchdog reboot.
    pub const boot_info: u32 = 0x0040;
};

/// What the chip says it is.
pub const ChipInfo = struct {
    /// `CHIP_INFO_PACKAGE_SEL`. Note this is the ROM's view; `silicon.package()`
    /// reads SYSINFO directly and the two should agree.
    package_sel: u32,
    /// The 64-bit device id. **This is the unique board id** — the C-SDK's
    /// `pico_get_unique_board_id` is these two words on RP2350, where RP2040
    /// had to borrow the flash chip's serial number instead.
    device_id: u64,

    /// The id as the SDK's `pico_get_unique_board_id_string` would print it:
    /// 16 uppercase hex digits, most significant nibble first.
    pub fn hex(self: ChipInfo) [16]u8 {
        var out: [16]u8 = undefined;
        var i: usize = 0;
        while (i < 16) : (i += 1) {
            const nibble: u4 = @truncate(self.device_id >> @intCast(60 - 4 * i));
            out[i] = "0123456789ABCDEF"[nibble];
        }
        return out;
    }
};

/// Ask the ROM for `flags`, into `out`.
///
/// Returns the slice actually filled, **including the leading supported-flags
/// word at index 0**, so a caller checking it does not have to remember an
/// offset.
pub fn getSysInfo(out: []u32, flags: u32) Error![]u32 {
    const f: GetSysInfoFn = @ptrCast(@alignCast(lookup(romCode('G', 'S')) orelse
        return Error.Unavailable));
    const rc = f(out.ptr, @intCast(out.len), flags);
    if (rc < 0) return Error.RomError;
    return out[0..@intCast(rc)];
}

/// The chip's identity, in one call.
pub fn chipInfo() Error!ChipInfo {
    var buf: [8]u32 = @splat(0);
    const got = try getSysInfo(&buf, sys_info.chip_info);
    // Leading word plus the three CHIP_INFO words.
    if (got.len < 4) return Error.RomError;
    if (got[0] & sys_info.chip_info == 0) return Error.Unsupported;
    return .{
        .package_sel = got[1],
        .device_id = @as(u64, got[2]) | (@as(u64, got[3]) << 32),
    };
}

/// Which architecture the ROM thinks is running. Always `.arm` in this
/// workspace — asked anyway, because a RISC-V answer would explain a great deal
/// about a board that is otherwise behaving impossibly.
pub const CpuArch = enum(u32) { arm = 0, riscv = 1, _ };

pub fn cpuArch() Error!CpuArch {
    var buf: [4]u32 = @splat(0);
    const got = try getSysInfo(&buf, sys_info.cpu_info);
    if (got.len < 2) return Error.RomError;
    if (got[0] & sys_info.cpu_info == 0) return Error.Unsupported;
    return @fromBackingInt(@intCast(got[1]));
}

// ---------------------------------------------------------------------------
// get_partition_table_info
// ---------------------------------------------------------------------------

/// `int get_partition_table_info(uint32_t *out, uint32_t out_words, uint32_t flags_and_partition)`
const GetPartitionTableInfoFn = *const fn (out: [*]u32, out_words: u32, flags: u32) callconv(.c) i32;

/// Flags for `get_partition_table_info`, datasheet §5.4.8.16.
pub const pt_info = struct {
    /// 3 words about the table as a whole.
    pub const table: u32 = 0x0001;
    /// 2 words per partition: permissions_and_location, permissions_and_flags.
    pub const location_and_flags: u32 = 0x0010;
    /// 2 words per partition, only if the partition's HAS_ID flag is set.
    pub const id: u32 = 0x0020;
    /// Extra UF2 family ids.
    pub const family_ids: u32 = 0x0040;
    /// The optional name, only if HAS_NAME is set.
    pub const name: u32 = 0x0080;
    /// Return data for one partition only; its number goes in the top 8 bits of
    /// the flags word.
    pub const single_partition: u32 = 0x8000;
};

/// Sector fields inside `permissions_and_location`, datasheet §5.9.4.2.
pub const location = struct {
    pub const first_sector_lsb: u5 = 0;
    pub const first_sector_bits: u32 = 0x0000_1fff;
    pub const last_sector_lsb: u5 = 13;
    pub const last_sector_bits: u32 = 0x03ff_e000;

    pub fn firstSector(w: u32) u32 {
        return (w & first_sector_bits) >> first_sector_lsb;
    }
    pub fn lastSector(w: u32) u32 {
        return (w & last_sector_bits) >> last_sector_lsb;
    }
};

pub const TableInfo = struct {
    partition_count: u8,
    /// **False on a stock board**, and that is normal rather than a fault: a
    /// partition table is something a project chooses to write, and nothing in
    /// this workspace does yet.
    present: bool,
    unpartitioned_first_sector: u32,
    unpartitioned_last_sector: u32,
    unpartitioned_flags: u32,
};

pub fn tableInfo() Error!TableInfo {
    const f: GetPartitionTableInfoFn = @ptrCast(@alignCast(lookup(romCode('G', 'P')) orelse
        return Error.Unavailable));
    var buf: [8]u32 = @splat(0);
    const rc = f(&buf, buf.len, pt_info.table);
    if (rc < 0) return Error.RomError;
    const got = buf[0..@intCast(rc)];
    if (got.len < 4) return Error.RomError;
    if (got[0] & pt_info.table == 0) return Error.Unsupported;
    return .{
        .partition_count = @truncate(got[1] & 0xFF),
        .present = got[1] & 0x100 != 0,
        .unpartitioned_first_sector = location.firstSector(got[2]),
        .unpartitioned_last_sector = location.lastSector(got[2]),
        .unpartitioned_flags = got[3],
    };
}

// ---------------------------------------------------------------------------
// Tests — the pure parts only; everything else needs a bootrom.
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the ROM table code is two characters, low byte first" {
    // 'G','S' is 0x5347, not 0x4753. Getting this backwards resolves to
    // nothing, which reads as "no bootrom" rather than as a typo.
    try testing.expectEqual(@as(u32, 0x5347), romCode('G', 'S'));
    try testing.expectEqual(@as(u32, 0x5047), romCode('G', 'P'));
}

test "the device id prints most significant nibble first" {
    const info = ChipInfo{ .package_sel = 0, .device_id = 0x0123_4567_89AB_CDEF };
    try testing.expectEqualSlices(u8, "0123456789ABCDEF", &info.hex());
}

test "a device id with leading zeroes keeps its width" {
    // A 16-digit field that silently shortens is one that cannot be compared
    // between two boards, which is the only thing a board id is for.
    const info = ChipInfo{ .package_sel = 0, .device_id = 0x0000_0000_0000_000A };
    try testing.expectEqualSlices(u8, "000000000000000A", &info.hex());
}

test "location fields split at bit 13, and neither bleeds into the other" {
    // first = 0x1FFF, last = 0x1FFF packed together.
    const w: u32 = 0x0000_1FFF | (0x1FFF << 13);
    try testing.expectEqual(@as(u32, 0x1FFF), location.firstSector(w));
    try testing.expectEqual(@as(u32, 0x1FFF), location.lastSector(w));

    // And a first sector alone does not show up as a last sector.
    try testing.expectEqual(@as(u32, 0), location.lastSector(0x0000_1FFF));
}
