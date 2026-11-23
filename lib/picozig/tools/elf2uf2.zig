//! ELF → UF2 converter for the RP2350 alert-display firmware.
//!
//! This exists so the Pico firmware needs no part of the pico-sdk toolchain —
//! not even `picotool`. UF2 is a deliberately trivial format (512-byte blocks
//! with a 32-byte header and a 4-byte trailer, 256 payload bytes each), and the
//! ELF side only needs the program headers, so hand-parsing beats depending on
//! std.elf's API surface staying still across Zig releases.
//!
//! Usage: elf2uf2 <input.elf> <output.uf2>
//!
//! Run as a build step, not by hand — see build.zig's `uf2` step.

const std = @import("std");
const Io = std.Io;

// ---------------------------------------------------------------------------
// UF2
// ---------------------------------------------------------------------------

const uf2_magic_start0: u32 = 0x0a32_4655; // "UF2\n"
const uf2_magic_start1: u32 = 0x9e5d_5157;
const uf2_magic_end: u32 = 0x0ab1_6f30;

/// The familyID field is present and meaningful (rather than being a file size).
const uf2_flag_family_id_present: u32 = 0x0000_2000;
/// The four bytes after the payload carry a vendor extension tag.
const uf2_flag_extension_flags_present: u32 = 0x0000_8000;

/// RP2350 in Arm secure mode — matching the IMAGE_DEF block's declared
/// security/CPU in pico/boot.zig. Getting this wrong makes the bootrom reject
/// the drag-and-drop with no useful feedback. Values from the canonical
/// microsoft/uf2 family list:
///   RP2040          0xe48bff56
///   RP2XXX_ABSOLUTE 0xe48bff57
///   RP2XXX_DATA     0xe48bff58
///   RP2350_ARM_S    0xe48bff59   <-- us
///   RP2350_RISCV    0xe48bff5a
///   RP2350_ARM_NS   0xe48bff5b
const uf2_family_rp2350_arm_s: u32 = 0xe48b_ff59;
/// `RP2XXX_ABSOLUTE`, used only by the erratum block below.
const uf2_family_absolute: u32 = 0xe48b_ff57;

/// Marks the E10 workaround block, so a bootrom that does not need it can
/// recognise and skip it rather than writing it. Added in picotool 2.1.0;
/// value from the pico-sdk's `boot/uf2.h`.
const uf2_extension_rp2_ignore_block: u32 = 0x9957_e304;

const uf2_payload_size: u32 = 256;

/// Where the erratum block is aimed: the last page of the 16 MB flash window.
/// picotool's default, and the datasheet's "targeting the end of Flash".
const abs_block_addr: u32 = 0x1100_0000 - uf2_payload_size;

const Uf2Block = extern struct {
    magic_start0: u32,
    magic_start1: u32,
    flags: u32,
    target_addr: u32,
    payload_size: u32,
    block_no: u32,
    num_blocks: u32,
    family_id: u32,
    data: [476]u8,
    magic_end: u32,
};

comptime {
    if (@sizeOf(Uf2Block) != 512) @compileError("UF2 block must be exactly 512 bytes");
}

/// **RP2350-E10.** On A2 silicon the bootrom does not set up flash before
/// checking the partition table, so a drag-and-drop UF2 fails outright when a
/// partition table is present — which it is on any board that shipped with
/// MicroPython and a filesystem.
///
/// The datasheet's workaround, verbatim in shape: one block at the *start* of
/// the UF2 with the Absolute family ID, targeting the end of flash, block
/// number 0 and block count 2. It is written to flash first and does not reboot
/// the device; writing it is what sets flash up for the rest of the download.
///
/// **It costs the last sector of flash**, which the datasheet warns "could
/// overwrite user data". That is the erratum's price, not ours.
///
/// A3 and later fix the erratum and do not need this. The block is emitted
/// anyway, because the tool cannot know which stepping it is building for, and
/// the `IGNORE_BLOCK` extension exists precisely so a newer bootrom recognises
/// and skips it — costing a fixed chip nothing but 512 bytes of file.
fn absBlock(target_addr: u32) Uf2Block {
    var block: Uf2Block = .{
        .magic_start0 = uf2_magic_start0,
        .magic_start1 = uf2_magic_start1,
        .flags = uf2_flag_family_id_present | uf2_flag_extension_flags_present,
        .target_addr = target_addr,
        .payload_size = uf2_payload_size,
        // Deliberately 0 of 1, not part of the real block numbering: the
        // bootrom treats an Absolute-family block as its own tiny download.
        .block_no = 0,
        .num_blocks = 1,
        .family_id = uf2_family_absolute,
        .data = @splat(0),
        .magic_end = uf2_magic_end,
    };
    // The payload is 0xef filler; `check_abs_block` in picotool identifies the
    // block by exactly this, so it is a recognisable marker rather than an
    // arbitrary choice.
    @memset(block.data[0..uf2_payload_size], 0xef);
    std.mem.writeInt(
        u32,
        block.data[uf2_payload_size..][0..4],
        uf2_extension_rp2_ignore_block,
        .little,
    );
    return block;
}

// ---------------------------------------------------------------------------
// Minimal ELF32 little-endian program-header reader
// ---------------------------------------------------------------------------

const pt_load: u32 = 1;

const Segment = struct {
    /// Load address — the *physical* address, not the virtual one. This
    /// distinction is load-bearing: .data has a virtual address in RAM but is
    /// stored in flash, and it is the flash address we must write.
    paddr: u32,
    file_off: u32,
    file_size: u32,
};

fn readU16(buf: []const u8, off: usize) u16 {
    return std.mem.readInt(u16, buf[off..][0..2], .little);
}

fn readU32(buf: []const u8, off: usize) u32 {
    return std.mem.readInt(u32, buf[off..][0..4], .little);
}

fn parseLoadSegments(gpa: std.mem.Allocator, elf: []const u8) ![]Segment {
    if (elf.len < 52) return error.NotAnElf;
    if (!std.mem.eql(u8, elf[0..4], "\x7fELF")) return error.NotAnElf;
    if (elf[4] != 1) return error.NotElf32; // EI_CLASS
    if (elf[5] != 1) return error.NotLittleEndian; // EI_DATA

    const e_phoff = readU32(elf, 28);
    const e_phentsize = readU16(elf, 42);
    const e_phnum = readU16(elf, 44);
    if (e_phentsize < 32) return error.BadProgramHeader;

    var segments: std.ArrayList(Segment) = .empty;
    errdefer segments.deinit(gpa);

    var i: u16 = 0;
    while (i < e_phnum) : (i += 1) {
        const off = e_phoff + @as(u32, i) * e_phentsize;
        if (off + 32 > elf.len) return error.BadProgramHeader;

        if (readU32(elf, off + 0) != pt_load) continue;
        const file_size = readU32(elf, off + 16);
        if (file_size == 0) continue; // .bss-only segment: nothing to store

        try segments.append(gpa, .{
            .paddr = readU32(elf, off + 12),
            .file_off = readU32(elf, off + 4),
            .file_size = file_size,
        });
    }

    return segments.toOwnedSlice(gpa);
}

// ---------------------------------------------------------------------------
// Conversion
// ---------------------------------------------------------------------------

const flash_base: u32 = 0x1000_0000;
const flash_limit: u32 = flash_base + 16 * 1024 * 1024;

pub const Options = struct {
    abs_block_addr: ?u32 = null,
};

/// Flattens the loadable segments into one contiguous image, then emits it as
/// 256-byte UF2 blocks starting at a 256-byte boundary.
///
/// By default matches canonical `picotool` (Block 0 starts at 0x10000000).
/// Prepends an E10 workaround block if `opts.abs_block_addr` is provided.
fn convert(gpa: std.mem.Allocator, elf: []const u8, opts: Options) ![]u8 {
    const segments = try parseLoadSegments(gpa, elf);
    defer gpa.free(segments);
    if (segments.len == 0) return error.NoLoadableSegments;

    var lo: u32 = std.math.maxInt(u32);
    var hi: u32 = 0;
    for (segments) |seg| {
        if (seg.paddr < flash_base or seg.paddr >= flash_limit) return error.SegmentOutsideFlash;
        lo = @min(lo, seg.paddr);
        hi = @max(hi, seg.paddr + seg.file_size);
    }

    // Round the window out to whole UF2 payloads.
    const base = lo - (lo % uf2_payload_size);
    const end = std.mem.alignForward(u32, hi, uf2_payload_size);
    const image = try gpa.alloc(u8, end - base);
    defer gpa.free(image);
    @memset(image, 0xff); // erased-flash value, so padding costs no wear

    for (segments) |seg| {
        if (seg.file_off + seg.file_size > elf.len) return error.TruncatedElf;
        const dst = seg.paddr - base;
        @memcpy(image[dst..][0..seg.file_size], elf[seg.file_off..][0..seg.file_size]);
    }

    const num_blocks: u32 = @intCast(image.len / uf2_payload_size);
    const total_blocks: u32 = if (opts.abs_block_addr != null) num_blocks + 1 else num_blocks;

    const out = try gpa.alloc(u8, total_blocks * @sizeOf(Uf2Block));
    errdefer gpa.free(out);

    var body = out;
    if (opts.abs_block_addr) |addr| {
        const abs = absBlock(addr);
        @memcpy(out[0..@sizeOf(Uf2Block)], std.mem.asBytes(&abs));
        body = out[@sizeOf(Uf2Block)..];
    }

    var block_no: u32 = 0;
    while (block_no < num_blocks) : (block_no += 1) {
        var block: Uf2Block = .{
            .magic_start0 = uf2_magic_start0,
            .magic_start1 = uf2_magic_start1,
            .flags = uf2_flag_family_id_present,
            .target_addr = base + block_no * uf2_payload_size,
            .payload_size = uf2_payload_size,
            .block_no = block_no,
            .num_blocks = num_blocks,
            .family_id = uf2_family_rp2350_arm_s,
            .data = @splat(0),
            .magic_end = uf2_magic_end,
        };
        @memcpy(
            block.data[0..uf2_payload_size],
            image[block_no * uf2_payload_size ..][0..uf2_payload_size],
        );
        @memcpy(body[block_no * @sizeOf(Uf2Block) ..][0..@sizeOf(Uf2Block)], std.mem.asBytes(&block));
    }

    return out;
}

// ---------------------------------------------------------------------------

fn usage(prog: []const u8, io: Io) noreturn {
    var buf: [256]u8 = undefined;
    var w: Io.File.Writer = .init(.stderr(), io, &buf);
    w.interface.print("usage: {s} <input.elf> <output.uf2> [--abs-block-addr <addr>]\n", .{prog}) catch {};
    w.interface.flush() catch {};
    std.process.exit(2);
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;
    const arena = init.arena.allocator();

    const args = try init.minimal.args.toSlice(arena);

    var opts: Options = .{};
    var input_path: ?[]const u8 = null;
    var output_path: ?[]const u8 = null;

    var idx: usize = 1;
    while (idx < args.len) : (idx += 1) {
        const arg = args[idx];
        if (std.mem.eql(u8, arg, "--abs-block-addr")) {
            if (idx + 1 >= args.len) usage(args[0], io);
            idx += 1;
            opts.abs_block_addr = std.fmt.parseInt(u32, args[idx], 0) catch usage(args[0], io);
        } else if (input_path == null) {
            input_path = arg;
        } else if (output_path == null) {
            output_path = arg;
        } else {
            usage(args[0], io);
        }
    }

    if (input_path == null or output_path == null) {
        usage(args[0], io);
    }

    const elf = try Io.Dir.cwd().readFileAlloc(io, input_path.?, gpa, .limited(16 * 1024 * 1024));
    defer gpa.free(elf);

    const uf2 = try convert(gpa, elf, opts);
    defer gpa.free(uf2);

    try Io.Dir.cwd().writeFile(io, .{ .sub_path = output_path.?, .data = uf2 });
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "convert: a single flash segment becomes correctly framed UF2 blocks" {
    const gpa = std.testing.allocator;

    // A hand-built ELF32 LE with one PT_LOAD segment of 300 bytes at
    // 0x10000000 — enough to span two UF2 blocks and exercise the padding.
    const payload_len = 300;
    const ph_off = 52;
    const seg_off = ph_off + 32;
    var elf: [seg_off + payload_len]u8 = @splat(0);

    @memcpy(elf[0..4], "\x7fELF");
    elf[4] = 1; // ELF32
    elf[5] = 1; // little endian
    std.mem.writeInt(u32, elf[28..32], ph_off, .little); // e_phoff
    std.mem.writeInt(u16, elf[42..44], 32, .little); // e_phentsize
    std.mem.writeInt(u16, elf[44..46], 1, .little); // e_phnum

    std.mem.writeInt(u32, elf[ph_off + 0 ..][0..4], pt_load, .little);
    std.mem.writeInt(u32, elf[ph_off + 4 ..][0..4], seg_off, .little);
    std.mem.writeInt(u32, elf[ph_off + 8 ..][0..4], 0x2000_0000, .little); // vaddr (ignored)
    std.mem.writeInt(u32, elf[ph_off + 12 ..][0..4], flash_base, .little); // paddr
    std.mem.writeInt(u32, elf[ph_off + 16 ..][0..4], payload_len, .little);

    for (elf[seg_off..], 0..) |*b, i| b.* = @truncate(i);

    const uf2 = try convert(gpa, &elf, .{});
    defer gpa.free(uf2);

    // Two canonical data blocks starting at flash_base.
    try std.testing.expectEqual(@as(usize, 2 * 512), uf2.len);

    const first: *const Uf2Block = @ptrCast(@alignCast(uf2.ptr));
    try std.testing.expectEqual(uf2_magic_start0, first.magic_start0);
    try std.testing.expectEqual(uf2_magic_start1, first.magic_start1);
    try std.testing.expectEqual(uf2_magic_end, first.magic_end);
    try std.testing.expectEqual(uf2_family_rp2350_arm_s, first.family_id);
    try std.testing.expectEqual(uf2_flag_family_id_present, first.flags);
    try std.testing.expectEqual(flash_base, first.target_addr);
    try std.testing.expectEqual(@as(u32, 2), first.num_blocks);
    try std.testing.expectEqual(@as(u32, 0), first.block_no);
    try std.testing.expectEqualSlices(u8, elf[seg_off..][0..256], first.data[0..256]);

    const second: *const Uf2Block = @ptrCast(@alignCast(uf2.ptr + 512));
    try std.testing.expectEqual(flash_base + 256, second.target_addr);
    try std.testing.expectEqual(@as(u32, 1), second.block_no);
    // The 44 real bytes, then erased-flash padding out to the 256-byte payload.
    try std.testing.expectEqualSlices(u8, elf[seg_off + 256 ..][0..44], second.data[0..44]);
    try std.testing.expectEqual(@as(u8, 0xff), second.data[44]);
    try std.testing.expectEqual(@as(u8, 0xff), second.data[255]);
}

test "convert: .bss-only segments are skipped, not written as zeros" {
    const gpa = std.testing.allocator;

    const ph_off = 52;
    const seg_off = ph_off + 64; // two program headers
    const payload_len = 16;
    var elf: [seg_off + payload_len]u8 = @splat(0);

    @memcpy(elf[0..4], "\x7fELF");
    elf[4] = 1;
    elf[5] = 1;
    std.mem.writeInt(u32, elf[28..32], ph_off, .little);
    std.mem.writeInt(u16, elf[42..44], 32, .little);
    std.mem.writeInt(u16, elf[44..46], 2, .little);

    // Real flash segment.
    std.mem.writeInt(u32, elf[ph_off + 0 ..][0..4], pt_load, .little);
    std.mem.writeInt(u32, elf[ph_off + 4 ..][0..4], seg_off, .little);
    std.mem.writeInt(u32, elf[ph_off + 12 ..][0..4], flash_base, .little);
    std.mem.writeInt(u32, elf[ph_off + 16 ..][0..4], payload_len, .little);

    // .bss: PT_LOAD, RAM address, filesz 0. Must be ignored — otherwise its
    // 0x2000_0000 paddr would trip the outside-flash check.
    std.mem.writeInt(u32, elf[ph_off + 32 ..][0..4], pt_load, .little);
    std.mem.writeInt(u32, elf[ph_off + 44 ..][0..4], 0x2000_0000, .little);
    std.mem.writeInt(u32, elf[ph_off + 48 ..][0..4], 0, .little);

    const uf2 = try convert(gpa, &elf, .{});
    defer gpa.free(uf2);

    try std.testing.expectEqual(@as(usize, 1 * 512), uf2.len);
}

test "convert: rejects a non-ELF input rather than emitting nonsense" {
    try std.testing.expectError(error.NotAnElf, convert(std.testing.allocator, "not an elf at all", .{}));
}

test "the RP2350-E10 block leads the file when requested" {
    const gpa = std.testing.allocator;

    const payload_len = 16;
    const ph_off = 52;
    const seg_off = ph_off + 32;
    var elf: [seg_off + payload_len]u8 = @splat(0);
    @memcpy(elf[0..4], "\x7fELF");
    elf[4] = 1;
    elf[5] = 1;
    std.mem.writeInt(u32, elf[28..32], ph_off, .little);
    std.mem.writeInt(u16, elf[42..44], 32, .little);
    std.mem.writeInt(u16, elf[44..46], 1, .little);
    std.mem.writeInt(u32, elf[ph_off + 0 ..][0..4], pt_load, .little);
    std.mem.writeInt(u32, elf[ph_off + 4 ..][0..4], seg_off, .little);
    std.mem.writeInt(u32, elf[ph_off + 12 ..][0..4], flash_base, .little);
    std.mem.writeInt(u32, elf[ph_off + 16 ..][0..4], payload_len, .little);

    const target_addr: u32 = 0x10ff_ff00;
    const uf2 = try convert(gpa, &elf, .{ .abs_block_addr = target_addr });
    defer gpa.free(uf2);

    const abs: *const Uf2Block = @ptrCast(@alignCast(uf2.ptr));
    // Every field the erratum text pins down.
    try std.testing.expectEqual(uf2_family_absolute, abs.family_id);
    try std.testing.expectEqual(target_addr, abs.target_addr);
    try std.testing.expectEqual(@as(u32, 0), abs.block_no);
    try std.testing.expectEqual(@as(u32, 1), abs.num_blocks);
    try std.testing.expectEqual(uf2_payload_size, abs.payload_size);
    try std.testing.expectEqual(
        uf2_flag_family_id_present | uf2_flag_extension_flags_present,
        abs.flags,
    );
    // Payload is 0xef filler, then the ignore-block tag — which is how a fixed
    // bootrom knows to skip it rather than burning the last flash sector.
    try std.testing.expectEqual(@as(u8, 0xef), abs.data[0]);
    try std.testing.expectEqual(@as(u8, 0xef), abs.data[uf2_payload_size - 1]);
    try std.testing.expectEqual(
        uf2_extension_rp2_ignore_block,
        std.mem.readInt(u32, abs.data[uf2_payload_size..][0..4], .little),
    );

    // **It must not disturb the real blocks.** They keep their own numbering
    // and their own family; a data block carrying the absolute family, or
    // counted as one of two, would be written to the wrong place.
    const data: *const Uf2Block = @ptrCast(@alignCast(uf2.ptr + 512));
    try std.testing.expectEqual(uf2_family_rp2350_arm_s, data.family_id);
    try std.testing.expectEqual(flash_base, data.target_addr);
    try std.testing.expectEqual(@as(u32, 0), data.block_no);
    try std.testing.expectEqual(@as(u32, 1), data.num_blocks);
    try std.testing.expectEqual(uf2_flag_family_id_present, data.flags);
}
