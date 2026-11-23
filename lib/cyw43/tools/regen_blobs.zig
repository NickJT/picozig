//! Convert one vendored firmware header into the raw blob it encodes.
//!
//!     regen-blobs image|hex|nvram HEADER OUT
//!
//! Run by `build.zig` once per blob, on every build, into the build cache: the
//! `.bin` files never exist in the source tree, so a fresh clone builds without
//! a manual step and nothing can go stale. It reproduces the images that have
//! been downloaded into the chip since rung 6 **byte for byte**:
//!
//!     w43439A0_combined.bin   225240 bytes  sha1 8c8d7fa1…
//!     wifi_nvram_43439.bin       742 bytes  sha1 c91ebb7a…
//!
//! The headers are cyw43-driver 2.0.0's, unmodified, so the driver can be
//! updated by copying files in rather than by converting them.
//!
//! ## Three formats, which is why a one-line `grep | xxd` does not do it
//!
//! * `image` — a WiFi image header holds **two** C arrays, the firmware then
//!   the CLM blob. The driver wants them as one: the firmware, zeros to the
//!   next 512-byte boundary, then the CLM, which is where the chip's loader
//!   expects to find it. `Image.check` in `cyw43.zig` rejects any other length.
//! * `hex` — one C array of `0x..` byte literals (the Bluetooth patchram).
//! * `nvram` — `wifi_nvram_43439.h` is a **concatenation of C string literals**, one
//!   NVRAM setting each, with explicit `\x00` terminators and a `\x00\x00` at
//!   the end. Parsing it as hex yields nothing; parsing it as text loses the
//!   escapes, and the two trailing NULs are what the firmware looks for.

const std = @import("std");

/// The whole 1.4 MB header, and room for what comes out of it.
const max_header: usize = 4 << 20;

pub fn main(init: std.process.Init) !void {
    const gpa = init.arena.allocator();
    const io = init.io;
    const cwd = std.Io.Dir.cwd();

    const args = try init.minimal.args.toSlice(gpa);
    if (args.len != 4) return error.Usage; // regen-blobs hex|nvram HEADER OUT
    const kind = args[1];
    const src = try cwd.readFileAlloc(io, args[2], gpa, std.Io.Limit.limited(max_header));
    const blob = if (std.mem.eql(u8, kind, "image"))
        try image(gpa, src)
    else if (std.mem.eql(u8, kind, "hex"))
        try parseHexArray(gpa, src)
    else if (std.mem.eql(u8, kind, "nvram"))
        try parseStringLiterals(gpa, src)
    else
        return error.Usage;
    try write(io, cwd, args[3], blob);
}

fn write(io: std.Io, dir: std.Io.Dir, path: []const u8, bytes: []const u8) !void {
    const file = try std.Io.Dir.createFile(dir, io, path, .{});
    defer file.close(io);
    var buf: [4096]u8 = undefined;
    var w = file.writer(io, &buf);
    try w.interface.writeAll(bytes);
    try w.interface.flush();
}

/// Firmware, zeros to the next 512-byte boundary, CLM: the header's two arrays
/// in the layout the driver loads. Exactly two arrays, or the header is not
/// the shape this was written for.
fn image(gpa: std.mem.Allocator, src: []const u8) ![]u8 {
    const fw_open = std.mem.indexOfScalar(u8, src, '{') orelse return error.NoArrayBody;
    const fw_close = std.mem.indexOfScalarPos(u8, src, fw_open, '}') orelse return error.NoArrayBody;
    const clm_open = std.mem.indexOfScalarPos(u8, src, fw_close, '{') orelse return error.NoClmArray;
    const clm_close = std.mem.indexOfScalarPos(u8, src, clm_open, '}') orelse return error.NoClmArray;
    if (std.mem.indexOfScalarPos(u8, src, clm_close, '{') != null) return error.UnexpectedArray;

    const fw = try parseHex(gpa, src[fw_open + 1 .. fw_close]);
    const clm = try parseHex(gpa, src[clm_open + 1 .. clm_close]);
    const clm_at = std.mem.alignForward(usize, fw.len, 512);
    const out = try gpa.alloc(u8, clm_at + clm.len);
    @memcpy(out[0..fw.len], fw);
    @memset(out[fw.len..clm_at], 0);
    @memcpy(out[clm_at..], clm);
    return out;
}

/// `{ 0x00, 0x01, ... }` — every `0x..` between the outermost braces.
fn parseHexArray(gpa: std.mem.Allocator, src: []const u8) ![]u8 {
    const open = std.mem.indexOfScalar(u8, src, '{') orelse return error.NoArrayBody;
    const close = std.mem.lastIndexOfScalar(u8, src, '}') orelse return error.NoArrayBody;
    if (close < open) return error.NoArrayBody;
    return parseHex(gpa, src[open + 1 .. close]);
}

/// Every `0x..` byte literal in `body`.
fn parseHex(gpa: std.mem.Allocator, body: []const u8) ![]u8 {
    var out: std.ArrayList(u8) = .empty;
    var i: usize = 0;
    while (i + 3 < body.len) {
        if (body[i] == '0' and (body[i + 1] == 'x' or body[i + 1] == 'X')) {
            const hi = std.fmt.charToDigit(body[i + 2], 16) catch {
                i += 1;
                continue;
            };
            const lo = std.fmt.charToDigit(body[i + 3], 16) catch {
                i += 1;
                continue;
            };
            try out.append(gpa, @as(u8, hi) * 16 + lo);
            i += 4;
            continue;
        }
        i += 1;
    }
    if (out.items.len == 0) return error.NoBytesFound;
    return out.items;
}

/// `"NVRAMRev=$Rev$" "\x00" "manfid=0x2d0" "\x00" ... "\x00\x00";`
///
/// Only the escapes this file actually uses are handled, and an unknown one is
/// an error rather than a guess — a silently mis-decoded NVRAM string is the
/// kind of thing that produces a chip that boots and misbehaves.
fn parseStringLiterals(gpa: std.mem.Allocator, src: []const u8) ![]u8 {
    // Start after the declaration's `=`, so the copyright comment's quotes are
    // not mistaken for the body.
    const decl = std.mem.indexOf(u8, src, "wifi_nvram_4343") orelse return error.NoNvramArray;
    const eq = std.mem.indexOfScalarPos(u8, src, decl, '=') orelse return error.NoNvramArray;
    const end = std.mem.lastIndexOfScalar(u8, src, ';') orelse return error.NoNvramArray;
    if (end < eq) return error.NoNvramArray;
    const body = src[eq + 1 .. end];

    var out: std.ArrayList(u8) = .empty;
    var i: usize = 0;
    var in_literal = false;
    while (i < body.len) : (i += 1) {
        const c = body[i];
        if (!in_literal) {
            if (c == '"') in_literal = true;
            continue;
        }
        switch (c) {
            '"' => in_literal = false,
            '\\' => {
                i += 1;
                if (i >= body.len) return error.TruncatedEscape;
                switch (body[i]) {
                    'x' => {
                        if (i + 2 >= body.len) return error.TruncatedEscape;
                        const hi = try std.fmt.charToDigit(body[i + 1], 16);
                        const lo = try std.fmt.charToDigit(body[i + 2], 16);
                        try out.append(gpa, @as(u8, hi) * 16 + lo);
                        i += 2;
                    },
                    'n' => try out.append(gpa, '\n'),
                    'r' => try out.append(gpa, '\r'),
                    't' => try out.append(gpa, '\t'),
                    '\\' => try out.append(gpa, '\\'),
                    '"' => try out.append(gpa, '"'),
                    '0' => try out.append(gpa, 0),
                    else => return error.UnknownEscape,
                }
            },
            else => try out.append(gpa, c),
        }
    }
    if (out.items.len == 0) return error.NoBytesFound;
    return out.items;
}
