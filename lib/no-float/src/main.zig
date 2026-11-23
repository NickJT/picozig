//! no-float — fails if a freestanding object references a soft-float symbol.
//!
//!     no-float OBJECT
//!
//! Run by the build, through `noFloat` in this package's `build.zig`; exits 1
//! with the explanation on stderr when the object uses soft float.
//!
//! Scans the object's bytes for the symbol names rather than parsing ELF. An
//! undefined symbol's name is in the object's string table verbatim, so a
//! substring search cannot miss one, and the names in the list are specific
//! enough that it cannot invent one either. Parsing the symbol table properly
//! would be more correct in principle and would trade about seventy lines for
//! no additional failure it can catch.

const std = @import("std");

/// The symbols a soft-float lowering leaves behind.
///
/// Matched as exact byte sequences rather than by prefix: `__aeabi_` alone
/// would also hit `__aeabi_unwind_cpp_pr0`, which is present in every object
/// and has nothing to do with floating point.
const soft_float_symbols = [_][]const u8{
    // ARM EABI helpers — what the compiler emits for `f32`/`f64` on this target.
    "__aeabi_fadd", "__aeabi_fsub", "__aeabi_fmul",  "__aeabi_fdiv",
    "__aeabi_dadd", "__aeabi_dsub", "__aeabi_dmul",  "__aeabi_ddiv",
    "__aeabi_i2f",  "__aeabi_i2d",  "__aeabi_ui2f",  "__aeabi_ui2d",
    "__aeabi_f2iz", "__aeabi_d2iz", "__aeabi_f2uiz", "__aeabi_d2uiz",
    "__aeabi_f2d",  "__aeabi_d2f",  "__aeabi_fcmp",  "__aeabi_dcmp",
    // The generic compiler-rt names, in case a lowering picks those instead.
    "__addsf3",     "__subsf3",     "__mulsf3",      "__divsf3",
    "__adddf3",     "__subdf3",     "__muldf3",      "__divdf3",
    "__floatsisf",  "__floatsidf",  "__fixsfsi",     "__fixdfsi",
};

/// The first soft-float symbol named in `bytes`, or null if there is none.
fn firstSoftFloat(bytes: []const u8) ?[]const u8 {
    for (soft_float_symbols) |sym| {
        if (std.mem.indexOf(u8, bytes, sym) != null) return sym;
    }
    return null;
}

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const arena = init.arena.allocator();
    const args = try init.minimal.args.toSlice(arena);
    if (args.len != 2) {
        std.debug.print("usage: no-float OBJECT\n", .{});
        std.process.exit(2);
    }
    const path = args[1];

    const bytes = std.Io.Dir.cwd().readFileAlloc(io, path, arena, .limited(64 * 1024 * 1024)) catch |err| {
        std.debug.print("no-float: unable to read {s}: {t}\n", .{ path, err });
        std.process.exit(1);
    };

    if (firstSoftFloat(bytes)) |sym| {
        std.debug.print(
            \\floating point reached the device build.
            \\
            \\  {s} references `{s}`, which is soft float out of compiler_rt --
            \\  tens to hundreds of cycles per operation on the M33.
            \\
            \\  The usual cause is an `@import("reference.zig")` that has been
            \\  hoisted out of a test body to the top of a file. Put it back
            \\  inside the test. The target flags do not catch this: subtracting
            \\  `fpregs` stops FP instructions being emitted, not floats being used.
            \\
        , .{ std.fs.path.basename(path), sym });
        std.process.exit(1);
    }
}

test "a soft-float helper is caught" {
    const object = "\x7fELF\x00padding\x00__aeabi_fmul\x00more";
    try std.testing.expectEqualStrings("__aeabi_fmul", firstSoftFloat(object).?);
}

test "the unwind personality routine is not mistaken for soft float" {
    // Present in every thumb object; a prefix match on `__aeabi_` would fail
    // every build.
    try std.testing.expectEqual(null, firstSoftFloat("\x00__aeabi_unwind_cpp_pr0\x00__aeabi_memcpy\x00"));
}
