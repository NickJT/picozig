//! pcm5122 — the PCM5122's control port, over I²C.
//!
//!     zig build test                 host tests for the register map and volume
//!     zig build check                typecheck for the device
//!     zig build swd-control-check    flash the control-port check over SWD
//!     zig build watch-control-check  read its result block over SWD
//!
//! `control-check` proves the driver against the real part over I²C alone — no
//! I²S, so it is everything that can be shown before the audio path exists.
//!
//! The I²S path is deliberately absent: audio reaches this part exactly as it
//! reaches the PCM5100A, and `lib/pcm5100` already drives that. What the 5122
//! adds is a control port, and a control port is all this package is.

const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);

    // The board the examples run on. **Defaults to `pico2`, the board the PCM5122 breakout is wired to**. Only the examples read it, and
    // nothing is forwarded: no dependency of this package takes a board.
    const board_name = b.option(
        []const u8,
        "board",
        "Board the examples run on: pico2 (default), or any of pico_lipo2, pico2, pico2w, pico_plus2w",
    ) orelse "pico2";

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency(board_name, .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    _ = b.addModule("pcm5122", .{
        .root_source_file = b.path("src/pcm5122.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    // ------------------------------------------------------------------
    // Checks and tests
    // ------------------------------------------------------------------
    // The device build is what catches a register map that does not compile
    // freestanding; the host build is what runs the arithmetic. The volume
    // encoding is the part worth testing — it runs downward from +24 dB, and
    // an off-by-one there is a level that is wrong everywhere and obviously
    // wrong nowhere.
    const check_step = b.step("check", "Typecheck pcm5122 for the device");
    const check_mod = b.createModule(.{
        .root_source_file = b.path("src/pcm5122.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{.{ .name = "rp2350", .module = fs_picozig.module("rp2350") }},
    });
    check_step.dependOn(&b.addObject(.{ .name = "pcm5122_check", .root_module = check_mod }).step);

    const host_picozig = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });
    const host_mod = b.createModule(.{
        .root_source_file = b.path("src/pcm5122.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{.{ .name = "rp2350", .module = host_picozig.module("rp2350") }},
    });
    const t = b.addTest(.{ .root_module = host_mod });
    const test_step = b.step("test", "Run the pcm5122 unit tests");
    test_step.dependOn(&b.addRunArtifact(t).step);
    check_step.dependOn(&t.step);

    // ------------------------------------------------------------------
    // example/control-check
    // ------------------------------------------------------------------
    const probe_serial = picozig_build.probeSerialOption(b);
    const probe_select = picozig_build.probeSelect(b, probe_serial);
    const probe_select_args = picozig_build.probeSelectArgs(b, probe_serial);
    const openocd = b.option([]const u8, "openocd", "openocd binary (default: openocd)") orelse "openocd";

    const example_mod = b.createModule(.{
        .root_source_file = b.path("example/control-check/src/main.zig"),
        .target = target,
        .optimize = .small,
        // Keep the symbol table: ReleaseSmall strips it, and a PC read over SWD
        // then resolves to nothing at exactly the moment it is wanted. Symbols
        // live in ELF metadata, so they cost zero flash.
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "pcm5122", .module = b.createModule(.{
                .root_source_file = b.path("src/pcm5122.zig"),
                .target = target,
                .optimize = .small,
                .imports = &.{.{ .name = "rp2350", .module = fs_picozig.module("rp2350") }},
            }) },
        },
    });
    const exe = b.addExecutable(.{ .name = "control-check", .root_module = example_mod });
    exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);
    check_step.dependOn(&exe.step);

    // **Never `-c halt` on RP2350**: it takes only cm0, the following resume
    // fails with `[rp2350.cm1] not halted`, and the chip is left stopped — which
    // looks exactly like the hang being diagnosed.
    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd.addFileArg(exe.getEmittedBin());
    swd.has_side_effects = true;
    b.step("swd-control-check", "Flash control-check over SWD").dependOn(&swd.step);

    const watch = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    watch.addArgs(probe_select_args);
    watch.addArgs(&.{
        "-f", "target/rp2350.cfg",
        "-c", "adapter speed 1000",
        "-c", "init",
        // Reads only; the core keeps running. 17 words is the Watch struct.
        "-c", "mdw 0x20000000 17",
        "-c", "shutdown",
    });
    watch.has_side_effects = true;
    b.step("watch-control-check", "Read control-check's result block over SWD").dependOn(&watch.step);
}
