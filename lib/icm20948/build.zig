const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    // **Two probes on one host enumerate identically.** openocd takes
    // whichever it finds first, so without this it can flash the wrong board
    // and report success — and on a bench where both run near-identical
    // firmware, every observation afterwards is of the other one. Empty means
    // one probe, and every command below is then byte for byte what it was.
    // picozig holds the helpers; the serial is the probe's USB serial number.
    const probe_serial = picozig_build.probeSerialOption(b);
    const probe_select = picozig_build.probeSelect(b, probe_serial);
    const probe_select_args = picozig_build.probeSelectArgs(b, probe_serial);
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // The board the examples run on. Only the examples read it, and nothing is
    // forwarded: no dependency of this package takes a board.
    const board_name = b.option(
        []const u8,
        "board",
        "Board the examples run on: pico_lipo2 (default), pico2, pico2w or pico_plus2w",
    ) orelse "pico_lipo2";

    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    // ------------------------------------------------------------------
    // Public Modules
    // ------------------------------------------------------------------
    const icm20948_mod = b.addModule("icm20948", .{
        .root_source_file = b.path("src/icm20948.zig"),
        .target = target,
        .optimize = optimize,
    });

    const picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = optimize,
    });

    _ = b.addModule("bus_picozig", .{
        .root_source_file = b.path("src/bus_picozig.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "rp2350", .module = picozig.module("rp2350") },
            .{ .name = "icm20948", .module = icm20948_mod },
        },
    });

    // ------------------------------------------------------------------
    // Freestanding Executable for RP2350 (Pico LiPo 2) Bring-Up
    // ------------------------------------------------------------------
    const freestanding = b.resolveTargetQuery(picozig_build.pico_target_query);
    const fs_picozig = b.dependency("picozig", .{
        .target = freestanding,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency(board_name, .{ .target = freestanding, .optimize = std.lang.Optimize.small }).module("board");

    const fs_icm20948_mod = b.createModule(.{
        .root_source_file = b.path("src/icm20948.zig"),
        .target = freestanding,
        .optimize = .small,
    });

    const fs_bus_mod = b.createModule(.{
        .root_source_file = b.path("src/bus_picozig.zig"),
        .target = freestanding,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "icm20948", .module = fs_icm20948_mod },
        },
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("example/main.zig"),
        .target = freestanding,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "icm20948", .module = fs_icm20948_mod },
            .{ .name = "bus_picozig", .module = fs_bus_mod },
        },
    });

    const exe = b.addExecutable(.{
        .name = "icm20948-bringup",
        .root_module = exe_mod,
    });
    // ReleaseSmall strips the symbol table, and a PC read over SWD then resolves
    // to nothing — a panic cannot be told from a spin. Symbols live in ELF
    // metadata, never in a PT_LOAD segment, so this costs exactly zero flash.
    exe.root_module.strip = false;
    exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    // The image's true entry is the reset vector; there is no _start.
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);

    const check_step = b.step("check", "Typecheck the driver and the bring-up image");
    check_step.dependOn(&exe.step);

    // ------------------------------------------------------------------
    // SWD: flash and watch
    // ------------------------------------------------------------------
    // Run through `sh -c` because OpenOCD wants the filename and its options as
    // one -c argument, which addFileArg cannot express on its own. $1 is the ELF.
    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd.addFileArg(exe.getEmittedBin());
    // Talking to hardware is never up to date, and never cacheable.
    swd_cmd.has_side_effects = true;
    const swd_step = b.step("swd-icm20948-bringup", "Flash the bring-up image over SWD via a Debug Probe");
    swd_step.dependOn(&swd_cmd.step);

    // `Watch` in example/main.zig is 23 words. If a field is added there, this
    // count must move with it or the tail of the block reads as absent.
    const watch_cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    watch_cmd.addArgs(probe_select_args);
    watch_cmd.addArgs(&.{
        "-f",
        "target/rp2350.cfg",
        "-c",
        "adapter speed 1000",
        "-c",
        "init",
        "-c",
        "mdw 0x20000000 23",
        "-c",
        "shutdown",
    });
    watch_cmd.has_side_effects = true;
    const watch_step = b.step("watch-icm20948-bringup", "Read the bring-up RAM block over SWD");
    watch_step.dependOn(&watch_cmd.step);

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run ICM-20948 driver unit tests");
    const host_icm20948 = b.createModule(.{
        .root_source_file = b.path("src/icm20948.zig"),
        .target = b.graph.host,
        .optimize = .debug,
    });
    const t = b.addTest(.{ .root_module = host_icm20948 });
    test_step.dependOn(&b.addRunArtifact(t).step);
}
