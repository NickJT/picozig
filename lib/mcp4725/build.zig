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
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_picozig_board = b.dependency("pico2w", .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    const mod_mcp4725 = b.addModule("mcp4725", .{
        .root_source_file = b.path("src/mcp4725.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck mcp4725 driver");
    const obj = b.addObject(.{ .name = "mcp4725_check", .root_module = mod_mcp4725 });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run mcp4725 unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_mcp4725_host = b.createModule(.{
        .root_source_file = b.path("src/mcp4725.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });

    const t = b.addTest(.{ .root_module = mod_mcp4725_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Example: dac-output
    // ------------------------------------------------------------------
    const example_mod = b.createModule(.{
        .root_source_file = b.path("example/dac-output/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_picozig_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "mcp4725", .module = mod_mcp4725 },
        },
    });

    const exe = b.addExecutable(.{ .name = "dac-output", .root_module = example_mod });
    exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);

    // SWD flash step
    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd.addFileArg(exe.getEmittedBin());
    swd_cmd.has_side_effects = true;
    const swd_step = b.step("swd-dac-output", "Flash dac-output example over SWD");
    swd_step.dependOn(&swd_cmd.step);

    // SWD watch step (reads RAM watch block at 0x20000000)
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
        "mdw 0x20000000 9",
        "-c",
        "shutdown",
    });
    watch_cmd.has_side_effects = true;
    const watch_step = b.step("watch-dac-output", "Read dac-output RAM block over SWD");
    watch_step.dependOn(&watch_cmd.step);
}
