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
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_picozig_board = b.dependency("pico2w", .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    const mod_drv8833c = b.addModule("drv8833c", .{
        .root_source_file = b.path("src/drv8833c.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck drv8833c driver");
    const obj = b.addObject(.{ .name = "drv8833c_check", .root_module = mod_drv8833c });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run drv8833c unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_drv8833c_host = b.createModule(.{
        .root_source_file = b.path("src/drv8833c.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });

    const t = b.addTest(.{ .root_module = mod_drv8833c_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Example: stepper
    // ------------------------------------------------------------------
    const example_mod = b.createModule(.{
        .root_source_file = b.path("example/stepper/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_picozig_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "drv8833c", .module = mod_drv8833c },
        },
    });

    const exe = b.addExecutable(.{ .name = "stepper", .root_module = example_mod });
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
    const swd_step = b.step("swd-stepper", "Flash stepper example over SWD");
    swd_step.dependOn(&swd_cmd.step);
}
