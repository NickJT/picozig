//! knob — Rung 1 of `lib/rotary`: one encoder and its switch, alone.
//!
//!     zig build          build
//!     zig build swd      flash over a Debug Probe
//!
//! A package of its own rather than a step in `lib/rotary`'s build, so the
//! library's `build.zig.zon` keeps no dependency on `picozig`: the decoder is
//! `std`-only, and a hardware example is not a reason to change that.

const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const probe_serial = picozig_build.probeSerialOption(b);
    const probe_select = picozig_build.probeSelect(b, probe_serial);
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    const optimize: std.lang.Optimize = .small;

    const picozig = b.dependency("picozig", .{ .target = target, .optimize = optimize });
    const rotary = b.dependency("rotary", .{ .target = target, .optimize = optimize });

    const mod = b.createModule(.{
        .root_source_file = b.path("main.zig"),
        .target = target,
        .optimize = optimize,
        // Keep symbols: a PC read over SWD must resolve to something.
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = picozig.module("rp2350") },
            .{ .name = "boot", .module = picozig.module("boot") },
            .{ .name = "rotary", .module = rotary.module("rotary") },
        },
    });

    const exe = b.addExecutable(.{ .name = "knob", .root_module = mod });
    exe.setLinkerScript(picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);

    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";
    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 5000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd.addFileArg(exe.getEmittedBin());
    swd_cmd.has_side_effects = true;
    const swd_step = b.step("swd", "Flash knob over SWD via a Debug Probe");
    swd_step.dependOn(&swd_cmd.step);
}
