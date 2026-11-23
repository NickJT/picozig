const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    // Accepted and unused: every package in this workspace takes it, so a
    // top-level `zig build -Dopenocd=...` reaches all of them uniformly. This
    // one has nothing to flash yet.
    _ = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)");

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });

    const mod_ds3231 = b.addModule("ds3231", .{
        .root_source_file = b.path("src/ds3231.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck ds3231 driver");
    const obj = b.addObject(.{ .name = "ds3231_check", .root_module = mod_ds3231 });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run ds3231 unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_ds3231_host = b.createModule(.{
        .root_source_file = b.path("src/ds3231.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });

    const t = b.addTest(.{ .root_module = mod_ds3231_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // **No example yet, on purpose.** The comparison against `lib/rv3028` is
    // the experiment worth running, and it wants one program driving both parts
    // on one chain rather than an example per driver. Until
    // there is a chain to run it on, this package is a library and its tests.
}
