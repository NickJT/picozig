const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });

    const mod_ws2812 = b.addModule("ws2812", .{
        .root_source_file = b.path("src/ws2812.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck ws2812 driver");
    const obj = b.addObject(.{ .name = "ws2812_check", .root_module = mod_ws2812 });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run ws2812 unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_ws2812_host = b.createModule(.{
        .root_source_file = b.path("src/ws2812.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });

    const t = b.addTest(.{ .root_module = mod_ws2812_host });
    test_step.dependOn(&b.addRunArtifact(t).step);
}
