//! `lib/ads1115` — the TI ADS1115 16-bit ADC over I2C.
//!
//!     zig build test    host tests: codecs, scaling, and the driver against a fake bus
//!     zig build check   typecheck host and freestanding, and run the float guard
//!
//! The source imports nothing but `std` and is generic over the bus, so the
//! host tests are the real test. The hardware bring-up is a package of its own
//! in `example/bring-up`, so this one takes no dependency on `picozig`.

const std = @import("std");
// The soft-float guard. **Build-time only** — nothing in `src/` imports it,
// so the source still imports nothing but `std`.
const no_float = @import("no_float");

/// The device target, spelled out rather than borrowed from `picozig` — a
/// driver with no platform dependency should not take one for a target query.
const device_target: std.Target.Query = .{
    .cpu_arch = .thumb,
    .os_tag = .freestanding,
    .abi = .eabi,
    .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m33 },
    .cpu_features_sub = std.Target.arm.featureSet(&.{.fpregs}),
};

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    _ = b.addModule("ads1115", .{
        .root_source_file = b.path("src/ads1115.zig"),
        .target = target,
        .optimize = optimize,
    });

    const test_step = b.step("test", "Run the ads1115 tests");
    const check_step = b.step("check", "Typecheck host and freestanding, and guard against floats");

    const tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/ads1115.zig"),
            .target = b.graph.host,
            .optimize = .debug,
        }),
    });
    test_step.dependOn(&b.addRunArtifact(tests).step);
    check_step.dependOn(&tests.step);

    const freestanding = b.addObject(.{
        .name = "ads1115-freestanding",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/freestanding_smoke.zig"),
            .target = b.resolveTargetQuery(device_target),
            .optimize = .small,
        }),
    });
    b.getInstallStep().dependOn(&freestanding.step);
    check_step.dependOn(&freestanding.step);

    const guard = no_float.noFloat(b, freestanding.getEmittedBin());
    check_step.dependOn(guard);
    test_step.dependOn(guard);
}
