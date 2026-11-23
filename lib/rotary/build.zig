//! `lib/rotary` — quadrature decoding for encoder knobs.
//!
//!     zig build test    host tests over the decoder
//!     zig build check   typecheck host and freestanding, and run the float guard
//!
//! The source imports nothing but `std`, so the host tests are the real test
//! and the freestanding build only has to prove the same code compiles and
//! links for the device. The float guard is what makes that second build worth
//! running: `lib/picozig` subtracts `fpregs`, which stops FP *instructions*
//! being emitted but does not stop a stray `f32` resolving to soft float out of
//! `compiler_rt`. This package is integer throughout; this is what keeps that
//! true.

const std = @import("std");
// The soft-float guard. **Build-time only** — nothing in `src/` imports it,
// so the source still imports nothing but `std`.
const no_float = @import("no_float");

/// The device target, spelled out rather than depending on `picozig` for it —
/// this package is a driver with no platform dependency yet, and taking one on
/// the whole of `picozig` to borrow a target query would invert the layering.
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

    _ = b.addModule("rotary", .{
        .root_source_file = b.path("src/rotary.zig"),
        .target = target,
        .optimize = optimize,
    });

    const test_step = b.step("test", "Run the rotary decoder tests");
    const check_step = b.step("check", "Typecheck host and freestanding, and guard against floats");

    const tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/rotary.zig"),
            .target = b.graph.host,
            .optimize = .debug,
        }),
    });
    test_step.dependOn(&b.addRunArtifact(tests).step);
    check_step.dependOn(&tests.step);

    // Rooted at the smoke file, not at `rotary.zig` — see that file for why an
    // object with no exported symbols makes the guard below vacuous.
    const freestanding = b.addObject(.{
        .name = "rotary-freestanding",
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
