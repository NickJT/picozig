//! no-float — the soft-float guard for a freestanding object.
//!
//!     zig build test    the scanner's own tests
//!
//! A build helper, not a module: a dependent's `build.zig` does
//! `@import("no_float")` and calls `noFloat`, so a package taking it acquires
//! no module dependency and its source can still claim to import only `std`.
//!
//! **Why it exists.** Subtracting `fpregs` from the target stops FP
//! *instructions* being emitted, but a stray `f32` still compiles cleanly — to
//! soft float out of `compiler_rt`, tens to hundreds of cycles per operation.
//! Nothing warns. This turns it into a build failure.
//!
//! **Why it is a package of its own.** Its callers are drivers and libraries
//! that deliberately take no platform dependency, so it cannot live in
//! `picozig`; and it is build-time only, so it has no business in a library of
//! modules either.

const std = @import("std");

pub fn build(b: *std.Build) void {
    const test_step = b.step("test", "Run the scanner's tests");
    const check_step = b.step("check", "Typecheck the scanner");

    // Always built for the machine running the build, whatever `-Dtarget` says,
    // and installed so `noFloat` can find it by name.
    const mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = b.graph.host,
        .optimize = .safe,
    });
    const exe = b.addExecutable(.{ .name = "no-float", .root_module = mod });
    b.installArtifact(exe);
    check_step.dependOn(&exe.step);
    test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = mod })).step);
}

/// A step that fails the build if the freestanding `object` references a
/// soft-float symbol. Depend `check` and `test` on it.
///
/// The scan is `src/main.zig`, a host executable this package installs; the
/// build system has no custom steps to run it in-process.
pub fn noFloat(b: *std.Build, object: std.Build.LazyPath) *std.Build.Step {
    const self = b.dependencyFromBuildZig(@This(), .{});
    const run = b.addRunArtifact(self.artifact("no-float"));
    run.setName("no-float: the freestanding object must not use soft float");
    run.addFileArg(object);
    return &run.step;
}
