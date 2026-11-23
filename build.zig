//! picozig — one command across every package.
//!
//!     zig build           build everything, examples and apps included
//!     zig build test      run every package's host tests
//!     zig build check     typecheck everything, for the device as well
//!     zig build list      print the packages and what each supports
//!
//! Each package is independent, with its own `build.zig` and `build.zig.zon`,
//! and builds and tests on its own: `cd lib/picozig && zig build test`. This
//! file only runs each one's own `zig build`, so it holds no build logic of its
//! own and deleting it breaks nothing.

const std = @import("std");

/// Every package, libraries before the apps that use them, so a failure
/// reports at its cause rather than downstream.
///
/// `tests` is false where a package has no test step on purpose: the apps are
/// freestanding firmware and cannot run on the host, so their logic lives in
/// libraries whose tests do.
const packages = [_]Package{
    .{ .path = "lib/no-float", .tests = true },
    .{ .path = "lib/picozig", .tests = true },
    // One package per board, each exporting a `board` module; the application
    // chooses. picozig takes no board, so nothing forwards a choice.
    .{ .path = "lib/pico2", .tests = true },
    .{ .path = "lib/pico2w", .tests = true },
    .{ .path = "lib/pico-lipo2", .tests = true },
    .{ .path = "lib/pico-plus2w", .tests = true },
    // picozig's getting-started firmware: a package of its own, because it
    // needs a board and every board needs picozig.
    .{ .path = "lib/picozig/example", .tests = false },
    .{ .path = "lib/usb", .tests = true },
    .{ .path = "lib/cyw43", .tests = true },
    .{ .path = "lib/ipnet", .tests = true },
    .{ .path = "lib/pcm5100", .tests = true },
    .{ .path = "lib/pcm5122", .tests = true },
    .{ .path = "lib/ads1115", .tests = true },
    .{ .path = "lib/bme688", .tests = true },
    .{ .path = "lib/drv8833c", .tests = true },
    .{ .path = "lib/ds3231", .tests = true },
    .{ .path = "lib/icm20948", .tests = true },
    .{ .path = "lib/mcp4725", .tests = true },
    .{ .path = "lib/rotary", .tests = true },
    .{ .path = "lib/rv3028", .tests = true },
    .{ .path = "lib/servo", .tests = true },
    .{ .path = "lib/ws2812", .tests = true },
    .{ .path = "lib/inventor", .tests = true },
    .{ .path = "app/hello-world/hello-uart", .tests = false },
    .{ .path = "app/hello-world/hello-usb", .tests = false },
    .{ .path = "app/bt-speaker", .tests = false },
};

const Package = struct {
    path: []const u8,
    /// Whether `zig build test` means anything here.
    tests: bool,
};

pub fn build(b: *std.Build) void {
    const all_step = b.getInstallStep();
    const test_step = b.step("test", "Run every package's tests");
    const check_step = b.step("check", "Typecheck every package");

    for (packages) |pkg| {
        all_step.dependOn(subBuild(b, pkg.path, &.{}));
        check_step.dependOn(subBuild(b, pkg.path, &.{"check"}));
        if (pkg.tests) test_step.dependOn(subBuild(b, pkg.path, &.{"test"}));
    }

    var listing: std.ArrayList(u8) = .empty;
    listing.appendSlice(b.allocator, "packages (dependency order):\n") catch @panic("OOM");
    for (packages) |pkg| {
        listing.appendSlice(b.allocator, b.fmt(
            "  {s: <28} {s}\n",
            .{ pkg.path, if (pkg.tests) "build check test" else "build check" },
        )) catch @panic("OOM");
    }
    const list = b.addSystemCommand(&.{ "printf", "%s" });
    list.addArg(listing.items);
    list.has_side_effects = true;
    b.step("list", "List the packages").dependOn(&list.step);
}

/// Runs `zig build [args...]` inside `path`, with the same zig that is running
/// this build, so a sub-build can never be compiled by a different toolchain.
fn subBuild(b: *std.Build, path: []const u8, args: []const []const u8) *std.Build.Step {
    const run = b.addSystemCommand(&.{ b.graph.zig_exe, "build" });
    run.addArgs(args);
    run.setCwd(b.path(path));
    // A sub-build's own cache decides what is up to date; this wrapper cannot
    // know, so it must not claim the result is cacheable.
    run.has_side_effects = true;
    run.setName(b.fmt("{s}: zig build {s}", .{ path, if (args.len > 0) args[0] else "" }));
    return &run.step;
}
