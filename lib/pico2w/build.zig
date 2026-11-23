//! `lib/pico2w` — one board, as a package exporting a `board` module.
//!
//!     zig build test    the board's facts and pin claims, on the host
//!     zig build check   the same, analysed for the device
//!
//! A dependent takes it with the target and optimize mode it gives picozig:
//!
//!     const board = b.dependency("pico2w", .{ .target = target, .optimize = optimize });
//!     ... .{ .name = "board", .module = board.module("board") } ...
//!
//! The build itself is picozig's `boardPackage`, shared by every board.

const std = @import("std");

pub fn build(b: *std.Build) void {
    @import("picozig").boardPackage(b);
}
