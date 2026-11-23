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

    // The board the examples run on. Only the examples read it, and
    // nothing is forwarded: no dependency of this package takes a board.
    const board_name = b.option(
        []const u8,
        "board",
        "Board the examples run on: pico_lipo2 (default), or any of pico_lipo2, pico2, pico2w, pico_plus2w",
    ) orelse "pico_lipo2";
    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency(board_name, .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    const mod_usb = b.addModule("usb", .{
        .root_source_file = b.path("src/usb.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck the USB device stack");
    const obj = b.addObject(.{ .name = "usb_check", .root_module = mod_usb });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (host)
    // ------------------------------------------------------------------
    // Descriptors are wire format and standard-request decoding is pure byte
    // arithmetic, so all of it host-tests. That is the whole reason this half
    // is a separate package from the controller: none of it needs a chip.
    const test_step = b.step("test", "Run USB device stack unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });
    const mod_usb_host = b.createModule(.{
        .root_source_file = b.path("src/usb.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });
    const t = b.addTest(.{ .root_module = mod_usb_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Example: loopback
    // ------------------------------------------------------------------
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    const example_mod = b.createModule(.{
        .root_source_file = b.path("example/loopback/src/main.zig"),
        .target = target,
        .optimize = .small,
        // ReleaseSmall strips the symbol table, and a PC read over SWD then
        // resolves to nothing — a panic cannot be told from a spin.
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "usb", .module = mod_usb },
        },
    });

    const exe = b.addExecutable(.{ .name = "usb-loopback", .root_module = example_mod });
    exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);
    check_step.dependOn(&exe.step);

    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd.addFileArg(exe.getEmittedBin());
    swd_cmd.has_side_effects = true;
    const swd_step = b.step("swd-loopback", "Flash the USB loopback example over SWD");
    swd_step.dependOn(&swd_cmd.step);

    // `Watch` in example/loopback/src/main.zig is 12 words.
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
        "mdw 0x20000000 12",
        "-c",
        "shutdown",
    });
    watch_cmd.has_side_effects = true;
    const watch_step = b.step("watch-loopback", "Read the loopback RAM block over SWD");
    watch_step.dependOn(&watch_cmd.step);

    // ------------------------------------------------------------------
    // Example: serial (CDC-ACM)
    // ------------------------------------------------------------------
    const serial_mod = b.createModule(.{
        .root_source_file = b.path("example/serial/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "usb", .module = mod_usb },
        },
    });

    const serial = b.addExecutable(.{ .name = "usb-serial", .root_module = serial_mod });
    serial.setLinkerScript(fs_picozig.path("rp2350.ld"));
    serial.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(serial);
    check_step.dependOn(&serial.step);

    const serial_swd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    serial_swd.addFileArg(serial.getEmittedBin());
    serial_swd.has_side_effects = true;
    const serial_swd_step = b.step("swd-serial", "Flash the USB serial example over SWD");
    serial_swd_step.dependOn(&serial_swd.step);

    // ------------------------------------------------------------------
    // Example: usb-trace
    // ------------------------------------------------------------------
    // The type-2 example: a serial port that narrates every control transfer
    // over the SWD UART. See `example/usb-trace/src/main.zig`.
    const trace_mod = b.createModule(.{
        .root_source_file = b.path("example/usb-trace/src/main.zig"),
        .target = target,
        .optimize = .small,
        // ReleaseSmall strips the symbol table, and a PC read over SWD then
        // resolves to nothing — a panic cannot be told from a spin.
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "usb", .module = mod_usb },
        },
    });

    const trace_exe = b.addExecutable(.{ .name = "usb-trace", .root_module = trace_mod });
    trace_exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    trace_exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(trace_exe);
    check_step.dependOn(&trace_exe.step);

    const trace_swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    trace_swd_cmd.addFileArg(trace_exe.getEmittedBin());
    trace_swd_cmd.has_side_effects = true;
    const trace_swd_step = b.step("swd-usb-trace", "Flash the usb-trace example over SWD");
    trace_swd_step.dependOn(&trace_swd_cmd.step);
}
