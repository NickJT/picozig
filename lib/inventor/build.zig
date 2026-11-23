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

    // The Inventor 2350 W carries a Pico 2 W, so its examples always build for
    // that board; there is nothing to choose.

    // ------------------------------------------------------------------
    // Public Modules
    // ------------------------------------------------------------------

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency("pico2w", .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    const fs_ws2812 = b.dependency("ws2812", .{});

    const fs_servo = b.dependency("servo", .{});

    const fs_inventor = b.createModule(.{
        .root_source_file = b.path("src/inventor.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "ws2812", .module = fs_ws2812.module("ws2812") },
            .{ .name = "servo", .module = fs_servo.module("servo") },
        },
    });

    const fs_bus = b.createModule(.{
        .root_source_file = b.path("src/bus_picozig.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const pico_volume = b.option(
        []const u8,
        "pico-volume",
        "Mounted BOOTSEL volume for the UF2 route (default: /Volumes/RP2350)",
    ) orelse "/Volumes/RP2350";

    // **The volume disappearing is the success signal, not `cp`'s exit code.**
    // macOS `cp` carries extended attributes FAT32 refuses with EPERM, and the
    // board unmounts itself the instant it takes the final block — so the copy
    // can report failure on a flash that worked, which is how a good flash came
    // to be retried. `-X` drops the attributes; a volume still mounted
    // afterwards is the real failure, and no volume at all is the BOOTSEL
    // button not having been held while the cable went in.
    const flash_script =
        \\set -u
        \\UF2="$1"
        \\VOL="$2"
        \\[ -d "$VOL" ] || { echo "$VOL is not mounted - hold BOOTSEL while connecting" >&2; exit 1; }
        \\cp -X "$UF2" "$VOL/" || echo "cp reported an error; checking whether the board took it anyway"
        \\for _ in 1 2 3 4 5; do
        \\  [ -d "$VOL" ] || { echo "$VOL unmounted - the board accepted the image and rebooted"; exit 0; }
        \\  sleep 1
        \\done
        \\echo "$VOL is still mounted - the image was NOT accepted" >&2
        \\exit 1
    ;
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    const FirmwareExample = struct { name: []const u8, watch_words: u32 };
    const firmware = [_]FirmwareExample{
        .{ .name = "board-check", .watch_words = 5 },
        .{ .name = "leds", .watch_words = 5 },
        .{ .name = "servo", .watch_words = 5 },
    };

    const check_step = b.step("check", "Typecheck for firmware");
    const fs_checks = [_]struct { name: []const u8, mod: *std.Build.Module }{
        .{ .name = "inventor_firmware", .mod = fs_inventor },
        .{ .name = "bus_firmware", .mod = fs_bus },
    };
    for (fs_checks) |c| {
        const obj = b.addObject(.{ .name = c.name, .root_module = c.mod });
        b.getInstallStep().dependOn(&obj.step);
        check_step.dependOn(&obj.step);
    }

    for (firmware) |fwe| {
        const mod = b.createModule(.{
            .root_source_file = b.path(b.fmt("example/{s}/src/main.zig", .{fwe.name})),
            .target = target,
            .optimize = .small,
            .strip = false,
            .imports = &.{
                .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
                .{ .name = "board", .module = fs_board },
                .{ .name = "boot", .module = fs_picozig.module("boot") },
                .{ .name = "inventor", .module = fs_inventor },
                .{ .name = "bus_picozig", .module = fs_bus },
                .{ .name = "ws2812", .module = fs_ws2812.module("ws2812") },
                .{ .name = "servo", .module = fs_servo.module("servo") },
            },
        });
        const exe = b.addExecutable(.{ .name = fwe.name, .root_module = mod });
        exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
        exe.entry = .{ .symbol_name = "_reset" };
        b.installArtifact(exe);

        check_step.dependOn(&exe.step);

        const uf2_run = b.addRunArtifact(fs_picozig.artifact("elf2uf2"));
        uf2_run.addFileArg(exe.getEmittedBin());
        const uf2_file = uf2_run.addOutputFileArg(b.fmt("{s}.uf2", .{fwe.name}));
        b.getInstallStep().dependOn(
            &b.addInstallFileWithDir(uf2_file, .prefix, b.fmt("{s}.uf2", .{fwe.name})).step,
        );

        const flash_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", flash_script, "flash" });
        flash_cmd.has_side_effects = true;
        flash_cmd.addFileArg(uf2_file);
        flash_cmd.addArg(pico_volume);
        const flash_step = b.step(
            b.fmt("flash-{s}", .{fwe.name}),
            b.fmt("Copy {s}.uf2 onto a BOOTSEL-mounted board", .{fwe.name}),
        );
        flash_step.dependOn(&flash_cmd.step);

        const swd_script = b.fmt(
            \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
            \\  -c "adapter speed 1000" \
            \\  -c "program $1 verify reset exit"
        , .{ openocd, probe_select });
        const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
        swd_cmd.addFileArg(exe.getEmittedBin());
        swd_cmd.has_side_effects = true;
        const swd_step = b.step(
            b.fmt("swd-{s}", .{fwe.name}),
            b.fmt("Flash {s} over SWD via a Raspberry Pi Debug Probe", .{fwe.name}),
        );
        swd_step.dependOn(b.getInstallStep());
        swd_step.dependOn(&swd_cmd.step);

        const watch_cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
        watch_cmd.addArgs(probe_select_args);
        watch_cmd.addArgs(&.{
            "-f",
            "target/rp2350.cfg",
            "-c",
            "adapter speed 1000",
            "-c",
            "init",
            b.fmt("-c", .{}),
            b.fmt("mdw 0x20000000 {d}", .{fwe.watch_words}),
            "-c",
            "shutdown",
        });
        watch_cmd.has_side_effects = true;
        const watch_step = b.step(
            b.fmt("watch-{s}", .{fwe.name}),
            b.fmt("Read {s}'s self-test block over SWD", .{fwe.name}),
        );
        watch_step.dependOn(&watch_cmd.step);
    }

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run inventor driver unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });
    const fs_ws2812_host = b.dependency("ws2812", .{});
    const fs_servo_host = b.dependency("servo", .{});

    const inventor_mod = b.createModule(.{
        .root_source_file = b.path("src/inventor.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
            .{ .name = "ws2812", .module = fs_ws2812_host.module("ws2812") },
            .{ .name = "servo", .module = fs_servo_host.module("servo") },
        },
    });

    const t = b.addTest(.{ .root_module = inventor_mod });
    test_step.dependOn(&b.addRunArtifact(t).step);
}
