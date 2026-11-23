//! hello-usb — the scratch app.
//!
//! A USB serial port that says which chip it is. Kept buildable so that a terminal and
//! `silicon.idHex` have somewhere to be exercised that is not a real project's
//! firmware.
//!
//! ```bash
//! zig build swd       # flash over a Debug Probe
//! zig build flash     # or drag the UF2 onto a BOOTSEL volume
//! ```
//!
//! **There is deliberately no `-Dboard` option here, and no `board`
//! import.** Everything this image prints is read from the chip; nothing is
//! read from a build flag. A board name comes from `-Dboard`, so an image
//! built with the wrong one states the wrong board confidently and in the same
//! breath as the facts that are true — which turns a banner meant to resolve
//! confusion into a source of it. This app cannot make that mistake because it
//! has no such setting: `rp2350` and `boot` do not read the board choice, and
//! `board.zig` is never imported.

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
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);

    // Fixed, and matching what `lib/usb` builds its own picozig with. Both
    // arguments have to agree exactly or `b.dependency` stops deduplicating
    // and the link fails with "file exists in modules 'rp2350' and 'rp2350'".
    const optimize: std.lang.Optimize = .small;

    const picozig = b.dependency("picozig", .{ .target = target, .optimize = optimize });
    // **No arguments**, and that is not an oversight: `lib/usb`'s build script
    // declares no `target` or `optimize` option — it resolves picozig's target
    // query itself and fixes ReleaseSmall — so passing them is an error. The
    // two lines still have to agree on what they resolve to, which is why the
    // constants above are picozig's query and ReleaseSmall rather than
    // `standardTargetOptions`.
    const usb = b.dependency("usb", .{});

    const mod = b.createModule(.{
        .root_source_file = b.path("main.zig"),
        .target = target,
        .optimize = optimize,
        // ReleaseSmall strips the symbol table, and a PC read over SWD then
        // resolves to nothing — a panic cannot be told from a spin.
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = picozig.module("rp2350") },
            // Side effects only: `boot` exports the vector table and `_reset`,
            // which nothing references by name.
            .{ .name = "boot", .module = picozig.module("boot") },
            .{ .name = "usb", .module = usb.module("usb") },
        },
    });

    const exe = b.addExecutable(.{ .name = "hello-usb", .root_module = mod });
    exe.setLinkerScript(picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);

    const check_step = b.step("check", "Check if hello-usb compiles");
    check_step.dependOn(&exe.step);

    const uf2_run = b.addRunArtifact(picozig.artifact("elf2uf2"));
    uf2_run.addFileArg(exe.getEmittedBin());
    const uf2_file = uf2_run.addOutputFileArg("hello-usb.uf2");
    b.getInstallStep().dependOn(
        &b.addInstallFileWithDir(uf2_file, .prefix, "hello-usb.uf2").step,
    );

    const default_volume = switch (@import("builtin").os.tag) {
        .macos => "/Volumes/RP2350",
        else => "/run/media/" ++ "$USER" ++ "/RP2350",
    };
    const pico_volume = b.option(
        []const u8,
        "pico-volume",
        "Mounted BOOTSEL volume to copy the UF2 onto (default: " ++ default_volume ++ ")",
    ) orelse default_volume;

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

    const flash_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", flash_script, "flash" });
    flash_cmd.has_side_effects = true;
    flash_cmd.addFileArg(uf2_file);
    flash_cmd.addArg(pico_volume);
    const flash_step = b.step("flash", "Copy hello-usb.uf2 onto a BOOTSEL-mounted board");
    flash_step.dependOn(&flash_cmd.step);

    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";
    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 5000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });
    const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd.addFileArg(exe.getEmittedBin());
    swd_cmd.has_side_effects = true;
    const swd_step = b.step("swd", "Flash hello-usb over SWD via a Raspberry Pi Debug Probe");
    swd_step.dependOn(&swd_cmd.step);
}
