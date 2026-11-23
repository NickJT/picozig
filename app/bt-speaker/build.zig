//! bt-speaker — a Bluetooth A2DP sink on a Pico 2 W with a Pimoroni Audio Pack.
//!
//! **An app rather than an example because it needs two libraries.**
//! `lib/cyw43` carries the Bluetooth stack and `lib/pcm5100` the I2S engine;
//! neither may depend on the other, so the place they meet is here.

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

    // **150 MHz, and not by default.** At the crystal's 12 MHz an SBC decoder
    // does not fit, and raising clk_sys
    // without `GspiBus.setSysHz` takes the gSPI bus with it and fails at rung 1.
    // `board.clock_plan` stays null for every other board in the workspace.
    const sys_mhz = b.option(u32, "sys-mhz", "clk_sys in MHz (default 150)") orelse 150;
    const required = b.option(u32, "required", "unused here; kept so the shared options module matches") orelse 41000;

    // **The gSPI bus rate. Default 4 — 18.75 MHz — since 2026-08-27.**
    // 0 restores the old behaviour: `setSysHz` holds the bus at 3 MHz whatever
    // clk_sys is, which is what every audio run before that date used. Any
    // other value is a PIO divisor, so `-Dbus-div=2` is `clk_sys / 4`, the
    // 37.5 MHz `lib/cyw43` rungs A through C2 established — and which still
    // fails here at the HCI stage.
    //
    // **Moved on a soak, as this file's own note required.** One album,
    // 51.9 minutes unbroken: 0 missed flips, 0 dropped frames, 0 overruns in
    // any phase, and 2,944 frames of silence padding in total — 67 ms in
    // 52 minutes. The number that decided it is **worst `bt.read`: 2,812 us at
    // 3 MHz against 602 us here**, measured against a 5,804 us buffer period.
    // That is the audio-relevant figure: a single read went from consuming
    // half the deadline to a tenth of it. Wall-time share moved much less
    // (6.6% to 4.7%), and is not the reason.
    const bus_div = b.option(u32, "bus-div", "gSPI PIO divisor (0 = the old 3 MHz)") orelse 4;

    const opts = b.addOptions();
    opts.addOption(u32, "sys_mhz", sys_mhz);
    opts.addOption(u32, "required", required);
    opts.addOption(u32, "bus_div", bus_div);

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency("pico2w", .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    // The combined firmware image: the WiFi-only build has no Bluetooth
    // subsystem for the shared bus to talk to.
    // **Neither package takes `target` or `optimize`**: both resolve the Pico
    // target themselves from picozig, so passing one is an error rather than a
    // no-op.
    // **`ReleaseFast`, and measured rather than assumed.** At `ReleaseSmall`
    // one media packet took 41.7 ms to decode against a 14.5 ms budget.
    const dep_cyw43 = b.dependency("cyw43", .{
        .fw = @as([]const u8, "combined"),
        .opt = std.lang.Optimize.fast,
    });
    const dep_pcm = b.dependency("pcm5100", .{});

    const mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = .fast,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = dep_cyw43.module("cyw43") },
            .{ .name = "pcm5100", .module = dep_pcm.module("pcm5100") },
            .{ .name = "throughput_config", .module = opts.createModule() },
        },
    });

    const exe = b.addExecutable(.{ .name = "bt-speaker", .root_module = mod });

    // **`zig build check` at the root has been broken since this app existed.**
    // The root build calls `check` on every package (`build.zig:88`) and this
    // one never defined it, so the workspace-wide typecheck has always failed
    // here — silently, because nobody
    // runs it when the thing they are working on builds. Found by the
    // reconciliation pass on 2026-08-27.
    //
    // Not the same omission as the missing `test` step, which is deliberate and
    // documented: there is nothing here to unit-test that does not belong in
    // `lib/cyw43`. A typecheck has no such argument against it.
    const check_obj = b.addObject(.{ .name = "bt_speaker_check", .root_module = mod });
    b.step("check", "Typecheck bt-speaker").dependOn(&check_obj.step);
    exe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe);

    // ------------------------------------------------------------------
    // UF2, for a board whose SWD pads are inside a box
    // ------------------------------------------------------------------
    // **Added because this is becoming dedicated hardware.** SWD is the better
    // route while a board is on the bench — it needs no BOOTSEL dance and
    // carries symbols for GDB — but it needs three wires onto pads that a
    // finished enclosure may not reach. A UF2 is a drag onto a mass-storage
    // volume with nothing but the USB cable that is already there.
    //
    // The packer is picozig's, since it is the thing that knows the RP2350
    // family id — an RP2040 bootloader silently ignores blocks carrying it,
    // which is a useful property rather than a hazard: the wrong board simply
    // does not take the image.
    const uf2 = b.addRunArtifact(fs_picozig.artifact("elf2uf2"));
    uf2.addFileArg(exe.getEmittedBin());
    const uf2_file = uf2.addOutputFileArg("bt-speaker.uf2");
    b.getInstallStep().dependOn(
        &b.addInstallFileWithDir(uf2_file, .prefix, "bt-speaker.uf2").step,
    );

    // The volume is the board's own label, not something this build controls,
    // so it is overridable.
    const default_volume = switch (@import("builtin").os.tag) {
        .macos => "/Volumes/RP2350",
        else => "/run/media/" ++ "$USER" ++ "/RP2350",
    };
    const pico_volume = b.option(
        []const u8,
        "pico-volume",
        "Mounted BOOTSEL volume to copy the UF2 onto (default: " ++ default_volume ++ ")",
    ) orelse default_volume;

    // **A plain `cp` reports failure on a flash that worked**, which is worse
    // than either outcome on its own — it teaches you to ignore the exit code.
    // Two things make it fail after the data has already landed: macOS `cp`
    // carries extended attributes, which FAT32 refuses with EPERM, and the
    // board unmounts itself the instant it accepts the final block, so the
    // close lands on a volume that is no longer there.
    //
    // So `-X` drops the attributes, and **the volume disappearing is the
    // success signal** rather than the exit code of the copy. A volume still
    // mounted afterwards means the image was not taken — wrong family id, or a
    // full volume — and that is the case worth failing on.
    const flash_script =
        \\set -u
        \\UF2="$1"
        \\VOL="$2"
        \\cp -X "$UF2" "$VOL/" || echo "cp reported an error; checking whether the board took it anyway"
        \\for _ in 1 2 3 4 5; do
        \\  [ -d "$VOL" ] || { echo "$VOL unmounted - the board accepted the image and rebooted"; exit 0; }
        \\  sleep 1
        \\done
        \\echo "$VOL is still mounted - the image was NOT accepted" >&2
        \\exit 1
    ;
    const flash_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", flash_script, "flash" });
    flash_cmd.addFileArg(uf2_file);
    flash_cmd.addArg(pico_volume);
    flash_cmd.has_side_effects = true;
    b.step("flash", "Copy bt-speaker.uf2 onto a BOOTSEL-mounted Pico 2 W")
        .dependOn(&flash_cmd.step);

    const swd_script =
        \\set -eu
        \\EXE="$1"
        \\SERIAL="${2:-}"
        \\if [ -n "$SERIAL" ]; then set -- -c "adapter serial $SERIAL"; else set --; fi
        \\openocd -f interface/cmsis-dap.cfg "$@" -f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "init" -c "targets rp2350.cm0" -c "reset halt" \
        \\  -c "program {$EXE} verify reset" \
        \\  -c "shutdown"
    ;
    const swd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd.addFileArg(exe.getEmittedBin());
    swd.addArg(probe_serial);
    swd.has_side_effects = true;
    b.step("swd", "Flash bt-speaker over SWD").dependOn(&swd.step);

    const watch_script = b.fmt(
        \\openocd -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "init" -c "targets rp2350.cm0" \
        \\  -c "mdw 0x20000000 32" \
        \\  -c "shutdown"
    , .{probe_select});
    const watch = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch.has_side_effects = true;
    b.step("watch", "Read the bt-speaker watch block over SWD").dependOn(&watch.step);
}
