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
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    // The clock the tone is generated at. `tone-check` has always run on the
    // bare crystal; an application that decodes audio runs at 150 MHz, and
    // "the I2S engine works" measured only at 12 MHz is a narrower claim than
    // it looks. 0 keeps the board default.
    const sys_mhz = b.option(u32, "sys-mhz", "clk_sys in MHz for tone-check (0 = board default)") orelse 0;
    // **Roadmap 2.1.** Off by default: a split tone is a diagnostic, not what
    // `tone-check` is for. On, it plays a different pitch in each channel so a
    // physical left ear settled a claim that had only ever been a host
    // assertion -- `packStereoFrame` putting left in the high half.
    const split = b.option(bool, "split", "different pitch per channel, for the channel-order check") orelse false;
    // **Quiet on purpose.** Straight into headphones with no volume control
    // anywhere in the path, so the table's amplitude is the only thing between
    // the DAC and someone's ears.
    const volume = b.option(u32, "volume-pct", "tone amplitude, percent of full scale") orelse 12;

    const tone_opts = b.addOptions();
    tone_opts.addOption(u32, "sys_mhz", sys_mhz);
    tone_opts.addOption(bool, "split", split);
    tone_opts.addOption(u32, "volume_pct", volume);

    // The board the examples run on — a Pico 2 W, which is what the Pico Audio Pack sits on. Only the examples read it, and
    // nothing is forwarded: no dependency of this package takes a board.
    const board_name = b.option(
        []const u8,
        "board",
        "Board the examples run on: pico2w (default), or any of pico_lipo2, pico2, pico2w, pico_plus2w",
    ) orelse "pico2w";

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency(board_name, .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    const mod_pcm5100 = b.addModule("pcm5100", .{
        .root_source_file = b.path("src/pcm5100.zig"),
        .target = target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
        },
    });

    const check_step = b.step("check", "Typecheck pcm5100 driver");
    const obj = b.addObject(.{ .name = "pcm5100_check", .root_module = mod_pcm5100 });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run pcm5100 unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_pcm5100_host = b.createModule(.{
        .root_source_file = b.path("src/pcm5100.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
        },
    });

    const t = b.addTest(.{ .root_module = mod_pcm5100_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Example: tone-check
    //
    // The only example in this package, and it is a measurement rather than a
    // demonstration. It reports the clock plan it computed and
    // the FIFO's own underrun flag, and says nothing about sound.
    // ------------------------------------------------------------------
    const tone_mod = b.createModule(.{
        .root_source_file = b.path("example/tone-check/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "pcm5100", .module = mod_pcm5100 },
            .{ .name = "tone_config", .module = tone_opts.createModule() },
        },
    });

    const exe_tone = b.addExecutable(.{ .name = "tone-check", .root_module = tone_mod });
    exe_tone.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_tone.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_tone);

    const swd_script = b.fmt(
        \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "program $1 verify reset exit"
    , .{ openocd, probe_select });

    const swd_cmd_tone = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_tone.addFileArg(exe_tone.getEmittedBin());
    swd_cmd_tone.has_side_effects = true;
    const swd_tone_step = b.step("swd-tone-check", "Flash tone-check example over SWD");
    swd_tone_step.dependOn(&swd_cmd_tone.step);

    const watch_cmd_tone = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    watch_cmd_tone.addArgs(probe_select_args);
    watch_cmd_tone.addArgs(&.{
        "-f",
        "target/rp2350.cfg",
        "-c",
        "adapter speed 1000",
        "-c",
        "init",
        "-c",
        "mdw 0x20000000 9",
        // **Leave the target running.** `init` halts the cores, and a halted
        // board makes no sound — which on an audio package turns every status
        // read into a silence that looks like a bug in the driver. `catch`
        // because resuming an already-running target is an error, not a no-op.
        "-c",
        "catch {resume}",
        "-c",
        "shutdown",
    });
    watch_cmd_tone.has_side_effects = true;
    const watch_tone_step = b.step("watch-tone-check", "Read tone-check RAM block over SWD");
    watch_tone_step.dependOn(&watch_cmd_tone.step);
}
