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

    // **Which board the examples compile against.** Only the examples read it,
    // and nothing is forwarded: no dependency of this package takes a board. The CYW43's four pins
    // are GP23/24/25/29 on every board this driver has met — a Pico 2 W, an
    // RM2 module and a Pico Plus 2 W all agree, and the pico-sdk's own board
    // headers say so — so this changes almost nothing about the wireless
    // driver. What it changes is whether the package and GPIO count the image
    // declares match the silicon it runs on, which is the difference between a
    // rig that reports its own conditions honestly and one that quietly
    // mislabels them. Rung A exists because of exactly that class of mistake.
    const board_name = b.option(
        []const u8,
        "board",
        "Board the examples run on: pico2w (default), pico_plus2w, pico_lipo2 or pico2",
    ) orelse "pico2w";

    const fs_picozig = b.dependency("picozig", .{
        .target = target,
        .optimize = std.lang.Optimize.small,
    });
    const fs_board = b.dependency(board_name, .{ .target = target, .optimize = std.lang.Optimize.small }).module("board");

    var ssid_opt = b.option([]const u8, "ssid", "Wi-Fi SSID") orelse "";
    var pass_opt = b.option([]const u8, "password", "Wi-Fi Password") orelse "";

    if (ssid_opt.len == 0) {
        // Read while configuring, so it must be declared: the configuration is
        // cached, and an undeclared read keeps stale credentials. The directory
        // catches the file appearing or going; a missing file cannot itself be
        // declared, which fails the configuration.
        b.dependOnDirectoryContents(b.path("."));
        if (b.root.root_dir.handle.readFileAlloc(b.graph.io, "wifi_credentials.json", b.allocator, .limited(4096))) |content| {
            b.dependOnFileContents(b.path("wifi_credentials.json"));
            const Creds = struct { ssid: []const u8 = "", password: []const u8 = "" };
            const parsed = std.json.parseFromSlice(Creds, b.allocator, content, .{ .ignore_unknown_fields = true }) catch null;
            if (parsed) |res| {
                ssid_opt = res.value.ssid;
                pass_opt = res.value.password;
            }
        } else |_| {}
    }

    if (ssid_opt.len == 0) {
        std.debug.print(
            \\
            \\================================================================================
            \\NOTICE: No Wi-Fi credentials found for the cyw43-blink target!
            \\
            \\Provide credentials via CLI flags:
            \\  zig build swd-cyw43-blink -Dssid="<SSID>" -Dpassword="<PASS>"
            \\
            \\OR create 'wifi_credentials.json' in lib/cyw43/:
            \\  {{"ssid": "<SSID>", "password": "<PASS>"}}
            \\================================================================================
            \\
        , .{});
    }

    // Credentials only.
    //
    // **Do not add a build-time clock here.** There was a `build_timestamp`
    // option carrying the host wall clock into the firmware, and the only thing
    // that ever read it was the fake NTP datagram the transport used to
    // synthesise: it reported `build_timestamp + uptime`, which is correct at
    // the moment of flashing and advances at exactly 1 Hz. That is
    // indistinguishable by eye from a working NTP client, and it was mistaken
    // for one. A device that needs the time must get it from the network or
    // from an RTC — being unable to fake it convincingly is the point.

    // Which firmware image to embed.
    //
    // **The two are different builds of the WiFi microcode**, not one with
    // Bluetooth appended — 224,190 bytes against 231,077 — so this is a real
    // change to what runs on the WLAN core, not a packaging detail. Rungs 1–16
    // were proven on the WiFi-only image and keep it by default; Bluetooth
    // needs the combined one, and says so.
    const fw_opt = b.option([]const u8, "fw", "firmware image: wifi (default) or combined (WiFi+BT)") orelse
        "wifi";
    const fw_combined = std.mem.eql(u8, fw_opt, "combined");
    if (!fw_combined and !std.mem.eql(u8, fw_opt, "wifi")) {
        std.debug.print("-Dfw must be 'wifi' or 'combined', got '{s}'\n", .{fw_opt});
        std.process.exit(1);
    }

    // **`-Dntp`, `-Drenew` and `-Ddrop` moved to `lib/ipnet` with the example
    // that read them.** They are falsifications of the IP stack — a dead
    // server, a shortened lease timer, a deliberate disassociation — and none
    // of them says anything about this driver. The reasoning behind each is
    // kept with the options, in `lib/ipnet/build.zig`.

    const wifi_opts = b.addOptions();
    wifi_opts.addOption([]const u8, "ssid", ssid_opt);
    wifi_opts.addOption([]const u8, "password", pass_opt);
    wifi_opts.addOption(bool, "fw_combined", fw_combined);

    const wifi_mod = wifi_opts.createModule();

    // **Size by default, speed on request.** Every rung through 25a is
    // protocol and fits comfortably; `src/bt/sbc_decode.zig` is a filterbank
    // and does not. Measured on hardware: one media packet took 41.7 ms to
    // decode at `ReleaseSmall` against a 14.5 ms budget, which stalled the
    // receive ring and eventually the link.
    const opt_opt = b.option(
        std.lang.Optimize,
        "opt",
        "optimize mode for the cyw43 module (default ReleaseSmall)",
    ) orelse .small;

    const mod_cyw43 = b.addModule("cyw43", .{
        .root_source_file = b.path("src/cyw43.zig"),
        .target = target,
        .optimize = opt_opt,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "wifi_config", .module = wifi_mod },
        },
    });

    // ------------------------------------------------------------------
    // Firmware blobs, generated from the vendored headers on every build
    // ------------------------------------------------------------------
    //
    // Into the build cache, never the source tree, so a fresh clone builds with
    // no manual step and no `.bin` can go stale. Each reaches `@embedFile`
    // through an anonymous import of the same name.
    const blobs_exe = b.addExecutable(.{ .name = "regen-blobs", .root_module = b.createModule(.{
        .root_source_file = b.path("tools/regen_blobs.zig"),
        .target = b.graph.host,
        .optimize = .debug,
    }) });
    const Blob = struct { name: []const u8, header: []const u8, kind: []const u8 };
    const blobs = [_]Blob{
        .{ .name = "w43439A0_combined.bin", .header = "reference/w43439A0_7_95_49_00_combined.h", .kind = "image" },
        .{ .name = "wifi_nvram_43439.bin", .header = "reference/wifi_nvram_43439.h", .kind = "nvram" },
        // The WiFi+BT image. **Not the WiFi image with Bluetooth appended**: its
        // WiFi microcode is 231,077 bytes against the WiFi-only build's 224,190,
        // a different firmware build that also carries the BT subsystem.
        .{ .name = "wb43439A0_combined.bin", .header = "reference/wb43439A0_7_95_49_00_combined.h", .kind = "image" },
        // The Bluetooth patchram, downloaded over the shared bus *after* the
        // combined image is running rather than being part of it.
        .{ .name = "btfw_43439.bin", .header = "reference/cyw43_btfw_43439.h", .kind = "hex" },
    };
    var blob_paths: [blobs.len]std.Build.LazyPath = undefined;
    for (blobs, &blob_paths) |blob, *path| {
        const run = b.addRunArtifact(blobs_exe);
        run.addArg(blob.kind);
        run.addFileArg(b.path(blob.header));
        path.* = run.addOutputFileArg(blob.name);
        mod_cyw43.addAnonymousImport(blob.name, .{ .root_source_file = path.* });
    }

    const check_step = b.step("check", "Typecheck cyw43 driver");
    const obj = b.addObject(.{ .name = "cyw43_check", .root_module = mod_cyw43 });
    check_step.dependOn(&obj.step);

    // ------------------------------------------------------------------
    // Tests (Host)
    // ------------------------------------------------------------------
    const test_step = b.step("test", "Run cyw43 unit tests");
    const fs_picozig_host = b.dependency("picozig", .{
        .target = b.graph.host,
        .optimize = std.lang.Optimize.debug,
    });

    const mod_cyw43_host = b.createModule(.{
        .root_source_file = b.path("src/cyw43.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig_host.module("rp2350") },
            .{ .name = "wifi_config", .module = wifi_mod },
        },
    });
    for (blobs, blob_paths) |blob, path| mod_cyw43_host.addAnonymousImport(blob.name, .{ .root_source_file = path });

    const t = b.addTest(.{ .root_module = mod_cyw43_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Example: cyw43-probe
    // ------------------------------------------------------------------
    const probe_mod = b.createModule(.{
        .root_source_file = b.path("example/cyw43-probe/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
        },
    });

    const exe_probe = b.addExecutable(.{ .name = "cyw43-probe", .root_module = probe_mod });
    exe_probe.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_probe.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_probe);

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

    const swd_cmd_probe = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_probe.addFileArg(exe_probe.getEmittedBin());
    swd_cmd_probe.addArg(probe_serial);
    swd_cmd_probe.has_side_effects = true;
    const swd_probe_step = b.step("swd-cyw43-probe", "Flash cyw43-probe example over SWD");
    swd_probe_step.dependOn(&swd_cmd_probe.step);

    const watch_script = b.fmt(
        \\openocd -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
        \\  -c "adapter speed 1000" \
        \\  -c "init" -c "targets rp2350.cm0" \
        \\  -c "mdw 0x20000000 32" \
        \\  -c "shutdown"
    , .{probe_select});

    const watch_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd.has_side_effects = true;
    const watch_step = b.step("watch-cyw43-probe", "Read cyw43 watch block over SWD");
    watch_step.dependOn(&watch_cmd.step);

    // ------------------------------------------------------------------
    // Example: cyw43-blink
    // ------------------------------------------------------------------
    const blink_mod = b.createModule(.{
        .root_source_file = b.path("example/cyw43-blink/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
            // The ladder reaches a real join at rung 13, so this example needs
            // credentials like the others do.
            .{ .name = "wifi_config", .module = wifi_mod },
        },
    });

    const exe_blink = b.addExecutable(.{ .name = "cyw43-blink", .root_module = blink_mod });
    exe_blink.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_blink.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_blink);

    const swd_cmd_blink = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_blink.addFileArg(exe_blink.getEmittedBin());
    swd_cmd_blink.has_side_effects = true;
    const swd_blink_step = b.step("swd-cyw43-blink", "Flash cyw43-blink example over SWD");
    swd_blink_step.dependOn(&swd_cmd_blink.step);

    const watch_cmd_blink = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd_blink.has_side_effects = true;
    const watch_blink_step = b.step("watch-cyw43-blink", "Read cyw43-blink watch block over SWD");
    watch_blink_step.dependOn(&watch_cmd_blink.step);

    // ------------------------------------------------------------------
    // Example: bt-probe — rung 17a. Needs -Dfw=combined.
    // ------------------------------------------------------------------
    const bt_mod = b.createModule(.{
        .root_source_file = b.path("example/bt-probe/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
        },
    });

    const exe_bt = b.addExecutable(.{ .name = "bt-probe", .root_module = bt_mod });
    exe_bt.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_bt.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_bt);

    const swd_cmd_bt = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_bt.addFileArg(exe_bt.getEmittedBin());
    swd_cmd_bt.has_side_effects = true;
    const swd_bt_step = b.step("swd-bt-probe", "Flash bt-probe example over SWD (use -Dfw=combined)");
    swd_bt_step.dependOn(&swd_cmd_bt.step);

    const watch_cmd_bt = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd_bt.has_side_effects = true;
    const watch_bt_step = b.step("watch-bt-probe", "Read bt-probe watch block over SWD");
    watch_bt_step.dependOn(&watch_cmd_bt.step);

    // ------------------------------------------------------------------
    // Example: bredr-probe — rung 21. Needs -Dfw=combined.
    // ------------------------------------------------------------------
    //
    // Separate from bt-probe rather than folded into it, deliberately.
    // bt-probe's claims are rungs 17 to 20 and every one of them has been
    // watched; adding an unproven rung to a proven example makes the whole file
    // ambiguous about which of its statements have been observed.
    const bredr_mod = b.createModule(.{
        .root_source_file = b.path("example/bredr-probe/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
        },
    });

    const exe_bredr = b.addExecutable(.{ .name = "bredr-probe", .root_module = bredr_mod });
    exe_bredr.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_bredr.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_bredr);

    const swd_cmd_bredr = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_bredr.addFileArg(exe_bredr.getEmittedBin());
    swd_cmd_bredr.has_side_effects = true;
    const swd_bredr_step = b.step("swd-bredr-probe", "Flash bredr-probe example over SWD (use -Dfw=combined)");
    swd_bredr_step.dependOn(&swd_cmd_bredr.step);

    const watch_cmd_bredr = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd_bredr.has_side_effects = true;
    const watch_bredr_step = b.step("watch-bredr-probe", "Read bredr-probe watch block over SWD");
    watch_bredr_step.dependOn(&watch_cmd_bredr.step);

    // ------------------------------------------------------------------
    // Example: bt-throughput — rung 21c. Needs -Dfw=combined.
    // ------------------------------------------------------------------
    //
    // **`-Drequired` exists so the failing run needs no source edit.** The
    // threshold is SBC's ~41,000 bytes a second; setting it to something
    // unreachable must make the same binary print FAIL. Same reasoning as
    // `-Dntp` and `-Ddrop`.
    const required_opt = b.option(
        u32,
        "required",
        "throughput pass threshold in payload bytes/sec (default 41000, SBC at 44.1 kHz)",
    ) orelse 41000;

    // **The clock the measurement is taken at**, because rung 21c's 88,840 B/s
    // turned out to have been measured at 12 MHz: `board.clock_plan` is `null`
    // for a Pico 2 W, so clk_sys runs straight off the crystal. That is a
    // deliberate board decision (picozig's `board.zig` says a PLL "gains
    // nothing but current draw"), so this option overrides it **for this
    // example only** and leaves every other build in the workspace alone.
    //
    // 0, the default, keeps the crystal and reproduces the original figure.
    const sys_mhz_opt = b.option(
        u32,
        "sys-mhz",
        "override clk_sys for bt-throughput, in MHz (0 = board default, the 12 MHz crystal)",
    ) orelse 0;

    // **The bus rate, separately from the core rate.** Rung 21d could only move
    // both at once, which is why its result took a session to interpret. 0 keeps
    // `setSysHz`'s behaviour — hold the bus at `proven_bus_hz` whatever clk_sys
    // is — and any other value is a PIO divisor, so `-Dbus-div=2` is `clk_sys/4`
    // and `-Dbus-div=25` at 150 MHz is the proven 3 MHz for a control run.
    const bus_div_opt = b.option(
        u32,
        "bus-div",
        "gSPI PIO divisor for bt-throughput (0 = hold the bus at its proven 3 MHz)",
    ) orelse 0;

    // **The falsification for rung A's pairing rule.** `auto` applies
    // `Variant.forBusHz`; naming a program overrides it, so the same binary can
    // be made to fail at a rate it passes at, which is the only way to show the
    // rule is doing work.
    const tp_variant_opt = b.option(
        []const u8,
        "variant",
        "gSPI PIO program for bt-throughput: auto (default), gap0_sample1, gap01_sample0, gap010_sample1, gap0101_sample0",
    ) orelse "auto";

    const tp_opts = b.addOptions();
    tp_opts.addOption(u32, "required", required_opt);
    tp_opts.addOption(u32, "sys_mhz", sys_mhz_opt);
    tp_opts.addOption(u32, "bus_div", bus_div_opt);
    tp_opts.addOption([]const u8, "variant", tp_variant_opt);

    const tp_mod = b.createModule(.{
        .root_source_file = b.path("example/bt-throughput/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
            .{ .name = "throughput_config", .module = tp_opts.createModule() },
        },
    });

    const exe_tp = b.addExecutable(.{ .name = "bt-throughput", .root_module = tp_mod });
    exe_tp.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_tp.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_tp);

    const swd_cmd_tp = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_tp.addFileArg(exe_tp.getEmittedBin());
    swd_cmd_tp.has_side_effects = true;
    const swd_tp_step = b.step("swd-bt-throughput", "Flash bt-throughput example over SWD (use -Dfw=combined)");
    swd_tp_step.dependOn(&swd_cmd_tp.step);

    const watch_cmd_tp = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd_tp.has_side_effects = true;
    const watch_tp_step = b.step("watch-bt-throughput", "Read bt-throughput watch block over SWD");
    watch_tp_step.dependOn(&watch_cmd_tp.step);

    // ------------------------------------------------------------------
    // Example: gspi-sweep — rung A. The bus-rate / program / edge-mode grid.
    // ------------------------------------------------------------------
    //
    // **Defaults to 150 MHz, unlike every other example here**, because the
    // question this rig exists to answer is what happens at `clk_sys / 4` and
    // a 12 MHz crystal cannot reach it. 0 still means "the board's own
    // decision", so the crystal case remains one flag away.
    const sweep_sys_mhz_opt = b.option(
        u32,
        "sweep-sys-mhz",
        "clk_sys for gspi-sweep, in MHz (default 150; 0 = board default, the 12 MHz crystal)",
    ) orelse 150;

    const sweep_opts = b.addOptions();
    sweep_opts.addOption(u32, "sys_mhz", sweep_sys_mhz_opt);

    const sweep_mod = b.createModule(.{
        .root_source_file = b.path("example/gspi-sweep/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = mod_cyw43 },
            .{ .name = "sweep_config", .module = sweep_opts.createModule() },
        },
    });

    const exe_sweep = b.addExecutable(.{ .name = "gspi-sweep", .root_module = sweep_mod });
    exe_sweep.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_sweep.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_sweep);

    const swd_cmd_sweep = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_sweep.addFileArg(exe_sweep.getEmittedBin());
    swd_cmd_sweep.has_side_effects = true;
    const swd_sweep_step = b.step("swd-gspi-sweep", "Flash gspi-sweep example over SWD");
    swd_sweep_step.dependOn(&swd_cmd_sweep.step);

    const watch_cmd_sweep = b.addSystemCommand(&.{ "/bin/sh", "-c", watch_script, "watch" });
    watch_cmd_sweep.has_side_effects = true;
    const watch_sweep_step = b.step("watch-gspi-sweep", "Read gspi-sweep watch block over SWD");
    watch_sweep_step.dependOn(&watch_cmd_sweep.step);

    // **`example/ntp-client` moved to `lib/ipnet` on 2026-08-31**, with the
    // IP stack it exercises. It is the ladder through rung 18 and it still
    // links this driver — `zig build swd-ntp-client` from `lib/ipnet` now.
    // It is not duplicated here: an L1 driver's example may not import an L2
    // library, and the example's whole subject is the L2 one.

    // ------------------------------------------------------------------
    // Rescue step: issue hardware SWD rescue reset for RP2350
    // ------------------------------------------------------------------
    const rescue_cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    rescue_cmd.addArgs(probe_select_args);
    rescue_cmd.addArgs(&.{
        "-c",
        "adapter speed 1000",
        "-c",
        "set RESCUE 1",
        "-f",
        "target/rp2350.cfg",
        "-c",
        "shutdown",
    });
    rescue_cmd.has_side_effects = true;
    const rescue_step = b.step("swd-rescue", "Issue an SWD Rescue DP reset via OpenOCD for RP2350");
    rescue_step.dependOn(&rescue_cmd.step);
}
