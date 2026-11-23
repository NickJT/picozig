//! ipnet — an IPv4 stack for a device that has an Ethernet frame and nothing
//! else.
//!
//!   net       Ethernet / ARP / IPv4 / UDP — formatting and parsing, no state
//!   dhcp      DHCP messages, RFC 2131 — likewise pure
//!   ipstack   the address, the lease and its renewal, ARP, filtered UDP
//!   session   keeping it up: join retry, link watch, rejoin
//!   ntp       RFC 5905
//!
//! **The library depends on no device and no driver.** `IpStack` and `Session`
//! are generic over a device type, so the module built here imports `std` and
//! nothing else — which is why `cyw43` appears below only in the example's
//! import list. That is what makes the layering compiler-enforced rather than remembered:
//! `@import("cyw43")` does not resolve from `src/`.
//!
//! Everything in `src/` was written and proven on hardware inside `lib/cyw43`
//! and moved here unchanged on 2026-08-31.

const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";

    // Two probes on one host enumerate identically; the serial is the only
    // thing that separates them. picozig holds the helpers so there is one
    // implementation rather than twenty.
    const probe_serial = picozig_build.probeSerialOption(b);
    const probe_select_args = picozig_build.probeSelectArgs(b, probe_serial);

    // The board the examples run on. Only the examples read it, and nothing is
    // forwarded: no dependency of this package takes a board.
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

    // ------------------------------------------------------------------
    // The library
    // ------------------------------------------------------------------
    //
    // No `.imports` at all. That is the layering, enforced.
    const mod_ipnet = b.addModule("ipnet", .{
        .root_source_file = b.path("src/ipnet.zig"),
        .target = target,
        .optimize = .small,
    });

    // ------------------------------------------------------------------
    // Standalone verification: the stack must build for the board as well as
    // for the host, and neither may be the only one checked
    // ------------------------------------------------------------------
    const check_step = b.step("check", "Typecheck ipnet for host and freestanding");
    const obj_fs = b.addObject(.{ .name = "ipnet-freestanding", .root_module = mod_ipnet });
    check_step.dependOn(&obj_fs.step);

    const mod_host = b.createModule(.{
        .root_source_file = b.path("src/ipnet.zig"),
        .target = b.graph.host,
        .optimize = .debug,
    });
    const obj_host = b.addObject(.{ .name = "ipnet-host", .root_module = mod_host });
    check_step.dependOn(&obj_host.step);

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    //
    // One module, and `src/ipnet.zig`'s `test { _ = ... }` block is what makes
    // the submodules' tests run at all — see the comment there.
    const test_step = b.step("test", "Run ipnet's unit tests");
    const t = b.addTest(.{ .root_module = mod_host });
    test_step.dependOn(&b.addRunArtifact(t).step);

    // ------------------------------------------------------------------
    // Credentials for the example
    // ------------------------------------------------------------------
    //
    // **`lib/cyw43`'s file is read rather than duplicated.** A second copy of a password is a second thing to keep
    // in step, and the failure when they drift is a join that does not happen.
    var ssid_opt = b.option([]const u8, "ssid", "Wi-Fi SSID") orelse "";
    var pass_opt = b.option([]const u8, "password", "Wi-Fi Password") orelse "";

    if (ssid_opt.len == 0) {
        for ([_][]const u8{ "wifi_credentials.json", "../cyw43/wifi_credentials.json" }) |path| {
            // Declared, because the configuration is cached and an undeclared
            // read keeps stale credentials. The directory catches the file
            // appearing or going; a missing file cannot itself be declared.
            b.dependOnDirectoryContents(b.path(std.fs.path.dirname(path) orelse "."));
            const content = b.root.root_dir.handle.readFileAlloc(b.graph.io, path, b.allocator, .limited(4096)) catch continue;
            b.dependOnFileContents(b.path(path));
            const Creds = struct { ssid: []const u8 = "", password: []const u8 = "" };
            const parsed = std.json.parseFromSlice(Creds, b.allocator, content, .{ .ignore_unknown_fields = true }) catch continue;
            ssid_opt = parsed.value.ssid;
            pass_opt = parsed.value.password;
            if (ssid_opt.len != 0) break;
        }
    }

    if (ssid_opt.len == 0) {
        std.debug.print(
            \\
            \\================================================================================
            \\NOTICE: No Wi-Fi credentials found for the ntp-client target!
            \\
            \\  zig build swd-ntp-client -Dssid="<SSID>" -Dpassword="<PASS>"
            \\
            \\or put {{"ssid": "...", "password": "..."}} in lib/cyw43/wifi_credentials.json
            \\================================================================================
            \\
        , .{});
    }

    // **The NTP server is an option so it can be pointed somewhere dead.**
    // Rung 16's evidence is a time that could have failed to arrive: the same
    // binary aimed at an address nobody answers must time out and print
    // nothing. A hard-coded server makes that test require an edit, and a test
    // that requires an edit does not get run.
    //
    // **Do not add a build-time clock here.** There was a `build_timestamp`
    // option carrying the host wall clock into firmware, and the only thing
    // that read it was a fabricated NTP datagram reporting
    // `build_timestamp + uptime` — right at the moment of flashing, advancing
    // at 1 Hz, and indistinguishable by eye from a working client. It was
    // mistaken for one.
    const ntp_opt = b.option([]const u8, "ntp", "NTP server IPv4 address (default 129.6.15.28, time.nist.gov)") orelse
        "129.6.15.28";

    // T1 on this network is twelve hours, so without this the renewal path
    // would ship host-tested and never once run on the board.
    const renew_opt = b.option(u32, "renew", "renew the DHCP lease N seconds after taking it (0 = at T1)") orelse 0;
    // Disassociate deliberately after N queries, to exercise the rejoin. The
    // alternative is rebooting a router that is not ours to reboot.
    const drop_opt = b.option(u32, "drop", "disassociate after N NTP queries, to exercise rejoin (0 = never)") orelse 0;

    // **The control for rung N2.** `wake-check` builds the same loop twice and
    // the option is the only difference, so a `polls` count from one run means
    // nothing without the other run to compare it against.
    const wake_opt = b.option(bool, "wake", "wake-check: service from the chip's interrupt rather than by polling") orelse true;

    // How long `wake-check` measures for. **An option because the run has to
    // outlast the stimulus**: `arping -c 90` takes rather more than 90 seconds
    // once retries are counted, and a window that closes first stops the
    // counters mid-experiment and makes the two sides uncomparable.
    const secs_opt = b.option(u32, "secs", "wake-check: seconds to measure for (default 90)") orelse 90;

    // net-check: which half to be, and who to talk to.
    const mode_opt = b.option([]const u8, "mode", "net-check: client (default) or server") orelse "client";
    if (!std.mem.eql(u8, mode_opt, "client") and !std.mem.eql(u8, mode_opt, "server")) {
        std.debug.print("-Dmode must be 'client' or 'server', got '{s}'\n", .{mode_opt});
        std.process.exit(1);
    }
    // **A name or an address.** A name is the observable proof that rung N9
    // ran: there is no resolver anywhere else in the image, so a page fetched
    // from a name could not have arrived any other way.
    const host_opt = b.option([]const u8, "host", "net-check: host to fetch from — a name or an address") orelse "192.168.1.2";
    const port_opt = b.option(u16, "port", "net-check: TCP port (default 8080)") orelse 8080;
    const path_opt = b.option([]const u8, "path", "net-check: request target (default /)") orelse "/";

    const net_opts = b.addOptions();
    net_opts.addOption([]const u8, "ssid", ssid_opt);
    net_opts.addOption([]const u8, "password", pass_opt);
    net_opts.addOption([]const u8, "ntp_server", ntp_opt);
    net_opts.addOption(u32, "renew_secs", renew_opt);
    net_opts.addOption(u32, "drop_after", drop_opt);
    net_opts.addOption(bool, "wake", wake_opt);
    net_opts.addOption(u32, "run_secs", secs_opt);
    net_opts.addOption([]const u8, "mode", mode_opt);
    net_opts.addOption([]const u8, "host", host_opt);
    net_opts.addOption(u16, "port", port_opt);
    net_opts.addOption([]const u8, "path", path_opt);
    const net_config = net_opts.createModule();

    // ------------------------------------------------------------------
    // example/ntp-client — the ladder through rung 18, ending in a remote clock
    // ------------------------------------------------------------------
    //
    // The one place in this package that names a radio.
    //
    // No arguments: cyw43 takes no board, and builds its picozig with the same
    // target and optimize mode as this file, so the two share one instance.
    const dep_cyw43 = b.dependency("cyw43", .{});

    const ntp_mod = b.createModule(.{
        .root_source_file = b.path("example/ntp-client/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = dep_cyw43.module("cyw43") },
            .{ .name = "ipnet", .module = mod_ipnet },
            .{ .name = "net_config", .module = net_config },
        },
    });

    const exe_ntp = b.addExecutable(.{ .name = "ntp-client", .root_module = ntp_mod });
    exe_ntp.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_ntp.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_ntp);
    check_step.dependOn(&exe_ntp.step);

    // **`adapter serial` has to land after the interface config and before the
    // adapter is initialised**, which is why it is spliced in rather than
    // appended. `set --` builds the argument list so an empty serial adds no
    // argument at all, rather than an empty one openocd would reject.
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

    const swd_cmd_ntp = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_ntp.addFileArg(exe_ntp.getEmittedBin());
    swd_cmd_ntp.addArg(probe_serial);
    swd_cmd_ntp.has_side_effects = true;
    const swd_ntp_step = b.step("swd-ntp-client", "Flash ntp-client example over SWD");
    swd_ntp_step.dependOn(&swd_cmd_ntp.step);

    // ------------------------------------------------------------------
    // example/wake-check — rung N2's falsification
    // ------------------------------------------------------------------
    //
    // Its `main` never calls `service`: the chip's own interrupt on GP24 is
    // what drives the stack. `-Dwake=false` builds the identical loop with the
    // guard removed, which is the control.
    const wake_mod = b.createModule(.{
        .root_source_file = b.path("example/wake-check/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = dep_cyw43.module("cyw43") },
            .{ .name = "ipnet", .module = mod_ipnet },
            .{ .name = "net_config", .module = net_config },
        },
    });

    const exe_wake = b.addExecutable(.{ .name = "wake-check", .root_module = wake_mod });
    exe_wake.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_wake.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_wake);
    check_step.dependOn(&exe_wake.step);

    const swd_cmd_wake = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_wake.addFileArg(exe_wake.getEmittedBin());
    swd_cmd_wake.addArg(probe_serial);
    swd_cmd_wake.has_side_effects = true;
    const swd_wake_step = b.step("swd-wake-check", "Flash wake-check over SWD (-Dwake=false for the polled control)");
    swd_wake_step.dependOn(&swd_cmd_wake.step);

    const watch_cmd_wake = watchCmd(b, openocd, probe_select_args);
    const watch_wake_step = b.step("watch-wake-check", "Read wake-check's watch block over SWD");
    watch_wake_step.dependOn(&watch_cmd_wake.step);

    // ------------------------------------------------------------------
    // example/net-check — rungs N5 to N11, on a wire for the first time
    // ------------------------------------------------------------------
    const nc_mod = b.createModule(.{
        .root_source_file = b.path("example/net-check/src/main.zig"),
        .target = target,
        .optimize = .small,
        .strip = false,
        .imports = &.{
            .{ .name = "rp2350", .module = fs_picozig.module("rp2350") },
            .{ .name = "board", .module = fs_board },
            .{ .name = "boot", .module = fs_picozig.module("boot") },
            .{ .name = "cyw43", .module = dep_cyw43.module("cyw43") },
            .{ .name = "ipnet", .module = mod_ipnet },
            .{ .name = "net_config", .module = net_config },
        },
    });

    const exe_nc = b.addExecutable(.{ .name = "net-check", .root_module = nc_mod });
    exe_nc.setLinkerScript(fs_picozig.path("rp2350.ld"));
    exe_nc.entry = .{ .symbol_name = "_reset" };
    b.installArtifact(exe_nc);
    check_step.dependOn(&exe_nc.step);

    const swd_cmd_nc = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
    swd_cmd_nc.addFileArg(exe_nc.getEmittedBin());
    swd_cmd_nc.addArg(probe_serial);
    swd_cmd_nc.has_side_effects = true;
    const swd_nc_step = b.step("swd-net-check", "Flash net-check over SWD (-Dmode=server for the other half)");
    swd_nc_step.dependOn(&swd_cmd_nc.step);

    // **A UF2 as well, for a board with no probe on it.** N12 is the first rung
    // that needs two boards at once; BOOTSEL and a drag onto a mass-storage
    // volume gets an image onto one that SWD cannot reach.
    //
    // The packer is picozig's, since it is the thing that knows the RP2350
    // family id: an RP2040 bootloader ignores blocks carrying it, so the wrong
    // board simply declines the image rather than bricking on it.
    //
    // **Written when the bench had one Debug Probe, and kept now that it has
    // two.** The original note said the UF2-flashed board had to be the server,
    // because UART0 is net-check's only output and it leaves on GP0/GP1 into a
    // probe that board did not have — so only a server, observable from outside
    // by curl, could be run blind. That constraint is gone: a second probe was
    // built on 2026-09-03 and both boards now
    // report over UART, so either can take either half. This step stays as the
    // fallback for a board without a probe, or a probe that misbehaves — and
    // the constraint is recorded rather than deleted, because it comes straight
    // back the day someone runs this on one probe.
    const uf2_nc = b.addRunArtifact(fs_picozig.artifact("elf2uf2"));
    uf2_nc.addFileArg(exe_nc.getEmittedBin());
    const uf2_nc_file = uf2_nc.addOutputFileArg("net-check.uf2");
    b.getInstallStep().dependOn(
        &b.addInstallFileWithDir(uf2_nc_file, .prefix, "net-check.uf2").step,
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
    // macOS `cp` carries extended attributes that FAT32 refuses with EPERM, and
    // the board unmounts itself the instant it takes the final block — so the
    // copy can report failure on a flash that worked, which is the one outcome
    // that teaches you to stop reading exit codes. `-X` drops the attributes;
    // a volume still mounted afterwards is the real failure. Same script as
    // `app/bt-speaker`, and for the same reasons.
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
    const flash_cmd_nc = b.addSystemCommand(&.{ "/bin/sh", "-c", flash_script, "flash" });
    flash_cmd_nc.addFileArg(uf2_nc_file);
    flash_cmd_nc.addArg(pico_volume);
    flash_cmd_nc.has_side_effects = true;
    b.step("flash-net-check", "Copy net-check.uf2 onto a BOOTSEL-mounted Pico 2 W")
        .dependOn(&flash_cmd_nc.step);

    const watch_cmd_nc = watchCmd(b, openocd, probe_select_args);
    const watch_nc_step = b.step("watch-net-check", "Read net-check's watch block over SWD");
    watch_nc_step.dependOn(&watch_cmd_nc.step);

    const watch_cmd_ntp = watchCmd(b, openocd, probe_select_args);
    const watch_ntp_step = b.step("watch-ntp-client", "Read ntp-client watch block over SWD");
    watch_ntp_step.dependOn(&watch_cmd_ntp.step);
}

/// The three `watch-*` steps read the same 32-word block at the same address
/// through the same adapter; only the step's name differed, and three copies of
/// an openocd invocation is three places to forget the probe serial.
fn watchCmd(b: *std.Build, openocd: []const u8, probe_select_args: []const []const u8) *std.Build.Step.Run {
    const cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    cmd.addArgs(probe_select_args);
    cmd.addArgs(&.{
        "-f", "target/rp2350.cfg",
        "-c", "adapter speed 1000",
        "-c", "init",
        "-c", "targets rp2350.cm0",
        "-c", "mdw 0x20000000 32",
        "-c", "shutdown",
    });
    cmd.has_side_effects = true;
    return cmd;
}
