//! picozig's getting-started applications — real, flashable firmware, one per
//! directory here, for whichever Pico board `-Dboard=` names.
//!
//!     zig build                         every example, as ELF and UF2
//!     zig build swd-blink-hello         flash one over a Debug Probe
//!     zig build flash-blink-hello       or the UF2 route, onto a BOOTSEL volume
//!     zig build watch-blink-hello       read its `.selftest` block over SWD
//!     zig build -Dboard=pico2w ...      pico_lipo2 (default), pico2, pico2w, pico_plus2w
//!
//! **A package of its own because the examples need a board and every board
//! package needs picozig**: inside picozig they would make a dependency cycle,
//! which Zig rejects.
//!
//! The target is `pico_target_query`, never `-Dtarget`: a getting-started image
//! that quietly followed a host target would produce an ELF that cannot run.

const std = @import("std");
const picozig_build = @import("picozig");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(picozig_build.pico_target_query);
    const optimize: std.lang.Optimize = .small;
    const board_name = b.option(
        []const u8,
        "board",
        "Pico board: pico_lipo2 (default), pico2, pico2w or pico_plus2w",
    ) orelse "pico_lipo2";

    const picozig = b.dependency("picozig", .{ .target = target, .optimize = optimize });
    const board = b.dependency(board_name, .{ .target = target, .optimize = optimize }).module("board");

    const steps = addExamples(b, target, picozig, board);
    const check_step = b.step("check", "Typecheck every example");
    for (steps) |s| check_step.dependOn(s);
}

/// The getting-started applications under `example/`.
///
/// Each gets an ELF, a UF2, and both flashing routes:
///
///     zig build swd-<name>      over a Debug Probe — preferred while working
///     zig build flash-<name>    UF2 onto a BOOTSEL volume — no probe needed
///
/// **These steps belong to an example, not to the platform layer** — picozig
/// itself has no image to flash, which is why `src/` has no such step. An example *is* an application; one that could not be got
/// onto a board would fail at the only job it has.
///
/// Returns each example's compile step, so `check` can typecheck them all.
fn addExamples(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    picozig: *std.Build.Dependency,
    board: *std.Build.Module,
) []const *std.Build.Step {
    const uf2_tool = picozig.artifact("elf2uf2");
    // Hold BOOTSEL while plugging the board in and it appears as a mass-storage
    // volume. The label is the board's, not something we choose, so override
    // with -Dpico-volume=<path> if yours differs.
    const default_volume = switch (@import("builtin").os.tag) {
        .macos => "/Volumes/RP2350",
        else => "/run/media/" ++ "$USER" ++ "/RP2350",
    };
    const pico_volume = b.option(
        []const u8,
        "pico-volume",
        "Mounted BOOTSEL volume to copy an example's UF2 onto (default: " ++ default_volume ++ ")",
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

    // Needs an OpenOCD new enough to carry target/rp2350.cfg — Homebrew's
    // 0.12.0 *bottle* predates RP2350 by well over a year and does not have it:
    // `brew install --HEAD open-ocd`, or build the Raspberry Pi fork
    // (`github.com/raspberrypi/openocd`, branch `sdk-2.0.0`). If the target is
    // not found, suspect the `rp2350.cfg` version before the wiring.
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";
    // **Two probes on one host enumerate identically.** openocd claims
    // whichever it finds first, so without this it can flash the wrong board
    // and report success. Declared once here and passed down, because
    // `b.option` may only be called once per name.
    const probe_serial = picozig_build.probeSerialOption(b);
    const probe_select = picozig_build.probeSelect(b, probe_serial);

    // `pio-tick` at 200 MHz off the PLL rather than at the board's own clock
    // plan, which on both Pico boards is the 12 MHz crystal.
    //
    // **This exists to close a variable rather than to configure anything.**
    // The image is a control for display scan-out work, which ran at 200 MHz
    // while these boards run at 12 — so a control that only ever ran
    // at 12 MHz leaves the clock untested, which is precisely the kind of gap
    // that investigation kept falling into. `clocks.Plan` is chip-level and
    // knows nothing about boards, so any RP2350 can be asked for 200 MHz.
    //
    // Only `pio-tick` reads it; every other example follows `board.clock_plan`.
    const tick_pll = b.option(
        bool,
        "tick-pll",
        "pio-tick: run at 200 MHz off the PLL instead of the board's clock plan",
    ) orelse false;
    const tick_opts = b.addOptions();
    tick_opts.addOption(bool, "use_pll", tick_pll);

    // `clock-check` at a PLL frequency the board did not ask for, so that a
    // board with no `clock_plan` can still be run at one.
    //
    // **This exists to cold-boot-test an overclock** — clk_sys above the
    // 150 MHz datasheet maximum, 200 MHz being the usual ask. The existing
    // evidence that 200 MHz is fine on a Pico board came entirely from
    // `swd-*` runs, which reset a *warm* chip with settled rails — precisely
    // the case a marginal overclock survives. Nobody has cold-booted one.
    //
    // `clock-check` is the right image for it because it needs no board
    // hardware and because FC0 **measures** clk_sys rather than reporting what
    // was requested — every other register in that block reads back the value
    // written to it whether or not the source behind it is running.
    const clock_pll_khz = b.option(
        u32,
        "clock-pll-khz",
        "clock-check: run clk_sys off the PLL at this many kHz instead of the board's plan (e.g. 200000)",
    ) orelse 0;
    const clock_opts = b.addOptions();
    clock_opts.addOption(u32, "pll_khz", clock_pll_khz);

    // `flash-wedge`: how hard to try to defeat the XIP cache. The first run of
    // that image completed the whole bootrom flash sequence from flash without
    // dying, because the sequence fitted in cache and never had to fetch from a
    // flash it had just made unreadable. Each mode removes one more of the
    // cache's help; see the example's header.
    const wedge_mode = b.option(
        []const u8,
        "wedge-mode",
        "flash-wedge: plain | flush | irq | erase-loop (see the example header)",
    ) orelse "plain";
    // Composable with any mode, because the XIP cache turned out to be what
    // rescued the board in all three of the first attempts: with the sequence
    // cached, nothing ever has to read the flash it made unreadable.
    const wedge_nocache = b.option(
        bool,
        "wedge-nocache",
        "flash-wedge: disable the XIP cache (CTRL.EN_SECURE) before the sequence",
    ) orelse false;
    // **The safety catch, and the experiment's second half.** A non-zero delay
    // gives every boot — including a boot into a broken state — a window in
    // which `zig build swd-blink-hello` can overwrite the image, so the fault
    // is always recoverable over SWD. Setting it to 0 removes that window
    // deliberately: the fault is then re-armed within milliseconds of every
    // reset, which is the defining property of a wedge that only BOOTSEL can
    // clear. Do not pass 0 without meaning it.
    const wedge_delay = b.option(
        u32,
        "wedge-delay",
        "flash-wedge: seconds of healthy, reflashable board before the fault (default 25; 0 = BOOTSEL-only)",
    ) orelse 25;
    const power_level = b.option(
        u32,
        "power-level",
        "low-power: how far up the ladder (0 baseline, 1 +core1 off, 2 +clocks gated, 3 +blocks reset)",
    ) orelse 0;
    // **The safety catch.** Rung 1 forces core 1 off, and OpenOCD's reset
    // sequence waits for both cores to halt -- so a board running rung 1 or
    // above cannot be reflashed until the bit is cleared. A non-zero delay
    // guarantees a window in which `-Dpower-level=0` gets it back.
    const power_delay = b.option(
        u32,
        "power-delay",
        "low-power: seconds of reflashable board before the rungs apply (default 15; 0 = no window)",
    ) orelse 15;
    const power_opts = b.addOptions();
    power_opts.addOption(u32, "level", power_level);
    power_opts.addOption(u32, "delay_s", power_delay);

    const wedge_opts = b.addOptions();
    wedge_opts.addOption([]const u8, "mode", wedge_mode);
    wedge_opts.addOption(bool, "nocache", wedge_nocache);
    wedge_opts.addOption(u32, "delay_s", wedge_delay);

    const Example = struct {
        name: []const u8,
        /// How many words its `.selftest` block holds, if it publishes one. An
        /// example with a block gets a `watch-<name>` step that reads it over
        /// SWD; see `addWatchStep` for why that is worth having.
        ///
        /// Must match the example's own struct, and there is no way to make the
        /// compiler check that from here. The example therefore prints
        /// `@sizeOf(Watch) / 4` in its boot banner: if the two disagree, the
        /// banner is right and this is stale.
        watch_words: ?u32 = null,
    };

    const examples = [_]Example{
        .{ .name = "blink-hello", .watch_words = 5 },
        // The least a board can do: no clocks, no UART, no peripheral beyond a
        // pad. **For a board that flashes cleanly and then says nothing** —
        // every other example starts with `clocks.init`, whose XOSC wait is
        // unbounded, so a dead crystal is indistinguishable from a dead board.
        // This one runs on the ROSC and asks the crystal with a bound.
        .{ .name = "blink-bare", .watch_words = 8 },
        // Is it the UART, the clock, the pad, or the wire? Reports over SWD,
        // because every other example reports over the UART and they all go
        // silent together when the UART is what is broken. Uses UARTCR's LBE
        // to test the block with no pins in the path at all.
        .{ .name = "uart-check", .watch_words = 24 },
        // Who is this chip, and how is its flash laid out? The three C-SDK
        // examples `unique_board_id`, `boot_info` and `partition_info` in one,
        // because nobody has ever wanted only one of them.
        .{ .name = "board-info", .watch_words = 10 },
        // Arm it, feed it, let it bite, and prove which happened -- across
        // three boots, reported through the scratch registers and a .selftest
        // block rather than a terminal, because a watchdog is exactly what you
        // want to test on a board whose terminal is the suspect.
        .{ .name = "watchdog-check", .watch_words = 10 },
        // A ladder of idle states, one rung per meter reading, cumulative and
        // selected by -Dpower-level. Read against `power-floor`: that says what
        // the board costs, this says how much of the rest firmware can remove.
        .{ .name = "low-power", .watch_words = 9 },
        // Proves POWMAN's always-on timer and its wake alarm **while awake**,
        // because that alarm is the only thing that can bring the chip back
        // from a power-down. The power-down itself is commented out until
        // `alarm_fired` says the wake works.
        .{ .name = "powman-check", .watch_words = 16 },
        .{ .name = "adc-battery", .watch_words = 5 },
        // Reports what clk_sys actually is, measured rather than assumed, into
        // a block read back over SWD. The only example that builds for every
        // board, because it is the one that needs no board hardware at all.
        .{ .name = "clock-check", .watch_words = 8 },
        // DMA moves bytes, at three widths and both increment modes.
        .{ .name = "dma-check", .watch_words = 8 },
        // A PIO state machine fetches, executes and moves data — proven
        // through the FIFOs, so it needs no pin and no instrument.
        .{ .name = "pio-check", .watch_words = 8 },
        // The four rungs between `pio-check` and a scan-out: a divisor, a
        // side-set, a joined TX FIFO under autopull, and `out exec`. Written
        // because a display scan-out had no rung between those two and spent
        // a fortnight reasoning about the top of the ladder.
        .{ .name = "pio-tick", .watch_words = 21 },
        // Scans all candidate I2C pin pairs across 0x00-0x7F and reports found devices.
        .{ .name = "i2c-check", .watch_words = 26 },
        // Reproduces the board wedge on purpose, by running the bootrom flash
        // sequence from flash instead of RAM. Deliberately the mistake
        // `src/rp2350/flash.zig` exists to prevent — read its header before
        // flashing it, and never imitate it anywhere else.
        .{ .name = "flash-wedge", .watch_words = 11 },
        // Clocks up, then `wfi` forever — no GPIO, no UART, no peripheral at
        // all. The control against which every idle-current measurement is
        // read: it separates what the firmware costs from what the board costs.
        .{ .name = "power-floor", .watch_words = 4 },
    };

    var steps = std.ArrayList(*std.Build.Step).empty;

    for (examples) |example| {
        const name = example.name;
        const mod = b.createModule(.{
            .root_source_file = b.path(b.fmt("{s}/src/main.zig", .{name})),
            .target = target,
            .optimize = .small,
            // Keep the symbol table:
            // ReleaseSmall strips it, and a PC read over SWD then resolves to
            // nothing at all, so a panic cannot be told from a spin. Symbols
            // live in ELF metadata, never in a PT_LOAD segment, so they cost
            // exactly zero flash and are never written to the device.
            .strip = false,
            .imports = &.{
                .{ .name = "rp2350", .module = picozig.module("rp2350") },
                .{ .name = "board", .module = board },
                // Side effects only: `boot` exports the vector table and
                // `_reset`, which nothing references by name.
                .{ .name = "boot", .module = picozig.module("boot") },
                .{ .name = "pio_selftest", .module = picozig.module("pio_selftest") },
            },
        });

        // Safe on an example module because `createModule` does not register a
        // name — a generated file is only a hazard when it hangs off a module a
        // dependent can resolve without building: anything that configures
        // without building (ZLS does) then panics on the ungenerated path.
        if (std.mem.eql(u8, name, "pio-tick")) mod.addOptions("build_options", tick_opts);
        if (std.mem.eql(u8, name, "clock-check")) mod.addOptions("build_options", clock_opts);
        if (std.mem.eql(u8, name, "flash-wedge")) mod.addOptions("build_options", wedge_opts);
        if (std.mem.eql(u8, name, "low-power")) mod.addOptions("build_options", power_opts);

        const exe = b.addExecutable(.{ .name = name, .root_module = mod });
        exe.setLinkerScript(picozig.path("rp2350.ld"));
        // The image's true entry is the reset vector; there is no _start.
        exe.entry = .{ .symbol_name = "_reset" };
        b.installArtifact(exe);
        steps.append(b.allocator, &exe.step) catch @panic("OOM");

        // ELF -> UF2, so flashing needs neither picotool nor a probe. The
        // packer knows the chip's family id; a wrong one is rejected by the
        // bootrom with no useful feedback.
        const uf2_run = b.addRunArtifact(uf2_tool);
        uf2_run.addFileArg(exe.getEmittedBin());
        const uf2_file = uf2_run.addOutputFileArg(b.fmt("{s}.uf2", .{name}));
        b.getInstallStep().dependOn(
            &b.addInstallFileWithDir(uf2_file, .prefix, b.fmt("{s}.uf2", .{name})).step,
        );

        const flash_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", flash_script, "flash" });
        flash_cmd.has_side_effects = true;
        flash_cmd.addFileArg(uf2_file);
        flash_cmd.addArg(pico_volume);
        const flash_step = b.step(
            b.fmt("flash-{s}", .{name}),
            b.fmt("Copy {s}.uf2 onto a BOOTSEL-mounted board", .{name}),
        );
        flash_step.dependOn(&flash_cmd.step);

        // SWD: build, flash, verify and start, with the board left plugged in.
        // Run through `sh -c` because OpenOCD wants the filename and its
        // options as a single -c argument, which addFileArg cannot express on
        // its own. $1 is the ELF.
        const swd_script = b.fmt(
            \\exec {s} -f interface/cmsis-dap.cfg {s}-f target/rp2350.cfg \
            \\  -c "adapter speed 1000" \
            \\  -c "program $1 verify reset exit"
        , .{ openocd, probe_select });
        const swd_cmd = b.addSystemCommand(&.{ "/bin/sh", "-c", swd_script, "swd" });
        swd_cmd.addFileArg(exe.getEmittedBin());
        // Talking to hardware is never up to date, and never cacheable.
        swd_cmd.has_side_effects = true;
        const swd_step = b.step(
            b.fmt("swd-{s}", .{name}),
            b.fmt("Flash {s} over SWD via a Raspberry Pi Debug Probe", .{name}),
        );
        swd_step.dependOn(&swd_cmd.step);

        if (example.watch_words) |words| addWatchStep(b, openocd, probe_serial, name, words);
    }

    return steps.toOwnedSlice(b.allocator) catch @panic("OOM");
}

/// `zig build watch-<name>` — read an example's `.selftest` block over SWD.
///
/// The block is pinned by the linker to the base of SRAM, so this needs no
/// symbols and no GDB: OpenOCD alone, which flashing already required. Run it
/// twice — a tick counter that advances is the liveness proof, and it is the
/// one diagnostic that still works when the UART has gone quiet.
///
/// **Deliberately no halt/resume.** Cortex-M memory reads go through the AHB
/// access port and work on a running target, so the counter keeps moving while
/// we read it, which is exactly what makes the check mean anything. Halting
/// would also break it outright: RP2350 is dual-core, `halt` takes only cm0,
/// and the following `resume` fails with `[rp2350.cm1] not halted`, leaving the
/// chip stopped and looking precisely like the hang under investigation.
fn addWatchStep(b: *std.Build, openocd: []const u8, probe_serial: []const u8, name: []const u8, words: u32) void {
    const probe_select_args = picozig_build.probeSelectArgs(b, probe_serial);
    const cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    cmd.addArgs(probe_select_args);
    cmd.addArgs(&.{
        "-f",
        "target/rp2350.cfg",
        "-c",
        "adapter speed 1000",
        "-c",
        "init",
        "-c",
        b.fmt("mdw 0x20000000 {d}", .{words}),
        "-c",
        "shutdown",
    });
    cmd.has_side_effects = true;
    const step = b.step(
        b.fmt("watch-{s}", .{name}),
        b.fmt("Read {s}'s RAM block over SWD, without halting the target", .{name}),
    );
    step.dependOn(&cmd.step);
}
