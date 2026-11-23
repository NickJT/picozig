//! picozig — bare-metal Zig for the RP2350B.
//!
//! Exports four modules and one host tool:
//!
//!   rp2350        the chip layer: registers, clocks, GPIO, SPI/I2C/UART, timers
//!   pico_board    what every Pico board package shares: the board list, the
//!                 reference wiring, and helpers for claims and configs
//!   boot          vector table, reset handler, .data/.bss/.ramfunc bring-up
//!   pio_selftest  a five-rung PIO ladder, depending on rp2350 and no board
//!   elf2uf2       ELF -> UF2 packer, so flashing is a drag-and-drop
//!
//! plus `rp2350.ld`, which a dependent reaches with `dep.path("rp2350.ld")`.
//!
//! **No board.** Each board is its own package (`lib/pico2`, `lib/pico2w`,
//! `lib/pico-lipo2`, `lib/pico-plus2w`) exporting a `board` module, and the
//! application chooses one. So this package takes no board option, every
//! `b.dependency("picozig", ...)` passes only `target` and `optimize`, and the
//! chip layer cannot reach for a board because none is here. The examples are
//! a package of their own, `example/`, because they need a board and a board
//! needs this package.

const std = @import("std");

/// The RP2350's core, as a query rather than a resolved target so it can serve
/// both as this package's default and as a dependent's explicit choice.
///
/// Soft float: cortex_m33 has an FPU, but using it needs CPACR enabled in the
/// reset handler first. Nothing here wants floating point, so `.eabi` plus
/// subtracting `fpregs` guarantees the compiler cannot quietly emit an FP
/// instruction that hard-faults on first use.
pub const pico_target_query: std.Target.Query = .{
    .cpu_arch = .thumb,
    .os_tag = .freestanding,
    .abi = .eabi,
    .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m33 },
    .cpu_features_sub = std.Target.arm.featureSet(&.{.fpregs}),
};

/// Declare `-Dprobe-serial` for a package that has SWD steps.
///
/// **Two Debug Probes on one host is a silent hazard.** They enumerate as the
/// same VID:PID and openocd claims whichever it finds first, so with both
/// attached it will flash the wrong board and report success — and on a bench
/// where both boards run near-identical firmware, every observation afterwards
/// is of the other one. The serial is the only field that separates them:
/// the probe's USB serial number, which openocd prints when it attaches.
///
/// It lives here rather than in each build file because there are twenty of
/// those, and an option that is missing from one of them is worse than one
/// nobody has: it fails only on the package you forgot.
///
/// Empty means "assume one probe", and then every command below is byte for
/// byte what it was before this existed.
///
/// **"Assume one probe" is a guess, and openocd does not check it.** With two
/// adapters attached it takes the first it enumerates and says nothing — no
/// error, no warning, just a successful flash of the wrong board. Measured on
/// the bench 2026-09-20 with two probes up, `-f interface/cmsis-dap.cfg` alone
/// picked the other rig's probe, and which one is first changes with USB
/// enumeration order. So on any bench with more than one probe the serial is
/// not an optimisation, it is the thing that makes the step correct, and it
/// belongs in the project's own documented build line rather than in whoever
/// is typing. Probes move between boards, so read which board is behind each
/// probe rather than recording it: `mdw 0x40130000 2` through a probe reads that
/// board's CHIPID0..1 from the OTP ECC alias, with no halt needed because
/// Cortex-M reads go through the AHB access port on a running target.
pub fn probeSerialOption(b: *std.Build) []const u8 {
    return b.option(
        []const u8,
        "probe-serial",
        "CMSIS-DAP adapter serial, when more than one probe is attached",
    ) orelse "";
}

/// The openocd fragment that picks one adapter, for a shell command line.
///
/// Trailing space and empty-when-unset are both deliberate: it splices between
/// `-f interface/cmsis-dap.cfg` and the target config, which is where `adapter
/// serial` has to go — after the driver exists, before `init` runs.
pub fn probeSelect(b: *std.Build, serial: []const u8) []const u8 {
    if (serial.len == 0) return "";
    return b.fmt("-c \"adapter serial {s}\" ", .{serial});
}

/// The same selection as argv, for `addSystemCommand` invocations. Append it
/// straight after the interface config with `addArgs`.
pub fn probeSelectArgs(b: *std.Build, serial: []const u8) []const []const u8 {
    if (serial.len == 0) return &.{};
    const args = b.allocator.alloc([]const u8, 2) catch @panic("OOM");
    args[0] = "-c";
    args[1] = b.fmt("adapter serial {s}", .{serial});
    return args;
}

/// The whole build of a board package (`lib/pico2`, `lib/pico2w`, ...): its
/// public `board` module, host tests of it, and a device typecheck. Each
/// board's `build.zig` is one call to this, so the four cannot drift apart.
///
/// A dependent passes the same `target` and `optimize` it gives picozig, so the
/// board and the dependent share one picozig — the only arguments either
/// takes, which is what keeps every `b.dependency("picozig", ...)` matching.
pub fn boardPackage(b: *std.Build) void {
    const target = b.standardTargetOptions(.{ .default_target = pico_target_query });
    const optimize = b.standardOptimizeOption(.{});

    const picozig = b.dependency("picozig", .{ .target = target, .optimize = optimize });
    const board = b.addModule("board", .{
        .root_source_file = b.path("src/board.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "rp2350", .module = picozig.module("rp2350") },
            .{ .name = "pico_board", .module = picozig.module("pico_board") },
        },
    });

    // The board's facts are constants and its claims are validated at comptime,
    // so analysing the module for the device is the check.
    const check_step = b.step("check", "Typecheck the board for the device");
    check_step.dependOn(&b.addObject(.{ .name = "board_check", .root_module = board }).step);
    b.getInstallStep().dependOn(check_step);

    const host = b.dependency("picozig", .{ .target = b.graph.host, .optimize = std.lang.Optimize.debug });
    const host_board = b.createModule(.{
        .root_source_file = b.path("src/board.zig"),
        .target = b.graph.host,
        .optimize = .debug,
        .imports = &.{
            .{ .name = "rp2350", .module = host.module("rp2350") },
            .{ .name = "pico_board", .module = host.module("pico_board") },
        },
    });
    const test_step = b.step("test", "Run the board's tests");
    test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = host_board })).step);
}

pub fn build(b: *std.Build) void { // Defaults to the Pico so `zig build` in this directory does the meaningful
    // thing with no flags. A dependent passes its own target through
    // `b.dependency("picozig", .{ .target = ... })`, which overrides this.
    const target = b.standardTargetOptions(.{ .default_target = pico_target_query });
    // Plain, with no `preferred_optimize_mode`. Setting one replaces `-Doptimize`
    // with `-Drelease`, and a dependent passing `.optimize` through
    // `b.dependency` would then fail with "invalid option: -Doptimize".
    // The smoke build below fixes its own mode regardless, which is what
    // actually matters: Debug is not a meaningful choice for firmware.
    const optimize = b.standardOptimizeOption(.{});

    // The public modules, registered under their names so a dependent can reach
    // them with `dep.module("rp2350")` and friends.
    _ = addModules(b, target, optimize, .public);

    // ------------------------------------------------------------------
    // Standalone build: prove the package compiles for freestanding on its own
    // ------------------------------------------------------------------
    // Not installed — the artifact is worthless, the type-checking is the
    // point. See src/freestanding_smoke.zig for why a root with `main` is
    // needed at all.
    const smoke_target = b.resolveTargetQuery(pico_target_query);
    const smoke_mods = addModules(b, smoke_target, .small, .private);
    const smoke_mod = b.createModule(.{
        .root_source_file = b.path("src/freestanding_smoke.zig"),
        .target = smoke_target,
        .optimize = .small,
        .imports = &.{
            .{ .name = "rp2350", .module = smoke_mods.rp2350 },
            .{ .name = "pico_board", .module = smoke_mods.pico_board },
            .{ .name = "boot", .module = smoke_mods.boot },
        },
    });
    const smoke = b.addExecutable(.{ .name = "freestanding-smoke", .root_module = smoke_mod });
    smoke.setLinkerScript(b.path("rp2350.ld"));
    // The image's true entry is the reset vector; there is no _start.
    smoke.entry = .{ .symbol_name = "_reset" };
    b.getInstallStep().dependOn(&smoke.step);

    // ------------------------------------------------------------------
    // elf2uf2 — host tool, installed so dependents can use it as an artifact
    // ------------------------------------------------------------------
    const uf2_mod = b.createModule(.{
        .root_source_file = b.path("tools/elf2uf2.zig"),
        .target = b.graph.host,
        .optimize = .safe,
    });
    const uf2_tool = b.addExecutable(.{ .name = "elf2uf2", .root_module = uf2_mod });
    b.installArtifact(uf2_tool);

    // ------------------------------------------------------------------
    // swd-rescue — the one SWD step that belongs to the platform, not an image
    // ------------------------------------------------------------------
    const openocd = b.option([]const u8, "openocd", "openocd binary to use (default: openocd)") orelse "openocd";
    const rescue_chip = b.option(
        []const u8,
        "rescue-chip",
        "Target chip for SWD rescue reset: rp2350 (default) or rp2040",
    ) orelse "rp2350";
    addRescueStep(b, openocd, probeSerialOption(b), rescue_chip);

    // ------------------------------------------------------------------
    // Check step (ZLS integration): typecheck without producing anything
    // ------------------------------------------------------------------
    const check_step = b.step("check", "Quickly check if everything compiles");
    check_step.dependOn(&smoke.step);
    check_step.dependOn(&uf2_tool.step);

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    // The chip layer is pure address arithmetic and divisor maths —
    // no MMIO is executed — so they build and run on the host. This is the part
    // of the platform where every possible bug is silent (a wrong SIO bank, a
    // wrong offset, an even-only prescaler), which makes it the part most worth
    // asserting rather than eyeballing.
    //
    // `boot` is absent on purpose: it is nothing but MMIO writes, linker
    // symbols and a reset vector, none of which mean anything on a host. The
    // freestanding smoke build above is what checks it.
    const test_step = b.step("test", "Run picozig's unit tests");
    const host_mods = addModules(b, b.graph.host, .debug, .private);
    // `pio_selftest` is here because its *encodings* and *arithmetic* are what
    // fail silently — a stage-E expectation that rounds to zero microseconds, or
    // a word whose halves are the wrong way round, would pass on hardware by
    // reporting nothing. The MMIO in it never executes under `zig test`; it is
    // only type-checked, which is the same bargain `rp2350` has always made.
    for ([_]*std.Build.Module{ host_mods.rp2350, host_mods.pico_board, host_mods.pio_selftest }) |mod| {
        const t = b.addTest(.{ .root_module = mod });
        test_step.dependOn(&b.addRunArtifact(t).step);
    }

    // The UF2 packer is a host tool, so its tests run like any other module's.
    const uf2_tests = b.addTest(.{ .root_module = uf2_mod });
    test_step.dependOn(&b.addRunArtifact(uf2_tests).step);
}

/// `zig build swd-rescue` — issue an SWD Rescue Debug Port reset via OpenOCD.
///
/// Clocked directly off SWCLK to reset the RP2040/RP2350 hardware and halt the
/// bootrom early in the boot flow before bad flash code can execute (e.g. code
/// that disabled clk_sys or entered unrecoverable sleep).
fn addRescueStep(b: *std.Build, openocd: []const u8, probe_serial: []const u8, target_chip: []const u8) void {
    const probe_select_args = probeSelectArgs(b, probe_serial);
    const target_cfg = b.fmt("target/{s}.cfg", .{target_chip});
    const cmd = b.addSystemCommand(&.{ openocd, "-f", "interface/cmsis-dap.cfg" });
    cmd.addArgs(probe_select_args);
    cmd.addArgs(&.{
        "-c",
        "adapter speed 1000",
        "-c",
        "set RESCUE 1",
        "-f",
        target_cfg,
        "-c",
        "shutdown",
    });
    cmd.has_side_effects = true;
    const step = b.step(
        "swd-rescue",
        b.fmt("Issue an SWD Rescue DP reset via OpenOCD for target {s}", .{target_chip}),
    );
    step.dependOn(&cmd.step);
}

const Modules = struct {
    rp2350: *std.Build.Module,
    pico_board: *std.Build.Module,
    boot: *std.Build.Module,
    pio_selftest: *std.Build.Module,
};

/// Whether this instance of the graph is the one dependents can see.
///
/// Only the `.public` call may register names: `addModule` puts a module in the
/// package's namespace, so calling it twice for `rp2350` would mean a dependent
/// silently got whichever instance was registered last — quite possibly the
/// host-targeted one built for the tests. `.private` instances are for this
/// package's own smoke build and test run and are reachable only from here.
const Visibility = enum { public, private };

/// The module graph, parameterised on target/optimize so it can be built once
/// for a dependent's chosen target and again for the host tests without either
/// build's modules leaking into the other's.
fn addModules(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.lang.Optimize,
    visibility: Visibility,
) Modules {
    const make = struct {
        fn f(
            bb: *std.Build,
            vis: Visibility,
            name: []const u8,
            options: std.Build.Module.CreateOptions,
        ) *std.Build.Module {
            return switch (vis) {
                .public => bb.addModule(name, options),
                .private => bb.createModule(options),
            };
        }
    }.f;

    const rp2350_mod = make(b, visibility, "rp2350", .{
        .root_source_file = b.path("src/rp2350/rp2350.zig"),
        .target = target,
        .optimize = optimize,
    });

    // What every Pico board package shares: the board list, the reference
    // wiring, and the helpers that turn a board's facts into claims.
    const pico_board_mod = make(b, visibility, "pico_board", .{
        .root_source_file = b.path("src/pico_board.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "rp2350", .module = rp2350_mod }},
    });

    const boot_mod = make(b, visibility, "boot", .{
        .root_source_file = b.path("src/boot.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "rp2350", .module = rp2350_mod }},
    });

    // Depends on `rp2350` and deliberately on no board. See
    // `src/pio_selftest.zig` for why the logic is here rather than in an
    // example's `main`.
    const pio_selftest_mod = make(b, visibility, "pio_selftest", .{
        .root_source_file = b.path("src/pio_selftest.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "rp2350", .module = rp2350_mod }},
    });

    return .{
        .rp2350 = rp2350_mod,
        .pico_board = pico_board_mod,
        .boot = boot_mod,
        .pio_selftest = pio_selftest_mod,
    };
}
