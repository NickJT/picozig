//! gspi-sweep — Rung A: where does the gSPI transport stop answering, and why?
//!
//! ```
//! zig build swd-gspi-sweep            # 150 MHz clk_sys, the whole grid
//! zig build watch-gspi-sweep          # read the packed results over SWD
//! ```
//!
//! ## The question
//!
//! `lib/cyw43` runs its bus at 3 MHz. The pico-sdk runs the same chip on the
//! same four pins at `clk_sys / 4` — 37.5 MHz on a 150 MHz Pico 2 W — and the
//! one attempt to follow it (rung 21d) died at rung 1 with
//! `ChipNotResponding`. The next thing recorded to try was drive
//! strength, slew rate and the input-synchroniser bypass. **All three were
//! already set, and had been for four commits before that attempt.** So the
//! written plan was spent before it was written, and this rig replaces it.
//!
//! ## Why a sweep and not a fix
//!
//! Every variant table in this package was taken at **one** bus rate, and
//! nobody noticed which. The Pico 2 W's famous `0/8, 7/8, 0/8` was measured at
//! **3 MHz** — `cyw43-probe` calls `clocks.init(board.clock_plan)`, which is
//! `null` for a Pico 2 W, so clk_sys was the bare 12 MHz crystal. An RM2
//! board's opposite answer was measured at **50 MHz**, because its test ran a
//! 200 MHz clk_sys through the bare `clk_sys / 4` divisor of the day. Two runs,
//! two boards, two bus rates, and the difference was written up as a property
//! of the boards.
//!
//! Line them up by rate instead and there is one variable moving one way. This
//! rig crosses rate with program on a **single board**, so the confound cannot
//! survive.
//!
//! ## The three axes
//!
//! * **Bus rate.** Six divisors at a fixed 150 MHz clk_sys: 3, 9.375, 12.5,
//!   18.75, 25 and 37.5 MHz. Fixing clk_sys and moving only the divisor is the
//!   point — rung 21d moved both at once. The datasheet caps the part at
//!   **50 MHz** (Table 43, T1 min 20.8 ns), so the top of the ladder is
//!   `clk_sys / 4` and there is deliberately no `clk_sys / 2`.
//!
//! * **The PIO program**, all four cells of the grid in `gspi.Variant`: two
//!   sample phases crossed with two bit offsets.
//!
//! * **The chip's own output edge** — `SPI_BUS_CONTROL` bit 4, "high-speed
//!   mode", datasheet Table 6. This axis is new. The host's sample edge is
//!   meaningless except against the edge the chip drives on, and this driver
//!   has been setting high-speed mode (chip outputs on the **rising** edge)
//!   while running `gap0_sample1` (host samples on the **rising** edge) since
//!   rung 1 — a mismatched pair that survives only because 3 MHz gives it a
//!   167 ns half-period to be wrong in.
//!
//! ## Three results per cell, because one is not enough
//!
//! **The test register is read before the handshake**, in 16-bit boot mode, at
//! whatever edge the chip powers up in. It is the only traffic in this driver
//! that does not run at the edge `bus_control` selects — so a variant can pass
//! the probe and fail everything after it. That is not hypothetical: it is
//! precisely what `gap010_sample1` did on the RM2 board, reading `0xFEEDBEAD`
//! perfectly and then failing at the CLM blob 224 KB later.
//!
//! So each cell reports:
//!
//!   1. **boot** — the 16-bit test register, 8 tries. What `cyw43-probe` has
//!      always measured, kept so this rig can be compared against it.
//!   2. **f0** — the *same* register read again after the handshake, as a plain
//!      32-bit command. First time this has ever been measured.
//!   3. **f1** — chipcommon's chip ID over a windowed Function 1 backplane
//!      read, which is the first traffic that resembles real work.
//!
//! ## The internal control
//!
//! The slowest rung, divisor 25 at 150 MHz, is **3.000 MHz — the same bus** as
//! divisor 2 at 12 MHz, which is what all twenty-one rungs were proven on. If
//! that cell does not reproduce `0/8, 7/8, 0/8` on `gap0_sample1` and its
//! neighbours, the rig is wrong and no other cell in the table means anything.
//! Read that row first.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const config = @import("sweep_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Dev = cyw43.Device(cyw43.PicogSpiBus);

// ---------------------------------------------------------------------------
// The grid
// ---------------------------------------------------------------------------

/// All four cells, slowest-changing axis first so the output groups by program.
const variants = [_]cyw43.gspi.Variant{
    .gap0_sample1,
    .gap01_sample0,
    .gap010_sample1,
    .gap0101_sample0,
};

/// **Slowest first, deliberately.** The first row printed is the control row;
/// if it disagrees with the proven 3 MHz result there is no point reading on.
///
/// **Divisor 5 is here for a clk_sys the default is not.** At 150 MHz the
/// achievable rates are `150 / 2d` and nothing lands between 18.75 and 25 MHz,
/// which is exactly where rung A bracketed the phase crossover — so the bracket
/// cannot be narrowed on this clock at all. At 200 MHz the same divisor list
/// reads 4, 12.5, 16.67, **20**, 25, 33.3 and **50** MHz: a point inside the
/// bracket, and one at the part's documented 50 MHz maximum. Changing the core
/// clock to reach bus rates the other clock cannot express is a legitimate use
/// of the two knobs; confusing the two is what rung 21d did.
const divisors = [_]u32{ 25, 8, 6, 5, 4, 3, 2 };

/// True is high-speed mode — chip outputs on the rising edge, and what every
/// rung through 27 has run.
const edge_modes = [_]bool{ true, false };

const cell_count = variants.len * divisors.len * edge_modes.len;

const tries: u32 = 8;

// ---------------------------------------------------------------------------
// The watch block
// ---------------------------------------------------------------------------

/// Results packed two cells to a word, so all 48 fit in the 128 bytes
/// `.selftest` allows.
///
/// **This exists because the UART has been the unreliable half of this rig.**
/// A sweep that only prints is a sweep that can be lost to a loose jumper, and
/// this one costs 48 nibble-writes.
const Watch = extern struct {
    magic: u32,
    /// Completed passes. A pass is the whole grid.
    pass: u32,
    /// The cell about to be attempted, written **before** it runs. If the
    /// firmware ever hangs inside a transfer, this names which cell did it —
    /// which a printed log cannot, because the line is printed after.
    cursor: u32,
    /// clk_sys as FC0 counted it, in kHz. Measured, not reported: every rate
    /// in the table below is derived from this number.
    clk_sys_khz: u32,
    /// Two cells per word, low half first. Each half is
    /// `boot(3:0) | f0(7:4) | f1(11:8) | handshake_ok(12)`.
    cells: [cell_count / 2]u32,
};

pub const watch_magic: u32 = 0x5357_5031; // 'SWP1'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
    if (cell_count % 2 != 0) @compileError("cells are packed in pairs");
}

fn recordCell(index: usize, boot_hits: u32, f0_hits: u32, f1_hits: u32, handshake_ok: bool) void {
    const packed_cell: u32 = (boot_hits & 0xF) |
        ((f0_hits & 0xF) << 4) |
        ((f1_hits & 0xF) << 8) |
        (@as(u32, @intFromBool(handshake_ok)) << 12);
    const word = index / 2;
    const shift: u5 = if (index % 2 == 0) 0 else 16;
    watch.cells[word] = (watch.cells[word] & ~(@as(u32, 0xFFFF) << shift)) |
        (packed_cell << shift);
}

// ---------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------

/// `-Dsweep-sys-mhz` overrides clk_sys **for this example only**; `board.clock_plan`
/// is untouched, as it was for `bt-throughput`. A measurement rig does not get
/// to overturn a board-wide decision.
const plan: ?clocks.Plan = if (config.sys_mhz == 0)
    board.clock_plan
else
    clocks.Plan.require(config.sys_mhz * 1_000_000);

const sys_hz: u32 = if (plan) |p| p.sys_hz else board.sys_hz;

// ---------------------------------------------------------------------------

pub fn main() noreturn {
    clocks.init(plan);
    gpio.init();
    uart.init(.uart0, .{
        .inst = .uart0,
        .tx = board.uart_tx,
        .rx = board.uart_rx,
        .clk_hz = sys_hz,
    });

    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;

    uart.print(.uart0, "\n=== gSPI sweep (rung A) ===\nboard {s}\n", .{board.name});

    watch.clk_sys_khz = clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_sys) orelse 0;
    uart.print(.uart0, "clk_sys {d} kHz (planned {d})\n", .{ watch.clk_sys_khz, sys_hz / 1000 });
    uart.print(.uart0, "silicon {s}, package {s}\n", .{
        @tagName(rp2350.silicon.stepping()),
        rp2350.silicon.package().name(),
    });
    uart.print(.uart0, "{d} cells: {d} variants x {d} rates x {d} edge modes\n\n", .{
        cell_count, variants.len, divisors.len, edge_modes.len,
    });

    const pins = cyw43.default_pins;

    var pass: u32 = 0;
    while (true) : (pass += 1) {
        var index: usize = 0;
        for (variants) |variant| {
            for (divisors) |div| {
                for (edge_modes) |high_speed| {
                    // Written before the cell runs, so a hang is attributable.
                    watch.cursor = @intCast(index);

                    const result = runCell(pins, variant, div, high_speed);

                    recordCell(index, result.boot_hits, result.f0_hits, result.f1_hits, result.handshake_ok);

                    uart.print(.uart0, "[{d}] div {d:>2} {d:>5} kHz hs={d} {s:<16} boot {d}/{d} f0 {d}/{d} f1 {d}/{d} {s} last=0x{X:0>8}\n", .{
                        // zig fmt: off
                        pass,
                        div,
                        busKhz(div),
                        @intFromBool(high_speed),
                        @tagName(variant),
                        result.boot_hits,   tries,
                        result.f0_hits,     tries,
                        result.f1_hits,     tries,
                        if (result.handshake_ok) "hs-ok " else "hs-ERR",
                        result.last,
                        // zig fmt: on
                    });

                    index += 1;
                }
            }
        }
        watch.pass = pass + 1;
        uart.print(.uart0, "-- pass {d} complete --\n\n", .{pass});
    }
}

fn busKhz(div: u32) u32 {
    return sys_hz / (2 * div) / 1000;
}

const CellResult = struct {
    boot_hits: u32 = 0,
    f0_hits: u32 = 0,
    f1_hits: u32 = 0,
    handshake_ok: bool = false,
    /// The last value the **boot** probe read, kept whatever it was. A wrong
    /// variant returns legible data, not noise: `0xFF765F56` is the wire stream
    /// shifted one bit, and reading it rather than discarding it is what named
    /// the variable on the RM2 board without a scope.
    last: u32 = 0,
};

/// One cell: a full power cycle, then the three reads.
///
/// **A fresh `GspiBus` and a real power cycle every time.** Reconfiguring a
/// live bus would leave the chip in whatever mode the previous cell's handshake
/// put it in, and the edge-mode axis would then measure the cell before it.
fn runCell(
    pins: cyw43.gspi.BoardPins,
    variant: cyw43.gspi.Variant,
    div: u32,
    high_speed: bool,
) CellResult {
    var result: CellResult = .{};

    var bus = cyw43.PicogSpiBus.init(pins);
    bus.setVariant(variant);
    bus.setBusDiv(div);
    bus.setHighSpeed(high_speed);

    var dev = Dev.init(bus, pins);

    dev.powerUp() catch return result;

    // 1. The 16-bit boot read, which is what cyw43-probe has always measured.
    //    Independent of `high_speed` by construction — it happens before the
    //    handshake — so the two edge-mode rows of a given (variant, rate) are a
    //    built-in replicate. If they ever differ, something is not repeatable.
    for (0..tries) |_| {
        result.last = dev.readTestRegister() catch 0;
        if (result.last == cyw43.gspi.test_pattern) result.boot_hits += 1;
    }

    // No point handshaking a chip that never answered; it would only add a
    // second failure downstream of the first.
    if (result.boot_hits == 0) {
        dev.powerDown() catch {};
        return result;
    }

    dev.busInit() catch {
        dev.powerDown() catch {};
        return result;
    };
    result.handshake_ok = true;

    // 2. The same register, now as a plain 32-bit command at the configured edge.
    for (0..tries) |_| {
        const v = dev.bus.readReg32(.bus, cyw43.gspi.reg.read_test) catch 0;
        if (v == cyw43.gspi.test_pattern) result.f0_hits += 1;
    }

    // 3. A windowed Function 1 backplane read — the first traffic that looks
    //    like work, and the one that carries a 16-byte response delay.
    result.f1_hits = dev.backplaneReadSuccessRate(tries) catch 0;

    dev.powerDown() catch {};
    return result;
}
