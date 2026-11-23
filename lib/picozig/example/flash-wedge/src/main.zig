//! flash-wedge — reproduces the board wedge **on purpose**, to find out what it is.
//!
//! For two months a Pico LiPo 2 intermittently entered a state where OpenOCD
//! read `SWD DPIDR` correctly and then failed every AP access, and the only
//! recovery anyone found was BOOTSEL plus a vendor UF2. It was worked around
//! rather than understood — first by closing `minicom`, then by moving to a
//! Pico 2 W. Both dodges were later shown to be coincidences.
//!
//! ## What it found, 2026-08-14
//!
//! **`clock-kill` is the answer.** Stopping `clk_sys` reproduces the recorded
//! wedge line for line: `DPIDR` reads, both cores fail examination, every AP
//! access stalls with `Connecting DP: stalled AP operation, issuing ABORT`, and
//! `reset run`, `cold_reset` and `swd-rescue` all fail at their first register
//! write. The DP is clocked from SWCLK so it keeps answering; every AP
//! transaction is synchronous to `clk_sys`, so none can complete.
//!
//! Recovery is a **power cycle** — nothing is damaged. BOOTSEL is needed only
//! when the image re-arms the fault faster than a reflash can land, which is
//! what `-Dwedge-delay=0` demonstrates: alive ~100 ms per boot, and a reflash
//! loop retrying once a second across many power cycles never landed once.
//!
//! **The flash modes were the wrong tree, and are kept because the negative
//! results are worth as much.** `plain`, `flush` and `irq` all *survived*:
//! `flash_exit_xip` leaves XIP readable, and the only direct-mode window is
//! inside the ROM erase call, during which the CPU runs from ROM and needs no
//! flash. What kills it is something *else* touching XIP meanwhile — and the
//! XIP cache normally hides even that. With the cache on, 85,509 flash-resident
//! interrupt handlers ran during erases without a fault; with `-Dwedge-nocache`
//! the first one killed the core. That asymmetry is the best explanation anyone
//! has for why the original failures looked random.
//!
//! `erase-loop` aims at the debugger rather than the target: one host read of
//! `0x10000000` during the erase window froze the loop and left QMI in direct
//! mode. Seen once, cleanly; not repeated enough to call reliable.
//!
//! ## Why it is safe to run
//!
//! * **`-Dwedge-delay` (default 25) is the safety catch.** Every boot offers
//!   that many seconds of healthy, reflashable board before the fault, so
//!   recovery never depends on the experiment behaving. **`-Dwedge-delay=0`
//!   removes the window deliberately** and produces a board only BOOTSEL can
//!   recover. Do not pass 0 without meaning it.
//! * **The flash modes erase at 1 MB**, far past this ~10 KB image, so the image
//!   itself always survives. `erase-loop` erases continuously — it is the only
//!   mode that puts real wear on the part.
//! * The stage marker is written to SRAM before and after each step, so the last
//!   value names the step that killed it. SRAM is reachable in every failure
//!   mode here except `clock-kill`, where nothing is.
//!
//! ```
//! zig build swd-flash-wedge -Dwedge-mode=clock-kill   # the real wedge
//! zig build watch-flash-wedge
//! ```
//!
//! `src/rp2350/flash.zig` does the flash sequence correctly, from `.ramfunc`.
//! **Nothing else in the workspace should imitate anything in this file.**

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const build_options = @import("build_options");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;
const flash = rp2350.flash;
const hw = rp2350.hw;

/// Seconds of healthy, reflashable board before the destructive call, from
/// `-Dwedge-delay`. Long enough to attach, read the block, and change your
/// mind. **Zero removes the window on purpose** — see `build.zig`.
const arm_delay_s: u32 = build_options.delay_s;

/// Flash offset for the erase, if execution ever gets that far. 1 MB in — this
/// image ends well before 64 KB.
const erase_offset: u32 = 0x0010_0000;

/// Where each step of the sequence is recorded. The last value written is the
/// step that killed the core.
const Stage = enum(u32) {
    boot = 0,
    funcs_resolved = 1,
    calling_connect = 2,
    connect_returned = 3,
    /// Reaching this means the fetch after `exit_xip` succeeded — which the
    /// XIP cache can allow, and which would refute the simple form of the
    /// hypothesis rather than confirm it.
    exit_xip_returned = 4,
    erase_returned = 5,
    program_returned = 6,
    flush_returned = 7,
    /// The whole sequence completed from flash. No wedge.
    survived = 8,
    /// `irq` mode: the alarm is running and the erase is about to be called.
    irq_armed = 10,
    /// `irq` mode: the erase returned with interrupts enabled throughout. No wedge.
    irq_survived = 11,
    /// `erase-loop` mode: erasing back-to-back, forever, correctly. The target
    /// is not the subject of this one — the **debugger** is.
    erase_looping = 20,
    /// `clock-kill` mode: the fatal write is next.
    clock_killing = 30,
    /// `clock-kill` mode: clk_sys survived being pointed at a dead source,
    /// which would mean the mux does something other than what `hw.zig` says.
    clock_survived = 31,
};

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    stage: u32,
    /// `flash.verifyRamResident()` — proves the *correct* routine really is in
    /// RAM in this build, so a wedge cannot be blamed on a broken linker script.
    ram_resident: u32,
    /// QMI as the bootrom left it, captured before anything is touched. This is
    /// the healthy fingerprint to compare a wedged board against.
    qmi_direct_csr: u32,
    qmi_m0_timing: u32,
    qmi_m0_rfmt: u32,
    qmi_m0_rcmd: u32,
    /// `irq` mode: interrupts taken. A value that stopped advancing at the
    /// moment the stage stopped advancing says the handler is where it died.
    irqs: u32,
    /// `erase-loop` mode: erases begun.
    erases: u32,
    /// `erase-loop` mode: 1 while the ROM erase is running, which is exactly
    /// the window in which QMI is in direct mode and the datasheet says a
    /// debugger reading XIP will take a bus fault. Written from RAM, so it
    /// stays truthful even while flash is unreachable.
    in_erase: u32,
};

const watch_magic: u32 = 0x5745_4447; // 'WEDG'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

/// How hard to try. Selected by `-Dwedge-mode`.
///
/// **`plain` was run first and the board survived it** (stage 8, 2026-08-14):
/// the whole sequence — erase and program included — executed from flash
/// without a fault, because it fitted in the XIP cache and never needed to
/// fetch from the flash it had just made unreadable. That is a real result
/// about how forgiving this mistake is, and the reason the other two modes
/// exist.
const Mode = enum {
    /// The sequence, straight through, from flash.
    plain,
    /// Invalidate the whole XIP cache immediately after `exit_xip`, so the very
    /// next instruction fetch *must* come from flash. Uses only the bootrom's
    /// own `flash_flush_cache`.
    flush,
    /// **The one the datasheet actually predicts.** `plain` and `flush` both
    /// survived, and re-reading the text says why: `flash_exit_xip` leaves XIP
    /// *readable* ("The QSPI device should be accessible for XIP reads after
    /// calling this function"), and the only window where QMI is in direct mode
    /// is inside the ROM erase call — during which the CPU is executing ROM and
    /// needs no flash at all. A single-threaded flash-resident caller is
    /// therefore safe by construction.
    ///
    /// What is *not* safe is anything touching XIP during that window. The
    /// datasheet names three: DMA, the other core, and — the one a single-core
    /// no-DMA program can still hit — an interrupt whose handler lives in
    /// flash. This mode runs the erase correctly from RAM and simply leaves a
    /// timer interrupt enabled, which is the mistake `flash.zig` masks
    /// interrupts to prevent.
    irq,
    /// **The debugger is the subject, not the target.** Everything on the board
    /// is done correctly: the erase runs from RAM with interrupts masked, and
    /// it survives indefinitely on its own. It simply erases back-to-back, so
    /// that QMI is in direct mode most of the time, and waits to see what
    /// happens when the *host* reads XIP through the AP during that window.
    ///
    /// The datasheet names three things that bus-fault if they touch XIP mid
    /// erase: DMA, the other core, and the debugger. This is the third, and the
    /// only one that was reliably present during every historical wedge —
    /// somebody was always attached with OpenOCD.
    erase_loop,
    /// **Aimed at the real wedge, which none of the flash modes reproduced.**
    ///
    /// Every flash mode above produced a hung CPU with a *working* debug
    /// interface: examination succeeded, SRAM and peripherals read fine, and
    /// `cold_reset` recovered it. The historical wedge is the opposite — the
    /// debug interface itself is gone. `Examination failed` on both cores,
    /// every AP access stalls, and only BOOTSEL clears it.
    ///
    /// What makes **every** AP access stall while `DPIDR` still answers? The DP
    /// is clocked from SWCLK and is independent of the chip. The AP's
    /// transactions are synchronous to **clk_sys**. Stop clk_sys and every AP
    /// transaction stalls forever while the DP answers perfectly — which is the
    /// recorded signature item for item, `CTRL/STAT = 0xf0000000` included.
    ///
    /// `hw.zig` already documents the mechanism, in the AUXSRC comment:
    /// "Switching AUXSRC under a running clk_sys is how you stop the core
    /// mid-instruction." And `build.zig`'s `swd-rescue` docstring already names
    /// this as the cause it exists for — "code that disabled clk_sys or entered
    /// unrecoverable sleep". Neither was ever tested.
    clock_kill,
};

const mode: Mode = if (std.mem.eql(u8, build_options.mode, "flush"))
    .flush
else if (std.mem.eql(u8, build_options.mode, "irq"))
    .irq
else if (std.mem.eql(u8, build_options.mode, "erase-loop"))
    .erase_loop
else if (std.mem.eql(u8, build_options.mode, "clock-kill"))
    .clock_kill
else
    .plain;

/// `CLK_SYS_CTRL.AUXSRC` = 4 selects **GPIN0**, a pin with no clock on it.
/// `hw.zig` only names the PLL because that is the only aux source picozig
/// legitimately uses; the mux value comes from the datasheet.
const sys_auxsrc_gpin0: u32 = 0x4 << 5;

/// How much to erase per call. Sixteen sectors at roughly 45 ms each puts QMI
/// in direct mode for the better part of a second at a time, so the host does
/// not have to win a race to land a read inside the window.
const erase_span: u32 = 16 * flash.sector_size;

/// Composes with any mode. **The cache is what saved the board in the first
/// three attempts** — with the sequence, the vector table and the interrupt
/// handler all cached, nothing ever had to read the flash that had been made
/// unreadable.
const nocache: bool = build_options.nocache;

/// TIMER0 alarm registers. **`INTR` and `INTE` are at 0x3c and 0x40 on
/// RP2350**, not RP2040's 0x34/0x38 — the register map was reshuffled, and
/// `hw.zig` does not carry the alarm set.
const timer0_base: u32 = 0x400b_0000;
const timer_timelr: u32 = timer0_base + 0x0c;
const timer_alarm0: u32 = timer0_base + 0x10;
const timer_intr: u32 = timer0_base + 0x3c;
const timer_inte: u32 = timer0_base + 0x40;

/// Short enough that the alarm is certain to fire inside a sector erase, which
/// takes tens of milliseconds.
const alarm_period_us: u32 = 500;

fn armAlarm() void {
    hw.write(timer_alarm0, hw.read(timer_timelr) +% alarm_period_us);
}

/// Claimed by name — `boot.zig` builds the vector table from this namespace at
/// comptime, so a misspelling is a compile error rather than a handler that
/// never runs.
///
/// **This handler lives in flash, and that is the entire point.** Taken while
/// the ROM erase has QMI in direct mode, both the vector fetch and the handler's
/// own instructions are reads of a flash that cannot answer.
pub const interrupts = struct {
    pub fn timer0_0() callconv(.c) void {
        hw.write(timer_intr, 1);
        watch.irqs +%= 1;
        armAlarm();
    }
};

/// The erase, correctly RAM-resident — `flash.zig`'s routine in every respect
/// **except** that interrupts are left enabled. Isolating that single variable
/// is what makes the result mean something: if this wedges and `flash.zig`'s
/// does not, the masking is proven to be the thing that matters.
/// The erase done **correctly** — RAM-resident, caller masks interrupts. The
/// `in_erase` flag is raised and lowered from RAM around the ROM call, so it
/// remains writable throughout the window in which flash is not.
fn eraseFromRamMasked(
    f: *const flash.Funcs,
    offset: u32,
    erase_len: u32,
) linksection(".ramfunc") callconv(.c) void {
    f.connect();
    f.exit_xip();
    watch.in_erase = 1;
    f.erase(offset, erase_len, 0, 0);
    watch.in_erase = 0;
    f.flush_cache();
    f.enter_xip();
}

fn eraseFromRamUnmasked(
    f: *const flash.Funcs,
    offset: u32,
    erase_len: u32,
) linksection(".ramfunc") callconv(.c) void {
    f.connect();
    f.exit_xip();
    f.erase(offset, erase_len, 0, 0);
    f.flush_cache();
    f.enter_xip();
}

/// XIP cache control, `XIP_CTRL_BASE`. **Bit 0 is `EN_SECURE`**, bit 1 is
/// `EN_NONSECURE`, bit 3 is `POWER_DOWN`. We run Secure, so bit 0 is the one.
const xip_ctrl: u32 = 0x400c_8000;

/// QMI registers, by their **real** datasheet offsets. `src/rp2350/qmi.zig`
/// has `m0_rfmt` and `m0_rcmd` transposed — its writes land correctly because
/// the values were matched to the wrong names, but the names are wrong. Read
/// straight from the datasheet here rather than inherit that.
const qmi_base: u32 = 0x400d_0000;
const qmi_direct_csr: u32 = qmi_base + 0x00;
const qmi_m0_timing: u32 = qmi_base + 0x0c;
const qmi_m0_rfmt: u32 = qmi_base + 0x10;
const qmi_m0_rcmd: u32 = qmi_base + 0x14;

fn mark(stage: Stage) void {
    watch.stage = @backingInt(stage);
}

/// **The deliberate mistake.** Identical to `flash.eraseAndProgramRam` except
/// for the one thing that matters: no `linksection(".ramfunc")`, so it executes
/// from flash. `callconv(.c)` keeps it a real function the optimiser will not
/// fold away.
///
/// Expected to die between `exit_xip()` and the store in `mark`, because that
/// store's own instruction has to be fetched from a flash the QMI can no longer
/// read.
fn eraseAndProgramFromFlash(
    f: *const flash.Funcs,
    offset: u32,
    data: [*]const u8,
    len: u32,
    erase_len: u32,
) callconv(.c) void {
    mark(.calling_connect);
    f.connect();
    mark(.connect_returned);

    f.exit_xip();
    // Invalidate every cache line while flash is unreadable, so the fetch of
    // the *next* instruction cannot be served from cache. `flush_cache` lives
    // in ROM, so calling it needs no flash.
    if (mode == .flush) f.flush_cache();
    // If this line executes, the instruction fetch after `exit_xip` succeeded.
    mark(.exit_xip_returned);

    f.erase(offset, erase_len, 0, 0);
    mark(.erase_returned);

    if (len > 0) f.program(offset, data, len);
    mark(.program_returned);

    f.flush_cache();
    mark(.flush_returned);

    f.enter_xip();
}

/// One sector of filler, only ever written if the sequence survives long enough
/// to reach the program step.
var payload: [flash.page_size]u8 = @splat(0xa5);

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .stage = @backingInt(Stage.boot),
        .ram_resident = @intFromBool(flash.verifyRamResident()),
        .qmi_direct_csr = hw.read(qmi_direct_csr),
        .qmi_m0_timing = hw.read(qmi_m0_timing),
        .qmi_m0_rfmt = hw.read(qmi_m0_rfmt),
        .qmi_m0_rcmd = hw.read(qmi_m0_rcmd),
        .irqs = 0,
        .erases = 0,
        .in_erase = 0,
    };

    uart.print(.uart0, "\n=== flash-wedge ===\n", .{});
    uart.print(.uart0, "board      {s}\n", .{board.name});
    uart.print(.uart0, "watch      {d} words at 0x20000000\n", .{@sizeOf(Watch) / 4});
    uart.print(.uart0, "ramfunc    correct routine in RAM: {s}\n", .{
        if (watch.ram_resident != 0) "yes" else "NO",
    });
    uart.print(.uart0, "QMI as the bootrom left it:\n", .{});
    uart.print(.uart0, "  DIRECT_CSR 0x{x:0>8}\n", .{watch.qmi_direct_csr});
    uart.print(.uart0, "  M0_TIMING  0x{x:0>8}\n", .{watch.qmi_m0_timing});
    uart.print(.uart0, "  M0_RFMT    0x{x:0>8}  (bit 12 PREFIX_LEN = {d})\n", .{
        watch.qmi_m0_rfmt,
        (watch.qmi_m0_rfmt >> 12) & 1,
    });
    uart.print(.uart0, "  M0_RCMD    0x{x:0>8}  (prefix 0x{x:0>2}, suffix 0x{x:0>2})\n", .{
        watch.qmi_m0_rcmd,
        watch.qmi_m0_rcmd & 0xff,
        (watch.qmi_m0_rcmd >> 8) & 0xff,
    });

    uart.print(.uart0,
        \\
        \\mode {s} is selected but DISARMED: the destructive section of this
        \\example is commented out in its source. Nothing is written to flash
        \\and the board is left exactly as it was found.
        \\
        \\Would arm after {d}s if enabled. See the block comment in
        \\example/flash-wedge/src/main.zig, and have BOOTSEL and a UF2 to hand
        \\before you take the comment markers off.
        \\
    , .{ @tagName(mode), arm_delay_s });

    // ------------------------------------------------------------------
    // EVERYTHING BELOW IS COMMENTED OUT ON PURPOSE.
    // ------------------------------------------------------------------
    //
    // **This example used to break the board, and an example must not do
    // that.** Disabled 2026-08-25 on a standing rule: *if an example cannot
    // bootstrap itself out of a state it put the board into, it does not ship
    // in a state that can reach it.* A new user working through the examples
    // without reading the header would otherwise arrive here and need BOOTSEL
    // and a UF2 to get out — and `-Dwedge-mode=clock-kill` with
    // `-Dwedge-delay=0` leaves no reflash window at all, by design.
    //
    // **The experiment and its findings are kept in full**, above and below,
    // because the negative results are the valuable part: `plain` and `flush`
    // both survived, and re-reading the datasheet explains why. Nothing here
    // is deleted, only disarmed.
    //
    // Uncommenting is the interlock. Anyone who reads this far, understands
    // what `clock-kill` does to a running chip, and takes the comment markers
    // off, can be trusted to have BOOTSEL and a UF2 to hand.
    //
    // ## Recovery, written out so nobody has to re-derive it
    //
    // Try these in order. The first two need no button and no cable change:
    //
    //   1. **Reflash inside the arming window** (default 25 s), which is the
    //      whole reason the window exists:
    //
    //          zig build swd-blink-hello -Dboard=pico2w
    //
    //   2. **An SWD Rescue DP reset**, which resets the chip from the debug
    //      port even when the core cannot be halted. `build.zig` wraps it:
    //
    //          zig build swd-rescue
    //
    //      or by hand:
    //
    //          openocd -f interface/cmsis-dap.cfg -c "adapter speed 1000" \
    //            -c "set RESCUE 1" -f target/rp2350.cfg -c "shutdown"
    //
    //   3. **BOOTSEL + a UF2.** Hold BOOTSEL while plugging USB in, then copy
    //      any working image onto the volume that appears:
    //
    //          zig build flash-blink-hello -Dboard=pico2w
    //
    //      This is the only route left after `clock-kill` with
    //      `-Dwedge-delay=0`, because clk_sys is stopped and there is no
    //      window: the chip cannot execute, so nothing over SWD can help.
    //
    // What still runs: the boot banner, `ram_resident`, and the QMI registers
    // as the bootrom left them — which is most of why anyone opens this
    // example, and none of it touches flash.
    // uart.print(.uart0, "\narmed -- mode {s}, calling the sequence from flash now\n", .{@tagName(mode)});
    //
    // // Nothing cached at any point, so every fetch after `exit_xip` goes to a
    // // flash that cannot answer.
    // if (nocache) {
    // const ctrl = hw.read(xip_ctrl);
    // hw.write(xip_ctrl, ctrl & ~@as(u32, 1));
    // }
    //
    // const funcs = flash.Funcs.resolve() orelse {
    // uart.print(.uart0, "bootrom lookup failed; nothing to test\n", .{});
    // while (true) timer.delayMs(1000);
    // };
    // mark(.funcs_resolved);
    //
    // if (mode == .clock_kill) {
    // mark(.clock_killing);
    // uart.print(.uart0, "pointing clk_sys at GPIN0. Goodbye.\n", .{});
    // // Let the UART drain before the clock that drives it stops.
    // timer.delayMs(50);
    //
    // // AUXSRC first, while SRC is still 0. This changes nothing on its own —
    // // aux is not selected — and is the order `hw.zig` prescribes, because
    // // AUXSRC is not glitchless.
    // hw.write(hw.Clocks.clk_sys_ctrl, sys_auxsrc_gpin0);
    // // The fatal write: select aux, which is now a pin with no clock.
    // hw.write(hw.Clocks.clk_sys_ctrl, sys_auxsrc_gpin0 | hw.Clocks.sys_src_aux);
    //
    // // Not reachable if the mux behaves as documented.
    // mark(.clock_survived);
    // while (true) {
    // gpio.toggle(led);
    // timer.delayMs(500);
    // watch.ticks +%= 1;
    // }
    // }
    //
    // if (mode == .erase_loop) {
    // mark(.erase_looping);
    // uart.print(.uart0, "erasing {d} KB back-to-back, forever. Hammer XIP from the host now.\n", .{erase_span / 1024});
    // while (true) {
    // watch.erases +%= 1;
    // rp2350.nvic.disableInterrupts();
    // eraseFromRamMasked(&funcs, erase_offset, erase_span);
    // rp2350.nvic.enableInterrupts();
    // // A short gap between erases, so a host read that lands outside the
    // // window still succeeds and the two cases stay distinguishable.
    // timer.delayMs(5);
    // gpio.toggle(led);
    // watch.ticks +%= 1;
    // }
    // }
    //
    // if (mode == .irq) {
    // // A timer interrupt at 2 kHz, handler in flash, left enabled across the
    // // erase. Everything else is done correctly.
    // hw.write(timer_intr, 1);
    // hw.write(timer_inte, 1);
    // rp2350.nvic.setPriority(rp2350.nvic.IrqNum.timer0_0, 0);
    // rp2350.nvic.enable(rp2350.nvic.IrqNum.timer0_0);
    // armAlarm();
    // mark(.irq_armed);
    //
    // eraseFromRamUnmasked(&funcs, erase_offset, flash.sector_size);
    //
    // mark(.irq_survived);
    // uart.print(.uart0, "\nSURVIVED -- erase completed with {d} interrupts taken.\n", .{watch.irqs});
    // while (true) {
    // gpio.toggle(led);
    // timer.delayMs(500);
    // watch.ticks += 1;
    // }
    // }
    //
    // // Interrupts off, exactly as the correct routine does: a handler living in
    // // flash, taken mid-sequence, would fault for the same reason and confuse
    // // the result.
    // rp2350.nvic.disableInterrupts();
    //
    // eraseAndProgramFromFlash(
    // &funcs,
    // erase_offset,
    // &payload,
    // payload.len,
    // flash.sector_size,
    // );
    //
    // // Only reachable if the whole sequence ran from flash without dying.
    // mark(.survived);
    // rp2350.nvic.enableInterrupts();
    //
    // uart.print(.uart0, "\nSURVIVED -- no wedge. The XIP cache covered the sequence.\n", .{});
    // while (true) {
    // gpio.toggle(led);
    // timer.delayMs(500);
    // watch.ticks += 1;
    // }

    uart.print(.uart0,
        \\
        \\The destructive section is commented out in this source file.
        \\Nothing has been written to flash and the board is unchanged.
        \\Read `example/flash-wedge/src/main.zig` before enabling it.
        \\
    , .{});

    // Blink slowly and keep `ticks` moving, so the board still says it is alive.
    const idle_led: rp2350.Pin = board.led orelse 22;
    gpio.initOutput(idle_led);
    while (true) {
        gpio.toggle(idle_led);
        timer.delayMs(500);
        watch.ticks +%= 1;
    }
}
