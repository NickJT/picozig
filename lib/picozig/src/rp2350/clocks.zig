//! Clock bring-up: the crystal, and optionally a PLL on top of it — `pll_sys`
//! for clk_sys, and `pll_usb` for the 48 MHz that USB requires.
//!
//! **Running straight off the 12 MHz crystal is still the default, and still a
//! deliberate choice.** A device that drives three LEDs and an SPI radio is
//! asleep almost all of the time and its wake work is trivial; 12 MHz rather
//! than 150 MHz means lower active current on every wake and one less block to
//! power back up coming out of dormant.
//!
//! The PLL arrived on 2026-07-30 for a display board, whose 480x480 panel needs a
//! continuously generated pixel clock and cannot be driven from a 12 MHz
//! clk_sys at all. It is opt-in: a board that wants it says so by declaring a
//! `clock_plan`, and a board that does not is unaffected — `init(null)` is
//! byte-for-byte the sequence that was here before.
//!
//! **The board chooses; this module only knows how.** `Plan` is pure
//! arithmetic over the datasheet's constraints, so it is comptime-evaluable and
//! host-tested, and an unreachable frequency is a compile error rather than a
//! chip that locks onto the wrong one.
//!
//! At reset the chip runs from the ROSC — a ring oscillator whose frequency
//! varies with process, voltage and temperature. That is fine for booting and
//! useless for timing anything, which is the whole reason this module exists.
//!
//! Order matters: clk_ref must be on the crystal before the tick generators are
//! started, because they divide clk_ref to produce their 1 us tick.

const std = @import("std");
const hw = @import("hw.zig");

/// The Pico family, and both boards here, use a 12 MHz crystal. RP2350 needs
/// exactly this for USB, so it is effectively fixed for Pico-compatible boards.
pub const xosc_hz: u32 = 12_000_000;

/// clk_sys when no PLL is configured. Equal to the crystal: clk_ref is the
/// crystal, clk_sys is clk_ref, and every divider is 1.
///
/// **This is not "the system clock" — it is the no-PLL case.** A board that
/// declares a `clock_plan` runs at `plan.sys_hz` instead, and callers computing
/// a divisor must read the frequency from the board rather than from here.
/// `board.sys_hz` is the value that is always right.
pub const crystal_sys_hz: u32 = xosc_hz;

// ---------------------------------------------------------------------------
// The system PLL
// ---------------------------------------------------------------------------

/// The datasheet's limits on `pll_sys`, all of which are silent when violated:
/// the PLL locks, and delivers a frequency nobody asked for.
pub const pll_limits = struct {
    /// Reference into the phase detector, after REFDIV.
    pub const ref_min_hz: u32 = 5_000_000;
    /// VCO range. Below `vco_min_hz` the loop may not lock at all; above
    /// `vco_max_hz` it is out of specification even when it appears to work.
    pub const vco_min_hz: u64 = 750_000_000;
    pub const vco_max_hz: u64 = 1_600_000_000;

    pub const fbdiv_min: u32 = 16;
    pub const fbdiv_max: u32 = 320;
    pub const refdiv_min: u32 = 1;
    pub const refdiv_max: u32 = 63;
    pub const postdiv_min: u32 = 1;
    pub const postdiv_max: u32 = 7;
};

/// One fully determined PLL configuration, plus the clk_sys it produces.
///
/// Built by `Plan.forSysHz`, which searches the divider space and refuses
/// anything it cannot hit **exactly**. Rounding to a near-miss would be the
/// wrong trade here: every consumer of `sys_hz` — UART baud, SPI prescaler, the
/// TICKS divisor, a pixel clock — derives an integer divisor from it, and an
/// approximate source frequency turns one wrong number into several, each
/// wrong in a different direction and none of them reported.
pub const Plan = struct {
    refdiv: u32,
    fbdiv: u32,
    postdiv1: u32,
    postdiv2: u32,
    /// The resulting clk_sys, in Hz. Exact by construction.
    sys_hz: u32,

    /// The VCO this plan runs the loop at — worth having for a boot banner,
    /// since it is the number that has to sit inside the legal range.
    pub fn vcoHz(self: Plan) u64 {
        return @as(u64, xosc_hz) / self.refdiv * self.fbdiv;
    }

    /// Find a plan producing exactly `target_hz` from the crystal, or null.
    ///
    /// The search is ordered to prefer, in this order: the **lowest VCO** that
    /// works, then the larger of the two post-dividers first. Both come from
    /// the datasheet's own guidance — a lower VCO draws less current, and
    /// POSTDIV1 >= POSTDIV2 is the documented convention.
    ///
    /// REFDIV is fixed at 1. With a 12 MHz crystal any larger value drops the
    /// phase-detector reference below 5 MHz for the FBDIV values that matter,
    /// and dividing a reference you have only 12 MHz of buys nothing.
    pub fn forSysHz(target_hz: u32) ?Plan {
        // The search is a nested sweep over ~70 x 7 x 7, which comfortably
        // exceeds the default comptime budget — and this is called at comptime
        // by every board that declares a plan.
        @setEvalBranchQuota(20_000);
        if (target_hz == 0) return null;
        const refdiv: u32 = 1;
        const ref_hz = xosc_hz / refdiv;
        if (ref_hz < pll_limits.ref_min_hz) return null;

        var fbdiv = pll_limits.fbdiv_min;
        while (fbdiv <= pll_limits.fbdiv_max) : (fbdiv += 1) {
            const vco = @as(u64, ref_hz) * fbdiv;
            if (vco < pll_limits.vco_min_hz) continue;
            if (vco > pll_limits.vco_max_hz) break; // monotonic — nothing further can fit

            var pd1 = pll_limits.postdiv_max;
            while (pd1 >= pll_limits.postdiv_min) : (pd1 -= 1) {
                var pd2 = pd1; // pd2 <= pd1, the documented convention
                while (pd2 >= pll_limits.postdiv_min) : (pd2 -= 1) {
                    const divisor = @as(u64, pd1) * pd2;
                    if (vco % divisor != 0) continue;
                    if (vco / divisor != target_hz) continue;
                    return .{
                        .refdiv = refdiv,
                        .fbdiv = fbdiv,
                        .postdiv1 = pd1,
                        .postdiv2 = pd2,
                        .sys_hz = target_hz,
                    };
                }
            }
        }
        return null;
    }

    /// `forSysHz` with the failure turned into a compile error naming the
    /// frequency that could not be reached. This is what a board file should
    /// call: a board's clock is a fact about the board, so it is known at
    /// comptime and a bad one should never reach a device.
    pub fn require(comptime target_hz: u32) Plan {
        comptime {
            return forSysHz(target_hz) orelse @compileError(
                std.fmt.comptimePrint(
                    "no exact PLL plan for {d} Hz from a {d} Hz crystal",
                    .{ target_hz, xosc_hz },
                ),
            );
        }
    }
};

// ---------------------------------------------------------------------------
// The USB PLL
// ---------------------------------------------------------------------------

/// USB requires **exactly** 48 MHz, and the tolerance is the specification's,
/// not ours: a Full Speed device's bit clock is 12 Mb/s +/- 0.25%. That is why
/// this is a fixed constant rather than something a board chooses — every other
/// clock in this module is a design decision, and this one is a requirement.
pub const usb_hz: u32 = 48_000_000;

/// `pll_usb`'s configuration.
///
/// **Derived, not transcribed.** It reuses `Plan`'s search, so the dividers here
/// are held to the same VCO and post-divider limits as `pll_sys` and are checked
/// by the same host tests. The `sys_hz` field means "the frequency this plan
/// produces", which for `pll_usb` is `usb_hz`; the name is `pll_sys`'s and is
/// left alone rather than renamed through fifty call sites.
///
/// It lands on a **768 MHz VCO** (fbdiv 64, both post-dividers 4), not the
/// 1200 MHz the C SDK uses for the same 48 MHz. Both are in specification and
/// the lower one is preferred here for the reason `forSysHz` already documents:
/// a lower VCO draws less current, and nothing downstream can tell them apart.
pub const usb_plan: Plan = Plan.require(usb_hz);

/// Bring up `pll_usb` and route clk_usb to it.
///
/// **Not called by `init`, and deliberately.** A board that never uses USB
/// should not pay for a second PLL: `adc.zig` sets out the standing argument for
/// running this project off the bare crystal, and a PLL nobody reads is exactly
/// the idle current that argument is about. USB is opt-in, and an application
/// that wants it calls this.
///
/// `sys_hz` is the board's clk_sys, and is checked rather than used —
/// see the erratum below. Pass `board.sys_hz`.
pub fn initUsb(comptime sys_hz: u32) void {
    comptime {
        // **RP2350-E12: the USB controller requires clk_sys > 48 MHz.**
        //
        // This is worth a compile error rather than a comment, because the
        // default configuration of this very library violates it. A board with
        // no `clock_plan` runs clk_sys from the 12 MHz crystal, and USB on it
        // would fail in the erratum's own manner — which is to say quietly.
        if (sys_hz <= usb_hz) @compileError(std.fmt.comptimePrint(
            "RP2350-E12: the USB controller needs clk_sys > {d} Hz, but this board runs at {d} Hz. " ++
                "Declare a clock_plan of at least 50 MHz before enabling USB.",
            .{ usb_hz, sys_hz },
        ));
    }

    startPll(hw.Pll.usb, usb_plan);

    // **clk_usb has no glitchless SRC mux** — only AUXSRC, which the datasheet
    // says "will glitch when switching". There is therefore nothing to park on
    // the way clk_sys parks on clk_ref, and the generator is instead stopped
    // outright while the source changes. ENABLE is cleared first for that
    // reason and not as a formality.
    hw.write(hw.Clocks.clk_usb_ctrl, 0);
    hw.write(hw.Clocks.clk_usb_div, hw.Clocks.div_by_1);
    hw.write(hw.Clocks.clk_usb_ctrl, hw.Clocks.usb_auxsrc_pll_usb);
    hw.write(hw.Clocks.clk_usb_ctrl, hw.Clocks.usb_auxsrc_pll_usb | hw.Clocks.usb_enable);

    // ENABLED is read-only and reports the generator actually running, which
    // CLK_USB_SELECTED cannot: with no SRC field that register is hardwired to
    // 0x1 and reads the same whether the clock runs or not.
    while (hw.read(hw.Clocks.clk_usb_ctrl) & hw.Clocks.usb_enabled == 0) {}
}

/// Whether clk_usb is running from a locked `pll_usb`. For a diagnostic block
/// or an example's banner; `measureKhz` is what answers "and at what rate".
pub fn usbRunning() bool {
    if (hw.read(hw.Pll.usb_cs) & hw.Pll.cs_lock == 0) return false;
    const ctrl = hw.read(hw.Clocks.clk_usb_ctrl);
    if (ctrl & hw.Clocks.usb_enabled == 0) return false;
    return ctrl & hw.Clocks.usb_auxsrc_mask == hw.Clocks.usb_auxsrc_pll_usb;
}

/// XOSC startup delay, in multiples of 256 crystal periods:
///     DELAY = (f_xtal_hz / 1000) * t_ms / 256
/// which for 12 MHz and 1 ms is 47. The datasheet states 1 ms is sufficient for
/// the RP2350 reference design at 12 MHz, and this matches what the SDK
/// computes. The reset value (0xc4 = ~50,000 cycles, about 4 ms at 12 MHz) also
/// works but costs four times as long on every wake that restarts the crystal —
/// which will matter once dormant sleep lands.
///
/// If a board ever fails to reach STABLE, raising this is the first thing to
/// try: crystal startup time is a property of the crystal, not the chip.
/// The `+ 128` rounds to nearest rather than truncating, matching what the SDK
/// computes; plain truncation gives 46, i.e. 0.98 ms, which is the wrong side of
/// the 1 ms target.
pub const startup_delay: u32 = 0xc4;

comptime {
    if (startup_delay != 0xc4) @compileError("unexpected XOSC startup delay for a 12 MHz crystal");
}

/// Bring up the crystal and switch clk_ref, clk_sys and clk_peri onto it, then
/// start the TIMER0 tick generator. Idempotent enough to re-run after a sleep
/// that powered the clocks down.
///
/// Pass a `Plan` to run clk_sys off the system PLL instead of the crystal, or
/// null to keep the crystal-direct behaviour. `board.clock_plan` is what a
/// caller should hand in — the board knows which it wants.
pub fn init(comptime plan: ?Plan) void {
    startXosc();

    // clk_sys is parked on clk_ref (the crystal) *before* the PLL is touched,
    // and stays there while it locks. This matters: AUXSRC is not a glitchless
    // mux, so it may only be changed while SRC selects clk_ref. Configuring the
    // PLL underneath a clk_sys that is already running from it is how a core
    // stops mid-instruction with no fault and no output.
    parkOnCrystal();

    if (plan) |p| {
        startPll(hw.Pll.sys, p);

        // AUXSRC first, while SRC is still 0 and clk_sys is on the crystal;
        // then SRC, which is glitchless and safe to switch live.
        hw.write(hw.Clocks.clk_sys_ctrl, hw.Clocks.sys_auxsrc_pll_sys | hw.Clocks.sys_src_ref);
        hw.write(hw.Clocks.clk_sys_div, hw.Clocks.div_by_1);
        hw.write(hw.Clocks.clk_sys_ctrl, hw.Clocks.sys_auxsrc_pll_sys | hw.Clocks.sys_src_aux);
        while (hw.read(hw.Clocks.clk_sys_selected) !=
            hw.Clocks.selectedMask(hw.Clocks.sys_src_aux))
        {}
    }

    // clk_peri <- clk_sys, enabled. Nothing uses it yet; the UART will.
    hw.write(hw.Clocks.clk_peri_ctrl, hw.Clocks.peri_auxsrc_clk_sys | hw.Clocks.peri_enable);

    startTimerTick();
    startWatchdogTick();
}

/// Put clk_ref and clk_sys on the crystal, with every divider at 1.
fn parkOnCrystal() void {

    // clk_ref <- XOSC. Glitchless mux, so this is safe to do live — but its
    // AUXSRC field gets the same treatment as clk_sys's below, and for the same
    // reason. The bootrom happens to leave clk_ref on ROSC rather than on aux,
    // so this one is defensive rather than a fix for anything measured.
    hw.write(hw.Clocks.clk_ref_div, hw.Clocks.div_by_1);
    const ref_ctrl = hw.read(hw.Clocks.clk_ref_ctrl);
    hw.write(hw.Clocks.clk_ref_ctrl, (ref_ctrl & ~hw.Clocks.ref_src_mask) | hw.Clocks.ref_src_xosc);
    while (hw.read(hw.Clocks.clk_ref_selected) !=
        hw.Clocks.selectedMask(hw.Clocks.ref_src_xosc))
    {}

    // clk_sys <- clk_ref. Also glitchless. We are executing from flash over XIP
    // clocked by clk_sys; slowing it down is always safe (flash timing has
    // margin to spare), which is why no XIP reconfiguration is needed.
    //
    // **AUXSRC must be left alone until SRC has finished moving off aux**, and
    // `sys_src_ref` is 0, so storing it bare would rewrite both fields in one
    // go. On a cold boot that is fatal: the bootrom hands over clk_sys running
    // from ROSC *through the aux mux* (`CLK_SYS_CTRL` = 0x41, measured on an
    // RP2350B 2026-08-16), and a bare store switches the aux input from the
    // running ROSC to a stopped PLL while the glitchless mux is still handing
    // over. The clock it is mid-handover on disappears, clk_sys stops for good,
    // and what is left is a chip whose DP answers and whose every AP access
    // stalls — the "wedge" this workspace chased for a fortnight.
    //
    // It survives a *warm* reset only because whatever ran before had already
    // left SRC on clk_ref or AUXSRC at 0, which is why flashing from the
    // debugger always worked and every cold boot did not.
    hw.write(hw.Clocks.clk_sys_div, hw.Clocks.div_by_1);
    const sys_ctrl = hw.read(hw.Clocks.clk_sys_ctrl);
    hw.write(hw.Clocks.clk_sys_ctrl, (sys_ctrl & ~hw.Clocks.sys_src_mask) | hw.Clocks.sys_src_ref);
    while (hw.read(hw.Clocks.clk_sys_selected) !=
        hw.Clocks.selectedMask(hw.Clocks.sys_src_ref))
    {}

    // Only now, with clk_sys demonstrably off the aux mux, is AUXSRC free to be
    // normalised. Nothing downstream depends on this value while SRC is 0; it
    // is written so the register is in a known state for `init`'s PLL path,
    // which sets AUXSRC deliberately.
    hw.write(hw.Clocks.clk_sys_ctrl, hw.Clocks.sys_src_ref);
}

/// Bring `pll_sys` up on the given plan and wait for lock.
///
/// The order is the datasheet's: reset the block, program the dividers while it
/// is powered down, then power up the loop, wait for LOCK, and only then let
/// the post-dividers out of power-down. Powering the post-dividers up before
/// LOCK puts an unsettled frequency on the output — which nothing downstream is
/// using yet in our case, but it costs one register write to do properly.
fn startPll(comptime regs: hw.Pll.Regs, comptime plan: Plan) void {
    comptime {
        const vco = @as(u64, xosc_hz) / plan.refdiv * plan.fbdiv;
        if (vco < pll_limits.vco_min_hz or vco > pll_limits.vco_max_hz)
            @compileError("PLL VCO outside the legal 750-1600 MHz range");
        if (vco / (@as(u64, plan.postdiv1) * plan.postdiv2) != plan.sys_hz)
            @compileError("PLL plan's output does not follow from its dividers");
    }

    // A held-in-reset PLL accepts writes to nothing at all. `unreset` spins on
    // RESET_DONE, which is the half of this that is easy to leave out.
    hw.Resets.unreset(regs.reset_bit);

    // Fully off, so the dividers below are programmed into a quiet block.
    hw.write(regs.pwr, hw.Pll.pwr_pd | hw.Pll.pwr_dsmpd |
        hw.Pll.pwr_postdivpd | hw.Pll.pwr_vcopd);

    hw.write(regs.cs, plan.refdiv & hw.Pll.cs_refdiv_mask);
    hw.write(regs.fbdiv_int, plan.fbdiv);

    // Power up the main loop: clear PD and VCOPD, leave POSTDIVPD set, and keep
    // DSMPD set because FBDIV is an integer (see hw.Pll).
    hw.write(regs.pwr, hw.Pll.pwr_dsmpd | hw.Pll.pwr_postdivpd);

    while (hw.read(regs.cs) & hw.Pll.cs_lock == 0) {}

    hw.write(regs.prim, hw.Pll.prim(plan.postdiv1, plan.postdiv2));
    hw.write(regs.pwr, hw.Pll.pwr_dsmpd);
}

/// Start the crystal oscillator and wait for it to stabilise.
///
/// **Public so it can be bisected against.** `init` switches clk_ref and then
/// clk_sys onto the crystal immediately afterwards, and if the crystal is not
/// really running that switch stops clk_sys mid-instruction — leaving a chip
/// whose DP still answers and whose every AP access stalls. Calling this alone
/// and then *not* switching is the one arrangement in which the outcome can
/// still be read out over SWD.
pub fn startXosc() void {
    // Clear the sticky BADWRITE flag first, so that reading it back afterwards
    // is an assertion about OUR writes rather than an inherited mystery.
    //
    // It arrives already set on a real chip. The likely author is the bootrom's
    // own XOSC bring-up, which (like the SDK's) writes FREQ_RANGE in one store
    // and ENABLE in another — and the first of those carries 0 in the ENABLE
    // field, which is not one of the two valid 12-bit codes, so the hardware
    // flags it. Harmless there, but it makes the flag useless as a check unless
    // cleared. Write-1-to-clear; every other bit in STATUS is read-only.
    hw.write(hw.Xosc.status, hw.Xosc.status_badwrite);

    // Set the startup delay BEFORE enabling — the datasheet is explicit that it
    // must be configured first, since it gates the STABLE flag.
    hw.write(hw.Xosc.startup, startup_delay);

    // 12 MHz falls in the 1-15 MHz range. Written together with ENABLE, both as
    // 12-bit magic codes.
    hw.write(hw.Xosc.ctrl, hw.Xosc.enable | hw.Xosc.freq_range_1_15mhz);

    // STABLE means the startup counter has expired and the output is usable.
    while (hw.read(hw.Xosc.status) & hw.Xosc.status_stable == 0) {}
}

/// Configure and start the TIMER0 tick generator: one tick per microsecond,
/// derived by dividing clk_ref.
///
/// **The generators divide clk_ref, not clk_sys, so the PLL does not enter
/// into it.** clk_ref stays on the crystal whether or not a `Plan` is in use,
/// which is why the divisor below is `xosc_hz` and stays correct on a board
/// running clk_sys at 200 MHz. If clk_ref ever moves, this moves with it.
///
/// This block is disabled at reset and has no RP2040 equivalent. Skip it and
/// TIMER0 never counts, so every delay built on it hangs forever.
fn startTimerTick() void {
    hw.Resets.unreset(hw.Resets.timer0);

    // "Before changing the cycle count, always stop the tick generator."
    hw.write(hw.Ticks.timer0_ctrl, 0);
    hw.write(hw.Ticks.timer0_cycles, xosc_hz / 1_000_000); // 12 -> 1 us
    hw.write(hw.Ticks.timer0_ctrl, hw.Ticks.ctrl_enable);

    while (hw.read(hw.Ticks.timer0_ctrl) & hw.Ticks.ctrl_running == 0) {}
}

/// Start the watchdog's tick generator.
///
/// Separate generator from TIMER0's, and just as disabled at reset — a watchdog
/// whose tick never arrives simply never fires, which is the most useless
/// possible state for a watchdog to be in. Started here so that enabling the
/// watchdog is a one-liner that cannot be half-done.
fn startWatchdogTick() void {
    hw.write(hw.Ticks.watchdog_ctrl, 0);
    hw.write(hw.Ticks.watchdog_cycles, xosc_hz / 1_000_000); // 1 us
    hw.write(hw.Ticks.watchdog_ctrl, hw.Ticks.ctrl_enable);
    while (hw.read(hw.Ticks.watchdog_ctrl) & hw.Ticks.ctrl_running == 0) {}
}

/// True once clk_ref and clk_sys are both running from the crystal, and our own
/// CTRL writes were accepted. Read back from the hardware rather than
/// remembered: a CTRL write carrying a bad 12-bit magic code is silently
/// ignored, leaving the previous value in place and only BADWRITE to show for
/// it — which is exactly the kind of failure this project keeps finding.
pub fn onCrystal() bool {
    const status = hw.read(hw.Xosc.status);
    if (status & hw.Xosc.status_badwrite != 0) return false;
    return status & hw.Xosc.status_stable != 0 and
        hw.read(hw.Clocks.clk_ref_selected) == hw.Clocks.selectedMask(hw.Clocks.ref_src_xosc) and
        hw.read(hw.Clocks.clk_sys_selected) == hw.Clocks.selectedMask(hw.Clocks.sys_src_ref);
}

/// True once clk_sys is running from a locked `pll_sys`, and the crystal
/// underneath it is stable and was configured by us.
///
/// The companion to `onCrystal` for a board that declared a `Plan` — for which
/// `onCrystal` returns *false* by design, since clk_sys is deliberately no
/// longer on clk_ref. Checking LOCK alone would not be enough: a PLL can be
/// locked and simply not selected, which reads as a working clock and runs the
/// core at 12 MHz.
pub fn onPll() bool {
    const status = hw.read(hw.Xosc.status);
    if (status & hw.Xosc.status_badwrite != 0) return false;
    if (status & hw.Xosc.status_stable == 0) return false;
    if (hw.read(hw.Clocks.clk_ref_selected) != hw.Clocks.selectedMask(hw.Clocks.ref_src_xosc)) return false;
    if (hw.read(hw.Pll.sys_cs) & hw.Pll.cs_lock == 0) return false;
    return hw.read(hw.Clocks.clk_sys_selected) == hw.Clocks.selectedMask(hw.Clocks.sys_src_aux);
}

/// Measure a clock with the FC0 frequency counter, in kHz.
///
/// **The point of this is that it measures rather than reports.** Every other
/// readback in this module tells you what was requested — a DIV register holds
/// whatever was written to it whether or not the source behind it is running,
/// and PLL LOCK says the loop closed, not that it closed where you meant. FC0
/// counts the clock against clk_ref, so it is the one number that can
/// contradict the plan.
///
/// Pass one of `hw.Clocks.fc0_src_*`. Returns null if the measurement did not
/// pass — which includes the test clock having stopped entirely, the case worth
/// distinguishing from "measured 0".
///
/// clk_ref must already be on the crystal, since it is the reference the count
/// is made against; a measurement taken against the ROSC is as vague as the
/// ROSC. Called after `init`, that is guaranteed.
pub fn measureKhz(src: u32) ?u32 {
    // The reference is clk_ref, which `init` has put on the crystal.
    hw.write(hw.Clocks.fc0_ref_khz, xosc_hz / 1000);

    // Accept anything: this is an instrument, not a gate. MIN/MAX exist to make
    // the counter itself flag an out-of-range clock, which would be useful in a
    // production self-test and is exactly wrong here — a clock that came out
    // unexpectedly is the result we most want to see reported, not rejected.
    hw.write(hw.Clocks.fc0_min_khz, 0);
    hw.write(hw.Clocks.fc0_max_khz, 0x1ff_ffff);

    // Writing SRC starts the count.
    hw.write(hw.Clocks.fc0_src, src);
    while (hw.read(hw.Clocks.fc0_status) & hw.Clocks.fc0_status_done == 0) {}

    const status = hw.read(hw.Clocks.fc0_status);
    const result = hw.read(hw.Clocks.fc0_result);

    // Release the counter, so a later measurement starts clean.
    hw.write(hw.Clocks.fc0_src, hw.Clocks.fc0_src_null);

    if (status & hw.Clocks.fc0_status_died != 0) return null;
    if (status & hw.Clocks.fc0_status_pass == 0) return null;
    return result >> hw.Clocks.fc0_result_khz_shift;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the plan for 200 MHz matches what a vendor's stock firmware runs" {
    // Read off the real board over SWD on 2026-07-30: CS = 0x80000001 (locked,
    // REFDIV 1), FBDIV_INT = 100, PRIM = 0x61000 (POSTDIV1 6, POSTDIV2 1).
    // An independent implementation arriving at the same dividers is the best
    // evidence available that the arithmetic here is right.
    const p = Plan.forSysHz(200_000_000).?;
    try testing.expectEqual(@as(u32, 1), p.refdiv);
    try testing.expectEqual(@as(u32, 100), p.fbdiv);
    try testing.expectEqual(@as(u32, 6), p.postdiv1);
    try testing.expectEqual(@as(u32, 1), p.postdiv2);
    try testing.expectEqual(@as(u64, 1_200_000_000), p.vcoHz());
}

test "the plan for 144 MHz is the one two bench projects rest on" {
    // **144 MHz, not 150.** An audio generator and an ADC instrument may share
    // a board, and `CLK_ADC_DIV` is a 4-bit integer field with no
    // fractional part: 150 MHz cannot reach the ADC's required 48 MHz by any
    // divide and 144 MHz does it exactly at /3. The audio side gains too —
    // 144 MHz divides the whole 48 kHz family exactly, 384 kHz included, where
    // 150 MHz runs out of divisor granularity at 384 kHz.
    //
    // Named here rather than derived at each call site because two projects now
    // depend on these dividers, and neither should discover they moved.
    const p = Plan.forSysHz(144_000_000).?;
    try testing.expectEqual(@as(u32, 1), p.refdiv);
    try testing.expectEqual(@as(u32, 72), p.fbdiv);
    try testing.expectEqual(@as(u32, 6), p.postdiv1);
    try testing.expectEqual(@as(u32, 1), p.postdiv2);
    try testing.expectEqual(@as(u64, 864_000_000), p.vcoHz());
    try testing.expectEqual(@as(u32, 144_000_000), p.sys_hz);
}

test "the SDK's default 150 MHz is reachable" {
    const p = Plan.forSysHz(150_000_000).?;
    // 900 MHz / 6, not the SDK's 1500 MHz / 5 / 2 — both are legal and exact,
    // and the lowest VCO wins here (see the preference test below).
    try testing.expectEqual(@as(u64, 900_000_000), p.vcoHz());
    try testing.expectEqual(@as(u32, 150_000_000), p.sys_hz);
}

test "every plan produced is internally consistent and inside the legal ranges" {
    // A sweep rather than spot checks: the failure this guards against is a
    // plan that looks plausible and runs the VCO out of specification, which
    // the hardware will not report.
    var target: u32 = 1_000_000;
    while (target <= 400_000_000) : (target += 1_000_000) {
        const p = Plan.forSysHz(target) orelse continue;
        const vco = p.vcoHz();
        try testing.expect(vco >= pll_limits.vco_min_hz);
        try testing.expect(vco <= pll_limits.vco_max_hz);
        try testing.expect(p.fbdiv >= pll_limits.fbdiv_min and p.fbdiv <= pll_limits.fbdiv_max);
        try testing.expect(p.postdiv1 >= pll_limits.postdiv_min and p.postdiv1 <= pll_limits.postdiv_max);
        try testing.expect(p.postdiv2 >= pll_limits.postdiv_min and p.postdiv2 <= pll_limits.postdiv_max);
        // The documented convention, and the reason the search orders pd1 first.
        try testing.expect(p.postdiv1 >= p.postdiv2);
        // Exact, never approximate — the whole contract of `forSysHz`.
        try testing.expectEqual(@as(u64, target), vco / (@as(u64, p.postdiv1) * p.postdiv2));
    }
}

test "an unreachable frequency is refused rather than approximated" {
    // 1 GHz needs a post-divisor of 1 from a VCO above the legal ceiling, and
    // no combination reaches it. Returning a near miss would be worse than
    // nothing: every downstream divisor would be computed from a lie.
    try testing.expect(Plan.forSysHz(1_000_000_000) == null);
    // A prime-ish target the divider space cannot land on exactly.
    try testing.expect(Plan.forSysHz(199_999_999) == null);
    try testing.expect(Plan.forSysHz(0) == null);
}

test "the search prefers the lowest VCO that works" {
    // 100 MHz is reachable from 900, 1200 and 1500 MHz. The lowest legal VCO
    // is the one that draws least, and is what we want.
    const p = Plan.forSysHz(100_000_000).?;
    try testing.expectEqual(@as(u64, 900_000_000), p.vcoHz());
}

test "pll_usb lands on exactly 48 MHz" {
    // USB's tolerance is +/-0.25% on a 12 Mb/s bit clock, so "exactly" is the
    // requirement rather than a preference. `Plan.require` refuses anything it
    // cannot hit on the nose, which is what makes this a real assertion.
    try testing.expectEqual(usb_hz, usb_plan.sys_hz);
    try testing.expectEqual(@as(u32, 48_000_000), usb_plan.sys_hz);
}

test "pll_usb's dividers are the ones a 768 MHz VCO implies" {
    // Pinned, because the search preferring a lower VCO is what makes these
    // differ from the C SDK's 1200 MHz / 5 / 5 for the same output. If the
    // preference is ever changed these move, and that should be a decision
    // rather than a surprise.
    try testing.expectEqual(@as(u32, 1), usb_plan.refdiv);
    try testing.expectEqual(@as(u32, 64), usb_plan.fbdiv);
    try testing.expectEqual(@as(u32, 4), usb_plan.postdiv1);
    try testing.expectEqual(@as(u32, 4), usb_plan.postdiv2);
    try testing.expectEqual(@as(u64, 768_000_000), usb_plan.vcoHz());
}

test "the C SDK's 1200 MHz route to 48 MHz is also legal, just not preferred" {
    // Both are correct. This pins the claim in `usb_plan`'s doc comment that
    // ours is a choice between two valid answers, not the only one available.
    const vco: u64 = 1_200_000_000;
    try testing.expect(vco >= pll_limits.vco_min_hz and vco <= pll_limits.vco_max_hz);
    try testing.expectEqual(usb_hz, vco / (5 * 5));
}

test "pll_usb's plan obeys every limit pll_sys is held to" {
    // The two PLLs are the same block, so the same constraints apply. Written
    // out rather than assumed because `usb_plan` is a fixed constant and would
    // otherwise be the one plan in this file nothing checks.
    const p = usb_plan;
    try testing.expect(p.refdiv >= pll_limits.refdiv_min and p.refdiv <= pll_limits.refdiv_max);
    try testing.expect(p.fbdiv >= pll_limits.fbdiv_min and p.fbdiv <= pll_limits.fbdiv_max);
    try testing.expect(p.postdiv1 >= pll_limits.postdiv_min and p.postdiv1 <= pll_limits.postdiv_max);
    try testing.expect(p.postdiv2 >= pll_limits.postdiv_min and p.postdiv2 <= pll_limits.postdiv_max);
    try testing.expect(p.postdiv2 <= p.postdiv1); // the documented convention
    try testing.expect(p.vcoHz() >= pll_limits.vco_min_hz and p.vcoHz() <= pll_limits.vco_max_hz);
    try testing.expect(xosc_hz / p.refdiv >= pll_limits.ref_min_hz);
    try testing.expectEqual(p.sys_hz, p.vcoHz() / (@as(u64, p.postdiv1) * p.postdiv2));
}

test "clk_usb's divider encoding is divide-by-one at INT=1" {
    // CLK_USB_DIV's INT is bits 19:16 with no fractional part, where clk_sys's
    // DIV is 16.16. The bit pattern for /1 coincides; this pins that it does,
    // since `initUsb` relies on one constant serving both.
    try testing.expectEqual(@as(u32, 0x0001_0000), hw.Clocks.div_by_1);
    try testing.expectEqual(@as(u32, 1), hw.Clocks.div_by_1 >> 16);
}

test "the USB clock registers are where the datasheet puts them" {
    // §12.7 and §8: CLOCKS at 0x40010000, PLL_USB at 0x40058000. Transcription
    // errors here are silent — a write lands somewhere harmless and the clock
    // simply never starts.
    try testing.expectEqual(@as(u32, 0x4001_0060), hw.Clocks.clk_usb_ctrl);
    try testing.expectEqual(@as(u32, 0x4001_0064), hw.Clocks.clk_usb_div);
    try testing.expectEqual(@as(u32, 0x4005_8000), hw.Pll.usb_cs);
    try testing.expectEqual(@as(u32, 0x4005_8004), hw.Pll.usb_pwr);
    try testing.expectEqual(@as(u32, 0x4005_8008), hw.Pll.usb_fbdiv_int);
    try testing.expectEqual(@as(u32, 0x4005_800c), hw.Pll.usb_prim);
    // AUXSRC 0 selects pll_usb, and ENABLE is bit 11 — not bit 28, which is the
    // read-only ENABLED that reports back.
    try testing.expectEqual(@as(u32, 0), hw.Clocks.usb_auxsrc_pll_usb);
    try testing.expectEqual(@as(u32, 1 << 11), hw.Clocks.usb_enable);
    try testing.expectEqual(@as(u32, 1 << 28), hw.Clocks.usb_enabled);
}

test "the two PLLs' register sets differ only by base" {
    try testing.expectEqual(hw.Pll.usb_cs - hw.Pll.sys_cs, hw.Pll.usb_prim - hw.Pll.sys_prim);
    try testing.expectEqual(hw.Pll.sys.cs, hw.Pll.sys_cs);
    try testing.expectEqual(hw.Pll.usb.cs, hw.Pll.usb_cs);
    try testing.expect(hw.Pll.sys.reset_bit != hw.Pll.usb.reset_bit);
}
