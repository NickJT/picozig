//! Runtime identification of the chip we are actually running on, and the
//! single home for stepping-dependent errata.
//!
//! **Why this exists.** Pimoroni's current Pico LiPo 2 stock is A2 silicon, and
//! a future batch will be A4 (the Pico Plus 2 W already shipped that way). The
//! difference matters — erratum E9 is a ~120 uA power bug on A2 and gone from A3
//! onwards — so the erratum reasoning belongs in exactly one place that the rest
//! of the code consults, rather than scattered through pad-setup comments where
//! it will drift or be missed. It was missed once already; see spi.zig's MISO
//! pull.
//!
//! **Everything here is read from the hardware, not configured.** The firmware
//! asks the chip what it is. That is one register read, it cannot go stale, and
//! it means a mixed drawer of boards is self-identifying instead of relying on
//! whoever flashed it to have picked the right build.
//!
//! Deliberately stateless — no cached values, no init call. A cache would need
//! initialising, and an uninitialised cache read is exactly the class of bug
//! (fine until something restarts in a different order) that this project has
//! made a point of designing out.
//!
//! **One exception to "a register read": `idHex`.** The unique device id is not
//! in SYSINFO — it comes from the bootrom's `get_sys_info`, so that one function
//! goes through `bootrom.zig` and is a ROM call rather than a load. It lives
//! here anyway because "which chip is this" is the question this file answers,
//! and a caller looking for the id will look here before it looks in a module
//! named after the boot ROM. The dependency runs one way only —
//! `bootrom.zig` does not import this file.

const hw = @import("hw.zig");
const bootrom = @import("bootrom.zig");

pub const chip_id_reg: u32 = hw.sysinfo_base + 0x00;
pub const package_sel_reg: u32 = hw.sysinfo_base + 0x04;

/// Silicon stepping, from CHIP_ID.REVISION (bits 31:28).
///
/// **The numbering is not contiguous**: A2 = 0x2, A3 = 0x3, and then A4 jumps to
/// **0x8**. Anything written as `rev >= 4` is wrong, and `rev == 4` is very
/// wrong. That is why this is an enum with explicit values rather than an
/// integer comparison anywhere.
pub const Stepping = enum(u4) {
    a2 = 0x2,
    a3 = 0x3,
    a4 = 0x8,
    /// Future silicon. Treated as "at least A4" for errata purposes — see
    /// `hasE9Leakage`, which is deliberately written as an A2 allowlist rather
    /// than an "is it new enough" test, so unknown steppings are assumed fixed
    /// rather than assumed broken.
    _,

    pub fn name(self: Stepping) []const u8 {
        return switch (self) {
            .a2 => "A2",
            .a3 => "A3",
            .a4 => "A4",
            _ => "unknown",
        };
    }
};

/// Package, from PACKAGE_SEL bit 0. Determines how many GPIOs physically exist.
pub const Package = enum(u1) {
    /// RP2350B — 80-pin, 48 GPIOs, 8 ADC channels. What the Pico LiPo 2 uses.
    qfn80 = 0,
    /// RP2350A — 60-pin, 30 GPIOs, 4 ADC channels. The Pico 2 / Pico 2 W.
    qfn60 = 1,

    pub fn name(self: Package) []const u8 {
        return switch (self) {
            .qfn80 => "QFN-80 (RP2350B)",
            .qfn60 => "QFN-60 (RP2350A)",
        };
    }

    /// Highest GPIO number that physically exists.
    pub fn maxGpio(self: Package) u6 {
        return switch (self) {
            .qfn80 => 47,
            .qfn60 => 29,
        };
    }
};

/// SYSINFO is held in reset at power-on like every other peripheral, and **that
/// is not harmless here** — it produces a wrong answer rather than an obvious
/// failure.
///
/// Measured on real A2 silicon in a QFN-60 package: with SYSINFO in reset,
/// `PACKAGE_SEL` reads `0x0`, which decodes as QFN-80 — confidently, plausibly,
/// and wrongly. Releasing the reset makes it read `0x1` = QFN-60, correct.
/// `CHIP_ID` meanwhile reads correctly either way, which is what makes the trap
/// convincing: the register next door looks fine.
///
/// Called from every accessor rather than from an `init()` so there is no
/// initialisation order to get wrong. `unreset` spins on RESET_DONE and returns
/// immediately once the peripheral is up, so repeat calls cost two register
/// accesses and cannot be sequenced incorrectly.
fn ensureUp() void {
    hw.Resets.unreset(hw.Resets.sysinfo);
}

pub fn chipId() u32 {
    ensureUp();
    return hw.read(chip_id_reg);
}

pub fn stepping() Stepping {
    return @fromBackingInt(@as(u4, @truncate(chipId() >> 28)));
}

pub fn package() Package {
    ensureUp();
    return @fromBackingInt(@as(u1, @truncate(hw.read(package_sel_reg))));
}

/// **The chip's unique 64-bit device id, as 16 uppercase hex digits — the one
/// thing that tells two boards apart.**
///
/// `chipId()` above does not: it is revision, part and manufacturer, identical
/// on every RP2350 of a given stepping. This is the id the C-SDK's
/// `pico_get_unique_board_id` returns, and what a serial number, a MAC address
/// or a per-device key should be derived from.
///
/// **Deliberately cannot fail**, because its caller is a banner line:
///
/// ```zig
/// uart.print(.uart0, "board id   {s}\n", .{silicon.idHex()});
/// ```
///
/// An error union here is the whole reason this was not already a one-liner,
/// and there is nothing a startup banner can usefully do with
/// `error.Unavailable` except print something. So it prints something — and the
/// filler is dashes rather than zeroes on purpose: `0000000000000000` is a
/// perfectly plausible id, and a fallback that can be mistaken for real data is
/// worse than no fallback at all. Anything that needs to *handle* the failure,
/// rather than report it, should call `bootrom.chipInfo()` directly and get the
/// `u64` and the error with it.
pub fn idHex() [16]u8 {
    const info = bootrom.chipInfo() catch return @splat('-');
    return info.hex();
}

// ---------------------------------------------------------------------------
// Errata
// ---------------------------------------------------------------------------

/// **RP2350-E9 — increased leakage on Bank 0 GPIO when the pad input is enabled.
/// Affects A2 only; fixed in the silicon from A3 onwards.**
///
/// An input-enabled pad sitting between VIL and VIH — i.e. floating — sources
/// roughly **120 uA** and self-latches near 2.2 V. On a device whose entire
/// sleep budget is 200-400 uA, one floating input dominates everything else.
///
/// The three consequences, which every pad decision in this codebase follows:
///
///   1. **Pull-up works** and is unaffected. It pulls the pad clear of the
///      problematic range. Prefer it wherever a pull is needed at all.
///   2. **Pull-down does NOT work on its own.** The internal pull-down is far
///      weaker than the leakage. Overcoming it needs an external pull of
///      8.2 kOhm or less. An internal `.down` on A2 is not a mitigation — it is
///      the failure.
///   3. **Clearing IE removes the leakage outright.** So output pins and unused
///      pins must have the input buffer off. `gpio.initOutput` does this, and it
///      is a power measure rather than tidiness.
///
/// Written as an explicit A2 test rather than "is the stepping older than A3",
/// so that unknown future silicon is assumed fixed. Guessing wrong in that
/// direction costs a slightly conservative pull choice; guessing wrong the other
/// way would silently reintroduce the bug on hardware that does not have it.
pub fn hasE9Leakage() bool {
    return stepping() == .a2;
}

/// True when an internal pull-down can be relied on to hold a floating input
/// low. False on A2 — see `hasE9Leakage`.
///
/// A pin that needs a defined level on A2 wants either a pull-**up**, an
/// external pull-down of 8.2 kOhm or less, or its input buffer disabled.
pub fn pullDownIsReliable() bool {
    return !hasE9Leakage();
}

/// **RP2350-E3 — GPIO_NSMASK controls the wrong PADS registers.** QFN-60 A2
/// only, fixed in A3.
///
/// Recorded for completeness and deliberately not acted on: it governs
/// *Non-secure* access to the pad registers, and this firmware declares Secure
/// in its IMAGE_DEF and never creates a Non-secure world. It cannot affect us.
pub fn hasE3NsmaskBug() bool {
    return stepping() == .a2 and package() == .qfn60;
}
