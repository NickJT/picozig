//! Bringing a pin up on RP2350 is a five-step dance and skipping any one of
//! them leaves a dead pin with no diagnostic:
//!
//!   1. Release IO_BANK0 and PADS_BANK0 from reset (once, at startup).
//!   2. Set the pad control  — input enable / output disable / drive strength.
//!   3. Set the pin's FUNCSEL to SIO, so software owns it rather than a
//!      peripheral.
//!   4. Set the SIO output-enable bit.
//!   5. **Clear the pad's ISO bit, last.** This step does not exist on RP2040
//!      and is the one an RP2040-shaped mental model omits. ISO resets to 1
//!      and freezes the pad: steps 2-4 all "succeed" and the pin still does
//!      nothing. Datasheet §9.7 is explicit that non-SDK applications must do
//!      this themselves, and that it belongs after the muxing is set up.
//!
//! Every SIO access goes through `hw.Sio.bank()`. On this board (an RP2350B)
//! GPIO 32-47 are real and used — VBAT sense on GP43, the BOOT button on
//! GP45 — and they are simply invisible to the low registers. Hand-writing
//! `1 << pin` against `gpio_out` would silently do nothing for those two.

const std = @import("std");
const hw = @import("hw.zig");

pub const Pin = hw.Pin;

/// Release the GPIO peripherals from reset. Call once, before any other
/// function here.
pub fn init() void {
    hw.Resets.unreset(hw.Resets.io_bank0 | hw.Resets.pads_bank0);
}

/// Configure `pin` as a software-driven push-pull output, initially low.
pub fn initOutput(pin: Pin) void {
    // Drive low before enabling the output, so a pin never glitches high on
    // the way up. Safe to do while still isolated — ISO latches the pad, it
    // does not block the SIO register write.
    clear(pin);

    // Pad: 4 mA drive, output enabled (OD clear), input buffer OFF.
    //
    // Clearing IE on a pure output is a power measure, not tidiness: it removes
    // the E9 leakage path outright on A2 silicon (see silicon.zig), and costs
    // nothing on any stepping.
    hw.write(hw.PadsBank0.pad(pin), hw.PadsBank0.drive_4ma | hw.PadsBank0.iso);

    // Hand the pin to SIO, then enable the output driver.
    hw.write(hw.IoBank0.ctrl(pin), hw.IoBank0.func_sio);
    const oe = hw.Sio.bank(pin, hw.Sio.gpio_oe_set);
    hw.write(oe.addr, oe.mask);

    deisolate(pin);
}

/// Configure `pin` as an input.
///
/// **Choose the pull direction with `silicon.hasE9Leakage()` in mind** — on A2
/// stock an internal `.down` does not hold a floating pad low and costs ~120 uA
/// doing so, while `.up` works normally. `silicon.zig` documents the erratum in
/// full and `silicon.pullDownIsReliable()` answers the question directly.
///
/// `.none` is only acceptable for a pin something else always drives hard, and
/// even then only while that driver is powered.
pub fn initInput(pin: Pin, pull: enum { none, up, down }) void {
    const oe = hw.Sio.bank(pin, hw.Sio.gpio_oe_clr);
    hw.write(oe.addr, oe.mask);

    var pad: u32 = hw.PadsBank0.ie | hw.PadsBank0.schmitt | hw.PadsBank0.iso;
    switch (pull) {
        .none => {},
        .up => pad |= hw.PadsBank0.pue,
        .down => pad |= hw.PadsBank0.pde,
    }
    hw.write(hw.PadsBank0.pad(pin), pad);

    hw.write(hw.IoBank0.ctrl(pin), hw.IoBank0.func_sio);

    deisolate(pin);
}

/// Hand `pin` to a peripheral (a FUNCSEL other than SIO) as an **output**.
///
/// The peripheral drives both the value and the output enable, so SIO's OE is
/// deliberately not touched. Input buffer off, per the E9 note above.
pub fn initFunctionOutput(pin: Pin, func: u32) void {
    hw.write(hw.PadsBank0.pad(pin), hw.PadsBank0.drive_4ma | hw.PadsBank0.iso);
    hw.write(hw.IoBank0.ctrl(pin), func);
    deisolate(pin);
}

/// Hand `pin` to a peripheral as an **input**.
///
/// A pull is not really optional on A2 silicon: an unconnected peripheral input
/// floats, and E9 then costs ~120 uA (see `silicon.zig`). For an idle-high
/// signal such as UART RX or SPI MISO, `.up` is both the correct resting level
/// and the direction the erratum leaves working.
pub fn initFunctionInput(pin: Pin, func: u32, pull: enum { none, up, down }) void {
    var pad: u32 = hw.PadsBank0.ie | hw.PadsBank0.schmitt | hw.PadsBank0.iso;
    switch (pull) {
        .none => {},
        .up => pad |= hw.PadsBank0.pue,
        .down => pad |= hw.PadsBank0.pde,
    }
    hw.write(hw.PadsBank0.pad(pin), pad);
    hw.write(hw.IoBank0.ctrl(pin), func);
    deisolate(pin);
}

/// Pad output drive strength.
pub const Drive = enum {
    ma_2,
    ma_4,
    ma_8,
    ma_12,

    fn bits(self: Drive) u32 {
        return switch (self) {
            .ma_2 => hw.PadsBank0.drive_2ma,
            .ma_4 => hw.PadsBank0.drive_4ma,
            .ma_8 => hw.PadsBank0.drive_8ma,
            .ma_12 => hw.PadsBank0.drive_12ma,
        };
    }
};

/// Change a pin's drive strength after `initOutput`.
///
/// The default is 4 mA, which is right for logic signals and marginal for
/// anything drawing real current: the setting is the load at which V_OH is
/// still guaranteed, so a pad asked for more than its setting sags, and an LED
/// on the end of it goes dim rather than failing outright.
///
/// Raising this does **not** force more current — the series resistor still
/// sets that. It only makes the pad a stiffer voltage source, so more of the
/// 3V3 rail reaches the load instead of being dropped across the driver.
pub fn setDriveStrength(pin: Pin, drive: Drive) void {
    const addr = hw.PadsBank0.pad(pin);
    hw.clrBits(addr, hw.PadsBank0.drive_12ma); // clear both DRIVE bits
    hw.setBits(addr, drive.bits());
}

/// Pad slew rate: fast, or the default slow.
///
/// The companion to `setDriveStrength`, and the two answer different questions.
/// Drive strength is how *hard* the pad holds a level once it gets there; slew
/// rate is how *quickly* it gets there. A clock that must stay square at tens of
/// MHz needs both — raising drive alone leaves the edge shallow, and a shallow
/// edge on a fast bus is read as a late one.
///
/// Slow is the default for a reason: a fast edge radiates more and rings harder
/// on an unterminated line, so this is for pins whose timing genuinely demands
/// it rather than for every output.
pub fn setSlewFast(pin: Pin, fast: bool) void {
    const addr = hw.PadsBank0.pad(pin);
    if (fast) {
        hw.setBits(addr, hw.PadsBank0.slewfast);
    } else {
        hw.clrBits(addr, hw.PadsBank0.slewfast);
    }
}

/// Park a pin as genuinely unused: no output drive, **no input buffer**, no
/// pulls.
///
/// This is the correct resting state for any pin nothing is connected to, and
/// it is a power decision rather than tidiness. Clearing IE removes E9's
/// leakage path outright on A2 silicon (see `silicon.zig`) and costs nothing on
/// later steppings — whereas parking a pin as an input with a pull-down, the
/// obvious-looking choice, is the exact configuration the erratum punishes.
///
/// Worth applying to every unconnected pin on the board once the sleep current
/// starts being measured.
pub fn parkUnused(pin: Pin) void {
    const oe = hw.Sio.bank(pin, hw.Sio.gpio_oe_clr);
    hw.write(oe.addr, oe.mask);
    hw.write(hw.PadsBank0.pad(pin), hw.PadsBank0.od | hw.PadsBank0.iso);
    hw.write(hw.IoBank0.ctrl(pin), hw.IoBank0.func_null);
    deisolate(pin);
}

/// Release a pad's isolation latch — the final step of any pin setup, and the
/// one with no RP2040 equivalent. Also needs re-running for every pad after
/// waking from a sleep state that powered down the switched core domain, since
/// that resets all ISO bits back to 1 (datasheet §9.7).
pub fn deisolate(pin: Pin) void {
    hw.clrBits(hw.PadsBank0.pad(pin), hw.PadsBank0.iso);
}

pub inline fn set(pin: Pin) void {
    const r = hw.Sio.bank(pin, hw.Sio.gpio_out_set);
    hw.write(r.addr, r.mask);
}

pub inline fn clear(pin: Pin) void {
    const r = hw.Sio.bank(pin, hw.Sio.gpio_out_clr);
    hw.write(r.addr, r.mask);
}

pub inline fn toggle(pin: Pin) void {
    const r = hw.Sio.bank(pin, hw.Sio.gpio_out_xor);
    hw.write(r.addr, r.mask);
}

pub inline fn put(pin: Pin, value: bool) void {
    if (value) set(pin) else clear(pin);
}

pub inline fn get(pin: Pin) bool {
    const r = hw.Sio.bank(pin, hw.Sio.gpio_in);
    return hw.read(r.addr) & r.mask != 0;
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------
//
// Added 2026-08-31 for `lib/ipnet` rung N2, whose whole design rests on being
// able to **mask** a level-triggered source. Datasheet §9.4.
//
// **The two kinds behave differently and the difference is the API.** Edge
// interrupts are latched in `INTR` and cleared by writing 1 to them. Level
// interrupts are not latched at all: the bit follows the pin, so a level-high
// interrupt whose pin is still high **re-fires the instant the handler
// returns, forever**. There is nothing to clear. The only way to stop it is to
// disable it, service whatever made the pin high, and enable it again — which
// is why `setIrqEnabled` is the load-bearing function here and not a
// convenience over some clear-and-continue.

/// What makes a pin interrupt. **The numbering is the hardware's**, and it is
/// not the order anyone guesses: the datasheet lays each GPIO's four bits out
/// low-to-high as LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH.
pub const Trigger = enum(u2) {
    level_low = 0,
    level_high = 1,
    edge_low = 2,
    edge_high = 3,

    /// True for the two that are not latched, and therefore cannot be cleared.
    pub fn isLevel(self: Trigger) bool {
        return self == .level_low or self == .level_high;
    }
};

/// Route `pin`'s `trigger` to proc0's `io_bank0` interrupt, or stop routing it.
///
/// **Enabling this is not enough on its own.** The NVIC line has to be enabled
/// too (`nvic.enable(nvic.IrqNum.io_bank0)`), and there has to be a handler for
/// it — see `boot.zig`, which checks the handler's *name* against
/// `nvic.IrqNum` at comptime so a misspelling is a compile error rather than a
/// vector that silently goes nowhere.
///
/// Read-modify-write on a shared register: this word carries eight GPIOs, so a
/// caller enabling GP24 must not clobber a neighbour. There is no atomic set
/// alias for IO_BANK0's interrupt registers, so **do not call this from an ISR
/// and the main loop at once** without a critical section of your own.
pub fn setIrqEnabled(pin: Pin, trigger: Trigger, enabled: bool) void {
    const word = hw.IoBank0.irqWord(pin);
    const bit = @as(u32, 1) << hw.IoBank0.irqBit(pin, @backingInt(trigger));
    const addr = hw.IoBank0.proc0_inte(word);
    const cur = hw.read(addr);
    hw.write(addr, if (enabled) cur | bit else cur & ~bit);
}

/// Whether `pin`'s `trigger` is currently asserted **and enabled** — the
/// post-mask status an ISR reads to find out who called it.
///
/// Deliberately not the raw `INTR`: a handler wants what was routed to it, and
/// reading raw would report a pin whose interrupt is disabled.
pub fn irqPending(pin: Pin, trigger: Trigger) bool {
    const word = hw.IoBank0.irqWord(pin);
    const bit = @as(u32, 1) << hw.IoBank0.irqBit(pin, @backingInt(trigger));
    return hw.read(hw.IoBank0.proc0_ints(word)) & bit != 0;
}

/// Clear a **latched edge**. Write-1-to-clear, so this touches no other bit in
/// the word and needs no read-modify-write.
///
/// **A level trigger has nothing to clear and this does nothing for one** —
/// the bit tracks the pin. Passing one is a programming error rather than a
/// no-op worth tolerating quietly, so it asserts: a handler that "clears" a
/// level interrupt and returns will be re-entered immediately, and that
/// presents as a wedged board rather than as a mistake.
pub fn clearIrq(pin: Pin, trigger: Trigger) void {
    std.debug.assert(!trigger.isLevel());
    const word = hw.IoBank0.irqWord(pin);
    const bit = @as(u32, 1) << hw.IoBank0.irqBit(pin, @backingInt(trigger));
    hw.write(hw.IoBank0.intr(word), bit);
}
