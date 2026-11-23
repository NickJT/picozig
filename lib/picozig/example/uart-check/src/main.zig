//! uart-check — is it the UART, the clock, the pad, or the wire?
//!
//! **Written 2026-08-25 for a factory-fresh Pico 2 W that flashed cleanly, ran,
//! blinked a pin, and put not one byte on the UART.** Every other example here
//! reports over that UART, so when the UART is the thing that is broken they all
//! go quiet together and none of them says why. This one reports over SWD, and
//! only uses the UART as its subject.
//!
//! It separates four things that all present as silence:
//!
//!   1. **clk_peri is not what the baud divisors were computed against.** The
//!      divisors come from `board.sys_hz`; if the real clock differs, the bits
//!      go out at the wrong rate. That normally produces *garbage* rather than
//!      silence, so it is measured here rather than assumed either way.
//!   2. **The UART block is not transmitting at all** — held in reset, wrong
//!      divisor, FIFO never draining.
//!   3. **The block transmits but the signal never reaches the pad** — wrong
//!      FUNCSEL, pad isolated, output disable still set.
//!   4. **The pad drives but nothing is listening** — wiring, or the wrong pin
//!      at the probe end.
//!
//! ## The loopback is the decisive step, and it needs no wiring
//!
//! UARTCR bit 7 is `LBE`. Per the datasheet (§12.1, UARTCR): "If this bit is set
//! to 1, and the SIRTEST bit is set to 0, the UARTTXD path is fed through to the
//! UARTRXD path." SIRTEST is 0 out of reset and nothing here sets it, so raising
//! LBE alone connects this UART's transmitter to its own receiver **inside the
//! block**, before the pads.
//!
//! So `loopback_ok` splits the four cases cleanly:
//!
//!   * **loopback fails** — the fault is at or below the block: clock, divisor,
//!     reset, FIFO. Cases 1 and 2. Nothing external can help.
//!   * **loopback passes and nothing arrives at the probe** — the block and the
//!     baud rate are both fine, and the fault is the pad, the mux or the wire.
//!     Cases 3 and 4. Read `pad_toggles` next.
//!
//! ## Then the pad, separately from the peripheral
//!
//! After the loopback, GP0 is taken *away* from the UART, driven as a plain SIO
//! output and toggled slowly for a few seconds. **Put the LED on GP0 for that
//! window** and the pad answers for itself, with no UART involved. Then GP0 goes
//! back to the UART and the banner streams forever, so a fixed wire starts
//! working with no reflash.
//!
//! ```bash
//! zig build swd-uart-check -Dboard=pico2w
//! zig build watch-uart-check      # the answer; run twice for liveness
//! ```

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const hw = rp2350.hw;
const gpio = rp2350.gpio;
const clocks = rp2350.clocks;
const uart = rp2350.uart;
const timer = rp2350.timer;

/// `LBE`, UARTCR bit 7. **Taken from the RP2350 datasheet §12.1's UARTCR table**,
/// not from PL011 folklore — the bit position and the SIRTEST interaction were
/// both read there before this was written.
const cr_lbe: u32 = 1 << 7;

/// How long GP0 is a plain toggling output before it goes back to the UART.
const pad_test_ms: u32 = 6000;
const pad_half_period_ms: u32 = 250;

const Watch = extern struct {
    magic: u32,
    /// Advances forever in the final loop. Liveness.
    ticks: u32,

    // -- what the clocks actually are, measured ---------------------------
    /// clk_sys in kHz, from the frequency counter. 0 if the measurement did
    /// not pass, which is distinct from measuring zero.
    clk_sys_khz: u32,
    /// clk_peri in kHz. **This is the one the baud rate is built from**, and
    /// the one worth doubting: clk_sys can be perfect while clk_peri is
    /// unglitched onto nothing.
    clk_peri_khz: u32,
    /// What `board.sys_hz` claims, in kHz, so the watch block carries the
    /// comparison rather than making the reader look it up.
    clk_expected_khz: u32,

    // -- the divisors, and what they really produce ------------------------
    ibrd: u32,
    fbrd: u32,
    /// The baud these divisors give at the *measured* clk_peri. Compare with
    /// 115200: more than about 3% out and a receiver will not frame it.
    actual_baud: u32,

    // -- the block itself --------------------------------------------------
    /// 1 if every byte sent with `LBE` set came back byte-exact.
    ///
    /// **This is the field that decides where to look.** 1 means the UART,
    /// its clock and its divisors are all sound and the fault is outside the
    /// block. 0 means it is inside, and no amount of rewiring will help.
    loopback_ok: u32,
    /// How many of the test bytes came back, and how many were correct:
    /// received | correct << 16.
    loopback_counts: u32,
    /// The first mismatch, as sent | got << 8, or 0xFFFF_FFFF if none.
    ///
    /// **A byte that comes back mangled rather than missing is a baud or
    /// framing fault**, and says something quite different from silence.
    loopback_first_bad: u32,
    /// `UARTFR` after the loopback: bit 3 BUSY, 4 RXFE, 5 TXFF, 7 TXFE.
    uart_fr: u32,

    // -- the pad, tested with no UART involved -----------------------------
    /// Half-cycles GP0 was toggled as a plain SIO output. **Put the LED on GP0
    /// during the first few seconds after reset** — if this counts up and the
    /// LED did nothing, the pad or the wire is the fault.
    pad_toggles: u32,
    /// `IO_BANK0.GPIO0_CTRL` once GP0 has been handed back to the UART.
    /// FUNCSEL is the low 5 bits and must read 2 for `func_uart`.
    gpio0_ctrl: u32,
    /// `PADS_BANK0.GPIO0` likewise: bit 7 is OD (output disable) and must be
    /// clear, bit 6 IE (input enable), bit 8 ISO — **ISO set is a pad that is
    /// electrically disconnected while every other register looks correct**,
    /// which is the trap this field exists for.
    pads0: u32,
    /// The same two for GP1, the receive pin.
    gpio1_ctrl: u32,
    pads1: u32,

    /// Bytes pushed out of the real UART since the pad test finished.
    sent: u32,
    /// `uart.print_errors` — formatting failures that `print` swallowed.
    print_errors: u32,
    /// `uart.dropped_bytes` — bytes given up on because the TX FIFO never
    /// drained. **Non-zero here with a healthy loopback means the transmitter
    /// is shifting nothing**, which is the pad or the clock to it.
    dropped_bytes: u32,
    /// The raw `UARTDR` of the first loopback byte, **including bits 8..11**:
    /// 8 FE (framing), 9 PE (parity), 10 BE (break), 11 OE (overrun).
    /// A byte that comes back as 0x00 with BE set is a line held low, which is
    /// a different fault from a byte that never arrives.
    first_dr: u32,
    /// **What is electrically attached to the pins**, before any peripheral
    /// touches them. One byte each, low to high: GP0, GP1, GP2, GP22. Within a
    /// byte, the low nibble is the reading under a weak pull-up and the high
    /// nibble the reading under a weak pull-down.
    ///
    /// **GP2 and GP22 are the calibration, and they are not optional.** This
    /// instrument was written in a hurry and its first readings were used to
    /// claim a wiring fault that did not exist — an inference from an
    /// uncalibrated instrument, which is precisely the mistake this workspace
    /// keeps re-learning. GP22 is unconnected on a bare board and must read
    /// up 1 / down 0. If it does not, **the instrument is wrong and nothing
    /// else in this field means anything.**
    ///
    /// How to read it — this is the field that tells a wiring fault from a
    /// firmware fault, and neither the loopback nor the FIFO flags can:
    ///
    ///   * **GP0 pull-up reads 0** — something is holding TX down. An LED to
    ///     ground will do it: an internal pull-up is ~50 kOhm and cannot reach
    ///     an LED's forward voltage, so the pin sits near 0 V. **The UART
    ///     cannot see this** — it shifts bytes out happily into a clamped line,
    ///     the FIFO drains, BUSY clears, and every counter reads perfect.
    ///   * **GP0 pull-up 1, pull-down 0** — nothing is loading TX. Normal.
    ///   * **GP1 pull-down reads 1** — the probe is driving our RX high, which
    ///     is what an idle UART line looks like. **This is the proof the cable
    ///     is actually connected**, and it is worth more than any amount of
    ///     staring at the transmit side.
    ///   * **GP1 pull-down 0 and pull-up 1** — nothing is driving RX: the
    ///     probe's TX wire is not connected, or the probe is not running.
    pin_probe: u32,
    /// Bytes drained from the RX FIFO *before* the loopback ran.
    ///
    /// **This is here because its absence produced a wrong answer.** The first
    /// version of this example read the FIFO without emptying it first, so it
    /// popped whatever a floating RX pin had already collected and reported
    /// "8 received, 0 correct" — which reads as a broken loopback and is
    /// nothing of the kind.
    stale_drained: u32,
};

pub const watch_magic: u32 = 0x75_61_72_74; // "uart"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

/// The loopback pattern. Deliberately includes 0x00, 0xFF and 0x55/0xAA:
/// a stuck-low line returns 0x00 for everything and would "pass" a test that
/// only ever sent zeroes, and 0x55/0xAA are the two that fail first when the
/// bit period is wrong.
const pattern = [_]u8{ 0x55, 0xAA, 0x00, 0xFF, 0x0F, 0xF0, 'U', '\n' };

pub fn main() noreturn {
    // `.selftest` is NOLOAD and survives a reset by design, so a fresh image
    // inherits the last one's values field for field. Initialise all of it.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .clk_sys_khz = 0,
        .clk_peri_khz = 0,
        .clk_expected_khz = 0,
        .ibrd = 0,
        .fbrd = 0,
        .actual_baud = 0,
        .loopback_ok = 0,
        .loopback_counts = 0,
        .loopback_first_bad = 0xFFFF_FFFF,
        .uart_fr = 0,
        .pad_toggles = 0,
        .gpio0_ctrl = 0,
        .pads0 = 0,
        .gpio1_ctrl = 0,
        .pads1 = 0,
        .sent = 0,
        .print_errors = 0,
        .dropped_bytes = 0,
        .first_dr = 0xFFFF_FFFF,
        .pin_probe = 0,
        .stale_drained = 0,
    };

    // Safe on this board: `blink-bare` established that the crystal reaches
    // STABLE on the first poll. On a board where that is *not* known, run
    // `blink-bare` first — this call contains an unbounded wait.
    clocks.init(board.clock_plan);
    gpio.init();

    watch.magic = watch_magic;

    // --- 0. what is attached to the pins, before anything claims them -------
    //
    // Done first and with no peripheral involved, because every later result
    // is about the *block* and none of them can see the wire.
    watch.pin_probe = probePin(0) | (probePin(1) << 8) |
        (probePin(2) << 16) | (probePin(22) << 24);

    watch.clk_expected_khz = board.sys_hz / 1000;
    watch.clk_sys_khz = clocks.measureKhz(hw.Clocks.fc0_src_clk_sys) orelse 0;
    watch.clk_peri_khz = clocks.measureKhz(hw.Clocks.fc0_src_clk_peri) orelse 0;

    // --- 1. the block, in loopback, with no pad in the path ----------------
    uart.init(.uart0, board.uartConfig());

    // What the hardware actually ended up with, rather than what was intended.
    watch.ibrd = hw.read(hw.Uart.ibrd(uartBase()));
    watch.fbrd = hw.read(hw.Uart.fbrd(uartBase()));
    if (watch.ibrd != 0 or watch.fbrd != 0) {
        watch.actual_baud = (4 * watch.clk_peri_khz * 1000) / (64 * watch.ibrd + watch.fbrd);
    }

    // **Empty the receive FIFO before testing it.** A floating or grounded RX
    // pin fills it with break characters, and popping those instead of the
    // loopback bytes reports a broken block that is working perfectly.
    while (hw.read(hw.Uart.fr(uartBase())) & hw.Uart.fr_rxfe == 0) {
        _ = hw.read(hw.Uart.dr(uartBase()));
        watch.stale_drained +%= 1;
    }

    const b = hw.Uart.cr(uartBase());
    const saved_cr = hw.read(b);
    // LBE on top of whatever `init` configured, so this tests the *real*
    // configuration rather than a special one that proves nothing about it.
    hw.write(b, saved_cr | cr_lbe);

    var received: u32 = 0;
    var correct: u32 = 0;
    for (pattern) |sent_byte| {
        uart.putc(.uart0, sent_byte);

        // The byte has to travel a whole character time through the block.
        // 87 us at 115200; a millisecond is ten times that and costs nothing
        // here.
        timer.delayMs(1);

        if (hw.read(hw.Uart.fr(uartBase())) & hw.Uart.fr_rxfe == 0) {
            const dr = hw.read(hw.Uart.dr(uartBase()));
            if (watch.first_dr == 0xFFFF_FFFF) watch.first_dr = dr;
            const got: u8 = @truncate(dr);
            received += 1;
            if (got == sent_byte) {
                correct += 1;
            } else if (watch.loopback_first_bad == 0xFFFF_FFFF) {
                watch.loopback_first_bad = @as(u32, sent_byte) | (@as(u32, got) << 8);
            }
        }
    }

    watch.uart_fr = hw.read(hw.Uart.fr(uartBase()));
    watch.loopback_counts = received | (correct << 16);
    watch.loopback_ok = @intFromBool(correct == pattern.len);

    // Back to normal. **Restoring rather than rewriting**: anything `init` set
    // that this file does not know about survives.
    hw.write(b, saved_cr);
    // Drain whatever the loopback left in the receive FIFO, so the first real
    // read after this is not the test pattern.
    while (hw.read(hw.Uart.fr(uartBase())) & hw.Uart.fr_rxfe == 0) {
        _ = hw.read(hw.Uart.dr(uartBase()));
    }

    // --- 2. the pad, with the UART taken out of the path --------------------
    //
    // GP0 becomes a plain SIO output. **Move the LED to GP0 for this window.**
    // If it does not flash while `pad_toggles` climbs, the pad or the wire is
    // the fault and the peripheral above it is innocent.
    gpio.initOutput(0);
    var elapsed: u32 = 0;
    while (elapsed < pad_test_ms) : (elapsed += pad_half_period_ms) {
        gpio.toggle(0);
        timer.delayMs(pad_half_period_ms);
        watch.pad_toggles +%= 1;
    }
    gpio.clear(0);

    // --- 3. hand GP0 back, and stream ---------------------------------------
    uart.init(.uart0, board.uartConfig());

    watch.gpio0_ctrl = hw.read(hw.IoBank0.ctrl(0));
    watch.pads0 = hw.read(hw.PadsBank0.pad(0));
    watch.gpio1_ctrl = hw.read(hw.IoBank0.ctrl(1));
    watch.pads1 = hw.read(hw.PadsBank0.pad(1));

    // Streams forever, so a wire fixed after this was flashed starts working
    // without a reflash. Short lines on purpose: a probe that is dropping
    // whole bursts is easier to spot against many small ones.
    while (true) {
        uart.print(.uart0, "uart-check {d}\n", .{watch.ticks});
        watch.sent +%= 1;
        watch.ticks +%= 1;
        watch.print_errors = uart.print_errors;
        watch.dropped_bytes = uart.dropped_bytes;
        timer.delayMs(500);
    }
}

fn uartBase() u32 {
    return hw.uart0_base;
}

/// What a weak pull reveals about whatever is attached to `pin`.
///
/// Returns the pull-up reading in bits 0..3 and the pull-down reading in bits
/// 4..7, where **0xF in the high nibble means "not measurable on this
/// silicon"** rather than a level.
///
/// ## Why the pull-down is often unmeasurable
///
/// **RP2350-E9: internal pull-downs on A2 are unreliable** —
/// `silicon.pullDownIsReliable()` exists to say so, and `silicon.zig` puts it
/// bluntly: "An internal `.down` on A2 is not a mitigation, it is the failure."
/// On A2 a pull-down does not hold a floating pin low, so the reading is not a
/// weak measurement, it is no measurement at all.
///
/// **This was found by the calibration below rather than by reading the code
/// that already documented it**, after an uncalibrated first version of this
/// probe was used to claim a wiring fault that did not exist. Hence GP22:
/// unconnected on a bare board, it *must* read up 1, and if it does not then
/// nothing else the probe says is worth reading.
///
/// So the pull-up reading is the one that carries information here:
///
///   * **up 1** — nothing is holding the pin low. For an output that is normal;
///     for an input expecting an idle-high signal it means nothing is driving.
///   * **up 0** — something is holding it low, and against a ~50 kOhm pull-up
///     that means a real load: a ground, a driven-low output, or an LED to
///     ground, which a pull-up cannot forward-bias.
fn probePin(pin: rp2350.Pin) u32 {
    gpio.initInput(pin, .up);
    // The settle delay matters: an internal pull is tens of kiloohms against
    // whatever capacitance the wire and the far end contribute, so reading
    // immediately after switching reports the *previous* state.
    timer.delayMs(2);
    const up: u32 = @intFromBool(gpio.get(pin));

    var down: u32 = 0xF;
    if (rp2350.silicon.pullDownIsReliable()) {
        gpio.initInput(pin, .down);
        timer.delayMs(2);
        down = @intFromBool(gpio.get(pin));
    }

    // Leave the pin as an input with no pull rather than holding one on a line
    // the UART is about to drive. **Not `.down` on A2** — that is the erratum's
    // ~120 uA leakage case exactly.
    gpio.initInput(pin, .none);
    return up | (down << 4);
}
