//! i2c-check — scan I2C bus pin pairs on RP2350 and report attached devices over SWD/UART.
//!
//! **The I2C equivalent of blinky**, and non-destructive: find out what is on a
//! bus before writing a driver for it. One pass per second, running for ever.
//!
//! ```bash
//! zig build swd-i2c-check -Dboard=pico2w
//! zig build watch-i2c-check
//! ```
//!
//! Verified 2026-08-26: **147 consecutive passes** over an ICM-20948 and a
//! BME688 on a Breakout Garden, both found on every pass, and `lib/icm20948`'s
//! full bring-up run straight afterwards without a power cycle — WHO_AM_I
//! 0xEA, AK09916 WIA2 0x09, accel/gyro/mag/temp all live, `error_code = 0`.
//!
//! ## It probes with a read, and skips the reserved ranges
//!
//! Both taken from the pico-sdk's `bus_scan`. A 1-byte read is a **single**
//! transaction — `START`, one address byte with the R/W bit set, the slave's
//! ACK, one byte, NACK, `STOP` — so it never leaves a register pointer moved
//! or a partial write behind. (A *register* read is two transactions and does
//! need `i2c.write`; a scan does not.) The skipped ranges are `0x00-0x07`,
//! which includes the High-Speed master codes every device on the bus listens
//! to, and `0x78-0x7F`, the 10-bit addressing prefixes — those do not NAK,
//! they begin a two-byte address sequence.
//!
//! ## Why the survey happens once, and the scan loop touches nothing else
//!
//! `gpio.initInput` and `riseTime` both write FUNCSEL, handing SDA and SCL from
//! the I2C block to SIO; only `i2c.init` hands them back. Anything of that kind
//! inside the scan loop disconnects the controller from the bus — see the
//! comment at the survey, which is where that mistake was made and what it
//! cost. Keep the loop pure I2C.
//!
//! ## Two real bugs this example found, both now fixed
//!
//!   1. **`i2c.init` configured the block in standard mode with `IC_SDA_HOLD`
//!      unwritten.** Transfers to an *absent* address then never started at
//!      all: the command sat in the TX FIFO, `IC_RAW_INTR_STAT` stayed flat
//!      zero, and no `TX_ABRT` ever arrived — so every absent address burned
//!      its full timeout and the abandoned transfer really did wedge live
//!      devices until power was removed. Present addresses worked, which is why
//!      every driver in the workspace was fine and only a scan ever showed it.
//!   2. **`setTarget` cached `IC_TAR`.** The SDK runs `enable=0; tar; enable=1`
//!      at the top of every transfer and never caches it, because taking
//!      `IC_ENABLE` low also flushes both FIFOs and returns the state machine
//!      to idle.
//!
//! **The failure signature worth recognising**: devices found on the first pass
//! and never again, with `dev0_info` (sticky) still holding them while
//! `dev_count` (live) reads 0. A sticky field beside a live one is what makes
//! that visible; either alone reads as "found" or as "gone".

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    scans: u32,
    dev_count: u32,

    dev0_info: u32, // (sda << 24) | (scl << 16) | (inst << 8) | addr
    dev1_info: u32,
    dev2_info: u32,
    dev3_info: u32,

    rv3028_found: u32, // 1 if 0x52 found
    rv3028_sda: u32,
    rv3028_scl: u32,

    bme688_found: u32, // 1 if 0x78 (or 0x76/0x77/0x3C) found
    bme688_sda: u32,
    bme688_scl: u32,

    last_addr_found: u32,
    /// **What is actually at 0x68**, which the address alone cannot say.
    ///
    /// The DS3231 and the ICM-20948 share that address, and the DS3231 has no
    /// address pins — so on a shared chain the IMU is the one that has to move
    /// to 0x69 with AD0. Distinguishing them needs a read, not a probe: the
    /// ICM-20948 answers `WHO_AM_I` (register 0) with **0xEA**
    /// (`lib/icm20948/reference/icm20948-datasheet.pdf` §8.1), while on a
    /// DS3231 register 0 is the seconds count and reads as plausible BCD.
    ///
    /// 0 = nothing at 0x68, 1 = ICM-20948, 2 = something else — most likely a
    /// DS3231, with its register 0 value in the high byte.
    at_0x68: u32,
    /// **The idle level of every pair's SDA and SCL, before any I2C traffic.**
    /// Two bits per pair in scan order, SDA low bit: `sda | scl << 1`.
    ///
    /// An I2C bus at rest is **high on both lines**, held there by pull-ups. So
    /// for a pair with devices on it, `0b11` is healthy and anything else is
    /// the fault:
    ///
    ///   * `0b00` — both lines held low. A short to ground, or a device that
    ///     has hung mid-transfer holding the bus.
    ///   * `0b01` / `0b10` — one line down. A short, or a broken conductor on
    ///     the line that reads low if it has no pull-up on it.
    ///   * `0b11` on an empty pair — no pull-ups, both lines floating and the
    ///     internal pull-up doing the work. Indistinguishable from healthy, so
    ///     read this **only for the pairs you have wired**.
    ///
    /// Measured with an internal pull-up, never a pull-down: RP2350-E9 makes
    /// pull-downs unreliable on A2 silicon and this board is A2.
    idle_levels: u32,
    /// The same pairs read with **no internal pull at all**, two bits each.
    ///
    /// **This was added to tell a powered bus from an empty one, and it does
    /// not.** The idea was that only external pull-ups would hold a line high
    /// with nothing of ours attached. Measured 2026-08-26: *every* pair reads
    /// `0b11` floating, including pins with nothing whatever connected — a
    /// floating CMOS input on this board simply sits high.
    ///
    /// Kept, because a reading of **0** here is still meaningful (something is
    /// actively pulling that line down), and because a field that was tried and
    /// found not to discriminate is worth recording as such rather than
    /// quietly deleting and re-inventing later. **Do not read `0b11` here as
    /// evidence of anything.**
    float_levels: u32,
    /// **How many devices each pair found, two bits each, this pass.**
    ///
    /// Added 2026-08-26 because `dev0_info` is sticky and `dev_count` is not,
    /// and reading a pattern out of those two produced a confident, wrong
    /// conclusion: that a harness had SDA and SCL swapped. It did not — a
    /// meter proved the wiring, and `i2c.init` treats the two pins
    /// symmetrically, so the two orderings cannot differ. What the sticky
    /// fields were actually showing was an **intermittent** bus.
    ///
    /// Per-pair counts show which pair saw what on *this* pass, so intermittent
    /// is distinguishable from positional without inference.
    per_pair_found: u32,
    /// `IC_TX_ABRT_SOURCE` after a targeted probe of **0x52** on the Qwiic
    /// pair, and of **0x68** in the next field.
    ///
    /// **"No device" and "the bus is broken" both read as a failed probe**, and
    /// only this register tells them apart. Bit 0 (`ABRT_7B_ADDR_NOACK`) means
    /// the address went out cleanly and nobody answered — the bus works, the
    /// part does not. ****A zero also means nobody answered.** The pico-sdk
    /// treats the two identically (`if (!abort_reason || abort_reason &
    /// ...ADDR_NOACK...)`) and its comment says a zero "seems to happen if
    /// there is nothing connected to the bus". The cause is a race both
    /// implementations share: `IC_TX_ABRT_SOURCE` is sampled one read *before*
    /// the `RAW_INTR_STAT` test that detects the abort, so on the iteration the
    /// abort lands the reason can still read 0. Both values were seen for a
    /// genuinely absent address on 2026-08-26 — 1 for the RV-3028's slot, 0 for
    /// the ICM's after it was unplugged.
    ///
    /// **Any other non-zero value** means the transfer never got that far:
    /// arbitration lost, a line held, or the controller never started.
    ///
    /// 0xFFFF_FFFF means the probe unexpectedly succeeded.
    abort_0x52: u32,
    abort_0x68: u32,
    /// `i2c.diagnostics()` taken on the Qwiic pair right after the two probes
    /// above fail: `IC_ENABLE`, `IC_ENABLE_STATUS`, `IC_STATUS`,
    /// `IC_RAW_INTR_STAT`.
    ///
    /// **`enable_status` is the field to read first.** `i2c.zig` records the
    /// trap in its own words: "`IC_ENABLE` reading 1 with the controller still
    /// down — writes then queue into a FIFO nobody is draining, which times out
    /// with no abort and looks exactly like a dead slave." That is precisely
    /// what a timeout with `abort_source` 0 looks like from outside, so the two
    /// have to be told apart here rather than guessed at.
    ///
    /// `IC_STATUS`: bit 0 ACTIVITY, 1 TFNF, 2 TFE, 3 RFNE, 5 MST_ACTIVITY.
    diag_enable: u32,
    diag_enable_status: u32,
    diag_status: u32,
    diag_raw_intr: u32,
    /// Polls taken for SDA to rise after being driven low and released, capped
    /// at `rise_limit`, with **bit 31 set if the line was confirmed low first**.
    ///
    /// Read bit 31 before the count. Without it a 0 is ambiguous — a strong
    /// pull-up and a pin that never went low both produce one. `rise_limit`
    /// with bit 31 set means the line was pulled down and **never came back
    /// up**: no pull-up on that line.
    rise_sda: u32,
    /// The same for SCL. **This is the one that stalls the controller**: the
    /// master releases SCL and waits for it to go high before clocking on.
    rise_scl: u32,
};

pub const watch_magic: u32 = 0x12c0_c4b0;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

const Pair = struct {
    sda: rp2350.Pin,
    scl: rp2350.Pin,
    inst: i2c.Instance,
};

/// **GP0/GP1 are deliberately absent, and that is a fix rather than an
/// oversight.** They are this board's debug UART, and scanning them muxes the
/// console's own pins to I2C mid-run — after which every line this example
/// prints arrives as noise. Found 2026-08-26: the scan worked perfectly and its
/// report was unreadable, which is a hard fault to attribute because the
/// evidence destroys itself.
///
/// A caller who genuinely has a bus there can say so with `-Di2c-scan-uart`.
///
/// ## There is no point listing a pair reversed, and here is why
///
/// A reversed entry — `.{ .sda = 5, .scl = 4 }` — was added on 2026-08-26 on
/// the theory that a scanner should try the obvious wiring mistake. **It cannot
/// work, and it found nothing that the forward entry could not.**
///
/// On RP2350 the I2C signal a pin carries is fixed by the *pin*, not by
/// software: the Bank 0 function table gives GP4 as `I2C0 SDA` and GP5 as
/// `I2C0 SCL`, and `FUNCSEL` only chooses *which peripheral* a pad belongs to.
/// `i2c.init` sets the same `func_i2c` on both pins and treats them
/// symmetrically, so `(sda=4, scl=5)` and `(sda=5, scl=4)` configure byte-for-
/// byte identical hardware.
///
/// **Physically swapped SDA and SCL therefore cannot be detected or corrected
/// from software.** It presents as an absent device and stays that way. The
/// only instruments for it are a meter and the schematic.
///
/// Recorded because the reversed entry appeared to work once — it was flashed
/// at the same moment a loose harness was reseated, and the coincidence
/// produced a confident, wrong diagnosis of swapped lines.
const pin_pairs = [_]Pair{
    // TEST: the Qwiic pair only. The other pairs are floating pins with no
    // pull-ups, where a transfer cannot complete and is abandoned -- and the
    // I2C0 block is shared with four of them.
    .{ .sda = 4, .scl = 5, .inst = .i2c0 },
};

/// How long a line takes to rise once released, in polls. Capped.
const rise_limit: u32 = 2000;

/// Drive `pin` low, release it with no pull, and count polls until it reads
/// high. See `rise_sda`.
fn riseTime(pin: rp2350.Pin) u32 {
    gpio.initOutput(pin);
    // `initOutput` clears IE for E9 leakage reasons, so the pad cannot be read
    // back. Turn it on for the duration of the measurement.
    rp2350.hw.setBits(rp2350.hw.PadsBank0.pad(pin), rp2350.hw.PadsBank0.ie);
    gpio.clear(pin);
    timer.delayMs(1);

    // **Confirm it actually went low before timing the rise.** Otherwise a
    // count of 0 is ambiguous: it reads the same whether a strong pull-up
    // snapped the line up instantly or the pin never went down at all. Bit 31
    // records which, so the number below is only ever read when it means
    // something.
    const went_low = !gpio.get(pin);

    // Release: input, no internal pull, so only an external one can lift it.
    gpio.initInput(pin, .none);
    var polls: u32 = 0;
    while (polls < rise_limit and !gpio.get(pin)) : (polls += 1) {}
    return polls | (@as(u32, @intFromBool(went_low)) << 31);
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== i2c-check ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .scans = 0,
        .dev_count = 0,
        .dev0_info = 0,
        .dev1_info = 0,
        .dev2_info = 0,
        .dev3_info = 0,
        .rv3028_found = 0,
        .rv3028_sda = 0,
        .rv3028_scl = 0,
        .bme688_found = 0,
        .bme688_sda = 0,
        .bme688_scl = 0,
        .last_addr_found = 0,
        .at_0x68 = 0,
        .idle_levels = 0,
        .float_levels = 0,
        .per_pair_found = 0,
        .abort_0x52 = 0,
        .abort_0x68 = 0,
        .diag_enable = 0,
        .diag_enable_status = 0,
        .diag_status = 0,
        .diag_raw_intr = 0,
        .rise_sda = 0,
        .rise_scl = 0,
    };

    // ------------------------------------------------------------------
    // Survey the wire ONCE, before anything drives it.
    // ------------------------------------------------------------------
    //
    // **Every test below takes SDA and SCL away from the I2C block.**
    // `gpio.initInput` and `riseTime` both write FUNCSEL, pointing the pad at
    // SIO; only `i2c.init` points it back. So running any of them inside the
    // scan loop disconnects the controller from the bus for every pass after
    // the one that last called `i2c.init`.
    //
    // That is not a hypothetical. It was done here on 2026-08-26 and produced
    // a flawless imitation of wedged hardware: pass 1 found every device, no
    // later pass found any, the lines read healthy high the whole time, and a
    // power cycle "recovered" it -- because a power cycle restarts the
    // firmware at pass 1. Most of a bench session went into suspecting the
    // breakouts, the harness and the driver before the instrument itself was
    // suspected. **When a fault appears only after the first iteration, look
    // at what the first iteration does that the others do not.**
    //
    // Once is also all these are worth: a resting level and a rise time are
    // properties of the wiring, and the wiring does not change between passes.
    var pair_index: u5 = 0;
    for (pin_pairs) |pair| {
        // **Look at the wire before driving it.** A bus held low reports no
        // devices in exactly the way an empty bus does, and the probe cannot
        // tell them apart -- only the resting level can.
        gpio.initInput(pair.sda, .up);
        gpio.initInput(pair.scl, .up);
        timer.delayMs(2);
        const levels: u32 = @as(u32, @intFromBool(gpio.get(pair.sda))) |
            (@as(u32, @intFromBool(gpio.get(pair.scl))) << 1);
        watch.idle_levels |= levels << (pair_index * 2);

        // And again with nothing of ours attached, so the answer is about the
        // wire rather than about us.
        gpio.initInput(pair.sda, .none);
        gpio.initInput(pair.scl, .none);
        timer.delayMs(2);
        const floating: u32 = @as(u32, @intFromBool(gpio.get(pair.sda))) |
            (@as(u32, @intFromBool(gpio.get(pair.scl))) << 1);
        watch.float_levels |= floating << (pair_index * 2);

        // Pull-ups, on the pair that is expected to have them.
        if (pair.sda == 4 and pair.scl == 5) {
            watch.rise_sda = riseTime(pair.sda);
            watch.rise_scl = riseTime(pair.scl);
        }

        pair_index += 1;

        // **And hand the pins back to the I2C block.** Initialising once, here,
        // is also what the pico-sdk's `bus_scan` does -- it calls `i2c_init`
        // before its loop, never inside it. Each call rewrites the pad
        // registers (ISO set, FUNCSEL, deisolate) and that puts edges on SDA
        // and SCL, which on a live bus is how a slave comes to believe a
        // transfer has started.
        i2c.init(pair.inst, .{
            .sda = pair.sda,
            .scl = pair.scl,
            .speed = .standard_100k,
            .clk_hz = board.sys_hz,
        });
    }

    var scan_count: u32 = 0;

    while (true) {
        scan_count += 1;
        watch.scans = scan_count;

        var dev_idx: u32 = 0;
        watch.dev_count = 0;
        watch.rv3028_found = 0;
        watch.bme688_found = 0;

        var pair_bit: u5 = 0;
        watch.per_pair_found = 0;
        for (pin_pairs) |pair| {
            var found_here: u32 = 0;

            var addr: u8 = 0;
            while (addr < 0x80) : (addr += 1) {
                // Skip the reserved ranges, as the pico-sdk's bus_scan does:
                // 0x00-0x07 (which includes the High-Speed mode master codes
                // 0x04-0x07, a bus-wide announcement every device sees) and
                // 0x78-0x7F (the 10-bit addressing prefixes).
                if (addr & 0x78 == 0 or addr & 0x78 == 0x78) continue;

                const addr7: u7 = @truncate(addr);
                if (i2c.probe(pair.inst, addr7, 10000)) {
                    const info: u32 = (@as(u32, pair.sda) << 24) |
                        (@as(u32, pair.scl) << 16) |
                        (@as(u32, @backingInt(pair.inst)) << 8) |
                        @as(u32, addr);

                    watch.last_addr_found = info;

                    if (dev_idx == 0) watch.dev0_info = info;
                    if (dev_idx == 1) watch.dev1_info = info;
                    if (dev_idx == 2) watch.dev2_info = info;
                    if (dev_idx == 3) watch.dev3_info = info;

                    dev_idx += 1;
                    watch.dev_count = dev_idx;
                    found_here += 1;

                    uart.print(.uart0, "Found device at 0x{x:0>2} on GP{d}(SDA)/GP{d}(SCL) [{s}]\n", .{
                        addr,
                        pair.sda,
                        pair.scl,
                        @tagName(pair.inst),
                    });

                    if (addr == 0x52) {
                        watch.rv3028_found = 1;
                        watch.rv3028_sda = pair.sda;
                        watch.rv3028_scl = pair.scl;
                    }

                    if (addr == 0x78 or addr == 0x76 or addr == 0x77 or addr == 0x3C) {
                        watch.bme688_found = 1;
                        watch.bme688_sda = pair.sda;
                        watch.bme688_scl = pair.scl;
                    }

                    // **0x68 is ambiguous, so ask — once.** See `at_0x68`.
                    //
                    // **`stop = true` on the pointer write, not a repeated
                    // start.** A held bus is the wrong risk to take in a
                    // scanner: `stop = false` keeps SCL/SDA owned until the
                    // matching read completes, so a read that fails leaves the
                    // bus locked and every later probe with it. That hung this
                    // example on 2026-08-26 after 29 clean passes — the scan
                    // stopped, `scans` froze, and the console went with it.
                    // `lib/icm20948`'s own bus layer uses `stop = true` here
                    // for the same reason.
                    //
                    // Asked once per boot rather than every pass: it is an
                    // identity, it cannot change, and a scanner should disturb
                    // an unknown device as little as it can.
                    if (addr == 0x68 and watch.at_0x68 == 0) {
                        var who: [1]u8 = .{0};
                        const reg0 = [1]u8{0x00};
                        if (i2c.write(pair.inst, addr7, &reg0, true, 10000)) |_| {
                            if (i2c.read(pair.inst, addr7, &who, true, 10000)) |_| {
                                watch.at_0x68 = if (who[0] == 0xEA)
                                    1
                                else
                                    2 | (@as(u32, who[0]) << 8);
                            } else |_| {}
                        } else |_| {}
                    }
                }
            }

            // **Release this pair before muxing the next.** `i2c.init` points a
            // pad at an I2C block and nothing ever points it away again, so
            // after one pass GP4, GP8, GP12, GP16 and GP20 were all still
            // muxed to I2C0 at once — five pads wired to one block, which is a
            // bus shorted across five pin pairs.
            //
            // The symptom was a device that appeared on the first pass and
            // intermittently afterwards: `dev0_info` held it (it is sticky)
            // while `dev_count`, which is not, read 0. **A sticky field beside
            // a live one is what made that visible**; either alone would have
            // read as "found" or as "gone".
            // **The cost is time, and it is worth it.** A pass went from about
            // a second to about twenty: probes on a genuinely floating pair now
            // run out their timeout instead of failing fast against a bus that
            // was shorted to a pair with pull-ups on it. The first pair is
            // scanned first, so a device on the Qwiic pins still shows up
            // within a second of boot; it is the empty pairs that are slow.
            watch.per_pair_found |= @as(u32, @min(found_here, 3)) <<
                @as(u5, @intCast(pair_bit * 2));
            pair_bit += 1;

            // On the Qwiic pair, ask *why* the two expected parts did not
            // answer. A failed probe alone cannot say.
            if (pair.sda == 4 and pair.scl == 5) {
                i2c.last_abort_source = 0;
                watch.abort_0x52 = if (i2c.probe(pair.inst, 0x52, 10000))
                    0xFFFF_FFFF
                else
                    i2c.last_abort_source;

                i2c.last_abort_source = 0;
                watch.abort_0x68 = if (i2c.probe(pair.inst, 0x68, 10000))
                    0xFFFF_FFFF
                else
                    i2c.last_abort_source;

                // --- are there pull-ups on this bus? ---
                //
                // **The question a probe cannot ask, and the one that matters
                // most.** I2C is open drain: the master pulls a line low and
                // then *releases* it, relying on a pull-up to bring it back.
                // With no pull-up the line never rises, the controller waits
                // for a clock that never comes, and the transfer times out
                // **with no abort** -- which is indistinguishable from a dead
                // slave from anywhere else, and is exactly what was seen here.
                //
                // So: drive the line low, release it with no internal pull, and
                // count how many polls it takes to read high. A real 2.2-4.7k
                // pull-up snaps it up in a handful; nothing but stray
                // capacitance takes forever or never gets there.
                //
                // An earlier version of this test switched each pin to an input
                // *before* reading it, which released the drive and made every
                // line read high. It measured nothing. Reading a pin you are
                // driving needs IE set on the pad -- `initOutput` clears it,
                // deliberately, as E9 leakage mitigation.
                // rise_sda/rise_scl are measured once, in the survey above:
                // riseTime drives the pin as SIO and would take the bus away
                // from the controller for every subsequent pass.

                const d = i2c.diagnostics(pair.inst);
                watch.diag_enable = d.enable;
                watch.diag_enable_status = d.enable_status;
                watch.diag_status = d.status;
                watch.diag_raw_intr = d.raw_intr_stat;
            }

            // Not parked any more: with a single pin pair and a one-shot
            // init there is nothing to release, and parking would undo the mux
            // that the next pass depends on.
        }

        timer.delayMs(1000);
    }
}
