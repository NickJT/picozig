//! bt-throughput — rung 21c: how fast can this driver take packets off the
//! shared bus?
//!
//! Build with `-Dfw=combined`.
//!
//! ## The question
//!
//! SBC joint stereo at 44.1 kHz is about 328 kbit/s — **~41,000 bytes a second**
//! of payload, before ACL and L2CAP overhead. Rung 21 established that this
//! chip does BR/EDR and that its ACL buffers are 1021 bytes; it explicitly did
//! not establish that the receive path can drain them fast enough for audio.
//! This does.
//!
//! It is the last question that can still say no. Everything above it — SSP,
//! L2CAP, SDP, AVDTP, an SBC decoder — is wasted if the bus cannot carry the
//! stream, so it is measured before any of that is written.
//!
//! ## What it measures, and what it does not
//!
//! **It drives the real receive path**: `btbus.injectInbound` puts a full
//! 1021-byte packet into the BT→host ring exactly where the controller would
//! have put it, and `btbus.read` takes it out through the same two `indices()`
//! reads, the same two `backplaneReadMem` calls and the same two index
//! writebacks a real packet costs. The bytes cross the gSPI bus for real.
//!
//! **It is a ceiling, not a demonstration.** Three things it leaves out:
//!
//! * **The controller's own pacing.** No radio, no peer, no scheduling — this
//!   is how fast the host *can* drain, not how fast packets *arrive*.
//! * **SBC decode.** There is no decoder in this workspace yet. Whatever it
//!   costs comes out of the margin measured here.
//! * **Audio DMA contention.** Measured before `lib/pcm5100` had an I2S engine,
//!   so the contention term is **not in this number**. Re-run this rig with
//!   one going.
//!
//! A number comfortably above 41,000 B/s does not prove audio works. A number
//! below it proves audio cannot work on this transport, which is the decision
//! this rung is for.
//!
//! ## Falsification
//!
//! `-Drequired=<bytes per second>` moves the pass threshold. The default is
//! 41,000. Set it to 10,000,000 and the same binary must report FAIL — a test
//! whose failing run needs a source edit is one nobody runs.
//!
//! ## The bus rate, and the program that has to go with it
//!
//! `-Dbus-div` sets the gSPI PIO divisor directly, **independently of clk_sys**;
//! 0, the default, keeps `setSysHz`'s behaviour of holding the bus at its proven
//! 3 MHz. At 150 MHz the ladder is `-Dbus-div=25,4,3,2` — 3, 18.75, 25 and
//! 37.5 MHz. Rung 21d could only move core and bus together, which is why its
//! result took a session to interpret.
//!
//! `-Dvariant` defaults to `auto`, which applies `Variant.forBusHz` — the
//! measured crossover from rung A. **Naming a program instead is the
//! falsification**: `-Dbus-div=2 -Dvariant=gap0_sample1` makes this same binary
//! print `FAILED at rung 1: ChipNotResponding` at a rate where `auto` carries
//! the whole ladder, which is the only way to show the pairing rule is doing
//! work rather than decorating a result that would have happened anyway.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const config = @import("throughput_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// One full BR/EDR ACL payload, as `Read_Buffer_Size` reports it.
const acl_payload = cyw43.hci.bredr_acl_data_max;

/// A Command Complete's worth, for the small-packet reading. Two points let the
/// fixed per-packet cost be separated from the per-byte one.
const small_payload = 6;

const iterations = 200;
const poll_iterations = 500;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// From `Read_Buffer_Size`, so this run's figures sit next to the buffers
    /// they were measured against.
    acl_len: u32,
    acl_count: u32,
    /// Microseconds for one full 1021-byte packet, and for a 6-byte one, each
    /// ×1000 so a sub-microsecond figure survives as an integer.
    large_ns: u32,
    small_ns: u32,
    /// Nanoseconds for one poll of an empty ring — the idle-loop cost.
    poll_ns: u32,
    /// The headline: payload bytes a second through the real read path.
    bytes_per_sec: u32,
    /// And the same as whole 1021-byte packets a second.
    packets_per_sec: u32,
    /// What was required of it, and whether it cleared the bar.
    required: u32,
    verdict: u32,
    /// How many times over. 100 = exactly the requirement.
    margin_pct: u32,
    /// **The figure that matters more than the margin**: what fraction of the
    /// core sustaining the required rate consumes, as a percentage. A pass at
    /// 216% of the requirement is the same thing as spending 46% of every
    /// second inside the receive path, and the second number is the one an
    /// SBC decoder has to share with.
    duty_pct: u32,
    /// Derived: the fixed per-packet overhead in ns, and the marginal cost of a
    /// byte in ns. Two packet sizes, one straight line.
    fixed_ns: u32,
    per_kb_ns: u32,
    /// Packets the read path refused, and packets it lost. Both must be zero.
    too_large: u32,
    mismatches: u32,
    /// **clk_sys as FC0 counted it**, in kHz — measured, not assumed. Every
    /// figure above scales with this, and the first run of this rig was taken
    /// at 12 MHz without anyone noticing.
    clk_sys_khz: u32,
    /// The gSPI bus rate this run was taken at, in kHz, and the PIO program it
    /// ran. **Every figure above is meaningless without these two**, which is
    /// the whole lesson of rung 21c being measured at 12 MHz and rung 21d
    /// discovering it.
    bus_khz: u32,
    variant_idx: u32,
    reserved: [8]u32,
};

pub const watch_magic: u32 = 0x4254_5450; // 'BTTP'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "FAILED at rung {d}: {s}\n", .{ rung, @errorName(err) });
    watch.magic = 0xDEAD_4254;
    watch.rung = rung;
    watch.err_code = switch (err) {
        error.ChipNotResponding => 2,
        error.AlpTimeout => 4,
        error.CoreNotInReset => 5,
        error.HtClockTimeout => 6,
        error.F2NotReady => 7,
        error.BackplaneUnreliable => 13,
        error.BtNotReady => 40,
        error.QueueFull => 41,
        error.PacketTooLarge => 42,
        error.BadPatchram => 43,
        error.NoHeaderRoom => 44,
        error.NoHciAnswer => 45,
        error.CommandFailed => 46,
        error.NoAnswer => 47,
        error.WrongOpcode => 48,
        error.Malformed => 49,
        error.BufferTooSmall => 51,
        error.ClassicNotSupported => 60,
        // This rung's own: a packet came back different from the one injected.
        error.ReadbackMismatch => 61,
        else => 99,
    };
    while (true) timer.delayMs(1000);
}

var payload: [acl_payload]u8 = undefined;
var rx: [cyw43.hci.rx_buf_len]u8 = undefined;

/// Inject a packet of `len` bytes, then time **only** the read. Returns total
/// microseconds over `count` reads.
fn timeReads(host: *cyw43.BtHost(Dev), len: usize, count: u32) !u64 {
    var total: u64 = 0;
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        try host.bt.injectInbound(.acl, payload[0..len]);

        const t0 = timer.now64();
        const n = host.bt.read(&rx) catch |err| return err;
        const t1 = timer.now64();
        total += t1 - t0;

        // **Checked every iteration, not once.** A read path that is fast
        // because it is dropping bytes is the failure this rig would otherwise
        // report as a pass.
        if (n != 4 + len) {
            watch.mismatches += 1;
            return error.ReadbackMismatch;
        }
        if (rx[4] != payload[0] or rx[4 + len - 1] != payload[len - 1]) {
            watch.mismatches += 1;
            return error.ReadbackMismatch;
        }
    }
    return total;
}

/// The clock this run is taken at.
///
/// **`board.clock_plan` is `null` for a Pico 2 W** — clk_sys runs straight off
/// the 12 MHz crystal, because picozig's board file says a PLL "gains nothing
/// but current draw" for boards that blink LEDs and sleep. That is right for
/// those boards and wrong for this one: rung 21c's 88,840 B/s and its 46% duty
/// cycle were both measured at 12 MHz, and an SBC decoder has to fit in what is
/// left of a **12 MHz** core.
///
/// `-Dsys-mhz=150` overrides it here and **only** here. The board's decision is
/// not overturned by a measurement rig.
const plan: ?clocks.Plan = if (config.sys_mhz == 0)
    board.clock_plan
else
    clocks.Plan.require(config.sys_mhz * 1_000_000);

/// What every divisor must follow. Getting this wrong garbles the UART at
/// exactly the moment the run has something to say.
const sys_hz: u32 = if (plan) |p| p.sys_hz else board.sys_hz;

fn strEq(a: []const u8, b: []const u8) bool {
    return std.mem.eql(u8, a, b);
}

/// `-Dvariant`, or null for "let the rate decide".
const variant_override: ?cyw43.gspi.Variant = if (strEq(config.variant, "auto"))
    null
else if (strEq(config.variant, "gap0_sample1"))
    .gap0_sample1
else if (strEq(config.variant, "gap01_sample0"))
    .gap01_sample0
else if (strEq(config.variant, "gap010_sample1"))
    .gap010_sample1
else if (strEq(config.variant, "gap0101_sample0"))
    .gap0101_sample0
else
    @compileError("-Dvariant must be auto or one of the four gSPI programs");

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
    watch.required = config.required;

    uart.print(.uart0, "\n=== BT receive-path throughput (rung 21c) ===\nboard {s}\n", .{board.name});

    // **Measured, not reported.** A DIV register holds whatever was written to
    // it; FC0 counts the clock against the crystal and is the one number that
    // can contradict the plan.
    watch.clk_sys_khz = clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_sys) orelse 0;
    uart.print(.uart0, "clk_sys {d} kHz (planned {d})\n", .{ watch.clk_sys_khz, sys_hz / 1000 });

    if (cyw43.fw_wifi.len != 231077) {
        uart.print(.uart0, "\nThis needs -Dfw=combined.\n", .{});
        die(error.BadPatchram, 21);
    }

    for (&payload, 0..) |*b, i| b.* = @truncate(i *% 31 +% 7);

    const pins = cyw43.default_pins;
    var bus = cyw43.PicogSpiBus.init(pins);

    // **The bus rate, chosen independently of clk_sys.** `-Dbus-div=0` keeps
    // `setSysHz`'s behaviour: hold the bus at its proven 3 MHz whatever clk_sys
    // is, which is what rungs 21c through 27 were measured on. Any other value
    // is a raw PIO divisor, so the rate ladder is `-Dbus-div=25,4,3,2` at
    // 150 MHz — 3, 18.75, 25 and 37.5 MHz.
    if (config.bus_div == 0) bus.setSysHz(sys_hz) else bus.setBusDiv(config.bus_div);

    // **The program follows the rate**, per rung A's measured crossover, unless
    // `-Dvariant` overrides it. Changing the divisor without changing this is
    // exactly what rung 21d did, and it fails at bring-up rather than in any
    // way that points at timing.
    const bus_hz = bus.busHz(sys_hz);
    bus.setVariant(if (variant_override) |v| v else cyw43.gspi.Variant.forBusHz(bus_hz));

    watch.bus_khz = bus_hz / 1000;
    watch.variant_idx = @backingInt(bus.variant);
    uart.print(.uart0, "gSPI divisor {d} -> {d} kHz, program {s}{s}\n", .{
        bus.clock_div,
        bus_hz / 1000,
        @tagName(bus.variant),
        if (variant_override != null) " (forced)" else "",
    });

    var dev = Dev.init(bus, pins);

    // The same bring-up bredr-probe uses, unchanged, so this number describes
    // the configuration rung 21 was proven in rather than a trimmed one.
    uart.print(.uart0, "1. Bus and firmware...\n", .{});
    dev.powerUp() catch |err| die(err, 0);

    var tr: u32 = 0;
    var tries: u32 = 10;
    while (tries > 0) : (tries -= 1) {
        tr = dev.readTestRegister() catch 0;
        if (tr == cyw43.gspi.test_pattern) break;
        timer.delayMs(1);
    }
    if (tr != cyw43.gspi.test_pattern) die(error.ChipNotResponding, 1);

    dev.busInit() catch |err| die(err, 2);
    dev.requestAlp() catch |err| die(err, 3);
    dev.disableCore(.wlan_arm) catch {};
    dev.disableCore(.socram) catch {};
    dev.resetCore(.socram, false) catch |err| die(err, 4);
    dev.disableSram3Remap() catch |err| die(err, 4);

    const bp = dev.backplaneSelfTest() catch |err| die(err, 5);
    if (bp.block_wrong != 0) die(error.BackplaneUnreliable, 5);

    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| die(err, 6);
    dev.waitF2Ready() catch |err| die(err, 7);
    dev.loadClmDefault() catch |err| die(err, 11);
    dev.wifiOn(.gb) catch |err| die(err, 12);

    uart.print(.uart0, "2. Starting the Bluetooth controller...\n", .{});
    var host = cyw43.BtHost(Dev).init(&dev);
    host.start(cyw43.fw_btfw, 2000) catch |err| die(err, 17);

    const bsp = host.commandWithResult(cyw43.bredr.op.read_buffer_size, &.{}) catch |err|
        die(err, 21);
    const bs = cyw43.bredr.parseBufferSize(bsp) catch |err| die(err, 21);
    watch.acl_len = bs.acl_len;
    watch.acl_count = bs.acl_count;

    if (!host.useClassicBuffers(bs.acl_len, bs.acl_count)) {
        uart.print(.uart0, "   controller ACL {d} > rx buffer {d}\n", .{ bs.acl_len, acl_payload });
        die(error.PacketTooLarge, 21);
    }
    uart.print(.uart0, "   BR/EDR ACL {d} bytes x {d}\n", .{ bs.acl_len, bs.acl_count });

    // -- the idle cost -----------------------------------------------------
    //
    // What a poll of an empty ring costs, which is what an application waiting
    // for audio pays between packets.
    uart.print(.uart0, "\n3. Polling an empty ring...\n", .{});
    const t0 = timer.now64();
    var i: u32 = 0;
    while (i < poll_iterations) : (i += 1) {
        const n = host.bt.read(&rx) catch |err| die(err, 21);
        if (n != 0) die(error.ReadbackMismatch, 21);
    }
    const t1 = timer.now64();
    watch.poll_ns = @intCast((t1 - t0) * 1000 / poll_iterations);
    uart.print(.uart0, "   {d} ns per empty poll\n", .{watch.poll_ns});

    // -- the real thing ----------------------------------------------------
    uart.print(.uart0, "\n4. Reading {d} packets of {d} bytes...\n", .{ iterations, acl_payload });
    const large_us = timeReads(&host, acl_payload, iterations) catch |err| die(err, 21);
    watch.large_ns = @intCast(large_us * 1000 / iterations);

    uart.print(.uart0, "5. Reading {d} packets of {d} bytes...\n", .{ iterations, small_payload });
    const small_us = timeReads(&host, small_payload, iterations) catch |err| die(err, 21);
    watch.small_ns = @intCast(small_us * 1000 / iterations);

    watch.too_large = host.bt.packets_too_large;

    // -- arithmetic --------------------------------------------------------
    //
    // Two sizes, one straight line: the difference is what the extra bytes
    // cost, and the small packet is almost all fixed overhead.
    const span: u64 = acl_payload - small_payload;
    const delta_ns: u64 = watch.large_ns - watch.small_ns;
    watch.per_kb_ns = @intCast(delta_ns * 1024 / span);
    watch.fixed_ns = @intCast(@as(u64, watch.small_ns) -| (delta_ns * small_payload / span));

    // Payload bytes a second, from the full-size figure — the one that matters,
    // because a media stream arrives in full-size packets.
    watch.bytes_per_sec = @intCast(@as(u64, acl_payload) * 1_000_000_000 / watch.large_ns);
    watch.packets_per_sec = @intCast(@as(u64, 1_000_000_000) / watch.large_ns);
    watch.margin_pct = @intCast(@as(u64, watch.bytes_per_sec) * 100 / watch.required);
    watch.duty_pct = @intCast(@as(u64, watch.required) * 100 / watch.bytes_per_sec);
    watch.verdict = @intFromBool(watch.bytes_per_sec >= watch.required);
    watch.rung = 0x21C;

    uart.print(.uart0,
        \\
        \\   per packet   {d} ns  ({d} bytes)
        \\   per 6 bytes  {d} ns
        \\   fixed cost   {d} ns per packet
        \\   marginal     {d} ns per KB
        \\
        \\   THROUGHPUT   {d} bytes/sec  ({d} packets/sec)
        \\   required     {d} bytes/sec
        \\   margin       {d}% of requirement
        \\   DUTY CYCLE   {d}% of the core, to sustain the requirement
        \\   refused {d}, mismatched {d}
        \\
    , .{
        watch.large_ns,        acl_payload,
        watch.small_ns,        watch.fixed_ns,
        watch.per_kb_ns,       watch.bytes_per_sec,
        watch.packets_per_sec, watch.required,
        watch.margin_pct,      watch.duty_pct,
        watch.too_large,       watch.mismatches,
    });

    if (watch.verdict != 0) {
        uart.print(.uart0,
            \\   PASS -- the transport can carry SBC, but read the duty cycle
            \\   before calling it comfortable: that share of the core is spent
            \\   in the receive path alone, and an SBC decoder has not been
            \\   written yet, nor an I2S engine (lib/pcm5100 has no PIO or DMA
            \\   in it). Controller pacing is also not in this number.
            \\
        , .{});
    } else {
        uart.print(.uart0,
            \\   FAIL -- this transport cannot carry the stream. Nothing above
            \\   it is worth building until this number changes.
            \\
        , .{});
    }

    var secs: u32 = 0;
    while (true) {
        timer.delayMs(1000);
        secs += 1;
        watch.ticks = secs;
    }
}

const std = @import("std");
