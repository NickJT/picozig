//! wake-check — does the chip's own interrupt do the servicing?
//!
//! Rung N2 of the ipnet bring-up ladder, and the falsification for it.
//!
//! ## The obligation this exists to remove
//!
//! Until now `service()` has been a thing an application must remember to
//! call, often enough, forever. Forget it and the board stops answering ARP,
//! stops renewing its lease, and never notices the link has gone — silently,
//! while still looking alive. That failure has already happened here once: an
//! ARP responder that only ran inside `awaitUdp` passed a test that called
//! `awaitUdp`, and on hardware the board took its lease and went deaf.
//!
//! The CYW43 has a wire for this. `WL_D` carries the chip's interrupt as well
//! as the data, weighted so the interrupt is only heard when the bus is idle,
//! and the Pico 2 W and Pico Plus 2 W both connect it to GP24.
//!
//! ## The experiment
//!
//! Two loops with **identical structure**, chosen by `-Dwake`, so the only
//! difference between them is whether `poll()` is guarded:
//!
//! ```zig
//! // -Dwake=false        // -Dwake=true
//! poll();                if (takeHostWake()) { poll(); rearm(); }
//! delayMs(1);            delayMs(1);
//! ```
//!
//! Same cadence, same counters, one conditional. `polls` then measures exactly
//! what the interrupt saves, and it is a real number rather than an argument:
//! every poll is a gSPI transaction that mostly finds nothing.
//!
//! ## What makes this falsifiable rather than a demo
//!
//! **`wakes` is the field that matters.** A handler that never fires and a
//! network with nothing to say produce identical output — a quiet board, no
//! errors — so without a count of interrupts actually taken, "it worked" would
//! mean nothing at all. A `-Dwake=true` run that reports `wakes = 0` has
//! proven the opposite of what it looks like.
//!
//! Then, from another host on the same network — **the address the board
//! printed**, not a literal from a previous run. It comes from DHCP and changes
//! with the network; the bench moved from the lab Wi-Fi to an isolated segment
//! on 2026-09-03 and every hard-coded address in a comment went stale at once:
//!
//! ```
//! arping <the address in the trace>        # must be answered
//! ```
//!
//! with nothing in `main` calling `service`. `arp_answered` climbing while
//! `main` only sleeps is the whole claim of this rung.
//!
//! **Run it both ways.** `-Dwake=false` is the control: same board, same
//! network, same minute, and the difference in `polls` is the measurement.
//!
//! **"Same board" stopped being automatic when the bench got a second probe.**
//! Give both runs the same `-Dprobe-serial`, or OpenOCD may flash whichever it
//! enumerates first — reporting success either way — and the control becomes a
//! measurement of two different boards.

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const ipnet = @import("ipnet");
const net_config = @import("net_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const nvic = rp2350.nvic;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// How long to measure for, from `-Dsecs`.
///
/// **The window has to outlast the stimulus.** `arping -c 90` takes noticeably
/// longer than 90 seconds once its own retries are counted, and a window that
/// closes first freezes the counters mid-experiment — the board then reports
/// fewer replies than `arping` says it received, and the two sides cannot be
/// compared. Servicing continues either way; it is only the counting that
/// stops. Default 90, long enough for a `-Drenew=30` renewal to land inside it.
const run_secs: u32 = net_config.run_secs;

/// **Poll at least this often even when the chip has said nothing.**
///
/// An interrupt fires because the chip has something. It never fires because a
/// lease is due — and `maybeRenew` lives inside `poll`, so a purely
/// interrupt-driven loop on a silent network would never check the clock and
/// would let its address expire. That is not hypothetical arithmetic: these
/// runs already show wake mode taking one fewer renewal than the polled control
/// over the same window, because the renewal only gets looked at when a frame
/// happens to arrive.
///
/// One a second against the ~2.4 the interrupt produces is a rounding error on
/// the 413x saving, and it makes the lease safe on a network that goes quiet.
const poll_floor_ms: u32 = 1000;

/// **File scope because the handler needs it**, not because it is convenient.
/// `boot.zig` builds the vector table from the *root* module's declarations, so
/// the handler has to live here, and a handler cannot reach a local in `main`.
var dev: Dev = undefined;
var wifi: ipnet.Session(Dev) = undefined;

/// Interrupts taken. **Written by the ISR, read by everything else**, so it is
/// touched atomically — not because the RP2350 will tear a 32-bit word, but
/// because the compiler is entitled to keep it in a register across the loop
/// that reads it, and then the count never moves.
var wakes: u32 = 0;

/// This is not really your code — it is the driver's handler, given a home.
///
/// The name is the wiring: `boot.zig` matches it against `nvic.IrqNum` at
/// comptime, so `io_bank0` misspelled is a compile error rather than a vector
/// that silently goes nowhere. Same shape as `lib/usb`'s `usbctrl`.
///
/// **It masks and records. It must not touch the bus.** The application may be
/// mid-transfer on the very bus this interrupt arrived over; `onHostWakeIrq`
/// disables the interrupt and sets a flag, and the servicing happens back in
/// the main loop where the bus has one owner.
pub const interrupts = struct {
    pub fn io_bank0() callconv(.c) void {
        dev.onHostWakeIrq();
        _ = @atomicRmw(u32, &wakes, .Add, 1, .monotonic);
    }
};

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// 1 when built `-Dwake=true`. Read this first: the two modes are the same
    /// binary shape and the counters below mean nothing without it.
    wake_mode: u32,
    /// **Interrupts actually taken.** Zero in wake mode means the handler never
    /// ran, which looks exactly like a quiet network and is not one.
    wakes: u32,
    /// Calls to `poll()`. This is the number the rung is about.
    polls: u32,
    /// Seconds the loop ran, so `polls` can be turned into a rate.
    secs: u32,
    /// Of those polls, how many came from the timer floor rather than from an
    /// interrupt. **On a silent network this is all of them**, and it is what
    /// keeps the lease alive when nothing is arriving to wake us.
    timed_polls: u32,
    ip: u32,
    /// ARP requests answered — **the falsification.** `arping` from another
    /// host must move this while `main` is only sleeping.
    arp_answered: u32,
    frames_ignored: u32,
    eth_tx: u32,
    eth_rx: u32,
    eth_dropped: u32,
    /// **Read this before believing `eth_dropped = 0`.** The deepest the
    /// driver's receive ring ever got: a mark that never leaves 1 says the
    /// depth was never exercised and the zero means nothing, and one that
    /// reaches `rx_ring_depth` says the next burst overflows. The two read
    /// identically without it.
    rx_high_water: u32,
    /// Polls that stopped at the drain bound with frames still waiting — the
    /// shape of a stack falling behind.
    drain_limited: u32,
    /// How many `connect` attempts it took, and how many of those failed.
    /// **Non-zero failures with the run otherwise healthy is the `NoDhcpOffer`
    /// intermittency**, and the pair says whether it recovered on its own.
    connect_tries: u32,
    connect_failures: u32,
    /// Pings seen and answered — rung N4. `ping` from another host is now a
    /// live instrument rather than a screen of timeouts.
    echo_seen: u32,
    echo_answered: u32,
    renewals: u32,
    renew_failures: u32,
    link_drops: u32,
    reserved: [8]u32,
};

pub const watch_magic: u32 = 0x57414B45; // 'WAKE'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "FAILED at rung {d}: {s}\n", .{ rung, @errorName(err) });
    watch.magic = 0xDEAD_5741;
    watch.rung = rung;
    while (true) timer.delayMs(1000);
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;
    watch.wake_mode = if (net_config.wake) 1 else 0;

    uart.print(.uart0, "\n=== CYW43439 host-wake check ===\nboard {s}\nmode  {s}\n", .{
        board.name,
        if (net_config.wake) "interrupt-driven (-Dwake=true)" else "polled (-Dwake=false)",
    });

    const pkg = rp2350.silicon.package();
    if (pkg != board.expected_package) {
        uart.print(.uart0, "WARNING:   package mismatch — image built for {s}, running on {s}\n", .{
            board.expected_package.name(), pkg.name(),
        });
    }

    if (net_config.ssid.len == 0) {
        uart.print(.uart0, "No Wi-Fi credentials — see lib/cyw43/wifi_credentials.json\n", .{});
        die(error.BadSsid, 0);
    }

    const pins = cyw43.default_pins;
    dev = Dev.init(cyw43.PicogSpiBus.init(pins), pins);

    // The ladder, in the order `cyw43-blink` proved it.
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
    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| die(err, 6);
    dev.waitF2Ready() catch |err| die(err, 7);
    dev.loadClmDefault() catch |err| die(err, 11);
    dev.wifiOn(.gb) catch |err| die(err, 12);
    watch.rung = 12;

    uart.print(.uart0, "joining \"{s}\"...\n", .{net_config.ssid});
    wifi = ipnet.Session(Dev).init(&dev, net_config.ssid, net_config.password, @truncate(timer.now64()));
    wifi.ip.renew_after_secs = net_config.renew_secs;

    // **Retry, and say what happened each time.** `NoDhcpOffer` at this rung
    // has been seen twice on a network that then leased normally seconds later,
    // so parking on the first failure costs a flash cycle to learn nothing.
    // The counters printed here are the diagnosis: `dhcp attempts` is how many
    // DISCOVERs went out, and `seen` against `ignored` says whether anything
    // came back at all — a link that is up but deaf looks completely different
    // from one where the server is quiet.
    var connect_try: u32 = 0;
    while (true) : (connect_try += 1) {
        if (wifi.connect(.gb)) |_| break else |err| {
            uart.print(
                .uart0,
                "   attempt {d} failed: {s} (link {s}, dhcp attempts {d}, frames seen {d}, ignored {d}, arp seen {d})\n",
                .{
                    connect_try + 1,           @errorName(err),
                    @tagName(dev.link_status), wifi.ip.dhcp_attempts,
                    wifi.ip.frames_seen,       wifi.ip.frames_ignored,
                    wifi.ip.arp_requests_seen,
                },
            );
            watch.connect_failures = connect_try + 1;
            watch.err_code = wifi.ip.dhcp_attempts;
            if (connect_try + 1 >= 4) die(err, 15);
            timer.delayMs(2000);
        }
    }
    watch.rung = 15;
    watch.connect_tries = connect_try + 1;
    watch.ip = pack4(wifi.ip.lease.ip);
    uart.print(.uart0, "address {d}.{d}.{d}.{d}, lease {d}s\n", .{
        wifi.ip.lease.ip[0],      wifi.ip.lease.ip[1],
        wifi.ip.lease.ip[2],      wifi.ip.lease.ip[3],
        wifi.ip.lease.lease_secs,
    });

    // **Arming comes after the join**, deliberately. Everything above this line
    // is a dense stream of transfers, and while the interrupt is masked for the
    // duration of each one, there is nothing for it to report until there is a
    // network to report about.
    if (net_config.wake) {
        dev.enableHostWake();
        nvic.enable(nvic.IrqNum.io_bank0);
        uart.print(.uart0, "host wake armed on GP24, io_bank0 enabled\n", .{});
    }

    uart.print(.uart0, "running {d}s — arping {d}.{d}.{d}.{d} from another host now\n", .{
        run_secs,
        wifi.ip.lease.ip[0],
        wifi.ip.lease.ip[1],
        wifi.ip.lease.ip[2],
        wifi.ip.lease.ip[3],
    });
    watch.rung = 20;

    var polls: u32 = 0;
    var timed_polls: u32 = 0;
    var last_poll_ms: u32 = 0;
    var ms: u32 = 0;
    while (ms < run_secs * 1000) : (ms += 1) {
        if (net_config.wake) {
            // **`main` never calls `service`.** The chip says when it has
            // something; until then this loop is a sleep and the bus is idle.
            const woke = dev.takeHostWake();
            // The floor. See `poll_floor_ms`: the interrupt cannot tell us a
            // timer has expired, only that a frame has arrived.
            const overdue = ms -% last_poll_ms >= poll_floor_ms;
            if (woke or overdue) {
                // **`wifi.poll`, not `wifi.ip.poll`.** Going straight to the IP
                // layer skips `Session`'s link check, so a board whose AP
                // rebooted would poll a dead link forever. That was the first
                // version of this loop and no ninety-second run would have
                // shown it.
                _ = wifi.poll() catch {};
                polls += 1;
                last_poll_ms = ms;
                if (woke) dev.rearmHostWake() else timed_polls += 1;
            }
        } else {
            _ = wifi.poll() catch {};
            polls += 1;
        }

        timer.delayMs(1);

        if (ms % 1000 == 0) {
            watch.ticks +%= 1;
            watch.secs = ms / 1000;
            watch.polls = polls;
            watch.timed_polls = timed_polls;
            watch.wakes = @atomicLoad(u32, &wakes, .monotonic);
            watch.arp_answered = wifi.ip.arp_replies_sent;
            watch.frames_ignored = wifi.ip.frames_ignored;
            watch.eth_tx = dev.eth_tx_count;
            watch.eth_rx = dev.eth_rx_count;
            watch.eth_dropped = dev.census.data_dropped;
            watch.rx_high_water = dev.rx_high_water;
            watch.drain_limited = wifi.ip.drain_limited;
            watch.echo_seen = wifi.ip.echo_requests_seen;
            watch.echo_answered = wifi.ip.echo_replies_sent;
            watch.renewals = wifi.ip.renewals;
            watch.renew_failures = wifi.ip.renew_failures;
            watch.link_drops = wifi.link_drops;
        }
    }

    const w = @atomicLoad(u32, &wakes, .monotonic);
    uart.print(.uart0,
        \\
        \\--- after {d}s ---
        \\wakes          {d}
        \\polls          {d}   ({d} per second, {d} from the timer floor)
        \\arp answered   {d}
        \\ping answered  {d} of {d} seen
        \\frames ignored {d}
        \\eth rx/tx/drop {d}/{d}/{d}
        \\rx high water   {d} of {d}
        \\drain limited   {d}
        \\renewals       {d} ({d} failures)
        \\connect        {d} tries, {d} failed
        \\
    , .{
        run_secs,                 w,                         polls,                      polls / run_secs,       timed_polls,
        wifi.ip.arp_replies_sent, wifi.ip.echo_replies_sent, wifi.ip.echo_requests_seen, wifi.ip.frames_ignored, dev.eth_rx_count,
        dev.eth_tx_count,         dev.census.data_dropped,   dev.rx_high_water,          cyw43.rx_ring_depth,    wifi.ip.drain_limited,
        wifi.ip.renewals,         wifi.ip.renew_failures,    watch.connect_tries,        watch.connect_failures,
    });

    if (net_config.wake and w == 0) {
        uart.print(.uart0,
            \\
            \\NOT PROVEN: zero interrupts taken. A handler that never fires looks
            \\exactly like this, so the quiet run above is not evidence of anything.
            \\
        , .{});
    }

    // The same loop, forever, so what is held is what was measured. **Using
    // `service` here instead would quietly return the board to polling a
    // thousand times a second** the moment the report printed, which is the
    // opposite of what this example is for.
    uart.print(.uart0, "holding the address. Ctrl-C when done.\n", .{});
    var idle_ms: u32 = 0;
    last_poll_ms = 0;
    while (true) : (idle_ms +%= 1) {
        if (net_config.wake) {
            const woke = dev.takeHostWake();
            if (woke or idle_ms -% last_poll_ms >= poll_floor_ms) {
                _ = wifi.poll() catch {};
                last_poll_ms = idle_ms;
                if (woke) dev.rearmHostWake();
            }
            timer.delayMs(1);
        } else {
            _ = wifi.poll() catch {};
            timer.delayMs(1);
        }
    }
}

fn pack4(v: [4]u8) u32 {
    return @as(u32, v[0]) | @as(u32, v[1]) << 8 |
        @as(u32, v[2]) << 16 | @as(u32, v[3]) << 24;
}
