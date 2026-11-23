//! ntp-client — ask a remote clock what time it is, over Wi-Fi.
//!
//! Rung 16 of the cyw43 bring-up ladder, and the first thing this
//! driver does that involves a host outside the building.
//!
//! ## Why this example exists, and why it was written twice
//!
//! The previous file with this name was demonstrated on hardware printing a
//! correct, ticking UTC clock, and every part of that was false: it loaded four
//! bytes of invented microcode, never brought the bus up at all, and the time
//! on screen was the build timestamp plus uptime. It was deleted rather than
//! repaired, and nothing was carried forward.
//!
//! So this one is built to be falsifiable, and the test is in the build:
//!
//! ```
//! zig build swd-ntp-client                      # answers, and the time is right
//! zig build swd-ntp-client -Dntp=192.0.2.1      # nothing answers, and it says so
//! ```
//!
//! **With more than one probe attached, both runs need the same
//! `-Dprobe-serial`.** The pair is only evidence if it is one board twice;
//! OpenOCD claims whichever probe it enumerates first and reports success, so
//! without the serial the second run can land on the other board and the
//! comparison quietly means nothing.
//!
//! The same binary aimed at a reserved address that cannot reply must print no
//! time at all. Anything computing the answer locally passes both runs
//! identically, which is exactly what happened last time.
//!
//! ## What has to work before the time arrives
//!
//! Everything below it, which is why this file walks the whole ladder rather
//! than jumping to the interesting part: bus, firmware, CLM, `wifi_on`, join,
//! frames, a DHCP lease, and the gateway's hardware address. The watch block at
//! 0x20000000 says how far it got.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const ipnet = @import("ipnet");
const net_config = @import("net_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// The port this client listens on. Fixed rather than ephemeral so the filter
/// in `ipstack` has something to match, and high enough not to collide with
/// anything the network expects to serve.
const local_port: u16 = 50123;

/// How long to wait for a server on the other side of the internet. Generous:
/// the failure being tested for is "no answer ever", not "a slow one".
const ntp_timeout_ms: u32 = 3000;

/// Parsed from `-Dntp` at build time. A literal rather than a lookup because
/// there is no DNS resolver in this driver.
const ntp_server: [4]u8 = parseIpv4(net_config.ntp_server);

fn parseIpv4(comptime s: []const u8) [4]u8 {
    var out: [4]u8 = undefined;
    var field: usize = 0;
    var acc: u16 = 0;
    var digits: usize = 0;
    for (s) |c| {
        if (c == '.') {
            if (digits == 0 or field >= 3) @compileError("malformed -Dntp address: " ++ s);
            out[field] = @intCast(acc);
            field += 1;
            acc = 0;
            digits = 0;
        } else if (c >= '0' and c <= '9') {
            acc = acc * 10 + (c - '0');
            if (acc > 255) @compileError("octet out of range in -Dntp address: " ++ s);
            digits += 1;
        } else {
            @compileError("malformed -Dntp address: " ++ s);
        }
    }
    if (field != 3 or digits == 0) @compileError("malformed -Dntp address: " ++ s);
    out[3] = @intCast(acc);
    return out;
}

/// Published over SWD, so a board that stopped partway says where without a
/// console attached. Same discipline as `cyw43-blink`: `rung` is the last one
/// reached, `err_code` is a stable number rather than `@intFromError`.
///
/// The region is 0x80 bytes — 32 words — because `.faultinfo` is pinned at
/// 0x20000080 for every image in this workspace. Adding a field means removing
/// one.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// The station MAC, low four bytes then high two.
    mac_lo: u32,
    mac_hi: u32,
    /// `LinkStatus` in the low half, `JoinState` in the high half. `0x0E010002`
    /// is a completed join.
    link: u32,
    /// The lease: address, mask, gateway, and how long it is good for.
    ip: u32,
    mask: u32,
    gateway: u32,
    lease_secs: u32,
    /// The gateway's hardware address, low four bytes then high two.
    gw_mac_lo: u32,
    gw_mac_hi: u32,
    /// The server asked, and the port asked from.
    server: u32,
    /// Queries sent, and answers that survived the filter and parsed.
    queries: u32,
    answers: u32,
    /// **The time itself, as Unix seconds** — 0 until one actually arrives.
    /// This is the field the whole example exists to fill, and a run against a
    /// dead server must leave it 0.
    unix_time: u32,
    /// Seconds of uptime when that timestamp was received, so a stale value is
    /// obvious rather than merely old.
    unix_at_tick: u32,
    /// Round-trip of the last query, in milliseconds. A plausible number here
    /// is one more thing a fabricated answer would have to invent.
    rtt_ms: u32,
    /// What the frame filter saw: ARP requests answered, and frames that were
    /// somebody else's.
    arp_answered: u32,
    frames_ignored: u32,
    /// Frames in and out at the driver, and frames the receive latch dropped.
    eth_tx: u32,
    eth_rx: u32,
    eth_dropped: u32,
    reserved: [8]u32,
};

pub const watch_magic: u32 = 0x4E54_5031; // 'NTP1'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    // `.faultinfo` is pinned at 0x20000080 for every image in this workspace,
    // so this block cannot cross 0x80. Without this check the overrun is
    // silent: the linker places both, and the fault record is quietly
    // overwritten by whatever field ran past the end.
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

fn pack4(v: [4]u8) u32 {
    return @as(u32, v[0]) | @as(u32, v[1]) << 8 |
        @as(u32, v[2]) << 16 | @as(u32, v[3]) << 24;
}

/// Report which rung failed and park. Continuing past a failed rung produces
/// pages of plausible output about a network that is not there.
fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "FAILED at rung {d}: {s}\n", .{ rung, @errorName(err) });
    watch.magic = 0xDEAD_4E54;
    watch.rung = rung;
    // Stable codes, matching `cyw43-blink`'s so the two can be read the same
    // way. `@intFromError` shifts when unrelated code changes.
    watch.err_code = switch (err) {
        error.NotPowered => 1,
        error.ChipNotResponding => 2,
        error.BusControlMismatch => 3,
        error.AlpTimeout => 4,
        error.CoreNotInReset => 5,
        error.HtClockTimeout => 6,
        error.F2NotReady => 7,
        error.CreditTimeout => 8,
        error.BadTransferLength => 9,
        error.PayloadTooLarge => 10,
        error.FirmwareNotLoaded => 11,
        error.BackplaneUnreliable => 13,
        error.ClmLoadFailed => 14,
        error.IoctlRejected => 15,
        error.IoctlTimeout => 16,
        error.CountryRejected => 17,
        error.EventMaskRejected => 18,
        error.WifiUpRejected => 19,
        error.WifiNotOn => 20,
        error.JoinRejected => 21,
        error.JoinTimeout => 22,
        error.JoinFailed => 23,
        error.BadSsid => 24,
        error.BadPassphrase => 25,
        error.LinkDown => 26,
        error.BufferTooSmall => 27,
        error.NoDhcpOffer => 30,
        error.DhcpNak => 31,
        error.IncompleteLease => 32,
        error.GatewayUnreachable => 33,
        error.NoAddress => 34,
        else => 99,
    };
    while (true) timer.delayMs(1000);
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .rung = 0,
        .err_code = 0,
        .mac_lo = 0,
        .mac_hi = 0,
        .link = 0,
        .ip = 0,
        .mask = 0,
        .gateway = 0,
        .lease_secs = 0,
        .gw_mac_lo = 0,
        .gw_mac_hi = 0,
        .server = pack4(ntp_server),
        .queries = 0,
        .answers = 0,
        .unix_time = 0,
        .unix_at_tick = 0,
        .rtt_ms = 0,
        .arp_answered = 0,
        .frames_ignored = 0,
        .eth_tx = 0,
        .eth_rx = 0,
        .eth_dropped = 0,
        .reserved = @splat(0),
    };

    uart.print(.uart0, "\n=== CYW43439 NTP client ===\nboard {s}\n", .{board.name});

    // **Say so when the image and the silicon disagree.** For most of
    // 2026-08-31 this example ran on a Pico Plus 2 W (RP2350B) while printing
    // "Raspberry Pi Pico 2 W (RP2350A)" from a board file chosen by default,
    // and nothing anywhere said otherwise — the wireless pins agree across
    // both boards, so it worked perfectly and mislabelled every trace it
    // produced. `blink-hello` has carried this check since long before; the
    // example that actually gets run overnight did not.
    const pkg = rp2350.silicon.package();
    if (pkg != board.expected_package) {
        uart.print(.uart0, "WARNING:   package mismatch — image built for {s}, running on {s}\n", .{
            board.expected_package.name(),
            pkg.name(),
        });
    }
    uart.print(.uart0, "server {d}.{d}.{d}.{d}:123, asking from port {d}\n", .{
        ntp_server[0], ntp_server[1], ntp_server[2], ntp_server[3], local_port,
    });

    if (net_config.ssid.len == 0) {
        uart.print(.uart0,
            \\
            \\No Wi-Fi credentials. Either:
            \\  zig build swd-ntp-client -Dssid="<SSID>" -Dpassword="<PASS>"
            \\or put {{"ssid": "...", "password": "..."}} in lib/cyw43/wifi_credentials.json
            \\
        , .{});
        die(error.BadSsid, 0);
    }

    const pins = cyw43.default_pins;
    const bus = cyw43.PicogSpiBus.init(pins);
    var dev = Dev.init(bus, pins);

    // -- rungs 1-7: the bus, the cores, the firmware --------------------
    //
    // Walked in full, in the order `cyw43-blink` proved them. The temptation
    // here is to wrap this in a helper shared between the examples; it is
    // resisted because the ladder is the thing being demonstrated, and a helper
    // that silently skipped a step would be invisible in exactly the way the
    // last version of this file was.
    uart.print(.uart0, "1. Powering up...\n", .{});
    dev.powerUp() catch |err| die(err, 0);

    // The first read after WL_REG_ON rises always fails — the chip is waking,
    // not absent. Measured as 0b11111110 across eight reads.
    var tr: u32 = 0;
    var tries: u32 = 10;
    while (tries > 0) : (tries -= 1) {
        tr = dev.readTestRegister() catch 0;
        if (tr == cyw43.gspi.test_pattern) break;
        timer.delayMs(1);
    }
    if (tr != cyw43.gspi.test_pattern) die(error.ChipNotResponding, 1);
    watch.rung = 1;

    dev.busInit() catch |err| die(err, 2);
    watch.rung = 2;

    dev.requestAlp() catch |err| die(err, 3);
    watch.rung = 3;

    dev.disableCore(.wlan_arm) catch {};
    dev.disableCore(.socram) catch {};
    dev.resetCore(.socram, false) catch |err| die(err, 4);
    dev.disableSram3Remap() catch |err| die(err, 4);
    watch.rung = 4;

    const bp = dev.backplaneSelfTest() catch |err| die(err, 5);
    if (bp.block_wrong != 0) die(error.BackplaneUnreliable, 5);
    watch.rung = 5;

    uart.print(.uart0, "2. Downloading firmware ({d} + {d} bytes)...\n", .{
        cyw43.fw_wifi.len, cyw43.fw_nvram.len,
    });
    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| die(err, 6);
    watch.rung = 6;

    dev.waitF2Ready() catch |err| die(err, 7);
    watch.rung = 7;

    // -- rungs 8-11: the control path and the CLM blob ------------------
    //
    // One LED write, checked. It is the cheapest proof that an IOCTL completes
    // and its CDC status comes back zero, and doing it here means a CLM failure
    // below is known to be the CLM rather than the control path in general.
    // Turned off again immediately: the light is the instrument, not the
    // result, and a board left glowing says nothing about whether it worked.
    dev.setLed(true) catch |err| die(err, 9);
    dev.setLed(false) catch |err| die(err, 9);
    watch.rung = 9;

    uart.print(.uart0, "3. Loading the CLM blob...\n", .{});
    dev.loadClmDefault() catch |err| die(err, 11);
    watch.rung = 11;

    // -- rung 12: wifi_on ------------------------------------------------
    uart.print(.uart0, "4. Bringing Wi-Fi up...\n", .{});
    dev.wifiOn(.gb) catch |err| die(err, 12);
    watch.mac_lo = @as(u32, dev.mac[0]) | @as(u32, dev.mac[1]) << 8 |
        @as(u32, dev.mac[2]) << 16 | @as(u32, dev.mac[3]) << 24;
    watch.mac_hi = @as(u32, dev.mac[4]) | @as(u32, dev.mac[5]) << 8;
    uart.print(.uart0, "   MAC {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
        dev.mac[0], dev.mac[1], dev.mac[2], dev.mac[3], dev.mac[4], dev.mac[5],
    });
    watch.rung = 12;

    // -- rungs 13, 15 and 18: the network, kept up by the library --------
    //
    // **One call, and the retry policy is not this file's.** `Session.connect`
    // joins, waits for the three events that say the link is really up, takes a
    // lease and resolves the gateway — and it retries the join, because the key
    // exchange is intermittent. That logic used to live here, inline, and
    // moving it into the library is rung 18's real point: the next consumer
    // calls `connect` and `service` rather than copying thirty lines of
    // recovery and getting one of them wrong.
    uart.print(.uart0, "5. Joining \"{s}\" and taking an address...\n", .{net_config.ssid});

    var wifi = ipnet.Session(Dev).init(&dev, net_config.ssid, net_config.password, @truncate(timer.now64()));
    // Rung 18's own knob: renew after this many seconds rather than at T1,
    // which on this network is twelve hours. Zero is the real behaviour.
    wifi.ip.renew_after_secs = net_config.renew_secs;

    wifi.connect(.gb) catch |err| {
        uart.print(.uart0, "   failed: {s} (link {s}, dhcp attempts {d})\n", .{
            @errorName(err), @tagName(dev.link_status), wifi.ip.dhcp_attempts,
        });
        die(err, if (dev.link_status == .joined) 15 else 13);
    };

    const link = dev.link_status;
    watch.link = @backingInt(link) | @as(u32, @as(u16, @bitCast(dev.join_state))) << 16;
    watch.rung = 15;

    const ip = &wifi.ip;

    const l = ip.lease;
    watch.ip = pack4(l.ip);
    watch.mask = pack4(l.mask);
    watch.gateway = pack4(l.gateway);
    watch.lease_secs = l.lease_secs;
    watch.gw_mac_lo = @as(u32, ip.gateway_mac[0]) | @as(u32, ip.gateway_mac[1]) << 8 |
        @as(u32, ip.gateway_mac[2]) << 16 | @as(u32, ip.gateway_mac[3]) << 24;
    watch.gw_mac_hi = @as(u32, ip.gateway_mac[4]) | @as(u32, ip.gateway_mac[5]) << 8;
    uart.print(.uart0, "   {d}.{d}.{d}.{d}/{d}.{d}.{d}.{d} via {d}.{d}.{d}.{d}, lease {d}s\n", .{
        l.ip[0],      l.ip[1],      l.ip[2],      l.ip[3],
        l.mask[0],    l.mask[1],    l.mask[2],    l.mask[3],
        l.gateway[0], l.gateway[1], l.gateway[2], l.gateway[3],
        l.lease_secs,
    });
    watch.rung = 15;

    // -- rung 16: the time ------------------------------------------------
    //
    // Six queries, thirty seconds apart, so a single lucky packet is not the
    // whole result and the ARP responder gets exercised in between.
    uart.print(.uart0, "6. Asking {d}.{d}.{d}.{d} what time it is...\n", .{
        ntp_server[0], ntp_server[1], ntp_server[2], ntp_server[3],
    });

    var query: u32 = 0;
    while (query < 6) : (query += 1) {
        const request = ipnet.ntp.createRequest();
        const sent_at = timer.now64();

        watch.queries += 1;
        wifi.sendUdp(ntp_server, ipnet.ntp.ntp_port, local_port, &request) catch |err| {
            uart.print(.uart0, "   [{d}] send failed: {s}\n", .{ query + 1, @errorName(err) });
            publishCounters(ip, &dev);
            waitBetween(&wifi);
            continue;
        };

        var reply: [ipnet.ntp.ntp_msg_len]u8 = undefined;
        const got = wifi.awaitUdp(local_port, &reply, ntp_timeout_ms) catch |err| {
            uart.print(.uart0, "   [{d}] receive failed: {s}\n", .{ query + 1, @errorName(err) });
            publishCounters(ip, &dev);
            waitBetween(&wifi);
            continue;
        };

        const rtt: u32 = @truncate((timer.now64() - sent_at) / 1000);

        if (got) |r| {
            // The datagram arrived on our port and for our address. Whether it
            // is an *NTP* datagram is a separate question, and one the server
            // could get wrong: mode 4, non-zero stratum, a timestamp past the
            // epoch.
            if (ipnet.ntp.parseResponse(reply[0..r.len])) |unix_ts| {
                watch.answers += 1;
                watch.unix_time = @truncate(unix_ts);
                watch.unix_at_tick = @truncate(timer.now64() / 1_000_000);
                watch.rtt_ms = rtt;
                watch.rung = 16;

                const days: u32 = @truncate(unix_ts / 86400);
                const sod: u32 = @truncate(unix_ts % 86400);
                uart.print(.uart0, "   [{d}] {d:0>2}:{d:0>2}:{d:0>2} UTC" ++
                    "  (unix {d}, day {d}, from {d}.{d}.{d}.{d}, {d} ms)\n", .{
                    query + 1,   sod / 3600,  (sod % 3600) / 60, sod % 60,
                    unix_ts,     days,        r.src_ip[0],       r.src_ip[1],
                    r.src_ip[2], r.src_ip[3], rtt,
                });
            } else |err| {
                uart.print(.uart0, "   [{d}] {d} bytes that are not an NTP answer: {s}\n", .{
                    query + 1, r.len, @errorName(err),
                });
            }
        } else {
            // **The falsifiable outcome.** No time is printed, nothing is
            // guessed, and `watch.unix_time` stays 0.
            uart.print(.uart0, "   [{d}] no answer in {d} ms (ignored {d} frames meanwhile)\n", .{
                query + 1, ntp_timeout_ms, ip.frames_ignored,
            });
        }

        publishCounters(ip, &dev);

        // **Rung 18's other half, made to happen.** The failure being tested is
        // the AP going away, and waiting for a real one is neither repeatable
        // nor ours to arrange. A deliberate disassociation is the same event as
        // far as the driver is concerned: the chip issues a real disassoc, the
        // link genuinely drops, and `service` has to notice and recover — with
        // the address kept, because the lease is still ours.
        if (net_config.drop_after != 0 and query + 1 == net_config.drop_after) {
            uart.print(.uart0, "[net] dropping the link deliberately...\n", .{});
            dev.disconnectWifi() catch |err| {
                uart.print(.uart0, "[net] disconnect failed: {s}\n", .{@errorName(err)});
            };
            uart.print(.uart0, "[net] link is now {s}, address {d}.{d}.{d}.{d}\n", .{
                @tagName(dev.link_status), ip.lease.ip[0], ip.lease.ip[1],
                ip.lease.ip[2],            ip.lease.ip[3],
            });
        }

        waitBetween(&wifi);
    }

    uart.print(.uart0, "\n{d} queries, {d} answers. ", .{ watch.queries, watch.answers });
    if (watch.answers == 0) {
        uart.print(.uart0, "No time was obtained, and none is being invented.\n", .{});
    } else {
        uart.print(.uart0, "Last time: unix {d}.\n", .{watch.unix_time});
    }

    // Stay up and keep answering ARP: the lease is held for as long as the
    // board defends it, and a silent board loses the peer's cache entry.
    uart.print(.uart0, "Holding the address. Ctrl-C when done.\n", .{});
    while (true) {
        _ = wifi.service(1000) catch {};
        watch.ticks += 1;
        publishCounters(ip, &dev);
    }
}

fn publishCounters(ip: *ipnet.IpStack(Dev), dev: *Dev) void {
    watch.arp_answered = ip.arp_replies_sent;
    watch.frames_ignored = ip.frames_ignored;
    watch.eth_tx = dev.eth_tx_count;
    watch.eth_rx = dev.eth_rx_count;
    watch.eth_dropped = dev.census.data_dropped;
}

/// Thirty seconds between queries, spent servicing rather than sleeping — see
/// `ipstack.service`.
fn waitBetween(wifi: *ipnet.Session(Dev)) void {
    // **Through the session, not the IP layer.** This is where rung 18 actually
    // runs: thirty seconds of servicing is where a lease gets renewed and a
    // dropped link gets noticed, and calling `ip.service` directly would skip
    // both while looking identical.
    const ev = wifi.service(30_000) catch |err| {
        uart.print(.uart0, "[net] service error: {s}\n", .{@errorName(err)});
        return;
    };
    switch (ev) {
        .idle => {},
        .renewed => uart.print(.uart0, "[net] lease renewed ({d} so far, {d} failures)\n", .{
            wifi.ip.renewals, wifi.ip.renew_failures,
        }),
        .rejoined => uart.print(.uart0, "[net] link recovered ({d} drops, {d} rejoins)\n", .{
            wifi.link_drops, wifi.rejoins,
        }),
        .offline => uart.print(.uart0, "[net] OFFLINE: {d} drops, {d} failed rejoins\n", .{
            wifi.link_drops, wifi.rejoin_failures,
        }),
    }
    watch.ticks += 30;
}
