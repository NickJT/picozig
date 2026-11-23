//! net-check — everything above the frame, on real hardware.
//!
//! Rungs N5 to N11 of the ipnet bring-up ladder. TCP, DNS and HTTP were built
//! against captured fixtures and mutation tests, and this is the binary that
//! found out whether that was enough.
//!
//! ## It was — and it also found a bug two layers down, 2026-09-03
//!
//! All of N5–N11 passed on hardware, then N12 board to board: 13 bytes from a
//! literal address, the same 13 from a name, **1500 bytes reassembled from two
//! segments**, a server answering `curl`'s HTTP/1.1, and finally the two boards
//! to each other.
//!
//! The 1500-byte case failed first, and **nothing was wrong with the TCP code**.
//! `cyw43`'s receive buffer was four bytes short of what the chip announces for
//! a full-size Ethernet frame, and a packet refused for size is never drained —
//! so the receive path wedged from the first big frame onward and the second
//! segment was never acknowledged, through the original and eight
//! retransmissions. An hour went on the layer above the fault, because **this
//! example printed `HTTP 200` over a body of zero bytes**. It no longer can:
//! the body is checked against `Content-Length` and a short one is
//! `error.ShortBody`.
//!
//! ## Run it against the laptop before running it against another board
//!
//! **Two boards agreeing proves nothing if both are wrong the same way**, which
//! is why N12 is last and why this defaults to talking to something else. A
//! laptop running `python3 -m http.server 8080` is a known-good peer with a
//! known-good body; a second Pico is neither until this has passed.
//!
//! Serve the committed fixtures rather than whatever is lying about — **their
//! lengths are the test**, and `index.html` is what stops `-Dpath=/` being a
//! directory listing whose size depends on the directory:
//!
//! ```
//! # on the laptop
//! cd lib/ipnet/example/net-check/www && python3 -m http.server 8080
//!
//! # client: resolve, connect, GET, and check what came back
//! zig build swd-net-check -Dprobe-serial=<serial> -Dhost=<peer> -Dport=8080 -Dpath=/hello.txt
//!
//! # then the multi-segment case, which is the one worth running
//! zig build swd-net-check -Dprobe-serial=<serial> -Dhost=<peer> -Dport=8080 -Dpath=/big.txt
//!
//! # server: listen, and answer curl from the laptop
//! zig build swd-net-check -Dprobe-serial=<other> -Dmode=server -Dport=8080
//! ```
//!
//! **`-Dboard` is absent on purpose.** It defaults to `pico2w`, which is what
//! the bench boards are; an earlier version of this comment carried a board
//! from a different rig. **The probe
//! serial is present on purpose** — two probes enumerate identically and
//! OpenOCD flashes whichever it finds first, reporting success either way.
//!
//! `-Dhost` takes a **name or an address**. A name is the observable proof that
//! rung N9 ran — there is no resolver anywhere else in this image, so a page
//! fetched from a name could not have arrived any other way.
//!
//! ## What each stage reports, and why separately
//!
//! Every stage prints and publishes its own result, because the failures are
//! different repairs: a lease that never arrives is the network, a name that
//! will not resolve is DNS, a connection refused is the peer, and a response
//! that will not parse is this stack. **A single "failed" would send the next
//! session looking in the wrong place**, which is what `rung` in the watch
//! block has been for since `cyw43-blink`.

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

const Mode = enum { client, server };
const mode: Mode = if (std.mem.eql(u8, net_config.mode, "server")) .server else .client;

/// Poll at least this often even when the chip has said nothing — rung N2's
/// floor. An interrupt fires because a frame arrived, never because a timer
/// expired.
const poll_floor_ms: u32 = 1000;

var dev: Dev = undefined;
var wifi: ipnet.Session(Dev) = undefined;
var wakes: u32 = 0;

pub const interrupts = struct {
    pub fn io_bank0() callconv(.c) void {
        dev.onHostWakeIrq();
        _ = @atomicRmw(u32, &wakes, .Add, 1, .monotonic);
    }
};

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// The last stage reached. **Read this first**; the numbers are in `die`.
    rung: u32,
    err_code: u32,
    ip: u32,
    /// The address `-Dhost` resolved to, or the literal if it was one. Zero
    /// until DNS has answered, which is what makes N9 observable.
    resolved: u32,
    /// HTTP status, and how many body bytes arrived.
    status: u32,
    body_len: u32,
    /// Bytes of response read in total, head included.
    rx_total: u32,
    /// Requests served, when `-Dmode=server`.
    served: u32,
    wakes: u32,
    polls: u32,
    tcp_tx: u32,
    tcp_rx_overflow: u32,
    dns_queries: u32,
    dns_answers: u32,
    arp_answered: u32,
    echo_answered: u32,
    eth_rx: u32,
    eth_tx: u32,
    eth_dropped: u32,
    rx_high_water: u32,
    renewals: u32,
    reserved: u32,
};

pub const watch_magic: u32 = 0x4E455443; // 'NETC'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

/// Stage numbers, and they are the diagnosis rather than decoration.
const Rung = struct {
    const bringup: u32 = 12;
    const joined: u32 = 15;
    const resolved: u32 = 90;
    const connected: u32 = 60;
    const requested: u32 = 100;
    const answered: u32 = 101;
    const listening: u32 = 110;
    const served: u32 = 111;
};

fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "\r\nFAILED at rung {d}: {s}\r\n", .{ rung, @errorName(err) });
    watch.magic = 0xDEAD_4E45;
    watch.rung = rung;
    while (true) timer.delayMs(1000);
}

fn pack4(v: [4]u8) u32 {
    return @as(u32, v[0]) | @as(u32, v[1]) << 8 |
        @as(u32, v[2]) << 16 | @as(u32, v[3]) << 24;
}

/// Sleep, servicing the stack whenever the chip asks and at least once a
/// second regardless — rung N2 plus its floor.
var last_poll_ms: u32 = 0;
var elapsed_ms: u32 = 0;
var polls: u32 = 0;

fn idle(ms: u32) void {
    var left = ms;
    while (left > 0) : (left -= 1) {
        const woke = dev.takeHostWake();
        if (woke or elapsed_ms -% last_poll_ms >= poll_floor_ms) {
            _ = wifi.poll() catch {};
            polls += 1;
            last_poll_ms = elapsed_ms;
            if (woke) dev.rearmHostWake();
        }
        timer.delayMs(1);
        elapsed_ms +%= 1;
    }
}

fn publish() void {
    watch.ticks +%= 1;
    watch.wakes = @atomicLoad(u32, &wakes, .monotonic);
    watch.polls = polls;
    watch.tcp_tx = wifi.ip.tcp_tx_count;
    watch.tcp_rx_overflow = wifi.ip.tcp_rx_overflow;
    watch.dns_queries = wifi.ip.dns_queries;
    watch.dns_answers = wifi.ip.dns_answers;
    watch.arp_answered = wifi.ip.arp_replies_sent;
    watch.echo_answered = wifi.ip.echo_replies_sent;
    watch.eth_rx = dev.eth_rx_count;
    watch.eth_tx = dev.eth_tx_count;
    watch.eth_dropped = dev.census.data_dropped;
    watch.rx_high_water = dev.rx_high_water;
    watch.renewals = wifi.ip.renewals;
}

/// Parse `-Dhost` as a literal address, or return null if it is a name.
fn literalIpv4(s: []const u8) ?[4]u8 {
    var out: [4]u8 = undefined;
    var field: usize = 0;
    var acc: u16 = 0;
    var digits: usize = 0;
    for (s) |c| {
        if (c == '.') {
            if (digits == 0 or field >= 3) return null;
            out[field] = @intCast(acc);
            field += 1;
            acc = 0;
            digits = 0;
        } else if (c >= '0' and c <= '9') {
            acc = acc * 10 + (c - '0');
            if (acc > 255) return null;
            digits += 1;
        } else return null;
    }
    if (field != 3 or digits == 0) return null;
    out[3] = @intCast(acc);
    return out;
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;

    uart.print(
        .uart0,
        "\r\n=== ipnet net-check ===\r\nboard {s}\r\nmode  {s}\r\n",
        .{ board.name, net_config.mode },
    );

    const pkg = rp2350.silicon.package();
    if (pkg != board.expected_package) {
        uart.print(.uart0, "WARNING:   package mismatch — image built for {s}, running on {s}\r\n", .{
            board.expected_package.name(), pkg.name(),
        });
    }
    if (net_config.ssid.len == 0) die(error.BadSsid, 0);

    // -- the ladder to a frame, in the order cyw43-blink proved it --------
    const pins = cyw43.default_pins;
    dev = Dev.init(cyw43.PicogSpiBus.init(pins), pins);

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
    uart.print(.uart0, "1. firmware...\r\n", .{});
    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| die(err, 6);
    dev.waitF2Ready() catch |err| die(err, 7);
    dev.loadClmDefault() catch |err| die(err, 11);
    dev.wifiOn(.gb) catch |err| die(err, 12);
    watch.rung = Rung.bringup;

    uart.print(.uart0, "2. joining \"{s}\"...\r\n", .{net_config.ssid});
    wifi = ipnet.Session(Dev).init(&dev, net_config.ssid, net_config.password, @truncate(timer.now64()));
    wifi.ip.renew_after_secs = net_config.renew_secs;

    var attempt: u32 = 0;
    while (true) : (attempt += 1) {
        if (wifi.connect(.gb)) |_| break else |err| {
            uart.print(.uart0, "   attempt {d}: {s} (dhcp attempts {d}, frames seen {d})\r\n", .{
                attempt + 1, @errorName(err), wifi.ip.dhcp_attempts, wifi.ip.frames_seen,
            });
            if (attempt + 1 >= 4) die(err, Rung.joined);
            timer.delayMs(2000);
        }
    }
    watch.rung = Rung.joined;
    watch.ip = pack4(wifi.ip.lease.ip);
    uart.print(.uart0, "   {d}.{d}.{d}.{d}, dns {d}.{d}.{d}.{d}\r\n", .{
        wifi.ip.lease.ip[0],  wifi.ip.lease.ip[1],  wifi.ip.lease.ip[2],  wifi.ip.lease.ip[3],
        wifi.ip.lease.dns[0], wifi.ip.lease.dns[1], wifi.ip.lease.dns[2], wifi.ip.lease.dns[3],
    });

    dev.enableHostWake();
    nvic.enable(nvic.IrqNum.io_bank0);
    publish();

    switch (mode) {
        .client => runClient(),
        .server => runServer(),
    }
}

fn runClient() noreturn {
    // -- rung N9: a name, if one was given -------------------------------
    const host_addr = if (literalIpv4(net_config.host)) |a| blk: {
        uart.print(.uart0, "3. {s} is already an address\r\n", .{net_config.host});
        break :blk a;
    } else blk: {
        uart.print(.uart0, "3. resolving {s}...\r\n", .{net_config.host});
        const a = wifi.ip.resolveName(net_config.host, 5000) catch |err| die(err, Rung.resolved);
        break :blk a;
    };
    watch.resolved = pack4(host_addr);
    watch.rung = Rung.resolved;
    uart.print(.uart0, "   {d}.{d}.{d}.{d}\r\n", .{ host_addr[0], host_addr[1], host_addr[2], host_addr[3] });

    // -- rungs N5-N7: a connection ---------------------------------------
    uart.print(.uart0, "4. connecting to port {d}...\r\n", .{net_config.port});
    wifi.ip.connect(host_addr, net_config.port, 50080, 8000) catch |err| die(err, Rung.connected);
    watch.rung = Rung.connected;

    // -- rung N10: the request -------------------------------------------
    var req_buf: [256]u8 = undefined;
    const get = ipnet.http.buildGet(net_config.host, net_config.path, &req_buf) catch |err| die(err, Rung.requested);
    uart.print(.uart0, "5. GET {s}\r\n", .{net_config.path});
    wifi.ip.tcpSend(get, 8000) catch |err| die(err, Rung.requested);
    watch.rung = Rung.requested;

    var rx: [2048]u8 = undefined;
    const n = wifi.ip.tcpRecv(&rx, 8000) catch |err| die(err, Rung.answered);
    watch.rx_total = @intCast(n);

    const res = ipnet.http.parseResponse(rx[0..n]) catch |err| {
        // **A response that will not parse is this stack's fault, not the
        // network's**, so it prints what arrived rather than only the error.
        uart.print(.uart0, "   {d} bytes, unparseable: {s}\r\n", .{ n, @errorName(err) });
        const show = @min(n, 96);
        uart.print(.uart0, "   [{s}]\r\n", .{rx[0..show]});
        die(err, Rung.answered);
    };
    const body_len = n - res.body_at;
    watch.status = res.status;
    watch.body_len = @intCast(body_len);

    // **A truncated transfer parses.** The status line is intact, the headers
    // are intact, and only the count disagrees — so a client that reports the
    // status and not the length reports success over a dead connection. That
    // is exactly what this printed while cyw43's receive path was wedged on a
    // 1536-byte packet: `HTTP 200, 0 bytes of body`, with the 200 doing all
    // the talking. Content-Length is the server telling us how to check, and
    // declining to check is choosing not to be told.
    switch (res.body) {
        .length => |want| if (body_len < want) {
            uart.print(.uart0, "   TRUNCATED: {d} of {d} bytes\r\n", .{ body_len, want });
            if (wifi.ip.tcpError()) |e|
                uart.print(.uart0, "   tcp stopped: {s}\r\n", .{@errorName(e)});
            die(error.ShortBody, Rung.answered);
        },
        // Rule 8 gives no length to check against, which the RFC says is
        // indistinguishable from truncation. Nothing to assert.
        .until_close, .none => {},
    }

    watch.rung = Rung.answered;
    publish();

    uart.print(.uart0,
        \\   HTTP {d}, {d} bytes of body ({d} total)
        \\
    , .{ res.status, body_len, n });
    const preview = @min(body_len, 64);
    uart.print(.uart0, "   [{s}]\r\n", .{rx[res.body_at..][0..preview]});

    wifi.ip.tcpClose();
    report();
}

fn runServer() noreturn {
    uart.print(.uart0, "3. listening on port {d}\r\n", .{net_config.port});
    watch.rung = Rung.listening;

    var served: u32 = 0;
    while (true) {
        wifi.ip.listenTcp(net_config.port);
        // **Accept in slices, servicing between them.** A single long accept
        // would stop answering ARP and stop renewing the lease while it
        // waited, which is the obligation rung N2 exists to meet.
        var waited: u32 = 0;
        while (waited < 600_000) : (waited += 1000) {
            if (wifi.ip.acceptTcp(1000)) |_| break else |_| {}
            publish();
        }
        if (wifi.ip.conn.state != .established) continue;

        var req_buf: [1024]u8 = undefined;
        const n = wifi.ip.tcpRecv(&req_buf, 5000) catch 0;
        if (n == 0) {
            wifi.ip.tcpClose();
            continue;
        }

        var body_buf: [192]u8 = undefined;
        var res_buf: [384]u8 = undefined;
        const req = ipnet.http.parseRequest(req_buf[0..n]) catch {
            const bad = ipnet.http.buildResponse(400, "Bad Request", "text/plain", "", &res_buf) catch unreachable;
            _ = wifi.ip.tcpSend(bad, 5000) catch {};
            wifi.ip.tcpClose();
            continue;
        };

        served += 1;
        watch.served = served;
        watch.rung = Rung.served;
        uart.print(.uart0, "   {s} {s} -> 200\r\n", .{ req.method, req.target });

        const body = std.fmt.bufPrint(
            &body_buf,
            "board {s}\r\nrequests {d}\r\nwakes {d}\r\npolls {d}\r\nrenewals {d}\r\n",
            .{ board.name, served, @atomicLoad(u32, &wakes, .monotonic), polls, wifi.ip.renewals },
        ) catch "board\r\n";
        const res = ipnet.http.buildResponse(200, "OK", "text/plain", body, &res_buf) catch unreachable;
        _ = wifi.ip.tcpSend(res, 5000) catch {};
        wifi.ip.tcpClose();
        publish();
    }
}

fn report() noreturn {
    publish();
    uart.print(.uart0,
        \\
        \\--- done ---
        \\wakes/polls    {d}/{d}
        \\dns            {d} queries, {d} answers
        \\tcp segments   {d} out
        \\rx overflow    {d}
        \\eth rx/tx/drop {d}/{d}/{d}
        \\rx high water  {d} of {d}
        \\renewals       {d}
        \\
        \\driver census  data {d}, too small {d}, dropped {d}, oversize {d}, other {d}
        \\largest packet {d} bytes on the bus
        \\
        \\holding the address. Ctrl-C when done.
        \\
    , .{
        watch.wakes,             watch.polls,
        watch.dns_queries,       watch.dns_answers,
        watch.tcp_tx,            watch.tcp_rx_overflow,
        watch.eth_rx,            watch.eth_tx,
        watch.eth_dropped,       watch.rx_high_water,
        cyw43.rx_ring_depth,     watch.renewals,
        // **Read straight from the driver, not the watch block**, which is
        // capped at 0x80 bytes. A frame the TCP layer never saw was lost
        // somewhere below it, and these five say where: refused at the bus for
        // size, mis-described by its BDC header, dropped for a full latch, or
        // never delivered at all — in which case `largest packet` stays small
        // and the frame never reached the chip's host interface.
        dev.census.data,         dev.census.data_too_small,
        dev.census.data_dropped, dev.census.oversize,
        dev.census.other,        dev.census.largest_seen,
    });
    while (true) {
        idle(1000);
        publish();
    }
}
