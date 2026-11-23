//! A network that stays up — rung 18.
//!
//! Everything below this file was proven in runs lasting minutes. A clock runs
//! for months, and the two failures that get it there both look identical from
//! outside: **it worked yesterday and it is dead this morning.**
//!
//!  * The DHCP lease expires and is never renewed. `ipstack` handles that.
//!  * The AP reboots, the link drops, and nothing rejoins. This file handles
//!    that, because rejoining needs the credentials and `ipstack` is an IP
//!    layer that has no business holding them.
//!
//! ## Why this is a library file and not an example
//!
//! Because the alternative is that the only correct copy of "keep the network
//! up" lives in an example's `main`, and every consumer copy-pastes it. That is
//! precisely how the `0a85b69` fiction spread through four files. A consumer should call `service` and
//! inherit the behaviour, not reimplement it.
//!
//! ## The shape
//!
//! `connect` does the whole bring-up above the firmware: `wifiOn`, join, wait
//! for the events that say it joined, take a lease, resolve the gateway.
//! `service` is what an application calls while it has nothing else to do, and
//! it answers ARP, renews the lease, and notices when the link has gone.

const std = @import("std");
const ipstack = @import("ipstack.zig");

pub const Error = error{
    /// The link went down and would not come back within the attempts allowed.
    RejoinFailed,
};

/// What `service` did, so a caller can log a recovery rather than discover it
/// from a counter later.
pub const Event = enum {
    /// Nothing happened, which is the usual answer.
    idle,
    /// The link was down and has been brought back, lease and all.
    rejoined,
    /// The lease was renewed.
    renewed,
    /// The link is down and the rejoin did not take. The caller is still
    /// offline; `service` will try again next time.
    offline,
};

pub fn Session(comptime Dev: type) type {
    return struct {
        const Self = @This();

        dev: *Dev,
        ip: ipstack.IpStack(Dev),

        ssid: []const u8,
        password: []const u8,

        /// How long to wait for the join's events, and for a lease.
        join_timeout_ms: u32 = 20_000,
        lease_timeout_ms: u32 = 8_000,

        /// Attempts per `connect`, and per rejoin. The key exchange is
        /// intermittent — measured at roughly one failure in
        /// three — so a single attempt turns a working board into a dead one
        /// often enough to matter.
        join_attempts: u32 = 3,

        rejoins: u32 = 0,
        rejoin_failures: u32 = 0,
        link_drops: u32 = 0,

        pub fn init(dev: *Dev, ssid: []const u8, password: []const u8, seed: u32) Self {
            return .{
                .dev = dev,
                .ip = ipstack.IpStack(Dev).init(dev, dev.mac, seed),
                .ssid = ssid,
                .password = password,
            };
        }

        /// Join, take an address, and find the gateway. Everything above the
        /// firmware download, in one call.
        ///
        /// `country` is passed rather than stored because it is only needed
        /// once: `wifiOn` stays on, so a rejoin never repeats it. Taking it as
        /// `anytype` keeps this file from having to name the driver's own enum,
        /// which would make an IP-layer file depend on the chip's.
        pub fn connect(self: *Self, country: anytype) !void {
            if (!self.dev.wifi_on) try self.dev.wifiOn(country);
            // The MAC is only real once `wifiOn` has read it back from the
            // chip, and the IP layer needs it for every frame it builds.
            self.ip.mac = self.dev.mac;

            try self.joinWithRetries();
            try self.ip.acquireLease(self.lease_timeout_ms);
            try self.ip.resolveGateway(2000);
        }

        fn joinWithRetries(self: *Self) !void {
            var attempt: u32 = 0;
            while (attempt < self.join_attempts) : (attempt += 1) {
                self.dev.joinWifi(self.ssid, self.password, .wpa2_aes) catch continue;
                const link = self.dev.awaitJoin(self.join_timeout_ms) catch continue;
                if (link == .joined) return;
            }
            return Error.RejoinFailed;
        }

        /// Keep the network up for `ms` milliseconds.
        ///
        /// **The link check comes first**, because everything else is pointless
        /// without it: renewing a lease over a dead link fails in a way that
        /// looks like a DHCP problem, and answering ARP on a link that is gone
        /// answers nobody.
        pub fn service(self: *Self, ms: u32) !Event {
            if (self.dev.link_status != .joined) {
                self.link_drops += 1;
                return self.rejoin();
            }

            const before = self.ip.renewals;
            try self.ip.service(ms);
            return self.settle(before);
        }

        /// One pass, no sleeping — `IpStack.poll`'s counterpart, and **the one
        /// an interrupt-driven consumer wants**.
        ///
        /// `example/wake-check` originally called `ip.poll()` directly and that
        /// was wrong in a way no short run would show: it reaches past this
        /// type, so the link check above never runs, and a board whose AP
        /// rebooted would poll a dead link forever without rejoining. Rung 18's
        /// whole point is that the recovery lives here rather than in a
        /// caller's loop; a caller that goes around it inherits none of it.
        pub fn poll(self: *Self) !Event {
            if (self.dev.link_status != .joined) {
                self.link_drops += 1;
                return self.rejoin();
            }

            const before = self.ip.renewals;
            try self.ip.poll();
            return self.settle(before);
        }

        /// What both of the above do once the IP layer has had its turn.
        fn settle(self: *Self, before: u32) Error!Event {

            // A lease lost while the link was up means the server took it back
            // — a NAK, or an expiry. The link is fine, so the way back is a new
            // lease rather than a new join.
            if (self.ip.lease_lost) {
                self.ip.acquireLease(self.lease_timeout_ms) catch return .offline;
                self.ip.resolveGateway(2000) catch return .offline;
                return .rejoined;
            }

            return if (self.ip.renewals != before) .renewed else .idle;
        }

        /// Bring the link back, and the address with it.
        ///
        /// **The lease is kept across a rejoin.** Same network, same hardware
        /// address, and the lease has not expired — asking for a new one would
        /// churn the address for no reason and lose whatever the peer had
        /// cached. What does have to be redone is the gateway's hardware
        /// address, because an AP that has just rebooted has forgotten us and
        /// our own ARP cache may be stale.
        fn rejoin(self: *Self) Error!Event {
            self.joinWithRetries() catch {
                self.rejoin_failures += 1;
                return .offline;
            };
            self.rejoins += 1;

            if (self.ip.have_lease) {
                self.ip.resolveGateway(2000) catch {
                    // Back on the air with an address nothing can be routed
                    // to. Not fatal — the next `service` tries again — but not
                    // a recovery either.
                    return .offline;
                };
                return .rejoined;
            }

            self.ip.acquireLease(self.lease_timeout_ms) catch return .offline;
            self.ip.resolveGateway(2000) catch return .offline;
            return .rejoined;
        }

        // -- what a caller actually wants -----------------------------------

        pub fn sendUdp(self: *Self, dest_ip: [4]u8, dest_port: u16, src_port: u16, payload: []const u8) !void {
            return self.ip.sendUdp(dest_ip, dest_port, src_port, payload);
        }

        pub fn awaitUdp(self: *Self, port: u16, buf: []u8, timeout_ms: u32) !?ipstack.Received {
            return self.ip.awaitUdp(port, buf, timeout_ms);
        }

        pub fn lease(self: *Self) @TypeOf(self.ip.lease) {
            return self.ip.lease;
        }

        pub fn online(self: *Self) bool {
            return self.dev.link_status == .joined and self.ip.have_lease;
        }
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
//
// The device is faked at the level `Session` actually uses: a link status it
// can drop, a join that can be made to fail, and the frame path `ipstack`
// needs. Reusing `ipstack`'s own `FakeNet` would not do — what is under test
// here is the behaviour when the link goes away, which that fake has no notion
// of.
// ---------------------------------------------------------------------------

const testing = std.testing;
const net = @import("net.zig");
const dhcp = @import("dhcp.zig");

const test_mac = [6]u8{ 0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F };
const gw_mac = [6]u8{ 0xE4, 0x5E, 0x1B, 0xFF, 0xEE, 0xD1 };
const gw_ip = [4]u8{ 192, 168, 86, 1 };
const offered_ip = [4]u8{ 192, 168, 86, 42 };

const LinkStatus = enum(u32) { down = 0, joining = 1, joined = 2, fail = 3, no_net = 4, bad_auth = 5 };
const Country = enum { gb };

/// A device and the network behind it.
const FakeDev = struct {
    mac: [6]u8 = test_mac,
    wifi_on: bool = false,
    link_status: LinkStatus = .down,

    /// Fail this many joins before letting one through — the intermittent key
    /// exchange, which is the reason `join_attempts` exists.
    fail_joins: u32 = 0,
    joins: u32 = 0,

    serve_dhcp: bool = true,
    serve_arp: bool = true,

    /// The fake clock, advanced by `delayMs`.
    now_ms: u64 = 0,

    queue: [8][1514]u8 = undefined,
    queue_len: [8]usize = @splat(0),
    head: usize = 0,
    tail: usize = 0,

    pub fn wifiOn(self: *FakeDev, c: Country) !void {
        _ = c;
        self.wifi_on = true;
    }

    pub fn joinWifi(self: *FakeDev, ssid: []const u8, pass: []const u8, auth: anytype) !void {
        _ = ssid;
        _ = pass;
        _ = auth;
        self.joins += 1;
        if (self.fail_joins > 0) {
            self.fail_joins -= 1;
            self.link_status = .down;
            return;
        }
        self.link_status = .joined;
    }

    pub fn awaitJoin(self: *FakeDev, timeout_ms: u32) !LinkStatus {
        _ = timeout_ms;
        if (self.link_status != .joined) return error.JoinTimeout;
        return .joined;
    }

    /// **This used to discard `ms` entirely**, which was harmless while every
    /// wait here counted iterations. It is not harmless now: `service` is
    /// bounded by the clock, so a device whose clock never advances makes it
    /// loop forever. The fake sleep has to move the fake clock.
    pub fn delayMs(self: *FakeDev, ms: u32) void {
        self.now_ms +|= ms;
    }

    /// Same clock-is-the-sleep contract as `ipstack.zig`'s `FakeNet`.
    pub fn nowMs(self: *FakeDev) u64 {
        return self.now_ms;
    }

    pub fn enqueue(self: *FakeDev, frame: []const u8) void {
        @memcpy(self.queue[self.tail][0..frame.len], frame);
        self.queue_len[self.tail] = frame.len;
        self.tail = (self.tail + 1) % self.queue.len;
    }

    pub fn sendEthFrame(self: *FakeDev, frame: []const u8) !void {
        // A frame sent over a dead link goes nowhere, which is what makes the
        // link check in `service` matter.
        if (self.link_status != .joined) return;

        if (self.serve_arp) {
            if (net.parseArp(frame)) |a| {
                if (a.op == net.ArpOp.request and std.mem.eql(u8, &a.target_ip, &gw_ip)) {
                    var out: [net.arp_frame_len]u8 = undefined;
                    const reply = net.buildArpReply(a, gw_mac, gw_ip, &out) catch return;
                    self.enqueue(reply);
                }
                return;
            } else |_| {}
        }

        if (!self.serve_dhcp) return;
        const d = net.parseUdpFrame(frame) catch return;
        if (d.dest_port != dhcp.server_port) return;
        self.answerDhcp(d.payload);
    }

    pub fn answerDhcp(self: *FakeDev, msg: []const u8) void {
        if (msg.len < dhcp.header_len) return;
        const xid = std.mem.readInt(u32, msg[4..8], .big);
        const kind: dhcp.MessageType = @fromBackingInt(@intCast(msg[dhcp.header_len + 2]));
        const reply: dhcp.MessageType = switch (kind) {
            .discover => .offer,
            .request => .ack,
            else => return,
        };

        var payload: [dhcp.max_message_len]u8 = @splat(0);
        payload[0] = 2;
        payload[1] = 1;
        payload[2] = 6;
        std.mem.writeInt(u32, payload[4..8], xid, .big);
        @memcpy(payload[16..20], &offered_ip);
        @memcpy(payload[28..34], msg[28..34]);
        @memcpy(payload[236..240], &[_]u8{ 0x63, 0x82, 0x53, 0x63 });
        const opts = [_]u8{
            // zig fmt: off
            53, 1, @backingInt(reply),
            1,  4, 255, 255, 255, 0,
            3,  4, 192, 168, 86,  1,
            54, 4, 192, 168, 86,  1,
            51, 4, 0,   0,   0x0E, 0x10,
            255,
            // zig fmt: on
        };
        @memcpy(payload[dhcp.header_len..][0..opts.len], &opts);

        var frame: [1514]u8 = undefined;
        const f = net.buildUdpFrame(.{
            .dest_mac = net.broadcast_mac,
            .src_mac = gw_mac,
            .src_ip = gw_ip,
            .dest_ip = net.broadcast_ip,
            .src_port = dhcp.server_port,
            .dest_port = dhcp.client_port,
            .payload = payload[0..dhcp.min_message_len],
        }, &frame) catch return;
        self.enqueue(f);
    }

    pub fn pollEthFrame(self: *FakeDev, buf: []u8) !usize {
        if (self.head == self.tail) return 0;
        const n = self.queue_len[self.head];
        @memcpy(buf[0..n], self.queue[self.head][0..n]);
        self.head = (self.head + 1) % self.queue.len;
        return n;
    }
};

test "poll checks the link, the same as service does" {
    var fake = FakeDev{};
    var s = Session(FakeDev).init(&fake, "ssid", "pass", 0x1234);
    try s.connect(Country.gb);
    try testing.expectEqual(@as(u32, 0), s.link_drops);

    // The AP goes away. **`poll` must notice**, because an interrupt-driven
    // consumer calls nothing else — and reaching past this type to
    // `ip.poll` was exactly the bug `example/wake-check` shipped with.
    fake.link_status = .down;
    const ev = try s.poll();

    try testing.expectEqual(@as(u32, 1), s.link_drops);
    try testing.expect(ev != .idle);
}

test "connect joins, takes an address, and finds the gateway" {
    var dev = FakeDev{};
    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);

    try testing.expect(s.online());
    try testing.expectEqual(offered_ip, s.lease().ip);
    try testing.expectEqual(gw_mac, s.ip.gateway_mac);
    try testing.expectEqual(@as(u32, 1), dev.joins);
}

test "a join that fails twice is retried, because the key exchange is intermittent" {
    var dev = FakeDev{};
    // Measured at roughly one failure in three on this board. One attempt
    // would turn that into a dead board a third of the time.
    dev.fail_joins = 2;

    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);

    try testing.expect(s.online());
    try testing.expectEqual(@as(u32, 3), dev.joins);
}

test "a join that never takes is an error rather than a silent offline board" {
    var dev = FakeDev{};
    dev.fail_joins = 99;

    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try testing.expectError(Error.RejoinFailed, s.connect(.gb));
    try testing.expect(!s.online());
}

test "a link that drops is rejoined, and the address is kept" {
    var dev = FakeDev{};
    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);
    const held = s.lease().ip;

    // The AP reboots. Nothing tells the application; the link status is all
    // there is to notice by.
    dev.link_status = .down;

    const ev = try s.service(10);
    try testing.expectEqual(Event.rejoined, ev);
    try testing.expect(s.online());
    try testing.expectEqual(@as(u32, 1), s.rejoins);
    try testing.expectEqual(@as(u32, 1), s.link_drops);

    // **The same address**, because the lease is still ours: same network, same
    // MAC, not expired. Re-DHCPing on every AP blip would churn the address and
    // throw away what peers have cached.
    try testing.expectEqual(held, s.lease().ip);
}

test "a rejoin that fails leaves the caller offline rather than pretending" {
    var dev = FakeDev{};
    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);

    dev.link_status = .down;
    dev.fail_joins = 99;

    try testing.expectEqual(Event.offline, try s.service(10));
    try testing.expect(!s.online());
    try testing.expectEqual(@as(u32, 1), s.rejoin_failures);

    // And it keeps trying rather than giving up for good.
    dev.fail_joins = 0;
    try testing.expectEqual(Event.rejoined, try s.service(10));
    try testing.expect(s.online());
}

test "servicing a healthy link renews the lease and says so" {
    var dev = FakeDev{};
    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);
    s.ip.renew_after_secs = 1;

    try testing.expectEqual(Event.idle, try s.service(500));
    try testing.expectEqual(Event.renewed, try s.service(600));
    try testing.expectEqual(@as(u32, 1), s.ip.renewals);
    // Still the same address, and still online.
    try testing.expectEqual(offered_ip, s.lease().ip);
    try testing.expect(s.online());
}

test "a lease taken back over a live link is replaced without rejoining" {
    var dev = FakeDev{};
    var s = Session(FakeDev).init(&dev, "HFC", "secret", 0x1234);
    try s.connect(.gb);
    const joins_before = dev.joins;

    // The server expires the lease while the link is perfectly healthy. The way
    // back is a new lease, not a new join — rejoining here would be treating a
    // DHCP problem as a radio problem.
    s.ip.have_lease = false;
    s.ip.lease_lost = true;

    try testing.expectEqual(Event.rejoined, try s.service(10));
    try testing.expect(s.online());
    try testing.expectEqual(joins_before, dev.joins);
}
