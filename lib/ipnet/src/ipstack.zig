//! The smallest thing above an 802.3 frame that can be called a host: an
//! address, a route to it, and UDP in both directions — with TCP, DNS and HTTP
//! layered on top of the same state by `tcp.zig`, `dns.zig` and `http.zig`.
//!
//! `net.zig` and `dhcp.zig` format bytes and know nothing about a device; this
//! file owns the state that makes those bytes correct for *this* board — its
//! address, its lease, and which MAC to hand a frame to — and drives the
//! device to get it.
//!
//! ## What this is not
//!
//! Not a general IP stack. There is **no fragment reassembly** — fragments are
//! refused, not misread — and **no routing table**. ICMP echo, TCP and DNS do
//! exist now (rungs N4 and N5-N9), so a remote host may be named rather than
//! addressed. What this file owns is the part a client cannot do without:
//!
//! - **DHCP**, so the address is the network's to give rather than ours to
//!   assume.
//! - **An ARP responder**, so the address survives the peer's cache expiring.
//!   Without it a client works for a few minutes and then goes quiet with no
//!   error anywhere — the peer asks who has our address, hears nothing, and
//!   stops sending.
//! - **A filter**, because `pollEthFrame` returns whatever arrived. On a real
//!   network that is mostly mDNS, SSDP and other people's ARP, and a client
//!   that parses the first frame it sees as its own reply is reading someone
//!   else's traffic.
//!
//! ## Waiting is where the work happens
//!
//! Every wait here drains the device rather than sleeping: the driver holds one
//! received frame, so time spent not reading is time spent
//! dropping frames. `pump` is the single place that reads, services ARP, and
//! decides whether a frame is the one the caller asked for.

const std = @import("std");
const net = @import("net.zig");
const dhcp = @import("dhcp.zig");
const dns = @import("dns.zig");
const tcp = @import("tcp.zig");
const http = @import("http.zig");

/// How many frames one `poll` will take before giving the caller its turn back.
///
/// **Eight, which is twice the driver's ring depth**, so a poll that arrives
/// just as the ring fills empties it and still has room for what lands during
/// the draining. Larger would let one poll hold an application for however long
/// the network cares to talk; smaller would leave the ring part-full every time
/// and reintroduce the overflow the ring exists to prevent.
const max_drain: u32 = 8;

/// Longest frame this builds: a padded DHCP message under its headers.
const max_tx_frame: usize = net.eth_header_len + net.ipv4_header_len +
    net.udp_header_len + dhcp.max_message_len;

pub const Error = error{
    /// No server answered the DISCOVER.
    NoDhcpOffer,
    /// The server answered the REQUEST with NAK: the address was refused.
    DhcpNak,
    /// A lease arrived without the fields needed to use it.
    IncompleteLease,
    /// Nobody answered the ARP for the gateway.
    GatewayUnreachable,
    /// A send was attempted before there was an address to send it from.
    NoAddress,
    /// The lease carried no DNS server, so there is nobody to ask.
    NoDnsServer,
    /// The resolver said there is no such name. **Distinct from a timeout**:
    /// one is an answer and the other is silence, and a caller retrying a
    /// typo forever is the failure that conflating them produces.
    NameNotFound,
    DnsTimeout,
    /// The connection did not complete, or the peer stopped answering.
    TcpTimeout,
    /// `tcpSend` while a segment is still outstanding — stop-and-wait.
    NotWritable,
    BufferTooSmall,
};

/// A UDP datagram that arrived for us, with its payload in the caller's buffer.
pub const Received = struct {
    src_ip: [4]u8,
    src_port: u16,
    len: usize,
};

pub fn IpStack(comptime Dev: type) type {
    return struct {
        const Self = @This();

        dev: *Dev,
        mac: [6]u8,

        /// The lease. All zero until DHCP has run — **not a default address**,
        /// because a plausible-looking one is indistinguishable from a real one
        /// in every log it appears in.
        lease: dhcp.Lease = .{},
        have_lease: bool = false,

        /// The gateway's hardware address, learned by ARP. Broadcast is not a
        /// usable default, so this stays zero until it is real.
        gateway_mac: [6]u8 = @splat(0),

        /// The name this board asks to be known by. It appears in the router's
        /// client table, which makes it evidence: a name chosen here cannot
        /// have come from a previous tenant of this MAC.
        hostname: []const u8 = "zig-pico2w",

        /// Seeds the DHCP transaction id. **Constant across reboots is wrong**:
        /// a reply to the previous boot's exchange would match this one. The
        /// caller passes something that varies — the timer will do.
        seed: u32,
        xid: u32 = 0,

        /// When the lease was granted, on the device's clock.
        ///
        /// **This used to be an accumulator**, `lease_age_ms`, counting the
        /// milliseconds `service()` had slept — the stack had no time source
        /// but `delayMs`, and rather than add a seam it counted its own naps.
        /// That was wrong in a way that only shows up under load: every wait
        /// loop here spent `pump()` plus a millisecond per iteration, so a busy
        /// network made the clock run slow and every timeout longer than it
        /// said. **TCP cannot be built on that** — RTO, delayed ACK and
        /// TIME-WAIT are wall-clock, and a stack that measures time by how long
        /// its caller idled gets all three wrong by an amount the caller
        /// controls. `Dev.nowMs` is the seam; rung N1.
        ///
        /// Absolute rather than elapsed, so a gap in servicing ages the lease
        /// correctly instead of pausing it.
        lease_taken_ms: u64 = 0,

        /// Renew this many seconds after the lease is granted, instead of at
        /// T1. Zero means use T1, which is the right behaviour and takes half
        /// a lease to observe — **twelve hours on this network**, which is why
        /// the override exists. `-Drenew` sets it.
        renew_after_secs: u32 = 0,

        renewals: u32 = 0,
        renew_failures: u32 = 0,
        rebinds: u32 = 0,
        /// Set when the server refused, or the lease ran out. The address is
        /// gone and only a fresh `acquireLease` will get one back.
        lease_lost: bool = false,

        /// What the filter saw, so a wait that returns nothing can say why.
        arp_replies_sent: u32 = 0,
        arp_requests_seen: u32 = 0,
        echo_requests_seen: u32 = 0,
        echo_replies_sent: u32 = 0,
        /// Echo requests too big to answer from `rx_echo`. **Counted rather
        /// than silently unanswered**: a ping that works at 56 bytes and stops
        /// at 1400 is a report someone will make, and this is the field that
        /// answers it in one look.
        echo_too_large: u32 = 0,
        udp_accepted: u32 = 0,
        frames_ignored: u32 = 0,
        /// Every frame this stack has looked at, whatever became of it.
        ///
        /// **`poll` needs to know it read nothing**, and none of the counters
        /// above can tell it: a frame that was ours, a frame that was ARP and a
        /// frame that was somebody else's all increment different things, and
        /// "no frame at all" increments none of them. Summing three counters to
        /// infer a fourth is the kind of arithmetic that goes wrong when a
        /// fourth outcome is added later.
        frames_seen: u32 = 0,
        /// How many polls stopped at `max_drain` with more still waiting — the
        /// shape of a stack falling behind rather than an error in itself.
        drain_limited: u32 = 0,
        dhcp_attempts: u32 = 0,
        dns_queries: u32 = 0,
        dns_answers: u32 = 0,
        tcp_tx_count: u32 = 0,
        /// The one connection. **One, and the type says so** — `lib/ipnet`
        /// ships stop-and-wait with a single socket, and a second field here
        /// would be the first step towards pretending otherwise.
        conn: tcp.Connection = .{},
        tcp_tx: [net.eth_header_len + net.ipv4_header_len + 64 + tcp.ethernet_mss]u8 = @splat(0),

        /// Received stream bytes, waiting for a caller to take them.
        ///
        /// **`poll` drains several frames per call, so a caller cannot read
        /// `conn.received` after it returns** — the second segment overwrites
        /// the first, and everything but the last is lost. That is exactly
        /// what happened the first time this was wired up, and the symptom was
        /// an HTTP response whose head had vanished: a 200 arriving in two
        /// segments delivered only the body.
        ///
        /// Two MSS, which is one maximal write — a captured TCP session
        /// shows 2048 bytes leaving as 1460 + 588. **A peer that sends faster
        /// than the caller reads overflows it**, and the count says so rather
        /// than the bytes going quietly missing. Real backpressure means
        /// shrinking the advertised window, which stop-and-wait does not do.
        tcp_rx: [2 * tcp.ethernet_mss]u8 = @splat(0),
        tcp_rx_len: usize = 0,
        tcp_rx_overflow: u32 = 0,
        /// **An error the connection raised while polling**, held until a
        /// caller is in a position to be told. `poll` has nowhere to report a
        /// refused connection to — it is servicing everything at once — so it
        /// is latched here and returned by whichever call was waiting.
        tcp_error: ?anyerror = null,

        tx: [max_tx_frame]u8 = @splat(0),
        rx: [1514]u8 = @splat(0),
        /// **A separate buffer, because a reply is built while the request is
        /// still being read.** `rx` holds the frame `parseEchoRequest` returned
        /// a payload slice into, and `buildEchoReply` copies that payload — so
        /// building into `rx` would be a `memcpy` between overlapping regions
        /// of one buffer, which is undefined and would corrupt exactly the
        /// large pings that are hardest to notice.
        rx_echo: [1514]u8 = @splat(0),

        pub fn init(dev: *Dev, mac: [6]u8, seed: u32) Self {
            return .{ .dev = dev, .mac = mac, .seed = seed };
        }

        /// How old the lease is, in milliseconds. Saturating, so a clock that
        /// goes backwards reads as zero rather than as an expired lease.
        pub fn leaseAgeMs(self: *Self) u64 {
            return self.dev.nowMs() -| self.lease_taken_ms;
        }

        // -- addresses ------------------------------------------------------

        /// Run the DHCP exchange: DISCOVER, OFFER, REQUEST, ACK.
        ///
        /// Retries the DISCOVER, because it is a broadcast on a shared medium
        /// and one lost frame is not a missing server. Each attempt uses a
        /// **new transaction id**, so a late answer to an earlier attempt is
        /// rejected rather than adopted.
        pub fn acquireLease(self: *Self, timeout_ms: u32) !void {
            var attempt: u32 = 0;
            while (attempt < 4) : (attempt += 1) {
                self.dhcp_attempts += 1;
                self.xid = self.seed ^ (0x9E37_79B9 *% (attempt +% 1));

                const offer = self.discover(timeout_ms / 4) catch |err| switch (err) {
                    error.NoDhcpOffer => continue,
                    else => return err,
                };

                // The REQUEST keeps the same transaction as the OFFER it
                // accepts — that is what makes it an answer rather than a new
                // conversation.
                const ack = try self.request(offer, timeout_ms / 4);
                if (ack.kind == .nak) return Error.DhcpNak;

                if (isZero(&ack.lease.ip) or isZero(&ack.lease.mask) or isZero(&ack.lease.gateway)) {
                    return Error.IncompleteLease;
                }

                self.lease = ack.lease;
                self.have_lease = true;
                self.lease_taken_ms = self.dev.nowMs();
                self.lease_lost = false;
                return;
            }
            return Error.NoDhcpOffer;
        }

        fn discover(self: *Self, timeout_ms: u32) !dhcp.Lease {
            var msg: [dhcp.max_message_len]u8 = undefined;
            const m = try dhcp.buildDiscover(self.mac, self.xid, self.hostname, &msg);
            try self.sendUdpFrom(net.broadcast_mac, net.any_ip, net.broadcast_ip, dhcp.client_port, dhcp.server_port, m);

            const r = try self.awaitDhcp(.offer, timeout_ms) orelse return Error.NoDhcpOffer;
            return r.lease;
        }

        fn request(self: *Self, offer: dhcp.Lease, timeout_ms: u32) !dhcp.Reply {
            var msg: [dhcp.max_message_len]u8 = undefined;
            const m = try dhcp.buildRequest(
                self.mac,
                self.xid,
                offer.ip,
                offer.server_id,
                self.hostname,
                &msg,
            );
            // **Still broadcast, and still from 0.0.0.0.** The address in the
            // OFFER is not ours until the ACK arrives, so using it as a source
            // here would be claiming an address the server has not yet given.
            try self.sendUdpFrom(net.broadcast_mac, net.any_ip, net.broadcast_ip, dhcp.client_port, dhcp.server_port, m);

            return (try self.awaitDhcp(.ack, timeout_ms)) orelse Error.NoDhcpOffer;
        }

        /// Wait for a DHCP reply of the kind expected — or a NAK, which is an
        /// answer too and is returned rather than ignored.
        fn awaitDhcp(self: *Self, want: dhcp.MessageType, timeout_ms: u32) !?dhcp.Reply {
            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                if (try self.pump(.{ .port = dhcp.client_port, .require_our_ip = false })) |got| {
                    // **Address filtering is off during acquisition**, because
                    // there is no address yet: a server is free to unicast the
                    // reply to the offer it is about to make. The transaction
                    // id and hardware address are what identify it, and
                    // `parseReply` checks both.
                    if (dhcp.parseReply(self.rx[got.payload_at..][0..got.payload_len], self.mac, self.xid)) |r| {
                        if (r.kind == want or r.kind == .nak) return r;
                    } else |_| {
                        self.frames_ignored += 1;
                    }
                }
                self.dev.delayMs(1);
            }
            return null;
        }

        // -- TCP, rungs N6 to N10 --------------------------------------------

        /// Route a destination the way `sendUdp` does, then put a segment on
        /// the wire. Shared rather than duplicated, because the on-link and
        /// gateway rules are the same for any protocol.
        fn routeTo(self: *Self, dest_ip: [4]u8) ![6]u8 {
            if (self.onLink(dest_ip)) {
                if (std.mem.eql(u8, &dest_ip, &self.lease.gateway)) return self.gateway_mac;
                return try self.resolve(dest_ip, 1000);
            }
            return self.gateway_mac;
        }

        fn emit(self: *Self, a: tcp.Action) !void {
            // **No guard for a listener with no remote address**, and the
            // mutation harness is why: removing one made no test fail, because
            // the path cannot be reached. `onSegment` sets `remote_ip` from the
            // SYN before it returns anything to send, and `tick` in `listen`
            // has no deadline so it never fires. A guard here would have
            // implied a hazard that does not exist.
            const dest_mac = try self.routeTo(self.conn.remote_ip);
            if (isZero(&dest_mac)) return Error.GatewayUnreachable;

            const seg = try tcp.buildSegment(.{
                .dest_mac = dest_mac,
                .src_mac = self.mac,
                .src_ip = self.lease.ip,
                .dest_ip = self.conn.remote_ip,
                .src_port = self.conn.local_port,
                .dest_port = self.conn.remote_port,
                .seq = a.seq,
                .ack = a.ack,
                .flags = a.flags,
                .window = self.conn.rcv_wnd,
                .mss = a.mss,
                .payload = a.data,
            }, &self.tcp_tx);
            try self.dev.sendEthFrame(seg);
            self.tcp_tx_count += 1;
        }

        /// Feed a received frame to the connection, if it is one. Returns true
        /// if the frame was TCP and has been dealt with.
        fn serviceTcp(self: *Self, frame: []const u8) bool {
            const seg = tcp.parseSegment(frame, self.lease.ip) catch return false;
            if (!self.conn.matches(seg)) {
                // **Not ours, and not swallowed.** A segment for a connection
                // this stack has never heard of falls through to the counters,
                // the same way an ARP for another host does.
                return false;
            }
            self.conn.received = &.{};
            const action = self.conn.onSegment(seg, self.dev.nowMs()) catch |err| {
                self.tcp_error = err;
                return true;
            };
            if (self.conn.received.len != 0) {
                const room = self.tcp_rx.len - self.tcp_rx_len;
                const take = @min(self.conn.received.len, room);
                @memcpy(self.tcp_rx[self.tcp_rx_len..][0..take], self.conn.received[0..take]);
                self.tcp_rx_len += take;
                if (take < self.conn.received.len) self.tcp_rx_overflow += 1;
            }
            if (action) |a| self.emit(a) catch {};
            return true;
        }

        /// Give the connection its clock. Called from `poll`.
        fn serviceTcpTimer(self: *Self) void {
            const action = self.conn.tick(self.dev.nowMs()) catch |err| {
                self.tcp_error = err;
                return;
            };
            if (action) |a| self.emit(a) catch {};
        }

        /// Open a connection to `dest_ip`, blocking until it is established or
        /// gives up.
        pub fn connect(self: *Self, dest_ip: [4]u8, dest_port: u16, local_port: u16, timeout_ms: u32) !void {
            if (!self.have_lease) return Error.NoAddress;
            self.tcp_error = null;
            // **The ISN varies**, because a board that restarts with the same
            // one can have an old connection's segments accepted by a new one.
            const isn = self.seed ^ (0x9E37_79B9 *% (self.tcp_tx_count +% 1));
            self.tcp_rx_len = 0;
            const syn = self.conn.connect(local_port, dest_ip, dest_port, isn, self.dev.nowMs());
            try self.emit(syn);

            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                try self.poll();
                if (self.tcp_error) |e| {
                    self.tcp_error = null;
                    return e;
                }
                if (self.conn.state == .established) return;
                self.dev.delayMs(1);
            }
            return Error.TcpTimeout;
        }

        /// Wait for a connection on `port` — rung N11.
        ///
        /// **The listen is separate from the wait**, so a caller can arm the
        /// socket once and then poll for arrivals as part of its ordinary idle
        /// loop. A single call that did both would force a server to block,
        /// and this stack's whole servicing model is that nothing blocks for
        /// longer than it must.
        pub fn listenTcp(self: *Self, port: u16) void {
            self.tcp_error = null;
            self.tcp_rx_len = 0;
            const isn = self.seed ^ (0x9E37_79B9 *% (self.tcp_tx_count +% 1));
            self.conn.listen(port, isn);
        }

        /// Poll until somebody connects, or the timeout expires.
        pub fn acceptTcp(self: *Self, timeout_ms: u32) !void {
            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                try self.poll();
                if (self.tcp_error) |e| {
                    self.tcp_error = null;
                    return e;
                }
                if (self.conn.state == .established) return;
                self.dev.delayMs(1);
            }
            return Error.TcpTimeout;
        }

        /// Send, waiting for the peer to acknowledge — stop-and-wait means
        /// there is nothing else to wait for.
        pub fn tcpSend(self: *Self, payload: []const u8, timeout_ms: u32) !void {
            const first = self.conn.send(payload, self.dev.nowMs()) orelse return Error.NotWritable;
            try self.emit(first);

            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                try self.poll();
                if (self.tcp_error) |e| {
                    self.tcp_error = null;
                    return e;
                }
                if (self.conn.writable()) return;
                self.dev.delayMs(1);
            }
            return Error.TcpTimeout;
        }

        /// Collect received bytes into `buf` until the peer closes, the buffer
        /// fills, or the timeout expires.
        ///
        /// **Returns what it got rather than erroring on a short read**, since
        /// a close-delimited HTTP body (RFC 9112 §6.3 rule 8) has no length to
        /// check against — the caller decides whether what arrived is enough.
        pub fn tcpRecv(self: *Self, buf: []u8, timeout_ms: u32) !usize {
            var n: usize = 0;
            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                try self.poll();
                if (self.tcp_error) |e| {
                    // **Data first, then the reason it stopped — but the
                    // reason has to survive.** This used to clear the latch
                    // before returning the bytes, so a connection that died
                    // mid-transfer handed the caller a short read and no way
                    // to know it was short. On 2026-09-03 that turned a wedged
                    // receive path into a printed `HTTP 200`, and cost an hour
                    // spent looking above the link for a fault below it.
                    //
                    // Left latched, `tcpError` returns it and the next call
                    // still reports it.
                    if (n != 0) return n;
                    self.tcp_error = null;
                    return e;
                }
                if (self.tcp_rx_len != 0) {
                    const take = @min(self.tcp_rx_len, buf.len - n);
                    @memcpy(buf[n..][0..take], self.tcp_rx[0..take]);
                    n += take;
                    // Shuffle down whatever did not fit in the caller's buffer.
                    const left = self.tcp_rx_len - take;
                    if (left != 0) std.mem.copyForwards(u8, self.tcp_rx[0..left], self.tcp_rx[take..][0..left]);
                    self.tcp_rx_len = left;
                    if (n == buf.len) return n;
                }
                if (self.conn.peer_closed) return n;
                self.dev.delayMs(1);
            }
            return n;
        }

        /// The latched connection error, if one is waiting.
        ///
        /// **Peek, not take.** `tcpRecv` returns whatever bytes it has before
        /// the reason it stopped, so a caller holding a short read needs to be
        /// able to ask why without the asking clearing it.
        pub fn tcpError(self: *const Self) ?anyerror {
            return self.tcp_error;
        }

        /// Close, and let TIME-WAIT run its course in the background.
        pub fn tcpClose(self: *Self) void {
            if (self.conn.close(self.dev.nowMs())) |a| self.emit(a) catch {};
        }

        /// Turn a name into an address — rung N9.
        ///
        /// **The DNS server comes from the DHCP lease**, which is the whole
        /// reason this can work without configuration: the router that gave us
        /// an address also told us who to ask.
        ///
        /// Retries like `acquireLease` does, and for the same reason — this is
        /// UDP over a shared medium and one lost datagram is not a missing
        /// server. **Each attempt uses a new transaction id**, so a late answer
        /// to an earlier attempt is refused rather than adopted.
        pub fn resolveName(self: *Self, name: []const u8, timeout_ms: u32) ![4]u8 {
            if (!self.have_lease) return Error.NoAddress;
            if (isZero(&self.lease.dns)) return Error.NoDnsServer;

            var attempt: u32 = 0;
            while (attempt < 3) : (attempt += 1) {
                const id: u16 = @truncate(self.seed ^ (0x9E37_79B9 *% (self.dns_queries +% 1)));
                self.dns_queries += 1;

                var q: [dns.max_name_len + 32]u8 = undefined;
                const query = try dns.buildQuery(name, id, &q);
                // A fixed source port, so the filter has something to match,
                // and high enough not to collide with anything served here.
                const src_port: u16 = 50_053;
                try self.sendUdp(self.lease.dns, dns.port, src_port, query);

                var buf: [512]u8 = undefined;
                const got = (try self.awaitUdp(src_port, &buf, timeout_ms / 3)) orelse continue;

                const r = dns.parseReply(buf[0..got.len]) catch continue;
                // **Ours is the one whose id matches.** Every DNS datagram on
                // this port parses perfectly; the id is what makes one of them
                // an answer to our question.
                if (r.id != id) continue;
                self.dns_answers += 1;

                if (r.rcode == dns.Rcode.name_error) return Error.NameNotFound;
                return r.address orelse Error.NoAddress;
            }
            return Error.DnsTimeout;
        }

        /// Learn the gateway's hardware address. Nothing can be routed off this
        /// subnet until it is known.
        pub fn resolveGateway(self: *Self, timeout_ms: u32) !void {
            if (!self.have_lease) return Error.NoAddress;
            self.gateway_mac = try self.resolve(self.lease.gateway, timeout_ms);
        }

        fn resolve(self: *Self, ip: [4]u8, timeout_ms: u32) ![6]u8 {
            var attempt: u32 = 0;
            while (attempt < 4) : (attempt += 1) {
                var frame: [net.arp_frame_len]u8 = undefined;
                const req = try net.buildArpRequest(self.mac, self.lease.ip, ip, &frame);
                try self.dev.sendEthFrame(req);

                const deadline = self.dev.nowMs() +| (timeout_ms / 4);
                while (self.dev.nowMs() < deadline) {
                    // The ARP reply is not UDP, so it never satisfies `pump`'s
                    // filter — but `pump` parses and records every ARP it sees,
                    // which is where this comes from.
                    if (try self.pumpArp(ip)) |mac| return mac;
                    self.dev.delayMs(1);
                }
            }
            return Error.GatewayUnreachable;
        }

        // -- UDP ------------------------------------------------------------

        /// Send a UDP datagram, routing it the way a host with one interface
        /// routes: on-link destinations go straight to their own hardware
        /// address, everything else goes to the gateway.
        pub fn sendUdp(self: *Self, dest_ip: [4]u8, dest_port: u16, src_port: u16, payload: []const u8) !void {
            if (!self.have_lease) return Error.NoAddress;

            const dest_mac = if (std.mem.eql(u8, &dest_ip, &net.broadcast_ip))
                net.broadcast_mac
            else if (self.onLink(dest_ip)) blk: {
                // The gateway is the only on-link host this driver talks to;
                // anything else on the subnet would need its own ARP, and there
                // is no cache here to put the answer in.
                if (std.mem.eql(u8, &dest_ip, &self.lease.gateway)) break :blk self.gateway_mac;
                break :blk try self.resolve(dest_ip, 1000);
            } else self.gateway_mac;

            if (isZero(&dest_mac)) return Error.GatewayUnreachable;
            try self.sendUdpFrom(dest_mac, self.lease.ip, dest_ip, src_port, dest_port, payload);
        }

        fn sendUdpFrom(
            self: *Self,
            dest_mac: [6]u8,
            src_ip: [4]u8,
            dest_ip: [4]u8,
            src_port: u16,
            dest_port: u16,
            payload: []const u8,
        ) !void {
            const frame = try net.buildUdpFrame(.{
                .dest_mac = dest_mac,
                .src_mac = self.mac,
                .src_ip = src_ip,
                .dest_ip = dest_ip,
                .src_port = src_port,
                .dest_port = dest_port,
                .payload = payload,
            }, &self.tx);
            try self.dev.sendEthFrame(frame);
        }

        /// Keep the host alive for `ms` milliseconds: read frames, answer ARP,
        /// drop the rest.
        ///
        /// **An ARP responder that only runs while you are waiting for a reply
        /// is not a responder.** That was this file's first version, and it
        /// passed its own test — the test does its work inside `awaitUdp`,
        /// which pumps. On hardware the board took a lease, went into its main
        /// loop, and stopped answering: this Mac asked who had 192.168.86.241
        /// and got `(incomplete)`, from a board that was sitting there holding
        /// exactly that address.
        ///
        /// So an application that wants to keep its address has to call this
        /// wherever it would otherwise idle. There is no interrupt here doing
        /// it behind the caller's back.
        /// **One pass, and it does not sleep.** Collect whatever has arrived,
        /// answer an ARP if that is what it was, and check the lease clock.
        ///
        /// This is the primitive an interrupt or an alarm calls (rung N2), and
        /// the one `service` is built from. Keeping it non-blocking is what
        /// makes it callable from a handler at all — `service` owns the waiting
        /// and this owns the work.
        ///
        /// **It drains, rather than taking one frame — rung N3.** Until the
        /// driver had a receive ring there was nothing to drain and a loop here
        /// would only have moved a single slot's overflow around; now a burst
        /// is held for us and leaving it there is how the next one overflows.
        ///
        /// Bounded by `max_drain`, because "read until empty" against a device
        /// that keeps producing is a loop with no exit. Hitting the bound is
        /// not an error — the next poll continues — but it is the shape of a
        /// stack falling behind, so it is counted.
        pub fn poll(self: *Self) !void {
            var drained: u32 = 0;
            while (drained < max_drain) : (drained += 1) {
                // The filter is one nothing matches, so every frame falls
                // through to the ARP path and the counters.
                const before = self.frames_seen;
                _ = try self.pump(.{ .port = 0, .require_our_ip = true });
                if (self.frames_seen == before) break;
            }
            if (drained == max_drain) self.drain_limited += 1;
            self.serviceTcpTimer();
            try self.maybeRenew();
        }

        /// Keep the network alive for `ms` milliseconds of **real time**.
        ///
        /// The loop is bounded by the clock rather than by an iteration count,
        /// which is the whole of rung N1: `service(1000)` now means a second,
        /// not "a thousand passes, each of which took a millisecond plus
        /// however long the network kept us busy".
        pub fn service(self: *Self, ms: u32) !void {
            const deadline = self.dev.nowMs() +| ms;
            // At least one pass, so `service(0)` still collects and still
            // checks the lease.
            while (true) {
                try self.poll();
                if (self.dev.nowMs() >= deadline) return;
                self.dev.delayMs(1);
            }
        }

        // -- keeping the lease ----------------------------------------------

        /// When to renew, and when to give up, in milliseconds since the lease
        /// was granted. **RFC 2131's fractions**, not round numbers: T1 is half
        /// the lease and T2 is seven eighths of it.
        fn t1(self: *Self) u64 {
            if (self.renew_after_secs != 0) return @as(u64, self.renew_after_secs) * 1000;
            return @as(u64, self.lease.lease_secs) * 500;
        }

        fn t2(self: *Self) u64 {
            if (self.renew_after_secs != 0) return @as(u64, self.renew_after_secs) * 1000 * 7 / 4;
            return @as(u64, self.lease.lease_secs) * 875;
        }

        fn expiry(self: *Self) u64 {
            if (self.renew_after_secs != 0) return @as(u64, self.renew_after_secs) * 1000 * 2;
            return @as(u64, self.lease.lease_secs) * 1000;
        }

        /// Renew if the lease is old enough, rebind if renewing has stopped
        /// working, and drop it if it runs out.
        ///
        /// **Failing to renew is not an error to the caller.** A server that
        /// does not answer at T1 leaves the lease valid until T2, and valid
        /// until expiry after that — the address still works, so a failure here
        /// is recorded and retried rather than propagated. Only a NAK or an
        /// expiry actually takes the address away.
        fn maybeRenew(self: *Self) !void {
            if (!self.have_lease or self.lease.lease_secs == 0) return;

            if (self.leaseAgeMs() >= self.expiry()) {
                // Out of time. The address is no longer ours to use, and saying
                // so is the whole point — a client that kept using it would be
                // squatting on an address the server may have given away.
                self.have_lease = false;
                self.lease_lost = true;
                return;
            }

            if (self.leaseAgeMs() < self.t1()) return;

            // **Unicast to the server at T1, broadcast at T2.** The message is
            // the same; where it goes is the difference between asking the
            // server that gave us this address and asking anyone who will
            // listen.
            const rebinding = self.leaseAgeMs() >= self.t2();
            self.renewOnce(rebinding) catch {
                self.renew_failures += 1;
            };

            // **A renewal attempt costs real time — two seconds of it when
            // nobody answers — and that time can carry the lease past expiry.**
            // Checking only on entry was correct while the clock was an
            // accumulator that `service` advanced after the fact; with a real
            // clock the attempt's own duration is visible, and a lease can go
            // from valid to expired inside this function. `service` may well
            // return before there is another poll to notice, so the address
            // would stay in use after it stopped being ours. Found by rung N1.
            if (self.have_lease and self.leaseAgeMs() >= self.expiry()) {
                self.have_lease = false;
                self.lease_lost = true;
            }
        }

        fn renewOnce(self: *Self, rebinding: bool) !void {
            self.xid = self.seed ^ (0x9E37_79B9 *% (self.renewals +% self.renew_failures +% 2));

            var msg: [dhcp.max_message_len]u8 = undefined;
            const m = try dhcp.buildRenew(self.mac, self.xid, self.lease.ip, self.hostname, &msg);

            if (rebinding) {
                self.rebinds += 1;
                try self.sendUdpFrom(net.broadcast_mac, self.lease.ip, net.broadcast_ip, dhcp.client_port, dhcp.server_port, m);
            } else {
                // The server's own address, which means its hardware address
                // has to be resolved like any other on-link host. It is almost
                // always the gateway, and almost always already known.
                const mac = if (std.mem.eql(u8, &self.lease.server_id, &self.lease.gateway) and !isZero(&self.gateway_mac))
                    self.gateway_mac
                else
                    try self.resolve(self.lease.server_id, 1000);
                try self.sendUdpFrom(mac, self.lease.ip, self.lease.server_id, dhcp.client_port, dhcp.server_port, m);
            }

            const reply = (try self.awaitDhcp(.ack, 2000)) orelse return Error.NoDhcpOffer;
            if (reply.kind == .nak) {
                // The server says this address is not ours. Nothing to argue
                // with: drop it, and let the caller start again.
                self.have_lease = false;
                self.lease_lost = true;
                return Error.DhcpNak;
            }

            // **Keep the address, take the new timings.** A renewal ACK may
            // carry a different lease length, and it always restarts the clock.
            if (!isZero(&reply.lease.ip)) self.lease.ip = reply.lease.ip;
            if (reply.lease.lease_secs != 0) self.lease.lease_secs = reply.lease.lease_secs;
            if (!isZero(&reply.lease.server_id)) self.lease.server_id = reply.lease.server_id;
            self.lease_taken_ms = self.dev.nowMs();
            self.renewals += 1;
        }

        /// Wait for a UDP datagram addressed to `port` on this board, copying
        /// its payload into `buf`. Returns null on timeout — a quiet network is
        /// not an error, and the caller knows what it was waiting for.
        ///
        /// ARP requests are answered throughout the wait. That is not a side
        /// effect: a wait is the longest this board goes without transmitting,
        /// and it is exactly when a peer's cache is most likely to expire.
        pub fn awaitUdp(self: *Self, port: u16, buf: []u8, timeout_ms: u32) !?Received {
            const deadline = self.dev.nowMs() +| timeout_ms;
            while (self.dev.nowMs() < deadline) {
                if (try self.pump(.{ .port = port, .require_our_ip = true })) |got| {
                    if (buf.len < got.payload_len) return Error.BufferTooSmall;
                    @memcpy(buf[0..got.payload_len], self.rx[got.payload_at..][0..got.payload_len]);
                    return .{ .src_ip = got.src_ip, .src_port = got.src_port, .len = got.payload_len };
                }
                self.dev.delayMs(1);
            }
            return null;
        }

        // -- the frame filter -----------------------------------------------

        const Filter = struct {
            port: u16,
            /// Off during DHCP, when there is no address to match against.
            require_our_ip: bool,
        };

        const Hit = struct {
            payload_at: usize,
            payload_len: usize,
            src_ip: [4]u8,
            src_port: u16,
        };

        /// Read one frame and decide what it is: our datagram, an ARP request
        /// to answer, or traffic that belongs to somebody else.
        ///
        /// **Everything not ours is counted, not silently dropped.** On a real
        /// network the ignored count runs into the hundreds, and a client that
        /// cannot say so has no way to tell "nothing arrived" from "nothing I
        /// recognised arrived".
        fn pump(self: *Self, f: Filter) !?Hit {
            const n = try self.dev.pollEthFrame(&self.rx);
            if (n == 0) return null;
            self.frames_seen += 1;
            const frame = self.rx[0..n];

            if (self.serviceArp(frame)) return null;
            if (self.serviceEcho(frame)) return null;
            if (self.serviceTcp(frame)) return null;

            const d = net.parseUdpFrame(frame) catch {
                self.frames_ignored += 1;
                return null;
            };
            if (d.dest_port != f.port) {
                self.frames_ignored += 1;
                return null;
            }
            if (f.require_our_ip and !std.mem.eql(u8, &d.dest_ip, &self.lease.ip)) {
                // Broadcast to our port is still ours to look at; a datagram
                // for a different host is not.
                if (!std.mem.eql(u8, &d.dest_ip, &net.broadcast_ip)) {
                    self.frames_ignored += 1;
                    return null;
                }
            }

            self.udp_accepted += 1;
            const at = @intFromPtr(d.payload.ptr) - @intFromPtr(frame.ptr);
            return .{
                .payload_at = at,
                .payload_len = d.payload.len,
                .src_ip = d.src_ip,
                .src_port = d.src_port,
            };
        }

        /// Answer an ARP request for our address. Returns true if the frame was
        /// ARP and has been dealt with.
        fn serviceArp(self: *Self, frame: []const u8) bool {
            const a = net.parseArp(frame) catch return false;

            if (a.op == net.ArpOp.request) {
                self.arp_requests_seen += 1;
                if (self.have_lease and std.mem.eql(u8, &a.target_ip, &self.lease.ip)) {
                    var out: [net.arp_frame_len]u8 = undefined;
                    if (net.buildArpReply(a, self.mac, self.lease.ip, &out)) |reply| {
                        if (self.dev.sendEthFrame(reply)) {
                            self.arp_replies_sent += 1;
                        } else |_| {}
                    } else |_| {}
                }
            }
            return true;
        }

        /// Answer an ICMP echo request addressed to us — rung N4.
        ///
        /// **In `pump`, beside the ARP responder, and for the same reason.**
        /// The responder that only ran inside `awaitUdp` is the bug this
        /// package remembers best: a board that answers only while its
        /// application happens to be waiting is a board that goes deaf the
        /// moment it is doing anything else. A ping has to be answered whenever
        /// the stack is serviced at all.
        ///
        /// Returns true if the frame was an echo request and has been dealt
        /// with. **A request that fails to parse returns false**, not true, so
        /// it falls through to the UDP path and the counters rather than being
        /// swallowed as "handled".
        fn serviceEcho(self: *Self, frame: []const u8) bool {
            if (!self.have_lease) return false;
            const e = net.parseEchoRequest(frame, self.lease.ip) catch return false;

            self.echo_requests_seen += 1;
            // The reply is the request's payload plus headers, so it cannot
            // exceed the frame that arrived — but `tx` is sized for a padded
            // DHCP message, and a full-MTU ping is larger.
            const reply = net.buildEchoReply(e, self.mac, self.lease.ip, &self.rx_echo) catch {
                self.echo_too_large += 1;
                return true;
            };
            if (self.dev.sendEthFrame(reply)) {
                self.echo_replies_sent += 1;
            } else |_| {}
            return true;
        }

        /// Look for an ARP reply from `ip` while draining everything else.
        fn pumpArp(self: *Self, ip: [4]u8) !?[6]u8 {
            const n = try self.dev.pollEthFrame(&self.rx);
            if (n == 0) return null;
            const frame = self.rx[0..n];

            const a = net.parseArp(frame) catch {
                self.frames_ignored += 1;
                return null;
            };
            if (a.op == net.ArpOp.request) {
                _ = self.serviceArp(frame);
                return null;
            }
            if (!std.mem.eql(u8, &a.sender_ip, &ip)) {
                self.frames_ignored += 1;
                return null;
            }
            return a.sender_mac;
        }

        fn onLink(self: *Self, ip: [4]u8) bool {
            for (ip, self.lease.ip, self.lease.mask) |d, o, m| {
                if ((d & m) != (o & m)) return false;
            }
            return true;
        }
    };
}

fn isZero(bytes: []const u8) bool {
    for (bytes) |b| {
        if (b != 0) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Host unit tests
//
// The device is faked at the **frame** level rather than the bus level: the
// bus is already covered by `cyw43.zig`'s `AssertingBus`, and what is under
// test here is which frames go out and which of the ones coming back are
// recognised. The fake answers DHCP and ARP the way the network does, so a
// filter that accepts anything, or a client that ignores the transaction id,
// fails rather than passes.
// ---------------------------------------------------------------------------

const testing = std.testing;

const test_mac = [6]u8{ 0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F };
const gw_mac = [6]u8{ 0xE4, 0x5E, 0x1B, 0xFF, 0xEE, 0xD1 };
const gw_ip = [4]u8{ 192, 168, 86, 1 };
const offered_ip = [4]u8{ 192, 168, 86, 42 };

/// A network: a DHCP server, an ARP peer, and as much unrelated broadcast
/// traffic as a test wants to put in front of the client.
const FakeNet = struct {
    sent: [16][1514]u8 = undefined,
    sent_len: [16]usize = @splat(0),
    sent_count: usize = 0,

    /// **Sixteen, and the size matters.** A test that stages more frames than
    /// this used to wrap the ring silently and hand the stack a mixture of new
    /// frames and stale ones — which reads as a draining bug in the code under
    /// test rather than as a broken fixture. `enqueue` panics now instead.
    queue: [16][1514]u8 = undefined,
    queue_len: [16]usize = @splat(0),
    queue_head: usize = 0,
    queue_tail: usize = 0,

    /// Answer DHCP as a server would.
    serve_dhcp: bool = true,
    serve_dns: bool = true,
    dns_answer: [4]u8 = .{ 93, 184, 216, 34 },
    dns_rcode: u4 = 0,
    dns_wrong_id: bool = false,

    serve_http: bool = true,
    /// When set, the fake acts as a **client** instead of a server: it opens a
    /// connection, sends `http_request`, and collects what comes back.
    act_as_client: bool = false,
    client_isn: u32 = 0,
    client_rcv: u32 = 0,
    client_got: [512]u8 = @splat(0),
    client_got_len: usize = 0,
    client_closed: bool = false,
    http_request: []const u8 = "GET /status HTTP/1.0\r\nHost: board\r\n\r\n",
    http_isn: u32 = 0,
    http_rcv: u32 = 0,
    http_request_seen: bool = false,
    http_head: []const u8 = "HTTP/1.0 200 OK\r\nServer: fake\r\nContent-Length: 13\r\n\r\n",
    http_body: []const u8 = "hello, world!",
    /// Answer ARP requests for the gateway.
    serve_arp: bool = true,
    /// Reply to the REQUEST with NAK instead of ACK.
    nak: bool = false,
    /// Answer with a transaction id that is not the client's — the case that
    /// matters, because these are broadcast and genuinely arrive.
    wrong_xid: bool = false,

    delay_total_ms: u32 = 0,

    /// Milliseconds each `pollEthFrame` costs, charged to the clock.
    ///
    /// **This is what makes rung N1 falsifiable.** The old timebase counted
    /// loop iterations and assumed each was a millisecond; a network that kept
    /// `pump` busy therefore stretched every timeout by however long the work
    /// took, invisibly. Set this and a deadline-bounded `service` still returns
    /// on time while an iteration-counting one runs five times too long.
    frame_cost_ms: u32 = 0,

    /// The last REQUEST's `ciaddr`, and whether it was unicast — the two things
    /// that tell a renewal from an initial request.
    last_ciaddr: [4]u8 = @splat(0),
    last_was_unicast: bool = false,
    requests_seen: u32 = 0,

    fn delayMs(self: *FakeNet, ms: u32) void {
        self.delay_total_ms += ms;
    }

    /// **The fake clock is the fake sleep.** Time advances only when the stack
    /// says it slept, which is what makes a renewal test deterministic — and
    /// it also means a stack that stops calling `delayMs` stops time, so a
    /// runaway loop shows up as a hang in the test rather than as a pass.
    fn nowMs(self: *FakeNet) u64 {
        return self.delay_total_ms;
    }

    fn enqueue(self: *FakeNet, frame: []const u8) void {
        const next = (self.queue_tail + 1) % self.queue.len;
        if (next == self.queue_head) @panic("FakeNet queue full — raise its size rather than wrapping");
        @memcpy(self.queue[self.queue_tail][0..frame.len], frame);
        self.queue_len[self.queue_tail] = frame.len;
        self.queue_tail = next;
    }

    fn sendEthFrame(self: *FakeNet, frame: []const u8) !void {
        if (self.sent_count < self.sent.len) {
            @memcpy(self.sent[self.sent_count][0..frame.len], frame);
            self.sent_len[self.sent_count] = frame.len;
            self.sent_count += 1;
        }

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

        if (self.serve_http) {
            if (tcp.parseSegment(frame, gw_ip)) |seg| {
                self.answerTcp(seg);
                return;
            } else |_| {}
        }

        const d = net.parseUdpFrame(frame) catch return;

        if (d.dest_port == dns.port) {
            self.answerDns(d);
            return;
        }

        if (!self.serve_dhcp) return;
        if (d.dest_port != dhcp.server_port) return;
        self.last_was_unicast = !std.mem.eql(u8, &d.dest_ip, &net.broadcast_ip);
        self.answerDhcp(d.payload);
    }

    /// A server at `gw_ip:80` that completes a handshake, answers one GET and
    /// closes — enough to prove a client, and no more.
    ///
    /// **It sends the response as two segments**, because that is what a real
    /// one does: a captured TCP session shows a 2048-byte write leaving as
    /// 1460 + 588. A fake that answered in one would let a client that only
    /// ever reads a single segment pass.
    fn answerTcp(self: *FakeNet, seg: tcp.Segment) void {
        if (seg.has(tcp.Flag.rst)) return;
        if (self.act_as_client) return self.driveClient(seg);

        if (seg.has(tcp.Flag.syn) and !seg.has(tcp.Flag.ack)) {
            self.http_isn = 900_000;
            self.http_rcv = seg.seq +% 1;
            self.emitTcp(seg, tcp.Flag.syn | tcp.Flag.ack, self.http_isn, self.http_rcv, &.{});
            self.http_isn +%= 1;
            return;
        }

        if (seg.payload.len != 0) {
            self.http_rcv = seg.seq +% @as(u32, @intCast(seg.payload.len));
            self.http_request_seen = true;
            // Acknowledge, then answer in two segments, then close.
            self.emitTcp(seg, tcp.Flag.ack, self.http_isn, self.http_rcv, &.{});

            const head = self.http_head;
            const body = self.http_body;
            self.emitTcp(seg, tcp.Flag.ack | tcp.Flag.psh, self.http_isn, self.http_rcv, head);
            self.http_isn +%= @intCast(head.len);
            self.emitTcp(seg, tcp.Flag.ack | tcp.Flag.psh, self.http_isn, self.http_rcv, body);
            self.http_isn +%= @intCast(body.len);
            self.emitTcp(seg, tcp.Flag.fin | tcp.Flag.ack, self.http_isn, self.http_rcv, &.{});
            self.http_isn +%= 1;
            return;
        }

        if (seg.has(tcp.Flag.fin)) {
            self.http_rcv = seg.seq +% 1;
            self.emitTcp(seg, tcp.Flag.ack, self.http_isn, self.http_rcv, &.{});
        }
    }

    /// The client half: answer the board's SYN-ACK, send a request, collect
    /// the response.
    fn driveClient(self: *FakeNet, seg: tcp.Segment) void {
        if (seg.has(tcp.Flag.syn) and seg.has(tcp.Flag.ack)) {
            self.client_rcv = seg.seq +% 1;
            self.client_isn +%= 1; // our SYN consumed one
            self.emitTcp(seg, tcp.Flag.ack, self.client_isn, self.client_rcv, &.{});
            self.emitTcp(seg, tcp.Flag.ack | tcp.Flag.psh, self.client_isn, self.client_rcv, self.http_request);
            self.client_isn +%= @intCast(self.http_request.len);
            return;
        }
        if (seg.payload.len != 0) {
            self.client_rcv = seg.seq +% @as(u32, @intCast(seg.payload.len));
            const room = self.client_got.len - self.client_got_len;
            const take = @min(seg.payload.len, room);
            @memcpy(self.client_got[self.client_got_len..][0..take], seg.payload[0..take]);
            self.client_got_len += take;
            self.emitTcp(seg, tcp.Flag.ack, self.client_isn, self.client_rcv, &.{});
            return;
        }
        if (seg.has(tcp.Flag.fin)) {
            self.client_rcv = seg.seq +% 1;
            self.client_closed = true;
            self.emitTcp(seg, tcp.Flag.ack, self.client_isn, self.client_rcv, &.{});
        }
    }

    /// Open a connection to the board, as a client does.
    fn clientConnect(self: *FakeNet, dest_port: u16) void {
        self.client_isn = 4_000_000;
        var frame: [128]u8 = undefined;
        const f = tcp.buildSegment(.{
            .dest_mac = test_mac,
            .src_mac = gw_mac,
            .src_ip = gw_ip,
            .dest_ip = offered_ip,
            .src_port = 40_000,
            .dest_port = dest_port,
            .seq = self.client_isn,
            .ack = 0,
            .flags = tcp.Flag.syn,
            .window = 65535,
            .mss = tcp.ethernet_mss,
        }, &frame) catch return;
        self.enqueue(f);
    }

    fn emitTcp(self: *FakeNet, to: tcp.Segment, flags: u8, seq: u32, ack_no: u32, payload: []const u8) void {
        var frame: [1514]u8 = undefined;
        const f = tcp.buildSegment(.{
            .dest_mac = test_mac,
            .src_mac = gw_mac,
            .src_ip = gw_ip,
            .dest_ip = to.src_ip,
            .src_port = to.dest_port,
            .dest_port = to.src_port,
            .seq = seq,
            .ack = ack_no,
            .flags = flags,
            .window = 65535,
            .mss = if (flags & tcp.Flag.syn != 0) tcp.ethernet_mss else null,
            .payload = payload,
        }, &frame) catch return;
        self.enqueue(f);
    }

    /// Answer a DNS query the way the resolver does — **including the
    /// compression pointer**, because a fake that emitted inline names would
    /// let a parser that cannot follow a pointer pass every test here.
    fn answerDns(self: *FakeNet, d: net.UdpDatagram) void {
        if (!self.serve_dns) return;
        if (d.payload.len < dns.header_len) return;

        var out: [512]u8 = undefined;
        const qlen = d.payload.len;
        @memcpy(out[0..qlen], d.payload);
        // QR=1, RD=1, RA=1, and the rcode this fake was told to give.
        std.mem.writeInt(u16, out[2..4], 0x8180 | @as(u16, self.dns_rcode), .big);

        if (self.dns_rcode != 0) {
            std.mem.writeInt(u16, out[6..8], 0, .big);
            self.enqueueUdp(out[0..qlen], d);
            return;
        }

        std.mem.writeInt(u16, out[6..8], 1, .big); // one answer
        var w = qlen;
        out[w] = 0xC0; // pointer to the question's name
        out[w + 1] = 0x0C;
        std.mem.writeInt(u16, out[w + 2 ..][0..2], dns.Type.a, .big);
        std.mem.writeInt(u16, out[w + 4 ..][0..2], dns.class_in, .big);
        std.mem.writeInt(u32, out[w + 6 ..][0..4], 60, .big);
        std.mem.writeInt(u16, out[w + 10 ..][0..2], 4, .big);
        @memcpy(out[w + 12 ..][0..4], &self.dns_answer);
        w += 16;

        // **The id is the fake's to corrupt.** A resolver that ignores it
        // accepts any answer that happens to arrive on its port.
        if (self.dns_wrong_id) std.mem.writeInt(u16, out[0..2], 0xDEAD, .big);
        self.enqueueUdp(out[0..w], d);
    }

    /// Wrap a payload as a UDP datagram back to whoever sent `d`.
    fn enqueueUdp(self: *FakeNet, payload: []const u8, d: net.UdpDatagram) void {
        var frame: [1514]u8 = undefined;
        const f = net.buildUdpFrame(.{
            .dest_mac = test_mac,
            .src_mac = gw_mac,
            .src_ip = d.dest_ip,
            .dest_ip = d.src_ip,
            .src_port = d.dest_port,
            .dest_port = d.src_port,
            .payload = payload,
        }, &frame) catch return;
        self.enqueue(f);
    }

    fn answerDhcp(self: *FakeNet, msg: []const u8) void {
        if (msg.len < dhcp.header_len) return;
        const xid = std.mem.readInt(u32, msg[4..8], .big);
        const kind: dhcp.MessageType = @fromBackingInt(@intCast(msg[dhcp.header_len + 2]));

        if (kind == .request) {
            self.requests_seen += 1;
            @memcpy(&self.last_ciaddr, msg[12..16]);
        }

        const reply: dhcp.MessageType = switch (kind) {
            .discover => .offer,
            .request => if (self.nak) .nak else .ack,
            else => return,
        };

        var payload: [dhcp.max_message_len]u8 = @splat(0);
        payload[0] = 2; // BOOTREPLY
        payload[1] = 1;
        payload[2] = 6;
        std.mem.writeInt(u32, payload[4..8], if (self.wrong_xid) xid ^ 0xFFFF else xid, .big);
        @memcpy(payload[16..20], &offered_ip);
        @memcpy(payload[28..34], msg[28..34]);
        @memcpy(payload[236..240], &[_]u8{ 0x63, 0x82, 0x53, 0x63 });
        const opts = [_]u8{
            53,  1,    @backingInt(reply),
            1,   4,    255,
            255, 255,  0,
            3,   4,    192,
            168, 86,   1,
            // **Option 6, the resolver.** Added at rung N9, and its absence
            // until then is why `lease.dns` had never been exercised by a
            // test: the parser read the option and nothing ever sent one.
            6,   4,    192,
            168, 86,   1,
            54,  4,    192,
            168, 86,   1,
            51,  4,    0,
            0,   0x0E, 0x10,
            255,
        };
        @memcpy(payload[dhcp.header_len..][0..opts.len], &opts);

        var frame: [1514]u8 = undefined;
        // Broadcast, as a server answering the broadcast flag does.
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

    fn pollEthFrame(self: *FakeNet, buf: []u8) !usize {
        self.delay_total_ms += self.frame_cost_ms;
        if (self.queue_head == self.queue_tail) return 0;
        const n = self.queue_len[self.queue_head];
        @memcpy(buf[0..n], self.queue[self.queue_head][0..n]);
        self.queue_head = (self.queue_head + 1) % self.queue.len;
        return n;
    }

    fn lastSent(self: *FakeNet) []const u8 {
        return self.sent[self.sent_count - 1][0..self.sent_len[self.sent_count - 1]];
    }
};

/// Traffic that is not ours: a broadcast UDP datagram on some other port, the
/// kind of thing a real network produces constantly.
fn noiseFrame(out: []u8, port: u16) []u8 {
    return net.buildUdpFrame(.{
        .dest_mac = net.broadcast_mac,
        .src_mac = .{ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF },
        .src_ip = .{ 192, 168, 86, 77 },
        .dest_ip = net.broadcast_ip,
        .src_port = port,
        .dest_port = port,
        .payload = "not for you",
    }, out) catch unreachable;
}

test "the DHCP exchange is two frames out, two back, and a usable lease" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);

    try ip.acquireLease(4000);

    try testing.expect(ip.have_lease);
    try testing.expectEqual(offered_ip, ip.lease.ip);
    try testing.expectEqual([4]u8{ 255, 255, 255, 0 }, ip.lease.mask);
    try testing.expectEqual(gw_ip, ip.lease.gateway);
    try testing.expectEqual(@as(u32, 3600), ip.lease.lease_secs);

    // Two frames out: DISCOVER then REQUEST. A third would mean the OFFER was
    // missed and the exchange restarted.
    try testing.expectEqual(@as(usize, 2), fake.sent_count);
    try testing.expectEqual(@as(u32, 1), ip.dhcp_attempts);

    // Both go out from 0.0.0.0 to 255.255.255.255 — the REQUEST too, because
    // the offered address is not ours until the ACK.
    for (0..2) |i| {
        const d = try net.parseUdpFrame(fake.sent[i][0..fake.sent_len[i]]);
        try testing.expectEqual(net.any_ip, d.src_ip);
        try testing.expectEqual(net.broadcast_ip, d.dest_ip);
        try testing.expectEqual(dhcp.server_port, d.dest_port);
        try testing.expectEqual(dhcp.client_port, d.src_port);
    }
}

test "another client's DHCP reply never becomes our lease" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);

    // Every reply carries a transaction id that is not ours — which is what a
    // neighbouring client's exchange looks like, and these are broadcast, so
    // they genuinely arrive here. A client matching on shape would take the
    // first one and configure itself with someone else's address.
    fake.wrong_xid = true;

    // No address is the right outcome. Having one would be the wrong one.
    try testing.expectError(Error.NoDhcpOffer, ip.acquireLease(400));
    try testing.expect(!ip.have_lease);
    try testing.expectEqual(net.any_ip, ip.lease.ip);
    // Four attempts, each with its own transaction id, all refused.
    try testing.expectEqual(@as(u32, 4), ip.dhcp_attempts);
    try testing.expect(ip.frames_ignored > 0);
}

test "a NAK is reported as a refusal, not as silence" {
    var fake = FakeNet{};
    fake.nak = true;
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x99);

    // The distinction matters to the caller: a NAK means the server is there
    // and said no, which is not the same as no server at all.
    try testing.expectError(Error.DhcpNak, ip.acquireLease(4000));
    try testing.expect(!ip.have_lease);
}

test "each DHCP attempt uses a new transaction id" {
    var fake = FakeNet{};
    fake.serve_dhcp = false;
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);

    try testing.expectError(Error.NoDhcpOffer, ip.acquireLease(400));

    // A stale reply to the first attempt must not satisfy the fourth. Four
    // DISCOVERs went out; no two carry the same id.
    try testing.expectEqual(@as(usize, 4), fake.sent_count);
    var seen: [4]u32 = undefined;
    for (0..4) |i| {
        const d = try net.parseUdpFrame(fake.sent[i][0..fake.sent_len[i]]);
        seen[i] = std.mem.readInt(u32, d.payload[4..8], .big);
    }
    for (0..4) |i| {
        for (i + 1..4) |j| try testing.expect(seen[i] != seen[j]);
    }
}

test "the gateway's hardware address is learned, not assumed" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // Before ARP it is zero rather than broadcast: an unresolved gateway has to
    // be distinguishable from a resolved one, and broadcast looks resolved.
    try testing.expectEqual([6]u8{ 0, 0, 0, 0, 0, 0 }, ip.gateway_mac);
    try ip.resolveGateway(1000);
    try testing.expectEqual(gw_mac, ip.gateway_mac);
}

test "an off-link datagram goes to the gateway's MAC with the remote address intact" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    const ntp_server = [4]u8{ 129, 6, 15, 28 };
    try ip.sendUdp(ntp_server, 123, 12345, &.{ 0x1B, 0, 0, 0 });

    const f = fake.lastSent();
    // **Addressed to the router, destined for the server.** Getting this
    // backwards — the remote MAC does not exist locally — produces a frame that
    // goes nowhere and no error at all.
    try testing.expectEqualSlices(u8, &gw_mac, f[0..6]);
    try testing.expectEqualSlices(u8, &test_mac, f[6..12]);

    const d = try net.parseUdpFrame(f);
    try testing.expectEqual(offered_ip, d.src_ip);
    try testing.expectEqual(ntp_server, d.dest_ip);
    try testing.expectEqual(@as(u16, 123), d.dest_port);
}

test "sending before there is an address is refused" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);

    // The alternative is a frame sourced from 0.0.0.0 that no reply can reach.
    try testing.expectError(Error.NoAddress, ip.sendUdp(.{ 8, 8, 8, 8 }, 53, 1234, "x"));
    try testing.expectEqual(@as(usize, 0), fake.sent_count);
}

test "an ARP request for our address is answered during a wait" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    fake.serve_arp = false;
    fake.sent_count = 0;

    // The gateway asking who has our address — which is what happens when its
    // cache expires, and where a client that never answers goes quiet.
    var req_buf: [net.arp_frame_len]u8 = undefined;
    const req = try net.buildArpRequest(gw_mac, gw_ip, offered_ip, &req_buf);
    fake.enqueue(req);

    var buf: [64]u8 = undefined;
    try testing.expectEqual(@as(?Received, null), try ip.awaitUdp(123, &buf, 5));

    try testing.expectEqual(@as(u32, 1), ip.arp_replies_sent);
    const reply = try net.parseArpReply(fake.lastSent());
    try testing.expectEqual(test_mac, reply.sender_mac);
    try testing.expectEqual(offered_ip, reply.sender_ip);
    try testing.expectEqual(gw_mac, reply.target_mac);
}

test "an idle board still answers ARP, which is the whole point of servicing" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    fake.serve_arp = false;
    fake.sent_count = 0;

    // Not inside a wait for anything — this is the board with nothing to do,
    // which is where it spends almost all of its time and where the first
    // version of this file stopped answering. Hardware caught that; this test
    // is the version that would have.
    var req_buf: [net.arp_frame_len]u8 = undefined;
    fake.enqueue(try net.buildArpRequest(gw_mac, gw_ip, offered_ip, &req_buf));

    try ip.service(5);

    try testing.expectEqual(@as(u32, 1), ip.arp_replies_sent);
    const reply = try net.parseArpReply(fake.lastSent());
    try testing.expectEqual(test_mac, reply.sender_mac);
    try testing.expectEqual(offered_ip, reply.sender_ip);
}

test "an ARP request for somebody else's address is not answered" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    fake.serve_arp = false;
    fake.sent_count = 0;

    var req_buf: [net.arp_frame_len]u8 = undefined;
    const req = try net.buildArpRequest(gw_mac, gw_ip, .{ 192, 168, 86, 99 }, &req_buf);
    fake.enqueue(req);

    var buf: [64]u8 = undefined;
    _ = try ip.awaitUdp(123, &buf, 5);
    try testing.expectEqual(@as(u32, 1), ip.arp_requests_seen);
    try testing.expectEqual(@as(u32, 0), ip.arp_replies_sent);
    try testing.expectEqual(@as(usize, 0), fake.sent_count);
}

test "somebody else's traffic is counted, and ours is the one returned" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // Three frames of noise, then the answer. A client that parsed the first
    // frame it saw would report the noise as its reply.
    var noise: [1514]u8 = undefined;
    fake.enqueue(noiseFrame(&noise, 5353)); // mDNS
    fake.enqueue(noiseFrame(&noise, 1900)); // SSDP
    var req_buf: [net.arp_frame_len]u8 = undefined;
    fake.enqueue(try net.buildArpRequest(gw_mac, gw_ip, .{ 192, 168, 86, 98 }, &req_buf));

    var answer: [1514]u8 = undefined;
    const a = try net.buildUdpFrame(.{
        .dest_mac = test_mac,
        .src_mac = gw_mac,
        .src_ip = .{ 129, 6, 15, 28 },
        .dest_ip = offered_ip,
        .src_port = 123,
        .dest_port = 12345,
        .payload = &.{ 0x24, 3, 0, 0 },
    }, &answer);
    fake.enqueue(a);

    var buf: [64]u8 = undefined;
    const got = (try ip.awaitUdp(12345, &buf, 50)) orelse return error.NothingReceived;
    try testing.expectEqual(@as(usize, 4), got.len);
    try testing.expectEqual([4]u8{ 129, 6, 15, 28 }, got.src_ip);
    try testing.expectEqual(@as(u16, 123), got.src_port);
    try testing.expectEqualSlices(u8, &.{ 0x24, 3, 0, 0 }, buf[0..got.len]);
    try testing.expect(ip.frames_ignored >= 2);
}

// ---------------------------------------------------------------------------
// Rung 18 — keeping the lease
// ---------------------------------------------------------------------------

/// A leased stack with the renewal timer shortened, which is what `-Drenew`
/// does on the board. Servicing for a second then crosses T1.
fn leasedWithRenewAfter(fake: *FakeNet, secs: u32) !IpStack(FakeNet) {
    var ip = IpStack(FakeNet).init(fake, test_mac, 0x1234);
    ip.renew_after_secs = secs;
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);
    fake.sent_count = 0;
    fake.requests_seen = 0;
    return ip;
}

test "the lease is renewed at T1, unicast, with the address in ciaddr" {
    var fake = FakeNet{};
    var ip = try leasedWithRenewAfter(&fake, 1);

    // Not yet: half a second of service is not T1.
    try ip.service(500);
    try testing.expectEqual(@as(u32, 0), ip.renewals);
    try testing.expectEqual(@as(u32, 0), fake.requests_seen);

    // Past it.
    try ip.service(600);
    try testing.expectEqual(@as(u32, 1), ip.renewals);
    try testing.expectEqual(@as(u32, 1), fake.requests_seen);

    // **Unicast, with the address in the header.** Broadcasting here, or
    // putting the address in option 50, is the initial REQUEST — which mostly
    // works, which is why it needs a test rather than a reading.
    try testing.expect(fake.last_was_unicast);
    try testing.expectEqual(offered_ip, fake.last_ciaddr);
    try testing.expectEqual(@as(u32, 0), ip.rebinds);

    // The address is unchanged and the clock restarted.
    try testing.expectEqual(offered_ip, ip.lease.ip);
    try testing.expect(ip.have_lease);
    try testing.expect(ip.leaseAgeMs() < 1000);
}

test "a renewal that goes unanswered is retried, and the address stays usable" {
    var fake = FakeNet{};
    // **Five seconds, not one, and the reason is rung N1.** An unanswered
    // renewal waits 2 s for an ACK, and that 2 s is now charged to the lease
    // clock because the clock is real. With a 1 s window the lease expires
    // *during* the attempt, so the old version of this test was asserting
    // something no longer physically true: t1 = 5 s and expiry = 10 s leaves
    // room for the attempt to fail and the address to still be valid.
    var ip = try leasedWithRenewAfter(&fake, 5);

    // The server goes quiet. T1 has passed; the lease has not expired.
    fake.serve_dhcp = false;
    try ip.service(5100);

    // **The address is still ours.** A server that misses T1 is not a reason to
    // stop using an address that is valid until expiry — this is the difference
    // between a failed renewal and a lost lease.
    try testing.expect(ip.have_lease);
    try testing.expect(!ip.lease_lost);
    try testing.expectEqual(offered_ip, ip.lease.ip);
    try testing.expect(ip.renew_failures > 0);
    try testing.expectEqual(@as(u32, 0), ip.renewals);
}

test "past T2 the renewal is broadcast instead of unicast" {
    var fake = FakeNet{};
    var ip = try leasedWithRenewAfter(&fake, 20);

    // **T2 has to be more than one failed attempt away from expiry.** Each
    // unanswered renewal costs 2 s of real clock, and the T2..expiry window is
    // a quarter of the lease — so with a short window every attempt steps over
    // it and the rebind path is unreachable. At 20 s: T1 = 20 s, T2 = 35 s,
    // expiry = 40 s, and the retries land at 22, 24 … 36, which is inside it.
    // The same arithmetic says **`-Drenew` below about 3 s cannot exercise
    // this path on the board either**, which is worth knowing before believing
    // a run that never rebinds.
    fake.serve_dhcp = false;
    try ip.service(37_000);
    try testing.expect(ip.rebinds > 0);

    fake.serve_dhcp = true;
    try ip.service(100);
    try testing.expect(!fake.last_was_unicast);
    try testing.expectEqual(@as(u32, 1), ip.renewals);
}

test "a NAK takes the address away rather than leaving it half-held" {
    var fake = FakeNet{};
    var ip = try leasedWithRenewAfter(&fake, 1);

    // The server refusing is different from the server being quiet: it knows
    // about this address and says it is not ours.
    fake.nak = true;
    try ip.service(1100);

    try testing.expect(!ip.have_lease);
    try testing.expect(ip.lease_lost);
    // And sending is refused, rather than going out from an address that has
    // been taken back.
    try testing.expectError(Error.NoAddress, ip.sendUdp(.{ 8, 8, 8, 8 }, 53, 1234, "x"));
}

test "a lease that runs out is dropped, not used forever" {
    var fake = FakeNet{};
    var ip = try leasedWithRenewAfter(&fake, 1);

    // Nobody answers, ever. T1, then T2, then expiry at 2 s.
    fake.serve_dhcp = false;
    try ip.service(2100);

    // **Expiry is not a formality.** The server may have given this address to
    // somebody else by now, so continuing to use it would be squatting.
    try testing.expect(!ip.have_lease);
    try testing.expect(ip.lease_lost);
}

// ---------------------------------------------------------------------------
// Rung N1 — a real clock, and a poll that does not sleep
// ---------------------------------------------------------------------------

test "one poll drains a burst, rather than taking one frame from it" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    const before = ip.arp_replies_sent;

    // Four ARP requests for our address, all waiting before anybody polls —
    // which is what a receive ring makes possible and a single slot did not.
    var req_buf: [net.arp_frame_len]u8 = undefined;
    for (0..4) |_| {
        fake.enqueue(try net.buildArpRequest(gw_mac, gw_ip, offered_ip, &req_buf));
    }

    try ip.poll();

    // **All four answered by one poll.** Taking one frame per poll would leave
    // three sitting in the ring for the next caller, and on a device whose ring
    // is four deep the burst after that overflows.
    try testing.expectEqual(before + 4, ip.arp_replies_sent);
    try testing.expectEqual(@as(u32, 0), ip.drain_limited);
}

test "draining is bounded, and says when it stopped early" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // More than `max_drain` waiting. **A poll must hand control back** — "read
    // until empty" against a device that keeps producing is a loop with no
    // exit, and an application that never returns from `poll` is
    // indistinguishable from a hang.
    var req_buf: [net.arp_frame_len]u8 = undefined;
    for (0..max_drain + 2) |_| {
        fake.enqueue(try net.buildArpRequest(gw_mac, gw_ip, offered_ip, &req_buf));
    }

    const before = ip.arp_replies_sent;
    try ip.poll();

    try testing.expectEqual(before + max_drain, ip.arp_replies_sent);
    // And it is counted, because falling behind should be visible rather than
    // inferred from a rate that looks slightly wrong.
    try testing.expectEqual(@as(u32, 1), ip.drain_limited);
}

test "a ping is answered from poll, not only while something is waiting" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    fake.sent_count = 0;

    var req: [128]u8 = undefined;
    fake.enqueue(net.testEchoRequest(0x1234, 9, "ping-payload", offered_ip, &req));

    // **`poll`, not `awaitUdp`.** The ARP responder that only ran while the
    // application happened to be waiting is this package's most-remembered bug;
    // an echo responder with the same shape would be the same bug again.
    try ip.poll();

    try testing.expectEqual(@as(u32, 1), ip.echo_requests_seen);
    try testing.expectEqual(@as(u32, 1), ip.echo_replies_sent);

    const sent = fake.lastSent();
    const icmp = sent[net.eth_header_len + net.ipv4_header_len ..];
    try testing.expectEqual(net.IcmpType.echo_reply, icmp[0]);
    try testing.expectEqual(@as(u16, 0x1234), std.mem.readInt(u16, icmp[4..6], .big));
    try testing.expectEqual(@as(u16, 9), std.mem.readInt(u16, icmp[6..8], .big));
    try testing.expectEqualSlices(u8, "ping-payload", icmp[net.icmp_header_len..]);
    try testing.expectEqual(@as(u16, 0), net.computeChecksum(icmp));
}

test "a ping for somebody else is ignored, not answered" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    fake.sent_count = 0;

    var req: [128]u8 = undefined;
    fake.enqueue(net.testEchoRequest(1, 1, "x", .{ 192, 168, 86, 99 }, &req));
    try ip.poll();

    // **Not answered, and not swallowed either.** A frame that is not ours must
    // fall through to the counters, or a stack that quietly ate other people's
    // traffic would look identical to one that never received it.
    try testing.expectEqual(@as(u32, 0), ip.echo_replies_sent);
    try testing.expectEqual(@as(u32, 0), ip.echo_requests_seen);
    try testing.expectEqual(@as(usize, 0), fake.sent_count);
    try testing.expectEqual(@as(u32, 1), ip.frames_ignored);
}

test "a name resolves, over the DNS server the lease named" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    // **The lease is where the resolver comes from**, which is the whole
    // reason this works with no configuration: the router that handed out an
    // address also said who to ask.
    try testing.expect(!isZero(&ip.lease.dns));

    const addr = try ip.resolveName("example.com", 3000);
    try testing.expectEqual([4]u8{ 93, 184, 216, 34 }, addr);
    try testing.expectEqual(@as(u32, 1), ip.dns_answers);
}

test "an answer with somebody else's id is not ours" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    // Every DNS datagram arriving on our port parses perfectly; **the
    // transaction id is what makes one of them an answer to our question**,
    // exactly as `xid` does for DHCP. Without the check, a late reply to an
    // earlier query — or somebody else's — is adopted as this one's.
    fake.dns_wrong_id = true;
    try testing.expectError(Error.DnsTimeout, ip.resolveName("example.com", 600));
    try testing.expect(ip.dns_queries > 1); // it retried rather than accepting
    try testing.expectEqual(@as(u32, 0), ip.dns_answers);
}

test "a name that does not exist fails differently from silence" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    // **NXDOMAIN is an answer.** Reporting it as a timeout would make a typo
    // indistinguishable from a resolver that is down, and a caller retrying a
    // misspelled name forever is what conflating them produces.
    fake.dns_rcode = 3;
    try testing.expectError(Error.NameNotFound, ip.resolveName("nope.invalid", 3000));
    try testing.expectEqual(@as(u32, 1), ip.dns_answers);

    // Silence is the other one, and it retries first.
    fake.serve_dns = false;
    try testing.expectError(Error.DnsTimeout, ip.resolveName("example.com", 600));
}

test "resolving without a lease is refused rather than attempted" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);

    // No address to send from, and no resolver to send to. **Both are refused
    // by name**, because "it timed out" would send the caller looking at the
    // network.
    try testing.expectError(Error.NoAddress, ip.resolveName("example.com", 100));

    try ip.acquireLease(4000);
    ip.lease.dns = @splat(0);
    try testing.expectError(Error.NoDnsServer, ip.resolveName("example.com", 100));
}

test "a GET completes: connect, request, response, close" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    try ip.connect(gw_ip, 80, 50080, 4000);
    try testing.expectEqual(tcp.State.established, ip.conn.state);

    var req: [128]u8 = undefined;
    const get = try http.buildGet("example.com", "/", &req);
    try ip.tcpSend(get, 4000);
    try testing.expect(fake.http_request_seen);

    // **The response arrives as two segments**, as a real server sends it —
    // a captured TCP session has a 2048-byte write leaving as 1460 + 588.
    // A client that read one segment and stopped would pass against a fake
    // that answered in one, and fail against anything real.
    var buf: [512]u8 = undefined;
    const n = try ip.tcpRecv(&buf, 4000);

    const r = try http.parseResponse(buf[0..n]);
    try testing.expectEqual(@as(u16, 200), r.status);
    try testing.expectEqual(@as(u64, 13), r.body.length);
    try testing.expectEqualStrings("hello, world!", buf[r.body_at..n]);

    // The server closed, so the connection followed it into CLOSE-WAIT.
    try testing.expect(ip.conn.peer_closed);
    ip.tcpClose();
}

test "a connection to a port nothing listens on is refused, not waited out" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    // Nothing answers the SYN at all. **Bounded, and it reports** — the
    // failure this stack refuses to have is a caller left waiting on a host
    // that is not there.
    fake.serve_http = false;
    // The state machine gives up after four SYNs, which is sooner than this
    // call's own deadline — so **the error is the connection's, not the
    // wrapper's**, and it says which one ran out.
    try testing.expectError(tcp.Error.ConnectTimeout, ip.connect(gw_ip, 80, 50080, 8000));
    try testing.expectEqual(tcp.State.closed, ip.conn.state);
}

test "a second send while one is outstanding is refused" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);
    try ip.connect(gw_ip, 80, 50080, 4000);

    // Stop-and-wait, surfaced at the stack's own API rather than only inside
    // the state machine.
    _ = ip.conn.send("first", ip.dev.nowMs());
    try testing.expectError(Error.NotWritable, ip.tcpSend("second", 100));
}

test "segments for a connection this stack has not opened fall through" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    const before = ip.frames_ignored;

    // **Not ours, and not swallowed.** A stack that silently ate other
    // people's segments would look identical to one that never received them,
    // and the counters are what make the difference visible.
    var frame: [128]u8 = undefined;
    const seg = try tcp.buildSegment(.{
        .dest_mac = test_mac,
        .src_mac = gw_mac,
        .src_ip = gw_ip,
        .dest_ip = offered_ip,
        .src_port = 1234,
        .dest_port = 5678,
        .seq = 1,
        .ack = 1,
        .flags = tcp.Flag.ack,
        .window = 100,
    }, &frame);
    fake.enqueue(seg);
    try ip.poll();

    try testing.expectEqual(before + 1, ip.frames_ignored);
}

test "a route is served: accept, read the request, answer, close" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    fake.act_as_client = true;
    ip.listenTcp(80);
    fake.clientConnect(80);

    try ip.acceptTcp(4000);
    try testing.expectEqual(tcp.State.established, ip.conn.state);

    // Read the request. **The client is a separate machine from the server**,
    // so this exercises N8's accept path rather than N6's connect path — the
    // two reach `established` by different routes and only one of them had
    // ever been driven end to end.
    var req_buf: [512]u8 = undefined;
    const n = try ip.tcpRecv(&req_buf, 4000);
    const req = try http.parseRequest(req_buf[0..n]);
    try testing.expectEqualStrings("GET", req.method);
    try testing.expectEqualStrings("/status", req.target);

    var res_buf: [256]u8 = undefined;
    const res = try http.buildResponse(200, "OK", "text/plain", "board ok", &res_buf);
    try ip.tcpSend(res, 4000);
    ip.tcpClose();

    // And the client got a whole, parseable response back.
    const got = try http.parseResponse(fake.client_got[0..fake.client_got_len]);
    try testing.expectEqual(@as(u16, 200), got.status);
    try testing.expectEqual(@as(u64, 8), got.body.length);
    try testing.expectEqualStrings("board ok", fake.client_got[got.body_at..fake.client_got_len]);
}

test "a listener survives a client that connects and resets" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);
    try ip.resolveGateway(1000);

    ip.listenTcp(80);

    // **The fake answers nothing**, so the handshake stalls where it is wanted.
    // Driving it as a client instead would not work: `poll` drains eight
    // frames a call, so the SYN, our SYN-ACK and the client's reply would all
    // be consumed by one call and the connection would be established before
    // this test could look at it.
    fake.serve_http = false;
    var syn_frame: [128]u8 = undefined;
    const syn_seg = try tcp.buildSegment(.{
        .dest_mac = test_mac,
        .src_mac = gw_mac,
        .src_ip = gw_ip,
        .dest_ip = offered_ip,
        .src_port = 40_000,
        .dest_port = 80,
        .seq = 4_000_000,
        .ack = 0,
        .flags = tcp.Flag.syn,
        .window = 65535,
        .mss = tcp.ethernet_mss,
    }, &syn_frame);
    fake.enqueue(syn_seg);
    try ip.poll();
    try testing.expectEqual(tcp.State.syn_received, ip.conn.state);

    // The client changes its mind. **Back to listening, and no error to the
    // caller** — a server that closed here would stop serving because one
    // client went away, which is N8's rule seen from the stack rather than
    // from the state machine.
    var frame: [128]u8 = undefined;
    const rst_seg = try tcp.buildSegment(.{
        .dest_mac = test_mac,
        .src_mac = gw_mac,
        .src_ip = gw_ip,
        .dest_ip = offered_ip,
        .src_port = 40_000,
        .dest_port = 80,
        .seq = 4_000_001,
        .ack = 0,
        .flags = tcp.Flag.rst,
        .window = 0,
    }, &frame);
    fake.enqueue(rst_seg);
    try ip.poll();

    try testing.expectEqual(tcp.State.listen, ip.conn.state);
    try testing.expectEqual(@as(?anyerror, null), ip.tcp_error);
}

test "poll does not sleep" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // **The whole point of the primitive.** An alarm handler calls this, and a
    // handler that sleeps is not a handler. Anything that made `poll` wait —
    // a `delayMs` for politeness, a retry loop — moves this number.
    const before = fake.nowMs();
    try ip.poll();
    try ip.poll();
    try ip.poll();
    try testing.expectEqual(before, fake.nowMs());
}

test "service is bounded by the clock, not by an iteration count" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // Four milliseconds of work per frame read — a busy network, or a slow
    // bus. **The old timebase counted iterations and assumed each was a
    // millisecond**, so this would run 1000 iterations at 5 ms each and
    // `service(1000)` would take five seconds while reporting one.
    fake.frame_cost_ms = 4;

    const before = fake.nowMs();
    try ip.service(1000);
    const elapsed = fake.nowMs() - before;

    // Bounded above by the request plus one final pass; bounded below because
    // it must actually wait rather than returning early.
    try testing.expect(elapsed >= 1000);
    try testing.expect(elapsed < 1100);
}

test "the lease ages across a gap in servicing" {
    var fake = FakeNet{};
    var ip = try leasedWithRenewAfter(&fake, 30);
    try testing.expect(ip.leaseAgeMs() < 1000);

    // An application that went off and did something else for ten seconds
    // without servicing. **The lease is ten seconds older**, because age is
    // now the difference between two clock readings rather than a count of
    // the milliseconds this stack was told to sleep.
    //
    // The accumulator it replaced would read zero here, and a board coming
    // back from a long unserviced stretch would think its lease was fresh.
    fake.delay_total_ms += 10_000;
    try testing.expect(ip.leaseAgeMs() >= 10_000);
}

test "T1 and T2 are RFC 2131's fractions of the lease, not round numbers" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // The fake grants 3600 s, so T1 is 1800 s and T2 is 3150 s.
    try testing.expectEqual(@as(u32, 3600), ip.lease.lease_secs);
    try testing.expectEqual(@as(u64, 1_800_000), ip.t1());
    try testing.expectEqual(@as(u64, 3_150_000), ip.t2());
    try testing.expectEqual(@as(u64, 3_600_000), ip.expiry());

    // And the override displaces all three together, or the ordering breaks.
    ip.renew_after_secs = 30;
    try testing.expect(ip.t1() < ip.t2());
    try testing.expect(ip.t2() < ip.expiry());
}

test "a datagram for a different host on our port is not ours" {
    var fake = FakeNet{};
    var ip = IpStack(FakeNet).init(&fake, test_mac, 0x1234);
    try ip.acquireLease(4000);

    // Right port, wrong destination address. This is the frame a filter that
    // checks only the port accepts.
    var other: [1514]u8 = undefined;
    fake.enqueue(try net.buildUdpFrame(.{
        .dest_mac = test_mac,
        .src_mac = gw_mac,
        .src_ip = .{ 129, 6, 15, 28 },
        .dest_ip = .{ 192, 168, 86, 43 },
        .src_port = 123,
        .dest_port = 12345,
        .payload = &.{ 0x24, 3, 0, 0 },
    }, &other));

    var buf: [64]u8 = undefined;
    try testing.expectEqual(@as(?Received, null), try ip.awaitUdp(12345, &buf, 5));
    try testing.expect(ip.frames_ignored >= 1);
}
