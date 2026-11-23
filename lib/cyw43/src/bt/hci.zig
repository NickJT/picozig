//! HCI commands and events — rungs 19 and 20.
//!
//! `btbus.zig` moves packets; this decides what is in them. It is the same
//! split as `net.zig` against `ipstack.zig`: the formatting and parsing here
//! are pure functions over bytes, and `Host` below is the small stateful part
//! that drives a device with them.
//!
//! ## The event mask is the trap
//!
//! **A controller does not report an LE connection unless it has been told
//! it may.** The default Event_Mask is `0x00001FFF_FFFFFFFF` — bits 0 to 44 —
//! and the LE Meta event is **bit 61**, so it is masked off in a controller
//! that has just been reset. Every LE event, including the one that says a
//! connection was established, arrives inside an LE Meta event.
//!
//! The failure that follows is quiet and misleading: the phone connects, the
//! controller knows, advertising stops because a connection was accepted, and
//! the host sits waiting for an event that was never going to come. It looks
//! like the connection failed. It did not.
//!
//! ## Framing
//!
//! Packets carry `btbus.packet_header_len` bytes in front, with the H4 type in
//! byte 3 — see `btbus.zig`. So an event's own header starts at byte 4:
//!
//!     [4]      event code
//!     [5]      parameter length
//!     [6...]   parameters

const std = @import("std");
const btbus = @import("btbus.zig");
const l2cap = @import("l2cap.zig");
const att = @import("att.zig");

/// Opcodes, as OGF and OCF already combined.
pub const op = struct {
    pub const reset: u16 = 0x0C03;
    pub const set_event_mask: u16 = 0x0C01;
    pub const read_local_version: u16 = 0x1001;
    pub const read_bd_addr: u16 = 0x1009;

    pub const le_set_event_mask: u16 = 0x2001;
    pub const le_read_buffer_size: u16 = 0x2002;
    pub const le_set_adv_params: u16 = 0x2006;
    pub const le_set_adv_data: u16 = 0x2008;
    pub const le_set_scan_rsp_data: u16 = 0x2009;
    pub const le_set_adv_enable: u16 = 0x200A;
    pub const disconnect: u16 = 0x0406;
};

pub const event = struct {
    pub const disconnection_complete: u8 = 0x05;
    pub const command_complete: u8 = 0x0E;
    pub const command_status: u8 = 0x0F;
    pub const number_of_completed_packets: u8 = 0x13;
    pub const le_meta: u8 = 0x3E;
};

pub const le_subevent = struct {
    pub const connection_complete: u8 = 0x01;
    pub const advertising_report: u8 = 0x02;
    pub const connection_update_complete: u8 = 0x03;
    pub const long_term_key_request: u8 = 0x05;
};

/// Bits 0 to 61. **Bit 61 is the LE Meta event** and is the reason this command
/// is sent at all; the rest are set because a controller that reports more than
/// we read is harmless, and one that reports less is a silent hang.
pub const event_mask_all: u64 = 0x3FFF_FFFF_FFFF_FFFF;

/// Bits 0 to 4: connection complete, advertising report, connection update,
/// read remote features, long-term key request. This is also the controller's
/// own default, and it is set explicitly because "the default happens to be
/// right" is not a thing this driver relies on.
pub const le_event_mask_default: u64 = 0x1F;

pub const AdvType = enum(u8) {
    /// Connectable and scannable, undirected. Rung 19 needs this one.
    ind = 0x00,
    direct_ind_high = 0x01,
    scan_ind = 0x02,
    /// Non-connectable, undirected — rung 17b, which had nothing to connect to.
    nonconn_ind = 0x03,
    direct_ind_low = 0x04,
};

pub const Error = error{
    /// The controller answered, and said no.
    CommandFailed,
    /// Nothing came back in the time allowed.
    NoAnswer,
    /// A reply arrived for a command we did not send.
    WrongOpcode,
    /// A packet too short to be the event it claims to be.
    Malformed,
    /// An ATT PDU was offered with nothing connected to send it on.
    NotConnected,
};

// ---------------------------------------------------------------------------
// Building
// ---------------------------------------------------------------------------

/// Room for the **largest legal** HCI command: the four-byte bus header, the
/// three-byte command header, and a full 255-byte parameter block. The length
/// field is a `u8`, so nothing can be bigger and this size can never be short
/// again.
///
/// **It used to be 64 bytes of parameters, and rung 21 found that on
/// hardware.** Every LE command this driver sends fits easily — the largest is
/// `LE_Set_Advertising_Data` at 32 — so the ceiling was invisible for four
/// rungs. `Write_Local_Name` is a *fixed 248-byte field*, and it overflowed,
/// and `buildCommand` returned `error.BufferTooSmall`. That error is not in
/// `bredr-probe`'s `die()` table, so the board halted reporting code 99 —
/// "something not on the list" — with no UART attached and nothing else to say
/// which of four commands had failed. It cost a flash cycle to narrow by
/// elimination from the watch block.
///
/// Two lessons, both already applied: **size a buffer to the protocol's
/// maximum rather than to the caller you have**, and a `die()` table that
/// omits an error its call graph can produce is a debugging instrument with a
/// hole in it.
pub const command_buf_len: usize = btbus.packet_header_len + 3 + 255;

/// The largest ACL payload a BR/EDR link on this controller carries, as
/// `Read_Buffer_Size` reported it at rung 21.
pub const bredr_acl_data_max: usize = 1021;

/// Room for the largest packet the bus can hand back: the four-byte bus header,
/// the four-byte HCI ACL header, and a full BR/EDR ACL payload. Events are far
/// smaller — 2 + 255 at most — so this is set by the ACL path.
///
/// **It was 260 bytes, and that was an LE number.** LE ACL payloads on this
/// chip are 251, so 260 fitted with four bytes to spare and nothing in rungs 17
/// to 20 could exceed it. A BR/EDR packet is up to 1021, and every one of them
/// would have come back `PacketTooLarge` — *and* desynchronised the ring on the
/// way out, because the header is consumed before the size is known. `btbus`
/// now resynchronises; this is the half that stops it happening at all.
pub const rx_buf_len: usize = btbus.packet_header_len + acl_header_len + bredr_acl_data_max;

/// Write a command into `out`, which must have room for the four-byte bus
/// header, the three-byte command header, and the parameters. Returns the slice
/// to hand to `btbus.write`.
pub fn buildCommand(opcode: u16, params: []const u8, out: []u8) ![]u8 {
    const total = btbus.packet_header_len + 3 + params.len;
    if (out.len < total) return error.BufferTooSmall;
    // Bytes 0..2 are the bus length, filled in by `btbus.write` — only the
    // packet type belongs to us here.
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = @backingInt(btbus.PacketType.command);
    out[4] = @truncate(opcode);
    out[5] = @truncate(opcode >> 8);
    out[6] = @intCast(params.len);
    @memcpy(out[7..][0..params.len], params);
    return out[0..total];
}

/// The fifteen bytes of `LE_Set_Advertising_Parameters`.
pub fn advParams(kind: AdvType, min_interval: u16, max_interval: u16) [15]u8 {
    var p: [15]u8 = @splat(0);
    std.mem.writeInt(u16, p[0..2], min_interval, .little);
    std.mem.writeInt(u16, p[2..4], max_interval, .little);
    p[4] = @backingInt(kind);
    p[5] = 0x00; // own address type: public, which is the BD_ADDR the chip has
    p[6] = 0x00; // peer address type — unused unless directed
    // p[7..13] peer address, zero
    p[13] = 0x07; // all three advertising channels
    p[14] = 0x00; // no filtering: anyone may scan or connect
    return p;
}

/// The 32 bytes of `LE_Set_Advertising_Data`: a significant-length byte and a
/// fixed 31-byte field, so short data is **padded rather than truncated**.
pub fn advData(name: []const u8, flags: u8) ![32]u8 {
    var d: [32]u8 = @splat(0);
    var n: usize = 1;

    // AD structure: flags. Scanners filter on this.
    d[n] = 2;
    d[n + 1] = 0x01;
    d[n + 2] = flags;
    n += 3;

    // AD structure: complete local name.
    if (1 + name.len > 255 or n + 2 + name.len > d.len) return error.BufferTooSmall;
    d[n] = @intCast(1 + name.len);
    d[n + 1] = 0x09;
    @memcpy(d[n + 2 ..][0..name.len], name);
    n += 2 + name.len;

    d[0] = @intCast(n - 1); // significant length, not counting itself
    return d;
}

// ---------------------------------------------------------------------------
// ACL data
// ---------------------------------------------------------------------------

/// The HCI ACL header: a handle with two flag fields packed above it, then the
/// payload length.
///
/// **The handle is twelve bits, not sixteen.** Bits 12–13 are the packet
/// boundary flag and 14–15 the broadcast flag, so a host that writes the handle
/// as a plain `u16` is right only while the flags happen to be zero — which is
/// exactly the case for the first fragment of every packet, so the bug survives
/// every test that never fragments.
pub const acl_header_len: usize = 4;

/// The packet boundary flag, bits 12–13 of the handle field.
///
/// **`continuation` was `0b00` here until 2026-08-27, and that was wrong.**
/// `0b00` is a *first* fragment — the non-automatically-flushable one — and
/// `0b01` is the continuation. Nothing ever built a `.continuation` packet, so
/// no wrong packet was ever sent; what the mislabel would have broken is the
/// first code to **route** on the flag, which is exactly what `bt-speaker`'s
/// `handleAcl` now does. It was caught by reading a real trace (`boundary
/// 0b01`) against this enum and finding they disagreed.
///
/// The authority is btstack's own incoming handler, `acl_handler` in its
/// `src/hci.c` (as vendored in pico-sdk): `0x01` continuation, `0x02`
/// first fragment, **anything else rejected as invalid packet boundary flags**.
/// The Bluetooth core specification is not in `reference/`; btstack is a
/// certified stack and its constants are being trusted in its place.
///
/// **The values are not symmetric between directions.** `0b00` is host to
/// controller only, so it should never arrive on a packet we are parsing.
pub const Boundary = enum(u2) {
    /// First fragment, **non**-automatically-flushable. Host to controller
    /// only, and only when the controller sets the LMP feature bit that
    /// `bredr.Features.nonFlushablePacketBoundary` reads.
    first_non_flushable = 0b00,
    /// A continuation of an L2CAP message already begun. **It carries no L2CAP
    /// header**, so it must never reach `l2cap.parse`.
    continuation = 0b01,
    /// First fragment, automatically flushable. What this stack sends, and what
    /// a peer's first fragment arrives as.
    first_flushable = 0b10,
    /// A complete L2CAP PDU. LE only — it should not appear on a BR/EDR link.
    complete_le = 0b11,
};

/// Build an ACL data packet around an already-assembled L2CAP payload.
pub fn buildAcl(handle: u16, boundary: Boundary, payload: []const u8, out: []u8) ![]u8 {
    const total = btbus.packet_header_len + acl_header_len + payload.len;
    if (out.len < total) return error.BufferTooSmall;

    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = @backingInt(btbus.PacketType.acl);
    const hf: u16 = (handle & 0x0FFF) | (@as(u16, @backingInt(boundary)) << 12);
    std.mem.writeInt(u16, out[4..6], hf, .little);
    std.mem.writeInt(u16, out[6..8], @intCast(payload.len), .little);
    @memcpy(out[8..][0..payload.len], payload);
    return out[0..total];
}

pub const Acl = struct {
    handle: u16,
    /// **The enum rather than a `u2`**, deliberately. The mislabel above went
    /// unnoticed because the one caller comparing this field did so against a
    /// raw bit pattern; all four values are named, so the conversion is total
    /// and a caller can `switch` without a bare `0b01` anywhere.
    boundary: Boundary,
    payload: []const u8,
};

pub fn parseAcl(pkt: []const u8) !Acl {
    if (pkt.len < btbus.packet_header_len + acl_header_len) return Error.Malformed;
    if (pkt[3] != @backingInt(btbus.PacketType.acl)) return Error.Malformed;

    const hf = std.mem.readInt(u16, pkt[4..6], .little);
    const len: usize = std.mem.readInt(u16, pkt[6..8], .little);
    if (btbus.packet_header_len + acl_header_len + len > pkt.len) return Error.Malformed;

    return .{
        .handle = hf & 0x0FFF,
        .boundary = @fromBackingInt(@as(u2, @truncate(hf >> 12))),
        .payload = pkt[8..][0..len],
    };
}

/// Reassemble an L2CAP PDU from the HCI ACL fragments it arrived in.
///
/// **Its absence silently destroyed audio.** Nothing here reassembled until
/// 2026-08-27, and a fragmented PDU was lost twice over: the first fragment
/// failed `l2cap.parse` (its header declares a length the fragment does not
/// carry) and every continuation failed too (a continuation has no header at
/// all). On a soak that showed as 665 lost PDUs and 435,328 frames of silence
/// padded into the stream — the two agree to about 2% — heard as burbling that
/// worsened as the link did. It was invisible because **fragmentation only
/// appears when the link degrades**: the same phone, on a good link, sends
/// every media packet whole and the counters stay at zero.
///
/// `max_pdu` is the largest PDU that can arrive, which is a property of the
/// MTU this device advertises plus the L2CAP header — not of any packet seen so
/// far. Callers pass `local_mtu + l2cap.header_len`.
///
/// Modelled on btstack's `acl_handler` (`src/hci.c`),
/// including its two sanity checks: a continuation with nothing in flight, and
/// a message larger than the buffer.
pub fn Reassembler(comptime max_pdu: usize) type {
    return struct {
        const Self = @This();
        pub const capacity: usize = max_pdu;

        buf: [max_pdu]u8 = undefined,
        /// Bytes accumulated so far. `want` of 0 means nothing is in flight.
        len: usize = 0,
        want: usize = 0,

        /// A continuation arrived with no first fragment in flight. btstack
        /// logs and drops; so do we. Non-zero means fragments are being lost
        /// before this ever sees them.
        orphans: u32 = 0,
        /// A PDU declared larger than `capacity`, or fragments overrunning it.
        /// Non-zero means `max_pdu` is wrong, not that the peer misbehaved.
        overflows: u32 = 0,
        /// PDUs that actually needed reassembling — the count that says whether
        /// this type is earning its place on a given link.
        assembled: u32 = 0,
        /// Boundary values a BR/EDR controller should never send us: `0b00` is
        /// host-to-controller only and `0b11` is LE. Counted rather than
        /// trusted, but still processed — dropping audio over a flag we merely
        /// disapprove of would be the worse failure.
        unexpected: u32 = 0,

        /// Returns a complete L2CAP PDU (header included) or null while still
        /// accumulating.
        ///
        /// **The result is valid only until the next `push`**: a whole-in-one
        /// fragment aliases the caller's packet buffer and a reassembled one
        /// aliases `buf`. Every caller parses it immediately, which is why this
        /// returns a slice rather than copying again.
        pub fn push(self: *Self, a: Acl) ?[]const u8 {
            switch (a.boundary) {
                .continuation => {
                    if (self.want == 0) {
                        self.orphans += 1;
                        return null;
                    }
                    if (self.len + a.payload.len > self.want) {
                        self.overflows += 1;
                        self.discard();
                        return null;
                    }
                    @memcpy(self.buf[self.len..][0..a.payload.len], a.payload);
                    self.len += a.payload.len;
                    if (self.len < self.want) return null;
                    const out = self.buf[0..self.len];
                    self.assembled += 1;
                    self.discard();
                    return out;
                },
                .first_non_flushable, .complete_le => {
                    self.unexpected += 1;
                    return self.begin(a.payload);
                },
                .first_flushable => return self.begin(a.payload),
            }
        }

        /// Forget anything in flight. Called on a disconnect: a half-assembled
        /// PDU belongs to a link that has ended, and carrying it into the next
        /// one would splice two messages together.
        pub fn discard(self: *Self) void {
            self.len = 0;
            self.want = 0;
        }

        /// A first fragment abandons whatever was in flight — btstack does the
        /// same, because the alternative is to splice unrelated messages.
        fn begin(self: *Self, payload: []const u8) ?[]const u8 {
            self.discard();
            // Too short to even hold the length field: nothing can be done with
            // it, and guessing would produce a frame rather than a dropped one.
            if (payload.len < l2cap.header_len) return null;
            const declared: usize = std.mem.readInt(u16, payload[0..2], .little);
            const total = l2cap.header_len + declared;
            // Already whole — the common case on a healthy link, and it costs
            // no copy at all.
            if (payload.len >= total) return payload[0..total];
            if (total > capacity) {
                self.overflows += 1;
                return null;
            }
            @memcpy(self.buf[0..payload.len], payload);
            self.len = payload.len;
            self.want = total;
            return null;
        }
    };
}

/// `LE_Read_Buffer_Size`'s answer: how big a payload the controller will take,
/// and how many it will hold.
///
/// **This is the ACL path's version of the SDPCM credit problem**: send past
/// the credits the chip granted and it drops what it cannot hold. The
/// controller accepts packets until its buffers are full and then drops them,
/// and `Number_Of_Completed_Packets` events are how it says one has gone out.
/// A host that transmits without counting works right up until it sends two
/// notifications quickly.
pub const BufferSize = struct {
    packet_len: u16,
    packet_count: u8,
};

pub fn parseBufferSize(params: []const u8) !BufferSize {
    if (params.len < 3) return Error.Malformed;
    return .{
        .packet_len = std.mem.readInt(u16, params[0..2], .little),
        .packet_count = params[2],
    };
}

pub const CommandStatus = struct {
    status: u8,
    opcode: u16,
    /// `Num_HCI_Command_Packets`, as on `CommandComplete`.
    credits: u8,
};

/// A Command Status event — **the answer to commands that cannot complete
/// immediately**, and a whole class of BR/EDR commands use it instead of
/// Command Complete: `Accept_Connection_Request`, `Authentication_Requested`,
/// `Set_Connection_Encryption`, `Disconnect`, `Create_Connection`.
///
/// Nothing parsed this until rung 22, because every command rungs 17–20 sent
/// answers with Command Complete. `commandWithResult` matches only that, so a
/// host that reaches for it here waits out its 400 attempts and reports
/// `NoAnswer` for a command the controller accepted — the failure looking
/// exactly like a controller that ignored it.
///
/// **Status 0 here means "started", not "succeeded".** The result arrives later
/// as its own event; for `Accept_Connection_Request` that is
/// `Connection_Complete`.
pub fn parseCommandStatus(ev: Event) !CommandStatus {
    if (ev.code != event.command_status) return Error.WrongOpcode;
    if (ev.params.len < 4) return Error.Malformed;
    return .{
        .status = ev.params[0],
        .opcode = std.mem.readInt(u16, ev.params[2..4], .little),
        .credits = ev.params[1],
    };
}

/// How many packets the `Number_Of_Completed_Packets` event says have left.
pub fn completedPackets(ev: Event) u16 {
    if (ev.code != event.number_of_completed_packets) return 0;
    if (ev.params.len < 5) return 0;
    var total: u16 = 0;
    const handles: usize = ev.params[0];
    var i: usize = 0;
    while (i < handles and 1 + i * 4 + 4 <= ev.params.len) : (i += 1) {
        total += std.mem.readInt(u16, ev.params[1 + handles * 2 + i * 2 ..][0..2], .little);
    }
    return total;
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

pub const Event = struct {
    code: u8,
    params: []const u8,
};

/// Pull an event out of a packet as `btbus.read` returned it.
pub fn parseEvent(pkt: []const u8) !Event {
    if (pkt.len < btbus.packet_header_len + 2) return Error.Malformed;
    if (pkt[3] != @backingInt(btbus.PacketType.event)) return Error.Malformed;
    const len: usize = pkt[5];
    if (btbus.packet_header_len + 2 + len > pkt.len) return Error.Malformed;
    return .{ .code = pkt[4], .params = pkt[6..][0..len] };
}

pub const CommandComplete = struct {
    opcode: u16,
    status: u8,
    /// `Num_HCI_Command_Packets` — how many commands the Host may now have
    /// outstanding. **It was sitting in `params[0]` being read past.** Feed it
    /// to `Host.creditsFrom`; see `Host.cmd_credits`.
    credits: u8,
    /// Whatever the command returns after its status byte.
    params: []const u8,
};

/// A Command Complete's parameters: how many commands the controller will now
/// accept, the opcode being answered, then the status and the rest.
///
/// **The echoed opcode is what ties the answer to the question.** Commands are
/// not necessarily answered in order, and a host that assumes they are will
/// eventually read one command's status as another's.
pub fn parseCommandComplete(ev: Event) !CommandComplete {
    if (ev.code != event.command_complete) return Error.WrongOpcode;
    if (ev.params.len < 4) return Error.Malformed;
    return .{
        .opcode = std.mem.readInt(u16, ev.params[1..3], .little),
        .status = ev.params[3],
        .credits = ev.params[0],
        .params = ev.params[4..],
    };
}

pub const Connection = struct {
    handle: u16,
    /// 0 is central, 1 is peripheral. This board is always the peripheral.
    role: u8,
    peer_addr_type: u8,
    peer_addr: [6]u8,
    interval: u16,
    latency: u16,
    timeout: u16,
};

/// `LE_Connection_Complete`, inside an LE Meta event.
pub fn parseConnectionComplete(ev: Event) !?Connection {
    if (ev.code != event.le_meta) return null;
    if (ev.params.len < 1) return Error.Malformed;
    if (ev.params[0] != le_subevent.connection_complete) return null;
    if (ev.params.len < 19) return Error.Malformed;
    if (ev.params[1] != 0) return Error.CommandFailed;

    var c: Connection = undefined;
    c.handle = std.mem.readInt(u16, ev.params[2..4], .little) & 0x0FFF;
    c.role = ev.params[4];
    c.peer_addr_type = ev.params[5];
    @memcpy(&c.peer_addr, ev.params[6..12]);
    c.interval = std.mem.readInt(u16, ev.params[12..14], .little);
    c.latency = std.mem.readInt(u16, ev.params[14..16], .little);
    c.timeout = std.mem.readInt(u16, ev.params[16..18], .little);
    return c;
}

pub const Disconnection = struct {
    handle: u16,
    reason: u8,
};

pub fn parseDisconnection(ev: Event) !?Disconnection {
    if (ev.code != event.disconnection_complete) return null;
    if (ev.params.len < 4) return Error.Malformed;
    return .{
        .handle = std.mem.readInt(u16, ev.params[1..3], .little) & 0x0FFF,
        .reason = ev.params[3],
    };
}

// ---------------------------------------------------------------------------
// The host
// ---------------------------------------------------------------------------

/// The stateful half: sends commands, waits for their answers, and keeps track
/// of whether anything is connected.
///
/// Generic over the device for the same reason everything else here is — the
/// tests drive it with a fake that never touches a bus.
/// How many commands may wait for a credit. See `Host.cmd_queue`.
pub const cmd_queue_len_max: usize = 8;

/// The largest parameter block any queued command carries. The biggest in use
/// is `Link_Key_Request_Reply` at 22 bytes (6 address + 16 key); 32 is the
/// next round number above it. **Sized to the protocol, not to the commands
/// this application happens to post today.**
pub const max_command_params: usize = 32;

/// A command held until the controller has room for it.
pub const QueuedCommand = struct {
    opcode: u16 = 0,
    len: u8 = 0,
    params: [max_command_params]u8 = @splat(0),
};

pub fn Host(comptime Dev: type) type {
    return struct {
        const Self = @This();

        bt: btbus.BtBus(Dev),

        /// The connection, once there is one. **`null` is the honest state**:
        /// a handle of zero is a perfectly valid handle, so it cannot be the
        /// sentinel.
        conn: ?Connection = null,

        /// Every Command Complete status OR'd together, so one non-zero says a
        /// command was refused without needing a field per command.
        status_all: u8 = 0,
        commands: u32 = 0,
        events: u32 = 0,
        /// Events that arrived while waiting for something else. On a live
        /// connection these are the interesting ones.
        events_unsolicited: u32 = 0,

        /// **HCI command flow control, which nothing here honoured until
        /// 2026-08-27.** The spec is not optional about it (Vol 4 Part E):
        /// "On initial power-on, and after a reset, the Host shall send a
        /// maximum of **one** outstanding HCI Command packet until an
        /// HCI_Command_Complete or HCI_Command_Status event has been
        /// received", and every such event then carries
        /// `Num_HCI_Command_Packets` saying how many may be outstanding from
        /// then on. Zero means stop.
        ///
        /// **Violating it has no error to observe** — the controller does not
        /// refuse, it degrades — which is why this went unnoticed while the
        /// application sent two and later three commands back to back.
        ///
        /// One is the spec's own starting value, and the only safe assumption
        /// before any event has arrived.
        cmd_credits: u8 = 1,

        /// Commands built but not yet sent, because no credit was free.
        ///
        /// **Eight, sized to the largest burst with headroom.** The paging path
        /// alone posts three back to back — `write_scan_enable`,
        /// `authentication_requested`, `switch_role` — and a queue that cannot
        /// hold a burst drops exactly the commands a busy moment produces.
        cmd_queue: [cmd_queue_len_max]QueuedCommand = @splat(.{}),
        cmd_queued: usize = 0,
        /// Commands lost because the queue was full. **Non-zero is a fault**,
        /// not backpressure: it means a burst outgrew the queue.
        cmd_dropped: u32 = 0,
        /// Commands that had to wait for a credit rather than going straight
        /// out. The number that says whether this machinery is doing anything.
        cmd_delayed: u32 = 0,

        /// How many `HCI_Reset` attempts the last `start` needed. **One at
        /// boot; more than one means the controller was restarted and needed
        /// time to become answerable.**
        reset_attempts_used: u32 = 0,

        /// ACL flow control. **The controller's buffers, not ours** — read from
        /// `LE_Read_Buffer_Size` at start-up and returned by
        /// `Number_Of_Completed_Packets`. One is the safe floor if the
        /// controller reports nothing.
        ///
        /// **These defaults are a floor, not a measurement of this chip**, and
        /// the distinction was worth a correction: 27 is the LE minimum from
        /// the spec, while this controller reports **251 × 8** on LE and
        /// **1021 × 8** on BR/EDR. `start` overwrites them on every real run.
        /// Anything reasoning about "27-byte packets" on this hardware is
        /// reasoning about a fallback that has never been used.
        acl_credits: u16 = 1,
        acl_max: u16 = 1,
        acl_payload_max: u16 = 27,
        /// Set once `useClassicBuffers` has been called, so a caller can tell
        /// the figures above apart from the LE ones they replaced.
        acl_is_classic: bool = false,

        att_sent: u32 = 0,
        att_received: u32 = 0,
        att_other_channel: u32 = 0,

        /// Room for a command, and for the largest event the controller sends.
        buf: [command_buf_len]u8 = @splat(0),
        rx: [rx_buf_len]u8 = @splat(0),

        pub fn init(dev: *Dev) Self {
            return .{ .bt = btbus.BtBus(Dev).init(dev) };
        }

        /// Bring the controller up: reset, then **unmask the events**, which is
        /// the step whose absence looks like a broken connection rather than a
        /// masked event. See the header.
        /// How many times `HCI_Reset` is offered before giving up, and how long
        /// to wait between.
        ///
        /// **`fw_ready` is not the same as "ready for HCI".** `awaitReady`
        /// returns the instant that bit goes high, and at boot the controller
        /// answers the first command anyway — so nothing here needed retrying
        /// until 2026-08-27, when a *mid-session* restart raised `fw_ready` and
        /// then returned `NoAnswer` to `HCI_Reset`. Boot still costs one
        /// attempt; only the restart pays.
        pub const reset_attempts: u32 = 6;
        pub const reset_settle_ms: u32 = 100;

        pub fn start(self: *Self, patchram: []const u8, timeout_ms: u32) !void {
            try self.bt.start(patchram, timeout_ms);

            var attempt: u32 = 0;
            while (true) : (attempt += 1) {
                if (self.command(op.reset, &.{})) {
                    break;
                } else |err| {
                    if (attempt + 1 >= reset_attempts) return err;
                    self.bt.dev.delayMs(reset_settle_ms);
                }
            }
            self.reset_attempts_used = attempt + 1;

            var mask: [8]u8 = undefined;
            std.mem.writeInt(u64, &mask, event_mask_all, .little);
            try self.command(op.set_event_mask, &mask);

            std.mem.writeInt(u64, &mask, le_event_mask_default, .little);
            try self.command(op.le_set_event_mask, &mask);

            // How much the controller will hold. Asking is cheap and assuming
            // is the SDPCM credit failure one layer up.
            const bs = self.commandWithResult(op.le_read_buffer_size, &.{}) catch return;
            if (parseBufferSize(bs)) |size| {
                if (size.packet_count > 0) {
                    self.acl_max = size.packet_count;
                    self.acl_credits = size.packet_count;
                }
                if (size.packet_len > 0) self.acl_payload_max = size.packet_len;
            } else |_| {}
        }

        /// Adopt the BR/EDR ACL figures from `Read_Buffer_Size` (0x1005), which
        /// `start` does not issue: it asks `LE_Read_Buffer_Size`, and on a
        /// Classic link that describes the wrong buffers.
        ///
        /// **It lives here rather than in `start` on purpose.** `bredr.zig`
        /// sits above this file and owns the Classic parsing; having `start`
        /// reach up for it would make the two import each other. The caller
        /// reads, parses, and hands the answer down.
        ///
        /// Returns `false` if the controller's ACL payload is larger than `rx`
        /// can hold — **a receive path that would drop packets, reported rather
        /// than discovered later as corruption.** Nothing on this board takes
        /// that branch; a controller with bigger buffers would.
        pub fn useClassicBuffers(self: *Self, payload_max: u16, count: u16) bool {
            if (count > 0) {
                self.acl_max = count;
                self.acl_credits = count;
            }
            if (payload_max > 0) self.acl_payload_max = payload_max;
            self.acl_is_classic = true;
            return payload_max <= bredr_acl_data_max;
        }

        /// Send a command and wait for the Command Complete that answers **this
        /// opcode**, returning its parameters after the status byte.
        /// Post a command, honouring `cmd_credits`.
        ///
        /// **Never blocks, deliberately.** Every caller is inside an event loop
        /// that is itself the only thing able to deliver the credit being
        /// waited for, so a blocking wait here would deadlock against the code
        /// that would have released it. Commands that cannot go now are queued
        /// and sent by `pumpCommands` when a credit arrives.
        ///
        /// Order is preserved: once anything is queued, everything queues,
        /// because the controller "shall start performing the commands in the
        /// order in which they are received".
        pub fn postCommand(self: *Self, opcode: u16, params: []const u8) void {
            if (params.len > max_command_params) {
                self.cmd_dropped += 1;
                return;
            }
            if (self.cmd_credits > 0 and self.cmd_queued == 0) {
                self.sendCommandNow(opcode, params);
                return;
            }
            if (self.cmd_queued == cmd_queue_len_max) {
                self.cmd_dropped += 1;
                return;
            }
            var q = &self.cmd_queue[self.cmd_queued];
            q.opcode = opcode;
            q.len = @intCast(params.len);
            @memcpy(q.params[0..params.len], params);
            self.cmd_queued += 1;
            self.cmd_delayed += 1;
        }

        /// Take `Num_HCI_Command_Packets` from a Command Complete or Command
        /// Status and release whatever it now allows.
        ///
        /// **It is an allowance, not an increment** — the spec has the
        /// controller state the current permitted number, and zero means stop
        /// sending entirely.
        pub fn creditsFrom(self: *Self, n: u8) void {
            self.cmd_credits = n;
            self.pumpCommands();
        }

        /// Send whatever the current credits allow. Safe to call every pass.
        pub fn pumpCommands(self: *Self) void {
            while (self.cmd_queued > 0 and self.cmd_credits > 0) {
                const q = self.cmd_queue[0];
                self.sendCommandNow(q.opcode, q.params[0..q.len]);
                // Shift rather than ring: the queue is eight deep and empty
                // almost always, so the copy is cheaper than the index
                // arithmetic is to get wrong.
                var i: usize = 1;
                while (i < self.cmd_queued) : (i += 1) self.cmd_queue[i - 1] = self.cmd_queue[i];
                self.cmd_queued -= 1;
            }
        }

        fn sendCommandNow(self: *Self, opcode: u16, params: []const u8) void {
            // `btbus.packet_header_len` plus the 3-byte HCI command header,
            // exactly as `buildCommand` lays it out.
            var buf: [max_command_params + btbus.packet_header_len + 3]u8 = undefined;
            const pkt = buildCommand(opcode, params, &buf) catch {
                self.cmd_dropped += 1;
                return;
            };
            self.bt.write(pkt) catch {
                self.cmd_dropped += 1;
                return;
            };
            // **Spend the credit even on a write that failed to reach the
            // wire**, because we cannot tell which side of the bus it died on.
            // Assuming it arrived is the conservative half: it costs one slot
            // of throughput, where assuming it did not would send past the
            // controller's allowance, which is the very thing this exists to
            // prevent.
            if (self.cmd_credits > 0) self.cmd_credits -= 1;
            self.commands += 1;
        }

        pub fn command(self: *Self, opcode: u16, params: []const u8) !void {
            _ = try self.commandWithResult(opcode, params);
        }

        pub fn commandWithResult(self: *Self, opcode: u16, params: []const u8) ![]const u8 {
            const pkt = try buildCommand(opcode, params, &self.buf);
            try self.bt.write(pkt);
            self.commands += 1;

            var attempt: u32 = 0;
            while (attempt < 400) : (attempt += 1) {
                const n = try self.bt.read(&self.rx);
                if (n == 0) {
                    self.bt.dev.delayMs(5);
                    continue;
                }
                self.events += 1;
                const ev = parseEvent(self.rx[0..n]) catch continue;

                if (parseCommandComplete(ev)) |cc| {
                    if (cc.opcode != opcode) {
                        // Somebody else's answer. Dropping it rather than
                        // taking it is the same rule the IOCTL path follows.
                        self.events_unsolicited += 1;
                        continue;
                    }
                    self.status_all |= cc.status;
                    if (cc.status != 0) return Error.CommandFailed;
                    return cc.params;
                } else |_| {
                    // Not a Command Complete: an event that arrived while we
                    // were waiting. Fold it in rather than dropping it — a
                    // disconnection during a command is real.
                    self.absorb(ev);
                }
            }
            return Error.NoAnswer;
        }

        /// Start advertising, connectably.
        pub fn advertise(self: *Self, name: []const u8, kind: AdvType) !void {
            const params = advParams(kind, 0x00A0, 0x00F0); // 100 ms to 150 ms
            try self.command(op.le_set_adv_params, &params);

            // 0x06: general discoverable, BR/EDR not supported.
            const data = try advData(name, 0x06);
            try self.command(op.le_set_adv_data, &data);

            try self.command(op.le_set_adv_enable, &.{0x01});
        }

        pub fn stopAdvertising(self: *Self) !void {
            try self.command(op.le_set_adv_enable, &.{0x00});
        }

        /// Read events until something connects, or the time runs out.
        ///
        /// Returns null on timeout rather than erroring: nobody connecting is
        /// an ordinary outcome, and the caller knows how long it waited.
        pub fn awaitConnection(self: *Self, timeout_ms: u32) !?Connection {
            var left = timeout_ms;
            while (left > 0) : (left -|= 5) {
                const n = try self.bt.read(&self.rx);
                if (n == 0) {
                    self.bt.dev.delayMs(5);
                    continue;
                }
                self.events += 1;
                const ev = parseEvent(self.rx[0..n]) catch continue;
                self.absorb(ev);
                if (self.conn) |c| return c;
            }
            return null;
        }

        /// Keep reading for `ms`, folding in whatever arrives. This is the
        /// Bluetooth side's `service()`: a connection that nobody reads from is
        /// a connection that never notices it has been dropped.
        pub fn service(self: *Self, ms: u32) !void {
            var left = ms;
            while (left > 0) : (left -|= 5) {
                const n = try self.bt.read(&self.rx);
                if (n == 0) {
                    self.bt.dev.delayMs(5);
                    continue;
                }
                self.events += 1;
                const ev = parseEvent(self.rx[0..n]) catch continue;
                self.absorb(ev);
            }
        }

        // -- ATT over ACL ---------------------------------------------------

        /// Send an ATT PDU on the current connection.
        ///
        /// **Bounded by the controller's buffers**, which is why `acl_credits`
        /// exists: the controller takes packets until it has no room and then
        /// drops them, and `Number_Of_Completed_Packets` is how it says one has
        /// gone. Transmitting without counting works until two notifications
        /// leave close together — the same failure as the SDPCM credits, one
        /// layer up.
        pub fn sendAtt(self: *Self, pdu: []const u8) !void {
            const conn = self.conn orelse return Error.NotConnected;

            var waited: u32 = 0;
            while (self.acl_credits == 0) {
                if (waited >= 500) return Error.NoAnswer;
                try self.service(5);
                waited += 5;
            }

            var l2: [64]u8 = undefined;
            const frame = try l2cap.build(l2cap.cid.att, pdu, &l2);
            const pkt = try buildAcl(conn.handle, .first_flushable, frame, &self.buf);
            try self.bt.write(pkt);
            self.acl_credits -= 1;
            self.att_sent += 1;
        }

        /// Read one packet and, if it is an ATT request, answer it.
        ///
        /// Returns the handle a client wrote to, so the caller can act on a
        /// write without this layer knowing what the attribute means.
        pub fn serveAtt(self: *Self, server: *att.Server, ms: u32) !?u16 {
            var written: ?u16 = null;
            var left = ms;
            while (left > 0) : (left -|= 5) {
                const n = try self.bt.read(&self.rx);
                if (n == 0) {
                    self.bt.dev.delayMs(5);
                    continue;
                }
                self.events += 1;

                // An event and an ACL packet arrive through the same call and
                // are told apart only by the type byte.
                if (self.rx[3] == @backingInt(btbus.PacketType.event)) {
                    const ev = parseEvent(self.rx[0..n]) catch continue;
                    self.absorb(ev);
                    continue;
                }

                const acl = parseAcl(self.rx[0..n]) catch continue;
                const frame = l2cap.parse(acl.payload) catch continue;
                if (frame.cid != l2cap.cid.att) {
                    // Signalling and SMP arrive here too. Neither is
                    // implemented, and ignoring them is honest — a phone that
                    // tries to pair gets nothing.
                    self.att_other_channel += 1;
                    continue;
                }

                self.att_received += 1;
                const out = server.handle(frame.payload);
                if (out.written_handle) |h| written = h;
                if (out.response.len > 0) try self.sendAtt(out.response);
            }
            return written;
        }

        /// Send a notification, if the client has subscribed.
        pub fn notify(self: *Self, server: *att.Server, handle: u16, value: []const u8) !bool {
            var pdu: [32]u8 = undefined;
            const n = server.notify(handle, value, &pdu) orelse return false;
            try self.sendAtt(n);
            return true;
        }

        /// Fold one event into the host's state.
        fn absorb(self: *Self, ev: Event) void {
            if (ev.code == event.number_of_completed_packets) {
                const done = completedPackets(ev);
                self.acl_credits = @min(self.acl_max, self.acl_credits + done);
                return;
            }

            if (parseConnectionComplete(ev)) |maybe| {
                if (maybe) |c| {
                    self.conn = c;
                    return;
                }
            } else |_| {}

            if (parseDisconnection(ev)) |maybe| {
                if (maybe) |d| {
                    if (self.conn) |c| {
                        if (c.handle == d.handle) self.conn = null;
                    }
                    return;
                }
            } else |_| {}

            self.events_unsolicited += 1;
        }

        pub fn connected(self: *Self) bool {
            return self.conn != null;
        }
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the event mask has bit 61 set, or no LE event ever arrives" {
    // **This is the whole reason `set_event_mask` is sent.** The controller's
    // default is bits 0..44, and every LE event — including the one saying a
    // connection was made — comes inside an LE Meta event at bit 61.
    try testing.expect(event_mask_all & (@as(u64, 1) << 61) != 0);

    var mask: [8]u8 = undefined;
    std.mem.writeInt(u64, &mask, event_mask_all, .little);
    // Little-endian on the wire, so bit 61 is in the last byte.
    try testing.expectEqual(@as(u8, 0x20), mask[7] & 0x20);
}

test "a command is built with the bus header in front and the type in byte 3" {
    var buf: [32]u8 = @splat(0xAA);
    const c = try buildCommand(op.reset, &.{}, &buf);

    try testing.expectEqual(@as(usize, 7), c.len);
    try testing.expectEqual(@as(u8, 0x01), c[3]); // HCI command
    try testing.expectEqual(@as(u8, 0x03), c[4]); // opcode, little-endian
    try testing.expectEqual(@as(u8, 0x0C), c[5]);
    try testing.expectEqual(@as(u8, 0), c[6]); // no parameters

    const with = try buildCommand(op.le_set_adv_enable, &.{0x01}, &buf);
    try testing.expectEqual(@as(usize, 8), with.len);
    try testing.expectEqual(@as(u8, 1), with[6]);
    try testing.expectEqual(@as(u8, 0x01), with[7]);
}

test "advertising parameters say connectable, and 17b's did not" {
    const conn = advParams(.ind, 0x00A0, 0x00F0);
    try testing.expectEqual(@as(u8, 0x00), conn[4]);
    try testing.expectEqual(@as(u8, 0x07), conn[13]); // all three channels
    try testing.expectEqual(@as(u16, 0x00A0), std.mem.readInt(u16, conn[0..2], .little));

    // Rung 17b advertised non-connectably on purpose — there was nothing above
    // HCI to accept a connection. Rung 19 is exactly this byte changing.
    const nonconn = advParams(.nonconn_ind, 0x00A0, 0x00F0);
    try testing.expectEqual(@as(u8, 0x03), nonconn[4]);
}

test "advertising data is padded to 31 bytes, not truncated" {
    const d = try advData("zig-pico2w", 0x06);
    try testing.expectEqual(@as(usize, 32), d.len);
    // Significant length: 3 for the flags structure, 12 for the name.
    try testing.expectEqual(@as(u8, 15), d[0]);
    try testing.expectEqualSlices(u8, &.{ 2, 0x01, 0x06 }, d[1..4]);
    try testing.expectEqual(@as(u8, 11), d[4]);
    try testing.expectEqual(@as(u8, 0x09), d[5]);
    try testing.expectEqualSlices(u8, "zig-pico2w", d[6..16]);
    // And the rest is zero rather than whatever was in the buffer.
    for (d[16..]) |b| try testing.expectEqual(@as(u8, 0), b);
}

/// An event as the chip delivers it, with the four-byte bus header.
fn eventPacket(out: []u8, code: u8, params: []const u8) []u8 {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = @backingInt(btbus.PacketType.event);
    out[4] = code;
    out[5] = @intCast(params.len);
    @memcpy(out[6..][0..params.len], params);
    return out[0 .. 6 + params.len];
}

test "the receive buffer holds a full BR/EDR ACL packet" {
    // 4 bus header + 4 HCI ACL header + 1021 payload. The old 260 could not
    // hold a quarter of one.
    try testing.expectEqual(@as(usize, 4 + 4 + 1021), rx_buf_len);
    try testing.expect(rx_buf_len > 260);
    // And it holds the largest event too, which is a much smaller bound.
    try testing.expect(rx_buf_len >= btbus.packet_header_len + 2 + 255);
}

// -- HCI command flow control ----------------------------------------------

test "Num_HCI_Command_Packets is read, not skipped past" {
    // **The field was in `params[0]` all along**, read past to reach the
    // opcode. Command Complete: credits, opcode, status. Command Status:
    // status, credits, opcode.
    const cc = try parseCommandComplete(.{
        .code = event.command_complete,
        .params = &.{ 0x02, 0x1A, 0x0C, 0x00 },
    });
    try testing.expectEqual(@as(u8, 2), cc.credits);
    try testing.expectEqual(@as(u16, 0x0C1A), cc.opcode);
    try testing.expectEqual(@as(u8, 0), cc.status);

    const cs = try parseCommandStatus(.{
        .code = event.command_status,
        .params = &.{ 0x00, 0x01, 0x0B, 0x08 },
    });
    try testing.expectEqual(@as(u8, 1), cs.credits);
    try testing.expectEqual(@as(u16, 0x080B), cs.opcode);
}

/// A real bus over `btbus`'s own fake chip, so the send path is exercised
/// rather than stubbed — the queue is only half the mechanism.
fn creditHost(chip: *btbus.FakeChip) !Host(btbus.FakeChip) {
    var host = Host(btbus.FakeChip).init(chip);
    host.bt = try btbus.startedBus(chip);
    return host;
}

test "with no credit, commands queue instead of going out" {
    var chip = btbus.FakeChip{};
    var host = try creditHost(&chip);

    // The spec's starting allowance, before any event has arrived.
    try testing.expectEqual(@as(u8, 1), host.cmd_credits);

    // Zero means "stop sending commands", so nothing may reach the bus --
    // which is also what makes this testable without one.
    host.cmd_credits = 0;
    host.postCommand(0x0C1A, &.{0x03});
    host.postCommand(0x080B, &.{ 0x0B, 0x00 });

    try testing.expectEqual(@as(usize, 2), host.cmd_queued);
    try testing.expectEqual(@as(u32, 2), host.cmd_delayed);
    try testing.expectEqual(@as(u32, 0), host.cmd_dropped);
    // Order is preserved: the controller performs them as received.
    try testing.expectEqual(@as(u16, 0x0C1A), host.cmd_queue[0].opcode);
    try testing.expectEqual(@as(u16, 0x080B), host.cmd_queue[1].opcode);
    try testing.expectEqual(@as(u8, 1), host.cmd_queue[0].len);
}

test "a credit releases the queue, in order, and is spent" {
    var chip = btbus.FakeChip{};
    var host = try creditHost(&chip);

    // One credit, three commands -- the paging burst that started all this.
    host.cmd_credits = 1;
    host.postCommand(0x0C1A, &.{0x02}); // write_scan_enable page
    host.postCommand(0x0419, &.{ 0x0B, 0x00 }); // authentication_requested
    host.postCommand(0x080B, &.{ 1, 2, 3, 4, 5, 6, 0 }); // switch_role

    // The first went out and spent the credit; the other two waited. **Before
    // this existed all three went straight to the bus**, two of them past an
    // allowance of one.
    try testing.expectEqual(@as(u8, 0), host.cmd_credits);
    try testing.expectEqual(@as(usize, 2), host.cmd_queued);
    try testing.expectEqual(@as(u32, 1), host.commands);
    try testing.expectEqual(@as(u32, 2), host.cmd_delayed);

    // The controller answers the first and allows one more.
    host.creditsFrom(1);
    try testing.expectEqual(@as(usize, 1), host.cmd_queued);
    try testing.expectEqual(@as(u32, 2), host.commands);
    try testing.expectEqual(@as(u16, 0x080B), host.cmd_queue[0].opcode);

    // An allowance of two drains the rest and leaves the spare credit.
    host.creditsFrom(2);
    try testing.expectEqual(@as(usize, 0), host.cmd_queued);
    try testing.expectEqual(@as(u32, 3), host.commands);
    try testing.expectEqual(@as(u8, 1), host.cmd_credits);
    try testing.expectEqual(@as(u32, 0), host.cmd_dropped);
}

test "zero credits stop sending, as the spec requires" {
    var chip = btbus.FakeChip{};
    var host = try creditHost(&chip);

    host.postCommand(0x0C1A, &.{0x03}); // spends the opening credit
    try testing.expectEqual(@as(u32, 1), host.commands);

    // "If the Controller generates ... Num_HCI_Command_Packets set to zero,
    // then the Host shall stop sending commands."
    host.creditsFrom(0);
    host.postCommand(0x0C1A, &.{0x02});
    try testing.expectEqual(@as(u32, 1), host.commands);
    try testing.expectEqual(@as(usize, 1), host.cmd_queued);

    host.creditsFrom(1);
    try testing.expectEqual(@as(u32, 2), host.commands);
}

test "a burst larger than the queue is counted, not silently lost" {
    var chip = btbus.FakeChip{};
    var host = try creditHost(&chip);
    host.cmd_credits = 0;

    for (0..cmd_queue_len_max + 3) |_| host.postCommand(0x0C1A, &.{0x03});

    try testing.expectEqual(@as(usize, cmd_queue_len_max), host.cmd_queued);
    try testing.expectEqual(@as(u32, 3), host.cmd_dropped);
}

test "a parameter block too large to queue is refused rather than truncated" {
    var chip = btbus.FakeChip{};
    var host = try creditHost(&chip);
    host.cmd_credits = 0;

    const too_big: [max_command_params + 1]u8 = @splat(0);
    host.postCommand(0x0C1A, &too_big);
    try testing.expectEqual(@as(usize, 0), host.cmd_queued);
    try testing.expectEqual(@as(u32, 1), host.cmd_dropped);

    // The largest thing actually posted -- Link_Key_Request_Reply, 6 address
    // bytes plus a 16-byte key -- must fit.
    const link_key_reply: [22]u8 = @splat(0);
    host.postCommand(0x040B, &link_key_reply);
    try testing.expectEqual(@as(usize, 1), host.cmd_queued);
}

test "Classic buffer figures replace the LE ones, and an oversize report is refused" {
    const Fake = struct {
        fn delayMs(_: *@This(), _: u32) void {}
    };
    var fake = Fake{};
    var host = Host(Fake).init(&fake);

    // The LE-shaped floor this starts at, which is not this chip's LE figure
    // either — see the field comment.
    try testing.expectEqual(@as(u16, 27), host.acl_payload_max);
    try testing.expect(!host.acl_is_classic);

    try testing.expect(host.useClassicBuffers(1021, 8));
    try testing.expectEqual(@as(u16, 1021), host.acl_payload_max);
    try testing.expectEqual(@as(u16, 8), host.acl_max);
    try testing.expectEqual(@as(u16, 8), host.acl_credits);
    try testing.expect(host.acl_is_classic);

    // A controller whose packets this driver's `rx` could not hold says so,
    // rather than being found out later as a stream full of holes.
    try testing.expect(!host.useClassicBuffers(2048, 8));
}

test "the command buffer holds Write_Local_Name's 248-byte parameter block" {
    // The regression for the rung 21 halt. `Write_Local_Name` (0x0C13) is the
    // first command this driver has ever sent whose parameters are a fixed
    // wide field rather than a handful of bytes.
    const params: [248]u8 = @splat('x');

    var buf: [command_buf_len]u8 = @splat(0);
    const pkt = try buildCommand(0x0C13, &params, &buf);
    try testing.expectEqual(@as(usize, btbus.packet_header_len + 3 + 248), pkt.len);
    try testing.expectEqual(@as(u8, 248), pkt[6]);

    // **This half is the point**: the same call against the buffer the driver
    // used to carry fails, so the assertion above is checked against the code
    // it was written to catch rather than only against the fix.
    var old: [btbus.packet_header_len + 3 + 64]u8 = @splat(0);
    try testing.expectError(error.BufferTooSmall, buildCommand(0x0C13, &params, &old));
}

test "the command buffer holds the largest command the length field can describe" {
    const params: [255]u8 = @splat(0xAA);
    var buf: [command_buf_len]u8 = @splat(0);
    const pkt = try buildCommand(0x0C13, &params, &buf);
    try testing.expectEqual(command_buf_len, pkt.len);
    try testing.expectEqual(@as(u8, 255), pkt[6]);
}

test "a Command Complete is matched by the opcode it echoes" {
    var buf: [32]u8 = undefined;
    // num_hci_command_packets, opcode, status.
    const pkt = eventPacket(&buf, event.command_complete, &.{ 0x01, 0x03, 0x0C, 0x00 });

    const ev = try parseEvent(pkt);
    const cc = try parseCommandComplete(ev);
    try testing.expectEqual(op.reset, cc.opcode);
    try testing.expectEqual(@as(u8, 0), cc.status);

    // The canonical trace's Read_BD_ADDR reply, which carries six more bytes.
    const bd = eventPacket(&buf, event.command_complete, &.{
        0x01, 0x09, 0x10, 0x00, 0x90, 0xdf, 0x00, 0x9e, 0xa2, 0x88,
    });
    const bcc = try parseCommandComplete(try parseEvent(bd));
    try testing.expectEqual(op.read_bd_addr, bcc.opcode);
    try testing.expectEqualSlices(u8, &.{ 0x90, 0xdf, 0x00, 0x9e, 0xa2, 0x88 }, bcc.params);
}

test "a connection complete yields the handle, and a rejected one is an error" {
    var buf: [64]u8 = undefined;
    const params = [_]u8{
        // zig fmt: off
        le_subevent.connection_complete,
        0x00, // status
        0x40, 0x00, // handle 0x0040
        0x01, // role: peripheral
        0x01, // peer address type: random, which is what a phone uses
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
        0x18, 0x00, // interval
        0x00, 0x00, // latency
        0x48, 0x00, // supervision timeout
        0x05, // clock accuracy
        // zig fmt: on
    };
    const c = (try parseConnectionComplete(try parseEvent(eventPacket(&buf, event.le_meta, &params)))) orelse
        return error.NoConnection;

    try testing.expectEqual(@as(u16, 0x40), c.handle);
    try testing.expectEqual(@as(u8, 1), c.role);
    try testing.expectEqual(@as(u8, 1), c.peer_addr_type);
    try testing.expectEqual(@as(u16, 0x18), c.interval);

    // A failed connection is not a connection.
    var bad = params;
    bad[1] = 0x3E; // connection failed to be established
    try testing.expectError(
        Error.CommandFailed,
        parseConnectionComplete(try parseEvent(eventPacket(&buf, event.le_meta, &bad))),
    );

    // And another LE subevent is not one either — advertising reports arrive on
    // the same event code.
    const other = [_]u8{ le_subevent.advertising_report, 0x01, 0x00 };
    try testing.expectEqual(
        @as(?Connection, null),
        try parseConnectionComplete(try parseEvent(eventPacket(&buf, event.le_meta, &other))),
    );
}

test "a disconnection clears the connection it names, and no other" {
    var buf: [32]u8 = undefined;
    const d = (try parseDisconnection(try parseEvent(
        eventPacket(&buf, event.disconnection_complete, &.{ 0x00, 0x40, 0x00, 0x13 }),
    ))) orelse return error.NoDisconnection;

    try testing.expectEqual(@as(u16, 0x40), d.handle);
    // 0x13 is "remote user terminated the connection", which is what a phone
    // tapping Disconnect produces.
    try testing.expectEqual(@as(u8, 0x13), d.reason);
}

test "an ACL packet keeps the handle to twelve bits and the flags above it" {
    var buf: [32]u8 = @splat(0xAA);
    const p = try buildAcl(0x0040, .first_flushable, &.{ 0x04, 0x00, 0x04, 0x00 }, &buf);

    try testing.expectEqual(@as(usize, 12), p.len);
    try testing.expectEqual(@as(u8, 0x02), p[3]); // ACL, not command
    // handle 0x040 with PB=0b10 in bits 12..13 -> 0x2040.
    try testing.expectEqual(@as(u16, 0x2040), std.mem.readInt(u16, p[4..6], .little));
    try testing.expectEqual(@as(u16, 4), std.mem.readInt(u16, p[6..8], .little));

    const a = try parseAcl(p);
    // **The handle comes back without the flags**, which is the whole point:
    // written as a plain u16 it would read as 0x2040 and match no connection.
    try testing.expectEqual(@as(u16, 0x0040), a.handle);
    try testing.expectEqual(Boundary.first_flushable, a.boundary);
    try testing.expectEqualSlices(u8, &.{ 0x04, 0x00, 0x04, 0x00 }, a.payload);
}

test "a continuation fragment is distinguishable from a first one" {
    var buf: [32]u8 = @splat(0);
    const first = try buildAcl(0x0040, .first_flushable, &.{0x01}, &buf);
    try testing.expectEqual(Boundary.first_flushable, (try parseAcl(first)).boundary);

    var buf2: [32]u8 = @splat(0);
    const cont = try buildAcl(0x0040, .continuation, &.{0x01}, &buf2);
    try testing.expectEqual(Boundary.continuation, (try parseAcl(cont)).boundary);
    // Same handle either way.
    try testing.expectEqual(@as(u16, 0x0040), (try parseAcl(cont)).handle);
}

// -- ACL reassembly ---------------------------------------------------------

const TestReasm = Reassembler(676); // bredr.local_mtu 672 + l2cap.header_len

/// Build an ACL packet carrying `payload` with the given boundary.
fn aclFrag(boundary: Boundary, payload: []const u8, out: []u8) !Acl {
    return parseAcl(try buildAcl(0x000B, boundary, payload, out));
}

/// An L2CAP PDU: 4-byte header (length, cid) then `len` bytes of body.
fn makePdu(len: u16, cid: u16, out: []u8) []u8 {
    std.mem.writeInt(u16, out[0..2], len, .little);
    std.mem.writeInt(u16, out[2..4], cid, .little);
    for (out[4..][0..len], 0..) |*b, i| b.* = @truncate(i);
    return out[0 .. 4 + len];
}

test "a whole PDU in one fragment is returned without copying" {
    var r = TestReasm{};
    var body: [64]u8 = undefined;
    const whole = makePdu(60, 0x0042, &body);

    var pkt: [128]u8 = @splat(0);
    const a = try aclFrag(.first_flushable, whole, &pkt);
    const out = r.push(a).?;

    try testing.expectEqualSlices(u8, whole, out);
    // The fast path aliases the caller's buffer rather than `buf`.
    try testing.expect(out.ptr == a.payload.ptr);
    try testing.expectEqual(@as(u32, 0), r.assembled);
}

test "the soak's own fragmentation: 83 bytes declaring 608" {
    // **The measured failure, 2026-08-27**: a first fragment carrying 83 bytes
    // whose L2CAP header declared 608. Before reassembly existed this PDU was
    // lost twice -- the first fragment failed `l2cap.parse` and every
    // continuation failed too.
    var r = TestReasm{};
    var body: [640]u8 = undefined;
    const whole = makePdu(608, 0x0042, &body); // 612 bytes on the wire

    var pkt: [128]u8 = @splat(0);
    const first = try aclFrag(.first_flushable, whole[0..83], &pkt);
    try testing.expectEqual(@as(?[]const u8, null), r.push(first));

    // The remaining 529 bytes, in continuations of the sizes a degraded link
    // actually produces.
    var at: usize = 83;
    var buf2: [1100]u8 = @splat(0);
    var out: ?[]const u8 = null;
    while (at < whole.len) {
        const take = @min(@as(usize, 183), whole.len - at);
        const frag = try aclFrag(.continuation, whole[at..][0..take], &buf2);
        out = r.push(frag);
        at += take;
        if (at < whole.len) try testing.expectEqual(@as(?[]const u8, null), out);
    }

    try testing.expectEqualSlices(u8, whole, out.?);
    try testing.expectEqual(@as(u32, 1), r.assembled);
    try testing.expectEqual(@as(u32, 0), r.orphans);
    try testing.expectEqual(@as(u32, 0), r.overflows);

    // And the reassembled bytes parse as the frame they always were.
    const frame = try l2cap.parse(out.?);
    try testing.expectEqual(@as(u16, 0x0042), frame.cid);
    try testing.expectEqual(@as(usize, 608), frame.payload.len);
}

test "a continuation with nothing in flight is dropped and counted" {
    var r = TestReasm{};
    var pkt: [64]u8 = @splat(0);
    const orphan = try aclFrag(.continuation, &.{ 1, 2, 3, 4 }, &pkt);
    try testing.expectEqual(@as(?[]const u8, null), r.push(orphan));
    try testing.expectEqual(@as(u32, 1), r.orphans);
}

test "a first fragment abandons a half-assembled message" {
    var r = TestReasm{};
    var body: [640]u8 = undefined;
    const long = makePdu(600, 0x0042, &body);

    var pkt: [128]u8 = @splat(0);
    _ = r.push(try aclFrag(.first_flushable, long[0..40], &pkt));
    try testing.expect(r.want != 0);

    // A new first fragment, complete in itself, arrives mid-message.
    var body2: [64]u8 = undefined;
    const short = makePdu(20, 0x0043, &body2);
    var pkt2: [64]u8 = @splat(0);
    const out = r.push(try aclFrag(.first_flushable, short, &pkt2)).?;

    try testing.expectEqualSlices(u8, short, out);
    try testing.expectEqual(@as(usize, 0), r.want);
}

test "a PDU larger than the buffer is refused, not overrun" {
    var r = TestReasm{};
    // Declare more than capacity while carrying only a header.
    var head: [8]u8 = @splat(0);
    std.mem.writeInt(u16, head[0..2], 1000, .little); // 1000 + 4 > 676
    std.mem.writeInt(u16, head[2..4], 0x0042, .little);
    var pkt: [64]u8 = @splat(0);
    try testing.expectEqual(@as(?[]const u8, null), r.push(try aclFrag(.first_flushable, &head, &pkt)));
    try testing.expectEqual(@as(u32, 1), r.overflows);
    try testing.expectEqual(@as(usize, 0), r.want);
}

test "discard forgets a half-assembled PDU across a disconnect" {
    var r = TestReasm{};
    var body: [640]u8 = undefined;
    const long = makePdu(600, 0x0042, &body);
    var pkt: [128]u8 = @splat(0);
    _ = r.push(try aclFrag(.first_flushable, long[0..40], &pkt));
    r.discard();

    // The tail of the abandoned message must not be adopted by the next link.
    var buf2: [128]u8 = @splat(0);
    _ = r.push(try aclFrag(.continuation, long[40..80], &buf2));
    try testing.expectEqual(@as(u32, 1), r.orphans);
}

test "the boundary flag carries the values btstack rejects and accepts" {
    // **Pinned to the wire values, not to the enum**, because the enum is what
    // was wrong: asserting `@backingInt(.continuation) == @backingInt(...)`
    // would have passed against the bug. See `Boundary` for the source.
    try testing.expectEqual(@as(u2, 0b00), @backingInt(Boundary.first_non_flushable));
    try testing.expectEqual(@as(u2, 0b01), @backingInt(Boundary.continuation));
    try testing.expectEqual(@as(u2, 0b10), @backingInt(Boundary.first_flushable));
    try testing.expectEqual(@as(u2, 0b11), @backingInt(Boundary.complete_le));

    // And the trace that found it: an ACL packet whose flag reads `0b01` must
    // come back as a continuation, whatever the payload length.
    var buf: [32]u8 = @splat(0);
    const empty = try buildAcl(0x0040, .continuation, &.{}, &buf);
    const a = try parseAcl(empty);
    try testing.expectEqual(Boundary.continuation, a.boundary);
    try testing.expectEqual(@as(usize, 0), a.payload.len);
}

test "the controller's buffer size is read, not assumed" {
    // 251-byte packets, 8 of them — a typical LE controller.
    const bs = try parseBufferSize(&.{ 0xFB, 0x00, 0x08 });
    try testing.expectEqual(@as(u16, 251), bs.packet_len);
    try testing.expectEqual(@as(u8, 8), bs.packet_count);
    try testing.expectError(Error.Malformed, parseBufferSize(&.{ 0x01, 0x02 }));
}

test "a truncated event is refused rather than read past" {
    var buf: [32]u8 = @splat(0);
    // Claims six parameters, carries two.
    buf[3] = @backingInt(btbus.PacketType.event);
    buf[4] = event.command_complete;
    buf[5] = 6;
    try testing.expectError(Error.Malformed, parseEvent(buf[0..8]));

    // And an ACL packet is not an event, however well-formed.
    buf[3] = @backingInt(btbus.PacketType.acl);
    try testing.expectError(Error.Malformed, parseEvent(buf[0..16]));
}
