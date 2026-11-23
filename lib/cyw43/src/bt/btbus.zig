//! The CYW43439's Bluetooth shared bus — rung 17a.
//!
//! Bluetooth on this part is **not a second bus**. It is a protocol in the
//! chip's own RAM, reached through the same backplane accessors that have
//! carried 224 KB of WLAN microcode on every boot since rung 6. Two circular
//! buffers and four indices; the host writes HCI packets into one and reads
//! events out of the other, and a doorbell register tells each side to look.
//!
//! Everything here is checked against a trace captured from
//! the SDK's own `cybt_shared_bus` on this board. The numbers in the comments
//! are that trace's.
//!
//! ## The shape, in the order it happens
//!
//! 1. Wake the BT core: write `0x03` to `0x19640894`. The reference's own
//!    comment says that address is "specific to 43012B0" — a different part —
//!    so it is borrowed, and it works here.
//! 2. Download the patchram, which is a **binary Intel-HEX**: four-byte records
//!    of `{len, addr_hi, addr_lo, type}` and then `len` bytes. See `Patchram`.
//! 3. Wait for `BT_CTRL_REG` to report `FW_RDY`. The trace polls fourteen times
//!    before it does, so this is a wait rather than a check.
//! 4. Read `WLAN_RAM_BASE_REG` — **a pointer into chip RAM**, `0x0006861C` on
//!    that run. The buffers and indices are all relative to it, so nothing here
//!    is at a fixed address.
//! 5. Zero the four indices, then set `SW_RDY` in `HOST_CTRL_REG`.
//!
//! ## What is easy to get wrong
//!
//! **The four-byte header on a packet is not the same header in both
//! directions.** Going out, bytes 0..2 are the payload length and byte 3 is the
//! H4 packet type. Coming back, bytes 0..2 are the length the chip is sending
//! and byte 3 is again the type — so the length field is written by whoever is
//! sending. A caller passes a buffer with four bytes of room at the front and
//! the type already in byte 3; `write` fills in the length.
//!
//! **`CIRC_BUF_SPACE` is not `size - count`.** It is
//! `(out - (in + 4)) & (size - 1)` — four bytes are always held back, so the
//! buffer is never allowed to look exactly full, because a full ring and an
//! empty one would otherwise be indistinguishable.
//!
//! **Both indices wrap, and the wrapped write is two writes.** A packet that
//! crosses the end of the ring is split, and the second half starts at offset 0
//! — not at the next 4-byte boundary, and not aborted.

const std = @import("std");

/// `BTSDIO_FWBUF_SIZE`. Each direction gets one of these, and every index is
/// masked to it.
pub const buf_size: u32 = 0x1000;

/// Registers, all in the chip's backplane space.
pub const reg = struct {
    /// Read: `FW_RDY` and `BT_AWAKE`. Read `0x01000100` when both are set.
    pub const bt_ctrl: u32 = 0x1800_0C7C;
    /// Write: `SW_RDY`, `WAKE_BT`, and the `DATA_VALID` bit that is toggled to
    /// ring the doorbell.
    pub const host_ctrl: u32 = 0x1800_0D6C;
    /// Read: where the ring control block lives. Everything else is an offset
    /// from what this returns.
    pub const wlan_ram_base: u32 = 0x1800_0D68;
};

/// Bits in those two registers.
pub const bit = struct {
    pub const data_valid: u32 = 1 << 1;
    pub const bt_awake: u32 = 1 << 8;
    pub const wake_bt: u32 = 1 << 17;
    /// The same bit number in both registers, meaning different things: the
    /// chip's "firmware ready" and the host's "software ready".
    pub const fw_ready: u32 = 1 << 24;
    pub const sw_ready: u32 = 1 << 24;
};

/// Offsets from the ring control base. The trace's base is `0x0006861C`, which
/// puts the four indices at `0x0006A61C`…`0x0006A628` — exactly the words it
/// zeroes.
const off = struct {
    const h2b_buf: u32 = 0;
    const b2h_buf: u32 = buf_size;
    const h2b_in: u32 = 0x2000;
    const h2b_out: u32 = 0x2004;
    const b2h_in: u32 = 0x2008;
    const b2h_out: u32 = 0x200C;
};

/// Where the patchram is written, and the wake that precedes it.
const btfw_mem_offset: u32 = 0x1900_0000;
const bt2wlan_pwrup_addr: u32 = 0x0064_0894;
const bt2wlan_pwrup_wake: u32 = 0x03;

/// The four-byte header in front of every packet, both directions.
pub const packet_header_len: usize = 4;

pub const PacketType = enum(u8) {
    command = 0x01,
    acl = 0x02,
    sco = 0x03,
    event = 0x04,
    _,
};

pub const Error = error{
    /// The BT core never reported `FW_RDY`.
    BtNotReady,
    /// The host→BT ring had no room for the packet.
    QueueFull,
    /// A packet arrived larger than the buffer offered for it.
    PacketTooLarge,
    /// The ring held less than the packet header declared. **Not a malformed
    /// packet** — a packet that was not all there yet, or a header read out of
    /// a ring that had lost its place.
    PacketShort,
    /// The patchram ended without a record telling us where it goes.
    BadPatchram,
    /// A caller passed a buffer with no room for the four-byte header.
    NoHeaderRoom,
};

/// The receive ring's two indices — the chip's write position and ours.
///
/// A named type rather than an anonymous one so `b2hIndices` and `readRingAt`
/// speak the same language: two structurally identical anonymous structs are
/// still two types, and carrying a position from one call into the next is the
/// whole point.
const B2hPos = struct { in: u32, out: u32 };

/// The four ring indices, read together because they are only meaningful
/// together.
pub const Indices = struct {
    h2b_in: u32 = 0,
    h2b_out: u32 = 0,
    b2h_in: u32 = 0,
    b2h_out: u32 = 0,
};

/// How many bytes are waiting, and how many will fit.
///
/// **Both are modular.** `count` is `(in - out) & (size - 1)`; `space` is the
/// same expression with the arguments crossed and `in + 4`, which is what holds
/// four bytes back so that a full ring never reads as an empty one.
pub fn circCount(in: u32, out: u32) u32 {
    return (in -% out) & (buf_size - 1);
}

pub fn circSpace(in: u32, out: u32) u32 {
    return circCount(out, in +% 4);
}

// ---------------------------------------------------------------------------
// The patchram
// ---------------------------------------------------------------------------

/// A record from the Bluetooth patchram.
pub const Record = struct {
    addr: u32,
    data: []const u8,
};

/// Walks the patchram blob, which is **binary Intel-HEX**: a four-byte record
/// header of `{len, addr_hi, addr_lo, type}` and then `len` bytes of body. The
/// address in a data record is only its low 16 bits — the high half comes from
/// whichever address record came before it, which is why this is a cursor with
/// state rather than a `for` loop over records.
///
/// The blob opens with a length-prefixed version string and a record count,
/// both skipped: `init` does that, and asserts the string is NUL-terminated
/// where the length says it should be.
pub const Patchram = struct {
    src: []const u8,
    at: usize,
    /// Set by a type-4 or type-2 record, applied to every data record after it.
    hi_addr: u32 = 0,
    /// Set by a type-5 record. Kept apart from `hi_addr` because the two are
    /// combined with the record address differently.
    abs_base: u32 = 0,
    mode: enum { unknown, extended, segment, linear32 } = .unknown,

    const type_data: u8 = 0;
    const type_end: u8 = 1;
    const type_extended_segment: u8 = 2;
    const type_extended: u8 = 4;
    const type_absolute32: u8 = 5;

    pub fn init(blob: []const u8) !Patchram {
        if (blob.len < 2) return Error.BadPatchram;
        // The version string's length includes its own NUL.
        const version_len: usize = blob[0];
        if (version_len + 2 > blob.len) return Error.BadPatchram;
        if (blob[version_len] != 0) return Error.BadPatchram;
        // Then the string, then a record count byte, neither of which is used.
        return .{ .src = blob, .at = version_len + 2 };
    }

    /// The version string, for a log line that says which firmware this is.
    pub fn version(self: Patchram) []const u8 {
        const version_len: usize = self.src[0];
        return self.src[1 .. version_len - 1 + 1];
    }

    /// The next data record, or null at the end. Address records are consumed
    /// silently, because they carry no bytes to write.
    pub fn next(self: *Patchram) ?Record {
        while (self.at + 4 <= self.src.len) {
            const len: usize = self.src[self.at];
            const addr: u32 = @as(u32, self.src[self.at + 1]) << 8 | self.src[self.at + 2];
            const kind = self.src[self.at + 3];
            self.at += 4;

            // A zero-length record ends the blob, as does running out.
            if (len == 0) return null;
            if (self.at + len > self.src.len) return null;
            const body = self.src[self.at..][0..len];
            self.at += len;

            switch (kind) {
                type_extended => {
                    if (len >= 2) {
                        self.hi_addr = @as(u32, body[0]) << 8 | body[1];
                        self.mode = .extended;
                    }
                },
                type_extended_segment => {
                    if (len >= 2) {
                        self.hi_addr = @as(u32, body[0]) << 8 | body[1];
                        self.mode = .segment;
                    }
                },
                type_absolute32 => {
                    if (len >= 4) {
                        self.abs_base = std.mem.readInt(u32, body[0..4], .big);
                        self.mode = .linear32;
                    }
                },
                type_data => {
                    const dest = switch (self.mode) {
                        .extended => addr + (self.hi_addr << 16),
                        .segment => addr + (self.hi_addr << 4),
                        .linear32 => addr + self.abs_base,
                        .unknown => addr,
                    };
                    return .{ .addr = dest, .data = body };
                },
                type_end => return null,
                else => {},
            }
        }
        return null;
    }
};

// ---------------------------------------------------------------------------
// The bus
// ---------------------------------------------------------------------------

/// Generic over the device, which must provide the backplane accessors rung 6
/// already proved:
///
///   `backplaneRead32(addr) !u32`      `backplaneWrite32(addr, val) !void`
///   `backplaneReadMem(addr, buf) !void`  `backplaneWriteMem(addr, data) !void`
///   `delayMs(ms) void`
pub fn BtBus(comptime Dev: type) type {
    return struct {
        const Self = @This();

        dev: *Dev,

        /// Read from `wlan_ram_base` at init. Zero until then, and every
        /// address below is relative to it.
        base: u32 = 0,
        ready: bool = false,

        /// What the host has written into `host_ctrl`. Kept because the
        /// doorbell is a **toggle** of one bit in it, so the previous value has
        /// to be known — and the trace shows the reference reading it back each
        /// time rather than remembering, which costs a bus transaction per
        /// packet.
        host_ctrl: u32 = 0,

        /// Diagnostics, in the census style of the WiFi side.
        patch_records: u32 = 0,
        patch_bytes: u32 = 0,
        ready_polls: u32 = 0,
        packets_sent: u32 = 0,
        packets_read: u32 = 0,
        /// Packets the caller's buffer could not hold. **Counted rather than
        /// silently skipped**, because the ring recovers from one of these and
        /// the stream above it does not: an ACL fragment discarded here is a
        /// hole in an L2CAP reassembly nothing downstream can see.
        packets_too_large: u32 = 0,
        /// Packets whose payload was shorter than their header declared.
        ///
        /// **This was silent until 2026-08-25.** `read` wrote `got` and never
        /// looked at it, so a short payload was returned as a whole packet with
        /// stale bytes on the end — reaching the parsers as a malformed frame
        /// with nothing to say where it came from. Worth watching alongside
        /// `acl_parse_fail`.
        packets_short: u32 = 0,

        /// The head of the last packet handed to the chip, and its length.
        /// **Kept so a firmware crash can be pinned to what preceded it** —
        /// "it stopped somewhere in the last thirty seconds" is not a
        /// diagnosis. `[3]` is the HCI packet type, `[4..6]` the opcode or
        /// connection handle.
        last_write: [8]u8 = @splat(0),
        last_write_len: usize = 0,

        pub fn init(dev: *Dev) Self {
            return .{ .dev = dev };
        }

        /// Bring the Bluetooth side up: wake, patchram, wait, rings, doorbell.
        pub fn start(self: *Self, patchram: []const u8, timeout_ms: u32) !void {
            try self.dev.backplaneWrite32(btfw_mem_offset + bt2wlan_pwrup_addr, bt2wlan_pwrup_wake);

            try self.downloadPatchram(patchram);
            try self.awaitReady(timeout_ms);

            self.base = try self.dev.backplaneRead32(reg.wlan_ram_base);

            // The four indices, zeroed. Nothing else in the control block is
            // touched — the buffers themselves are left as they are.
            try self.dev.backplaneWrite32(self.base + off.h2b_in, 0);
            try self.dev.backplaneWrite32(self.base + off.h2b_out, 0);
            try self.dev.backplaneWrite32(self.base + off.b2h_in, 0);
            try self.dev.backplaneWrite32(self.base + off.b2h_out, 0);

            // "The host is ready." Read-modify-write rather than a bare store:
            // the other bits in this register are the chip's business.
            self.host_ctrl = try self.dev.backplaneRead32(reg.host_ctrl);
            self.host_ctrl |= bit.sw_ready;
            try self.dev.backplaneWrite32(reg.host_ctrl, self.host_ctrl);

            self.ready = true;
        }

        /// Write the patchram, record by record.
        ///
        /// **Every write is word-aligned at both ends, and the padding is read
        /// back from the chip rather than invented.** The records are not
        /// aligned — they are Intel-HEX, so a record can start and end
        /// anywhere — and the backplane moves words. So a record that starts
        /// mid-word is preceded by the bytes already there, and one that ends
        /// mid-word is followed by the same.
        ///
        /// That read-modify-write is what the `cybt_mem_read` calls in the
        /// trace are. Taking them for a handshake, and writing each record raw
        /// at its own address, is what the first version of this did: the
        /// patchram went down 30 records deep and the BT core never reported
        /// ready, with nothing to say why.
        ///
        /// A write that would cross a 4 KB boundary is split, because the
        /// backplane window moves in 32 KB pages and the reference splits here
        /// rather than relying on that.
        fn downloadPatchram(self: *Self, blob: []const u8) !void {
            var it = try Patchram.init(blob);
            // A record's length byte is one byte, so 255 is the most it can
            // carry; the extra room is for up to three bytes of padding at each
            // end.
            var buf: [255 + 8]u8 = undefined;

            while (it.next()) |rec| {
                var at = btfw_mem_offset + rec.addr;
                var len: usize = 0;

                const head = at % 4;
                if (head != 0) {
                    at -= head;
                    var word: [4]u8 = undefined;
                    try self.dev.backplaneReadMem(at, &word);
                    @memcpy(buf[0..head], word[0..head]);
                    len = head;
                }

                @memcpy(buf[len..][0..rec.data.len], rec.data);
                len += rec.data.len;

                const end: u32 = at + @as(u32, @intCast(len));
                const tail = end % 4;
                if (tail != 0) {
                    var word: [4]u8 = undefined;
                    try self.dev.backplaneReadMem(end - tail, &word);
                    var i: usize = tail;
                    while (i < 4) : (i += 1) {
                        buf[len] = word[i];
                        len += 1;
                    }
                }

                if ((at & 0xFFF) + len <= 0x1000) {
                    try self.dev.backplaneWriteMem(at, buf[0..len]);
                } else {
                    const first = 0x1000 - (at & 0xFFF);
                    try self.dev.backplaneWriteMem(at, buf[0..first]);
                    try self.dev.backplaneWriteMem(at + first, buf[first..len]);
                }

                self.patch_records += 1;
                self.patch_bytes += @intCast(rec.data.len);
            }
            if (self.patch_records == 0) return Error.BadPatchram;
        }

        /// Poll `bt_ctrl` until the firmware says it is up. The trace shows
        /// fourteen zero reads before `0x01000100`, so a single check here
        /// reports a working chip as a dead one.
        fn awaitReady(self: *Self, timeout_ms: u32) !void {
            var left = timeout_ms;
            while (left > 0) : (left -= 1) {
                self.ready_polls += 1;
                const v = self.dev.backplaneRead32(reg.bt_ctrl) catch 0;
                if ((v & bit.fw_ready) != 0) return;
                self.dev.delayMs(1);
            }
            return Error.BtNotReady;
        }

        /// All four indices, in **one** bus transaction rather than four.
        ///
        /// They are contiguous at `0x2000`, and they always were — this read
        /// four registers separately because it was written one register at a
        /// time and nothing measured the cost until `app/bt-speaker` accounted
        /// for where its loop went: **half of all wall time inside `read`**,
        /// against a 5.8 ms audio deadline. Almost none of that is payload;
        /// it is per-transfer overhead, paid four times over.
        fn indices(self: *Self) !Indices {
            var raw: [16]u8 align(4) = undefined;
            try self.dev.backplaneReadMem(self.base + off.h2b_in, &raw);
            return .{
                .h2b_in = std.mem.readInt(u32, raw[0..4], .little),
                .h2b_out = std.mem.readInt(u32, raw[4..8], .little),
                .b2h_in = std.mem.readInt(u32, raw[8..12], .little),
                .b2h_out = std.mem.readInt(u32, raw[12..16], .little),
            };
        }

        /// Only the two the receive path uses, in one 8-byte transfer.
        ///
        /// **`h2b_in` and `h2b_out` are the host→controller side and mean
        /// nothing when reading.** Fetching them cost two of every four
        /// transfers on the hottest path in the system, including on an idle
        /// poll that exists only to learn there is nothing to collect.
        fn b2hIndices(self: *Self) !B2hPos {
            var raw: [8]u8 align(4) = undefined;
            try self.dev.backplaneReadMem(self.base + off.b2h_in, &raw);
            return .{
                .in = std.mem.readInt(u32, raw[0..4], .little),
                .out = std.mem.readInt(u32, raw[4..8], .little),
            };
        }

        /// Ring the doorbell by **toggling** `DATA_VALID`. Not setting it: the
        /// chip watches for the change, so writing the same value twice says
        /// nothing.
        fn toggleIntr(self: *Self) !void {
            self.host_ctrl = try self.dev.backplaneRead32(reg.host_ctrl);
            self.host_ctrl ^= bit.data_valid;
            try self.dev.backplaneWrite32(reg.host_ctrl, self.host_ctrl);
        }

        /// Send one HCI packet.
        ///
        /// `buf` must have `packet_header_len` bytes of room at the front, with
        /// the packet type already in byte 3 — the same contract the SDK
        /// documents ("Must include a 4 byte header. The last byte of the header
        /// should be the packet type"). The length goes in bytes 0..2 here,
        /// because only this layer knows it.
        pub fn write(self: *Self, buf: []u8) !void {
            if (buf.len < packet_header_len) return Error.NoHeaderRoom;
            const keep = @min(buf.len, self.last_write.len);
            @memcpy(self.last_write[0..keep], buf[0..keep]);
            self.last_write_len = buf.len;
            const payload_len = buf.len - packet_header_len;
            buf[0] = @truncate(payload_len);
            buf[1] = @truncate(payload_len >> 8);
            buf[2] = 0;

            // The chip reads whole words, so a short packet is padded up.
            const len: u32 = @intCast(std.mem.alignForward(usize, buf.len, 4));

            const idx = try self.indices();
            if (len > circSpace(idx.h2b_in, idx.h2b_out)) return Error.QueueFull;

            var in = idx.h2b_in;
            if (in + len <= buf_size) {
                try self.dev.backplaneWriteMem(self.base + off.h2b_buf + in, buf);
                in += len;
            } else {
                // Split at the end of the ring. A tail of fewer than four bytes
                // is skipped rather than written, because the chip reads words.
                var first = buf_size - in;
                if (first >= 4) {
                    try self.dev.backplaneWriteMem(self.base + off.h2b_buf + in, buf[0..first]);
                    in += first;
                } else {
                    first = 0;
                }
                const second = len - first;
                if (second > 0) {
                    const end = @min(buf.len, first + second);
                    try self.dev.backplaneWriteMem(self.base + off.h2b_buf, buf[first..end]);
                    in += second;
                }
            }

            try self.dev.backplaneWrite32(self.base + off.h2b_in, in & (buf_size - 1));
            try self.toggleIntr();
            self.packets_sent += 1;
        }

        /// Read one packet into `buf`, or return 0 if the chip has sent
        /// nothing. The four-byte header is included in what is returned, so
        /// `buf[3]` is the packet type and the payload starts at 4.
        /// Everything needed to tell a quiet bus from a wedged one, read live
        /// off the chip.
        ///
        /// **Built because two plausible fixes in a row failed to cure a
        /// wedge**, 2026-08-27: after several connect/disconnect cycles the
        /// controller stops delivering packets entirely — `read` returns
        /// nothing, `read_errors` stays 0, the loop keeps running, and the
        /// phone still reaches "Connected" at baseband before dropping. From
        /// outside that is indistinguishable from an idle link, which is
        /// exactly why it kept being guessed at. These four indices say which
        /// it is: **`b2h_in == b2h_out` is a genuinely empty ring** (the
        /// controller has produced nothing), while `b2h_in != b2h_out` with
        /// nothing coming out is a driver-side fault.
        pub const Diagnostics = struct {
            h2b_in: u32,
            h2b_out: u32,
            b2h_in: u32,
            b2h_out: u32,
            bt_ctrl: u32,
            packets_read: u32,
            packets_short: u32,
            packets_too_large: u32,
        };

        /// Is the controller's Bluetooth firmware still up?
        ///
        /// **One backplane read, so it is cheap enough to poll.** `fw_ready`
        /// going low is the controller's firmware having *stopped* — not a busy
        /// bus, not a host-side stall. Everything downstream follows from it:
        /// the h2b ring stops being consumed, no command ever completes, and
        /// every `Accept_Connection_Request` times out. Seen 2026-08-27 after a
        /// MacBook paired and reached AVDTP.
        /// **Returns the raw register, not a verdict.** The first version
        /// answered `bool` and did `catch return false`, which made a failed
        /// bus read indistinguishable from firmware that had stopped — and the
        /// two want opposite investigations. It reported 322 transitions in
        /// 16 seconds, which is not how firmware dies and was the tell.
        pub fn btControl(self: *Self) !u32 {
            return self.dev.backplaneRead32(reg.bt_ctrl);
        }

        /// **Costs one backplane read of four contiguous words**, the same
        /// fetch `read` makes, so it is cheap enough to print on a timer but
        /// should not go in the hot path.
        pub fn diagnostics(self: *Self) !Diagnostics {
            const idx = try self.indices();
            return .{
                .h2b_in = idx.h2b_in,
                .h2b_out = idx.h2b_out,
                .b2h_in = idx.b2h_in,
                .b2h_out = idx.b2h_out,
                .bt_ctrl = self.dev.backplaneRead32(reg.bt_ctrl) catch 0xFFFF_FFFF,
                .packets_read = self.packets_read,
                .packets_short = self.packets_short,
                .packets_too_large = self.packets_too_large,
            };
        }

        pub fn read(self: *Self, buf: []u8) !usize {
            if (buf.len < packet_header_len) return Error.NoHeaderRoom;

            // The header first: three length bytes and the type.
            //
            // **One index fetch for the whole packet, not two.** `readRing`
            // used to fetch them per call, so every packet paid for two
            // transfers where one would do — and this is the hottest path in
            // the system. The position after the header is carried into the
            // payload read; see `readRingAt` for why that is sound.
            var n: usize = packet_header_len;
            var pos = try self.readRingAt(buf[0..packet_header_len], &n, try self.b2hIndices());
            if (n == 0) return 0;

            const payload_len: usize = @as(usize, buf[0]) |
                @as(usize, buf[1]) << 8 | @as(usize, buf[2]) << 16;
            if (payload_len > buf.len - packet_header_len) {
                // **The header is already out of the ring**, because `readRing`
                // advances `b2h_out` before this check can run. Returning here
                // without taking the payload too leaves the tail pointing into
                // the middle of a packet, and *every subsequent read is
                // garbage* — lengths read out of payload bytes, packet types
                // read out of nowhere. The ring never resynchronises on its
                // own.
                //
                // This was unreachable for rungs 17 to 20: every LE packet fits
                // in 260 bytes. A BR/EDR ACL packet is up to 1021, so a Classic
                // link makes it the first thing that happens.
                try self.skipRing(payload_len);
                self.packets_too_large += 1;
                return Error.PacketTooLarge;
            }

            var got: usize = payload_len;
            pos = try self.readRingAt(buf[packet_header_len..][0..payload_len], &got, pos);

            // **A short payload used to be returned as a whole packet.** `got`
            // was written and never read, so a ring holding less than the
            // header claimed produced a buffer with a correct length and stale
            // bytes on the end — which reaches the parsers as a malformed frame
            // with no clue where it came from. The tail is already consumed by
            // the time this is known, so the ring stays in step; only the
            // packet is lost.
            if (got != payload_len) {
                self.packets_short += 1;
                return Error.PacketShort;
            }

            self.packets_read += 1;
            return packet_header_len + payload_len;
        }

        /// Put a packet into the BT→host ring as if the controller had sent it.
        ///
        /// **This is a measurement hook, not part of the transport**, and it is
        /// here because the receive path cannot otherwise be exercised at speed
        /// without a paired peer and a working L2CAP — which is the tower the
        /// measurement exists to decide whether to build. It writes `b2h_in`,
        /// which belongs to the chip.
        ///
        /// **Never call this on a live link.** Racing the controller for its
        /// own write index corrupts a real stream in a way that looks like a
        /// radio fault. It refuses unless the ring is already empty, which
        /// catches the common case of a controller that spoke while a test was
        /// setting up, and both indices are reset to zero so a packet can never
        /// straddle the end of the ring.
        pub fn injectInbound(self: *Self, kind: PacketType, payload: []const u8) !void {
            const aligned = std.mem.alignForward(usize, payload.len, 4);
            if (packet_header_len + aligned > buf_size - 4) return Error.PacketTooLarge;

            const idx = try self.indices();
            if (circCount(idx.b2h_in, idx.b2h_out) != 0) return Error.QueueFull;

            try self.dev.backplaneWrite32(self.base + off.b2h_out, 0);

            const hdr = [packet_header_len]u8{
                @truncate(payload.len),
                @truncate(payload.len >> 8),
                0,
                @backingInt(kind),
            };
            try self.dev.backplaneWriteMem(self.base + off.b2h_buf, &hdr);
            try self.dev.backplaneWriteMem(
                self.base + off.b2h_buf + @as(u32, @intCast(packet_header_len)),
                payload,
            );
            try self.dev.backplaneWrite32(
                self.base + off.b2h_in,
                @intCast(packet_header_len + aligned),
            );
        }

        /// Advance the ring's tail past `len` bytes without reading them, so a
        /// packet too big for the caller's buffer costs one packet rather than
        /// the stream. Word-aligned like every other index move here, and
        /// bounded by what is actually in the ring so a corrupt length cannot
        /// walk the tail past the head.
        fn skipRing(self: *Self, len: usize) !void {
            const want: u32 = @intCast(std.mem.alignForward(usize, len, 4));
            const idx = try self.indices();
            const take = @min(want, circCount(idx.b2h_in, idx.b2h_out));
            if (take == 0) return;
            const out = (idx.b2h_out + take) & (buf_size - 1);
            try self.dev.backplaneWrite32(self.base + off.b2h_out, out);
        }

        /// Take up to `n.*` bytes out of the BT→host ring, setting `n.*` to
        /// what was actually there.
        ///
        /// Fetches the indices itself. `read` uses `readRingAt` instead, to
        /// avoid paying for a second fetch it can derive — see there.
        fn readRing(self: *Self, dest: []u8, n: *usize) !void {
            _ = try self.readRingAt(dest, n, try self.b2hIndices());
        }

        /// `readRing` against indices the caller already has, returning the ring
        /// position afterwards so a second call can carry on from it.
        ///
        /// **`in` may be carried across calls; `out` must be the value this
        /// returns.** The chip advances `b2h_in` only once a whole packet is in
        /// the ring — the same discipline `write` follows on the other side —
        /// so an `in` fetched before the header was taken still accounts for
        /// that packet's payload. `out` is ours and is exact, because this is
        /// the only thing that moves it.
        fn readRingAt(self: *Self, dest: []u8, n: *usize, idx: B2hPos) !B2hPos {
            const want: u32 = @intCast(std.mem.alignForward(usize, n.*, 4));

            const count = circCount(idx.in, idx.out);
            if (count == 0) {
                n.* = 0;
                return idx;
            }
            const len = @min(want, count);

            var out = idx.out;
            if (out + len <= buf_size) {
                try self.dev.backplaneReadMem(self.base + off.b2h_buf + out, dest[0..@min(dest.len, len)]);
                out += len;
            } else {
                const first = buf_size - out;
                try self.dev.backplaneReadMem(self.base + off.b2h_buf + out, dest[0..@min(dest.len, first)]);
                const second = len - first;
                if (second > 0 and first < dest.len) {
                    const end = @min(dest.len, first + second);
                    try self.dev.backplaneReadMem(self.base + off.b2h_buf, dest[first..end]);
                }
                out = second;
            }

            out &= buf_size - 1;
            try self.dev.backplaneWrite32(self.base + off.b2h_out, out);
            n.* = @min(n.*, len);
            return .{ .in = idx.in, .out = out };
        }
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the ring arithmetic holds four bytes back, so full never looks empty" {
    // An empty ring: in == out.
    try testing.expectEqual(@as(u32, 0), circCount(0, 0));
    // And its space is four short of the buffer, which is the whole point —
    // `size - count` here would be 0x1000 and a full ring would then be
    // indistinguishable from this one.
    try testing.expectEqual(buf_size - 4, circSpace(0, 0));

    try testing.expectEqual(@as(u32, 16), circCount(16, 0));
    try testing.expectEqual(buf_size - 20, circSpace(16, 0));

    // Wrapped: in has gone round past out.
    try testing.expectEqual(@as(u32, 8), circCount(4, buf_size - 4));
    // And a ring with four bytes left reports no space, not four.
    try testing.expectEqual(@as(u32, 0), circSpace(buf_size - 4, 0));
}

test "the patchram is binary Intel-HEX, and the high address carries forward" {
    // A type-4 extended-address record setting the high half, then two data
    // records that inherit it. Reading the address as sixteen bits alone puts
    // every byte of the patch at the wrong end of memory.
    const blob = [_]u8{
        // The header, whose shape is checked against the real blob: a length
        // byte, that many bytes of string *including* the length byte itself,
        // a NUL sitting exactly at index `length`, then a record count. The
        // real one reads 74, 73 characters, NUL at 74, count 35 at 75.
        3, 'v', '1', 0,
        2, // record count, unused
        2, 0x00, 0x00, 4, 0x00, 0x21, // extended address 0x0021
        4, 0xe0, 0x00, 0, 0xAA, 0xBB, 0xCC, 0xDD, // data at 0x0021e000
        2, 0xf0, 0x00, 0, 0x11, 0x22, // data at 0x0021f000
        0, 0x00, 0x00, 1, // end
    };

    var it = try Patchram.init(&blob);
    const first = it.next() orelse return error.NoRecord;
    try testing.expectEqual(@as(u32, 0x0021_E000), first.addr);
    try testing.expectEqualSlices(u8, &.{ 0xAA, 0xBB, 0xCC, 0xDD }, first.data);

    const second = it.next() orelse return error.NoRecord;
    try testing.expectEqual(@as(u32, 0x0021_F000), second.addr);
    try testing.expectEqualSlices(u8, &.{ 0x11, 0x22 }, second.data);

    try testing.expectEqual(@as(?Record, null), it.next());
}

test "the real patchram parses, and lands where the trace says it does" {
    const blob = @embedFile("btfw_43439.bin");
    var it = try Patchram.init(blob);

    var records: u32 = 0;
    var bytes: u32 = 0;
    var first_addr: ?u32 = null;
    while (it.next()) |rec| {
        if (first_addr == null) first_addr = rec.addr;
        records += 1;
        bytes += @intCast(rec.data.len);
    }

    // The trace's first `cybt_mem_write` is at 0x1921e000, and the patchram is
    // written at `btfw_mem_offset + addr` — so the first record's address must
    // be 0x0021e000. That single number checks the version-string skip, the
    // record header layout and the extended-address handling at once.
    try testing.expectEqual(@as(u32, 0x0021_E000), first_addr.?);
    try testing.expect(records > 0);
    // Every byte of a 6,970-byte blob is either header or body.
    try testing.expect(bytes > 6000 and bytes < blob.len);
}

/// A chip: two rings in a byte array, four indices, and the two control
/// registers. It answers reads and records writes the way the real one does,
/// so a wrapped packet or a mis-set doorbell fails here rather than on the
/// board.
pub const FakeChip = struct {
    const base: u32 = 0x0006_861C;

    ram: [0x3000]u8 = @splat(0),
    h2b_in: u32 = 0,
    h2b_out: u32 = 0,
    b2h_in: u32 = 0,
    b2h_out: u32 = 0,
    bt_ctrl: u32 = 0,
    host_ctrl: u32 = 0,

    /// Reads of `bt_ctrl` return not-ready this many times first, as the real
    /// one does.
    not_ready_for: u32 = 0,

    /// Times the driver fetched the receive ring's index pair.
    ///
    /// **Counted so the "one fetch per packet" claim is a test rather than a
    /// comment.** This is the hottest path in the system, and a refactor that
    /// quietly reintroduces the second fetch would otherwise cost only
    /// performance — the least visible thing to lose.
    b2h_index_fetches: u32 = 0,

    patch_writes: u32 = 0,
    patch_bytes: u32 = 0,
    doorbells: u32 = 0,
    delay_ms: u32 = 0,

    /// Patchram writes, recorded so a test can check they are word-aligned.
    patch_addrs: [64]u32 = @splat(0),
    patch_lens: [64]usize = @splat(0),

    fn delayMs(self: *FakeChip, ms: u32) void {
        self.delay_ms += ms;
    }

    fn backplaneRead32(self: *FakeChip, addr: u32) !u32 {
        return switch (addr) {
            reg.bt_ctrl => blk: {
                if (self.not_ready_for > 0) {
                    self.not_ready_for -= 1;
                    break :blk 0;
                }
                break :blk self.bt_ctrl;
            },
            reg.host_ctrl => self.host_ctrl,
            reg.wlan_ram_base => base,
            base + off.h2b_in => self.h2b_in,
            base + off.h2b_out => self.h2b_out,
            base + off.b2h_in => self.b2h_in,
            base + off.b2h_out => self.b2h_out,
            else => 0,
        };
    }

    fn backplaneWrite32(self: *FakeChip, addr: u32, val: u32) !void {
        switch (addr) {
            reg.host_ctrl => {
                if ((val ^ self.host_ctrl) & bit.data_valid != 0) self.doorbells += 1;
                self.host_ctrl = val;
            },
            base + off.h2b_in => self.h2b_in = val,
            base + off.h2b_out => self.h2b_out = val,
            base + off.b2h_in => self.b2h_in = val,
            base + off.b2h_out => self.b2h_out = val,
            else => {},
        }
    }

    fn backplaneWriteMem(self: *FakeChip, addr: u32, data: []const u8) !void {
        if (addr >= btfw_mem_offset) {
            if (self.patch_writes < self.patch_addrs.len) {
                self.patch_addrs[self.patch_writes] = addr;
                self.patch_lens[self.patch_writes] = data.len;
            }
            self.patch_writes += 1;
            self.patch_bytes += @intCast(data.len);
            return;
        }
        const at = addr - base;
        @memcpy(self.ram[at..][0..data.len], data);
    }

    fn backplaneReadMem(self: *FakeChip, addr: u32, buf: []u8) !void {
        // Patchram space: the alignment read-modify-write reads back words it
        // is about to partly overwrite. Returning zeroes is enough — what the
        // test is checking is that the writes come out word-aligned, not what
        // the padding bytes contain.
        if (addr >= btfw_mem_offset) {
            @memset(buf, 0);
            return;
        }
        const at = addr - base;
        // **The index registers are memory on the real chip, and this fake has
        // to agree with itself about that.** It served them only through
        // `backplaneRead32` while `backplaneReadMem` came from `ram`, so the
        // two disagreed — and a driver reading all four in one transfer, which
        // is legitimate and four times cheaper on the hottest path in the
        // system, saw zeroes here and a working ring on hardware. A fake that
        // answers the same address differently depending on which call you use
        // will pass tests the part fails.
        if (at >= off.h2b_in and at + buf.len <= off.b2h_out + 4) {
            if (at == off.b2h_in) self.b2h_index_fetches += 1;
            var i: usize = 0;
            while (i + 4 <= buf.len) : (i += 4) {
                const v = try self.backplaneRead32(base + @as(u32, @intCast(at + i)));
                std.mem.writeInt(u32, buf[i..][0..4], v, .little);
            }
            return;
        }
        @memcpy(buf, self.ram[at..][0..buf.len]);
    }

    /// Put a packet in the BT→host ring, as the chip does.
    fn giveEvent(self: *FakeChip, payload: []const u8) void {
        self.givePacket(.event, payload);
    }

    /// Any packet type, so a test can put an ACL packet in the ring rather
    /// than only the events rungs 17–20 ever saw.
    fn givePacket(self: *FakeChip, kind: PacketType, payload: []const u8) void {
        var hdr: [4]u8 = .{ @truncate(payload.len), @truncate(payload.len >> 8), 0, @backingInt(kind) };
        const at = off.b2h_buf + self.b2h_in;
        @memcpy(self.ram[at..][0..4], &hdr);
        @memcpy(self.ram[at + 4 ..][0..payload.len], payload);
        self.b2h_in += @intCast(std.mem.alignForward(usize, 4 + payload.len, 4));
    }
};

pub fn startedBus(chip: *FakeChip) !BtBus(FakeChip) {
    chip.bt_ctrl = bit.fw_ready | bit.bt_awake;
    var bus = BtBus(FakeChip).init(chip);
    try bus.start(@embedFile("btfw_43439.bin"), 100);
    return bus;
}

test "bring-up zeroes the four indices and says the host is ready" {
    var chip = FakeChip{};
    // Not ready for the first fourteen polls, as the trace shows.
    chip.not_ready_for = 14;
    const bus = try startedBus(&chip);

    try testing.expectEqual(FakeChip.base, bus.base);
    try testing.expect(bus.ready);
    try testing.expectEqual(@as(u32, 15), bus.ready_polls);

    try testing.expectEqual(@as(u32, 0), chip.h2b_in);
    try testing.expectEqual(@as(u32, 0), chip.h2b_out);
    try testing.expectEqual(@as(u32, 0), chip.b2h_in);
    try testing.expectEqual(@as(u32, 0), chip.b2h_out);

    // SW_RDY set, and the patchram actually written.
    try testing.expect(chip.host_ctrl & bit.sw_ready != 0);
    try testing.expect(chip.patch_bytes > 6000);
}

test "every patchram write is word-aligned at both ends" {
    var chip = FakeChip{};
    const bus = try startedBus(&chip);
    _ = bus;

    // **The records are not aligned; the writes have to be.** Intel-HEX records
    // start and end wherever the linker left them, and the backplane moves
    // words. This is the fault that cost a hardware run: without the
    // read-modify-write padding, the patch went down 30 records deep and the BT
    // core simply never reported ready.
    try testing.expect(chip.patch_writes > 0);
    var i: usize = 0;
    while (i < @min(chip.patch_writes, chip.patch_addrs.len)) : (i += 1) {
        try testing.expectEqual(@as(u32, 0), chip.patch_addrs[i] % 4);
        try testing.expectEqual(@as(usize, 0), chip.patch_lens[i] % 4);
    }
}

test "a chip that never reports ready is an error, not a hang" {
    var chip = FakeChip{};
    chip.bt_ctrl = 0; // never sets FW_RDY
    var bus = BtBus(FakeChip).init(&chip);
    try testing.expectError(Error.BtNotReady, bus.start(@embedFile("btfw_43439.bin"), 50));
    try testing.expect(!bus.ready);
}

test "an HCI command goes into the ring with its length filled in, and rings the bell" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);
    const before = chip.doorbells;

    // HCI_Reset: four bytes of header with the type in byte 3, then opcode and
    // parameter length.
    var pkt = [_]u8{ 0, 0, 0, @backingInt(PacketType.command), 0x03, 0x0C, 0x00 };
    try bus.write(&pkt);

    // The length is the payload only — three, not seven.
    try testing.expectEqual(@as(u8, 3), pkt[0]);
    try testing.expectEqual(@as(u8, 0), pkt[1]);
    try testing.expectEqual(@as(u8, 0), pkt[2]);

    // In the ring, at offset 0, and the index advanced to a 4-byte boundary.
    try testing.expectEqualSlices(u8, &pkt, chip.ram[off.h2b_buf..][0..pkt.len]);
    try testing.expectEqual(@as(u32, 8), chip.h2b_in);

    // **The doorbell is a toggle.** A driver that sets the bit rings once and
    // then goes quiet.
    try testing.expectEqual(before + 1, chip.doorbells);
    try bus.write(&pkt);
    try testing.expectEqual(before + 2, chip.doorbells);
}

test "an event is read back with its header, and the out index follows it" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // Command Complete for HCI_Reset, exactly as the canonical trace shows it.
    chip.giveEvent(&.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 });

    var buf: [64]u8 = undefined;
    const n = try bus.read(&buf);

    try testing.expectEqual(@as(usize, 4 + 6), n);
    try testing.expectEqual(@as(u8, @backingInt(PacketType.event)), buf[3]);
    try testing.expectEqualSlices(u8, &.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 }, buf[4..n]);
    // Read to the end of what the chip wrote, rounded to a word.
    try testing.expectEqual(chip.b2h_in, chip.b2h_out);
}

test "one packet costs one index fetch, not two" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    chip.giveEvent(&.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 });

    const before = chip.b2h_index_fetches;
    var buf: [64]u8 = undefined;
    _ = try bus.read(&buf);

    // **The point of `readRingAt`.** The header read and the payload read used
    // to fetch the index pair separately, so every packet paid two transfers on
    // the hottest path in the system. The position after the header is carried
    // into the payload read instead.
    try testing.expectEqual(@as(u32, 1), chip.b2h_index_fetches - before);
}

test "an idle poll still costs exactly one index fetch" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // Nothing to collect: this is the common case on a quiet link, and it must
    // not have become *more* expensive in the course of making the busy case
    // cheaper.
    const before = chip.b2h_index_fetches;
    var buf: [64]u8 = undefined;
    try testing.expectEqual(@as(usize, 0), try bus.read(&buf));
    try testing.expectEqual(@as(u32, 1), chip.b2h_index_fetches - before);
}

test "a payload shorter than its header claims is an error, not a short packet" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // A header declaring 32 bytes with only 8 in the ring. Before this was
    // checked, `read` returned 4 + 32 and the caller parsed 24 bytes of
    // whatever the buffer happened to hold.
    const hdr = [_]u8{ 32, 0x00, 0x00, @backingInt(PacketType.acl) };
    @memcpy(chip.ram[off.b2h_buf..][0..4], &hdr);
    @memset(chip.ram[off.b2h_buf + 4 ..][0..8], 0xAB);
    chip.b2h_in = 12;

    var buf: [64]u8 = undefined;
    try testing.expectError(Error.PacketShort, bus.read(&buf));
    try testing.expectEqual(@as(u32, 1), bus.packets_short);
    // Not counted as a good packet.
    try testing.expectEqual(@as(u32, 0), bus.packets_read);
    // And the ring is drained rather than left mid-packet, so the next read
    // starts on a boundary instead of reading a length out of payload bytes.
    try testing.expectEqual(chip.b2h_in, chip.b2h_out);
}

test "an empty ring reads as nothing, not as a zero-length packet" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    var buf: [64]u8 = undefined;
    try testing.expectEqual(@as(usize, 0), try bus.read(&buf));
    try testing.expectEqual(@as(u32, 0), bus.packets_read);
}

test "an oversize packet costs one packet, not the whole stream" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // A 600-byte ACL packet — bigger than the caller's buffer, and well within
    // what a BR/EDR link sends — followed by an ordinary event.
    var big: [600]u8 = @splat(0xAB);
    chip.givePacket(.acl, &big);
    chip.giveEvent(&.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 });

    var buf: [260]u8 = undefined;
    try testing.expectError(Error.PacketTooLarge, bus.read(&buf));
    try testing.expectEqual(@as(u32, 1), bus.packets_too_large);

    // **This is the assertion that fails against the old code.** The next read
    // has to return the event that followed. Before `skipRing`, the tail was
    // left four bytes into a 600-byte packet, so this read took a length out of
    // payload bytes and the ring never recovered.
    const n = try bus.read(&buf);
    try testing.expectEqual(@as(usize, 4 + 6), n);
    try testing.expectEqual(@as(u8, @backingInt(PacketType.event)), buf[3]);
    try testing.expectEqualSlices(u8, &.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 }, buf[4..n]);
    try testing.expectEqual(chip.b2h_in, chip.b2h_out);
}

test "an injected packet reads back exactly, at full BR/EDR size" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    var payload: [1021]u8 = undefined;
    for (&payload, 0..) |*b, i| b.* = @truncate(i);
    try bus.injectInbound(.acl, &payload);

    var buf: [1029]u8 = undefined;
    const n = try bus.read(&buf);
    try testing.expectEqual(@as(usize, 4 + 1021), n);
    try testing.expectEqual(@as(u8, @backingInt(PacketType.acl)), buf[3]);
    try testing.expectEqualSlices(u8, &payload, buf[4..n]);
    try testing.expectEqual(chip.b2h_in, chip.b2h_out);
}

test "injecting refuses a ring that is not empty" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    chip.giveEvent(&.{ 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 });
    // The controller spoke first. Overwriting its index here is the corruption
    // the guard exists to prevent.
    try testing.expectError(Error.QueueFull, bus.injectInbound(.acl, &.{ 1, 2, 3, 4 }));
}

test "skipping never walks the tail past the head" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // A header claiming 65535 bytes with eight in the ring — the shape a
    // corrupt length takes. The tail must stop at the head, not run past it
    // and leave the ring looking full of a packet nobody wrote.
    const hdr = [_]u8{ 0xFF, 0xFF, 0x00, @backingInt(PacketType.acl) };
    @memcpy(chip.ram[off.b2h_buf..][0..4], &hdr);
    @memset(chip.ram[off.b2h_buf + 4 ..][0..4], 0);
    chip.b2h_in = 8;

    var buf: [16]u8 = undefined;
    try testing.expectError(Error.PacketTooLarge, bus.read(&buf));
    try testing.expectEqual(chip.b2h_in, chip.b2h_out);
    try testing.expectEqual(@as(u32, 0), circCount(chip.b2h_in, chip.b2h_out));
}

test "a packet that would not fit is refused rather than overwriting the ring" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // Claim the ring is nearly full: in has run right up behind out.
    chip.h2b_out = 8;
    chip.h2b_in = 0;
    try testing.expectEqual(@as(u32, 4), circSpace(0, 8));

    var pkt = [_]u8{ 0, 0, 0, @backingInt(PacketType.command), 1, 2, 3, 4, 5, 6, 7, 8 };
    try testing.expectError(Error.QueueFull, bus.write(&pkt));
    // Nothing moved.
    try testing.expectEqual(@as(u32, 0), chip.h2b_in);
}

test "a write that crosses the end of the ring is split, not truncated" {
    var chip = FakeChip{};
    var bus = try startedBus(&chip);

    // Eight bytes from the end, with a twelve-byte packet to place — and `out`
    // far enough ahead that there is genuinely room, or this tests the
    // queue-full path by accident.
    chip.h2b_in = buf_size - 8;
    chip.h2b_out = 0x100;

    var pkt = [_]u8{ 0, 0, 0, @backingInt(PacketType.command), 1, 2, 3, 4, 5, 6, 7, 8 };
    try bus.write(&pkt);

    // First eight bytes at the end of the ring, last four at the start.
    try testing.expectEqualSlices(u8, pkt[0..8], chip.ram[off.h2b_buf + buf_size - 8 ..][0..8]);
    try testing.expectEqualSlices(u8, pkt[8..12], chip.ram[off.h2b_buf..][0..4]);
    // And the index wrapped rather than running past the buffer.
    try testing.expectEqual(@as(u32, 4), chip.h2b_in);
}

test "the fake answers the index registers the same way through either call" {
    var chip = FakeChip{};
    chip.h2b_in = 0x11;
    chip.h2b_out = 0x22;
    chip.b2h_in = 0x33;
    chip.b2h_out = 0x44;

    var raw: [16]u8 align(4) = undefined;
    try chip.backplaneReadMem(FakeChip.base + off.h2b_in, &raw);
    try testing.expectEqual(@as(u32, 0x11), std.mem.readInt(u32, raw[0..4], .little));
    try testing.expectEqual(@as(u32, 0x22), std.mem.readInt(u32, raw[4..8], .little));
    try testing.expectEqual(@as(u32, 0x33), std.mem.readInt(u32, raw[8..12], .little));
    try testing.expectEqual(@as(u32, 0x44), std.mem.readInt(u32, raw[12..16], .little));

    // And the pair the receive path asks for on its own.
    var pair: [8]u8 align(4) = undefined;
    try chip.backplaneReadMem(FakeChip.base + off.b2h_in, &pair);
    try testing.expectEqual(@as(u32, 0x33), std.mem.readInt(u32, pair[0..4], .little));
    try testing.expectEqual(@as(u32, 0x44), std.mem.readInt(u32, pair[4..8], .little));
}
