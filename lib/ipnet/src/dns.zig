//! DNS — RFC 1035. Rung N9.
//!
//! **Pure, like `net.zig`, `dhcp.zig` and `tcp.zig`**: functions over
//! caller-supplied bytes, no device, no state, no allocator. `A` records only,
//! and only enough of the format to turn a name into an address.
//!
//! ## What it is checked against
//!
//! Raw datagrams queried from the real resolver on
//! 2026-08-31 and kept as bytes, not as a description. The `time.nist.gov`
//! response is the valuable one: it carries a **CNAME chain**, a pointer to
//! the middle of the question's name, and a second pointer into the *rdata* of
//! the answer before it. A parser that handles only the simple `c0 0c` case
//! passes against `pool.ntp.org` and fails against it.

const std = @import("std");

pub const header_len: usize = 12;
pub const port: u16 = 53;

pub const Type = struct {
    pub const a: u16 = 1;
    pub const cname: u16 = 5;
};

pub const class_in: u16 = 1;

/// **Names are bounded twice over, and both bounds matter.** RFC 1035 §2.3.4
/// caps a label at 63 octets and a whole name at 255 — but the reason for the
/// jump limit below is not the RFC's, it is that a pointer loop is otherwise
/// an unbounded walk driven by a stranger's bytes.
pub const max_name_len: usize = 255;
const max_label_len: usize = 63;

/// How many compression pointers one name may follow.
///
/// **The backwards rule already makes a cycle impossible, so this is not what
/// stops the walk — it is what bounds the work.** Each jump must land strictly
/// lower than the last, so a chain terminates after at most one hop per byte
/// of the message; on a 512-byte datagram that is 512 jumps to resolve a
/// single name, chosen by whoever sent it. Sixteen is far more than any real
/// encoder emits and turns a linear cost into a constant one.
///
/// This distinction is not decoration: a mutation removing this limit passes
/// every test about *termination*, because termination is the other rule's
/// job. The test below is about the bound.
const max_jumps: usize = 16;

pub const Error = error{
    Truncated,
    Malformed,
    /// A pointer that goes forward or sideways. See `readName`.
    BadPointer,
    /// The server answered, and the answer is that there is no such name.
    NameNotFound,
    /// The reply is not the one we asked for.
    IdMismatch,
    NoAddress,
    BufferTooSmall,
};

/// Response codes worth telling apart. RFC 1035 §4.1.1.
pub const Rcode = struct {
    pub const no_error: u4 = 0;
    pub const name_error: u4 = 3; // NXDOMAIN
};

/// Build a standard recursive `A` query for `name` into `out`.
pub fn buildQuery(name: []const u8, id: u16, out: []u8) ![]u8 {
    const total = header_len + name.len + 2 + 4;
    if (out.len < total) return Error.BufferTooSmall;
    if (name.len == 0 or name.len > max_name_len) return Error.Malformed;

    std.mem.writeInt(u16, out[0..2], id, .big);
    // QR=0, OPCODE=0, RD=1 — **recursion desired**, because this stack has no
    // referral machinery and would not know what to do with one.
    std.mem.writeInt(u16, out[2..4], 0x0100, .big);
    std.mem.writeInt(u16, out[4..6], 1, .big); // QDCOUNT
    std.mem.writeInt(u16, out[6..8], 0, .big);
    std.mem.writeInt(u16, out[8..10], 0, .big);
    std.mem.writeInt(u16, out[10..12], 0, .big);

    var w: usize = header_len;
    var it = std.mem.splitScalar(u8, name, '.');
    while (it.next()) |label| {
        if (label.len == 0 or label.len > max_label_len) return Error.Malformed;
        out[w] = @intCast(label.len);
        @memcpy(out[w + 1 ..][0..label.len], label);
        w += 1 + label.len;
    }
    out[w] = 0;
    w += 1;

    std.mem.writeInt(u16, out[w..][0..2], Type.a, .big);
    std.mem.writeInt(u16, out[w + 2 ..][0..2], class_in, .big);
    return out[0 .. w + 4];
}

/// Walk a name at `off`, returning the offset just past it **in the record
/// stream** — which is not the same as where the name ended, once a pointer is
/// involved.
///
/// **Two rules keep this terminating, and only one of them is the RFC's.**
///
///  * A pointer's two high bits are `11`; `10` and `01` are reserved and are
///    refused rather than guessed at.
///  * **A pointer must point strictly backwards.** RFC 1035 §4.1.4 describes
///    pointers as referring to "a prior occurance", and enforcing that makes a
///    cycle arithmetically impossible: each jump strictly decreases the
///    offset, so the walk cannot revisit anything. A parser that allowed a
///    forward pointer would hang on `c0 0c` at offset 12 — four bytes, from
///    anyone.
fn skipName(msg: []const u8, off: usize) !usize {
    var i = off;
    while (true) {
        if (i >= msg.len) return Error.Truncated;
        const len = msg[i];
        if (len == 0) return i + 1;
        switch (len & 0xC0) {
            0x00 => {
                if (i + 1 + len > msg.len) return Error.Truncated;
                i += 1 + len;
            },
            0xC0 => {
                if (i + 2 > msg.len) return Error.Truncated;
                // A pointer always ends the name in the record stream.
                return i + 2;
            },
            else => return Error.Malformed,
        }
    }
}

/// Read a name at `off` into `out` as dotted text, following pointers.
pub fn readName(msg: []const u8, off: usize, out: []u8) ![]u8 {
    var i = off;
    var w: usize = 0;
    var jumps: usize = 0;
    // **Seeded with where this name starts, not with the message length.**
    // The first version used `msg.len`, so the *first* jump was only checked
    // against the end of the buffer — a pointer at offset 12 aimed at offset
    // 20 sailed through, which is a forward pointer and exactly what §4.1.4's
    // "prior occurance" forbids. It took a test to notice, because every real
    // capture points backwards anyway.
    var lowest = off;

    while (true) {
        if (i >= msg.len) return Error.Truncated;
        const len = msg[i];
        if (len == 0) break;

        switch (len & 0xC0) {
            0x00 => {
                if (i + 1 + len > msg.len) return Error.Truncated;
                if (w + len + 1 > out.len or w + len + 1 > max_name_len) return Error.BufferTooSmall;
                if (w != 0) {
                    out[w] = '.';
                    w += 1;
                }
                @memcpy(out[w..][0..len], msg[i + 1 ..][0..len]);
                w += len;
                i += 1 + len;
            },
            0xC0 => {
                if (i + 2 > msg.len) return Error.Truncated;
                const target = (@as(usize, len & 0x3F) << 8) | msg[i + 1];
                // **Strictly backwards, and strictly below every previous
                // target.** Either condition alone bounds the walk; together
                // they make a loop unrepresentable rather than merely
                // unlikely.
                if (target >= lowest) return Error.BadPointer;
                lowest = target;
                jumps += 1;
                if (jumps > max_jumps) return Error.BadPointer;
                i = target;
            },
            else => return Error.Malformed,
        }
    }
    return out[0..w];
}

pub const Reply = struct {
    id: u16,
    rcode: u4,
    /// The first `A` record in the answer section.
    address: ?[4]u8,
    /// Seconds the answer may be cached. Zero when there is no address.
    ttl: u32,
};

/// Parse a response.
///
/// **The transaction id is checked by the caller, not here** — this returns it
/// so a caller can compare against what it sent, the same way `dhcp.zig`
/// treats an `xid`. Every DNS reply on a shared link is a plausible-looking
/// datagram; the id is what makes one of them ours.
///
/// **The first `A` record wins, and the CNAME chain is not verified.** A
/// response could in principle carry an `A` for a name nobody asked about.
/// This stack queries one configured resolver and matches the id, which is the
/// same trust this package extends to its DHCP server; verifying the chain
/// means name comparison and its own bugs, and nothing here needs it yet. It
/// is a limitation rather than an oversight, and it is written down.
pub fn parseReply(msg: []const u8) !Reply {
    if (msg.len < header_len) return Error.Truncated;

    const id = std.mem.readInt(u16, msg[0..2], .big);
    const flags = std.mem.readInt(u16, msg[2..4], .big);
    if ((flags & 0x8000) == 0) return Error.Malformed; // not a response
    // **TC: the answer did not fit in a datagram.** Using the truncated part
    // is worse than failing — the addresses that *are* present may not be the
    // ones the server would have ranked first.
    if ((flags & 0x0200) != 0) return Error.Truncated;
    const rcode: u4 = @truncate(flags);

    const qd = std.mem.readInt(u16, msg[4..6], .big);
    const an = std.mem.readInt(u16, msg[6..8], .big);

    if (rcode == Rcode.name_error) return .{ .id = id, .rcode = rcode, .address = null, .ttl = 0 };
    if (rcode != Rcode.no_error) return .{ .id = id, .rcode = rcode, .address = null, .ttl = 0 };

    var off: usize = header_len;
    var q: u16 = 0;
    while (q < qd) : (q += 1) {
        off = try skipName(msg, off);
        if (off + 4 > msg.len) return Error.Truncated;
        off += 4;
    }

    var a: u16 = 0;
    while (a < an) : (a += 1) {
        off = try skipName(msg, off);
        if (off + 10 > msg.len) return Error.Truncated;
        const rtype = std.mem.readInt(u16, msg[off..][0..2], .big);
        const rclass = std.mem.readInt(u16, msg[off + 2 ..][0..2], .big);
        const ttl = std.mem.readInt(u32, msg[off + 4 ..][0..4], .big);
        const rdlen = std.mem.readInt(u16, msg[off + 8 ..][0..2], .big);
        off += 10;
        if (off + rdlen > msg.len) return Error.Truncated;

        if (rtype == Type.a and rclass == class_in) {
            // **Four bytes, and an `A` claiming otherwise is malformed.** A
            // record whose length disagrees with its type is the shape of
            // either a broken server or a deliberate one.
            if (rdlen != 4) return Error.Malformed;
            var addr: [4]u8 = undefined;
            @memcpy(&addr, msg[off..][0..4]);
            return .{ .id = id, .rcode = rcode, .address = addr, .ttl = ttl };
        }
        off += rdlen;
    }

    return .{ .id = id, .rcode = rcode, .address = null, .ttl = 0 };
}

// ---------------------------------------------------------------------------
// Rung N9 — asserted against the captured resolver datagrams
//
// Raw datagrams from the real resolver, kept as bytes. The `time.nist.gov`
// reply is the one that matters: a CNAME chain, a pointer into the middle of
// the question's name, and a second pointer into the *rdata* of the answer
// before it.
// ---------------------------------------------------------------------------

const testing = std.testing;

const pool_query = [_]u8{
    0x12, 0x34, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x70, 0x6F, 0x6F, 0x6C, 0x03, 0x6E, 0x74, 0x70, 0x03, 0x6F, 0x72,
    0x67, 0x00, 0x00, 0x01, 0x00, 0x01,
};
const pool_reply = [_]u8{
    0x12, 0x34, 0x81, 0x80, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x70, 0x6F, 0x6F, 0x6C, 0x03, 0x6E, 0x74, 0x70, 0x03, 0x6F, 0x72,
    0x67, 0x00, 0x00, 0x01, 0x00, 0x01, 0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x53, 0x00, 0x04, 0xB0, 0x3A, 0x7F, 0x83, 0xC0, 0x0C,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x53, 0x00, 0x04, 0xB0, 0x3A,
    0x78, 0x22, 0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x53,
    0x00, 0x04, 0xB9, 0x33, 0xC0, 0x3E, 0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x53, 0x00, 0x04, 0x4E, 0x8D, 0xC9, 0xA1,
};
const nist_reply = [_]u8{
    0x12, 0x34, 0x81, 0x80, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x74, 0x69, 0x6D, 0x65, 0x04, 0x6E, 0x69, 0x73, 0x74, 0x03, 0x67,
    0x6F, 0x76, 0x00, 0x00, 0x01, 0x00, 0x01, 0xC0, 0x0C, 0x00, 0x05, 0x00,
    0x01, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x0B, 0x04, 0x6E, 0x74, 0x70, 0x31,
    0x03, 0x67, 0x6C, 0x62, 0xC0, 0x11, 0xC0, 0x2B, 0x00, 0x01, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x1E, 0x00, 0x04, 0x84, 0xA3, 0x60, 0x03,
};

test "the captured pool.ntp.org query is what this builds" {
    var buf: [64]u8 = undefined;
    const q = try buildQuery("pool.ntp.org", 0x1234, &buf);

    // **Byte-identical to the datagram that got an answer.** Asserting the
    // shape field by field would pass against a query no server replies to.
    try testing.expectEqualSlices(u8, &pool_query, q);
}

test "the captured pool.ntp.org reply yields the first address" {
    const r = try parseReply(&pool_reply);
    try testing.expectEqual(@as(u16, 0x1234), r.id);
    try testing.expectEqual(@as(u4, 0), r.rcode);
    try testing.expectEqual([4]u8{ 176, 58, 127, 131 }, r.address.?);
    try testing.expectEqual(@as(u32, 83), r.ttl);
}

test "every answer's name is a pointer, and all four resolve" {
    // Four answers, each named `c0 0c` — a pointer to offset 12, the
    // question's name. **The parser resolves a pointer four times in one
    // message**, and one that handled only inline labels would read the first
    // answer's type field as a label length and produce garbage that still
    // parses.
    var name: [max_name_len]u8 = undefined;
    var off: usize = header_len;
    off = try skipName(&pool_reply, off);
    off += 4; // qtype, qclass

    var seen: usize = 0;
    while (seen < 4) : (seen += 1) {
        try testing.expectEqual(@as(u8, 0xC0), pool_reply[off] & 0xC0);
        const n = try readName(&pool_reply, off, &name);
        try testing.expectEqualSlices(u8, "pool.ntp.org", n);
        off = try skipName(&pool_reply, off);
        const rdlen = std.mem.readInt(u16, pool_reply[off + 8 ..][0..2], .big);
        off += 10 + rdlen;
    }
    try testing.expectEqual(pool_reply.len, off);
}

test "a CNAME chain is walked past to the address behind it" {
    // `time.nist.gov` answers with a CNAME to `ntp1.glb.nist.gov` and then an
    // A record for it. **The A record is the second answer**, so a parser that
    // took the first would return a name where an address belongs.
    const r = try parseReply(&nist_reply);
    try testing.expectEqual(@as(u16, 0x1234), r.id);
    try testing.expectEqual([4]u8{ 132, 163, 96, 3 }, r.address.?);
    try testing.expectEqual(@as(u32, 30), r.ttl);
}

test "a pointer to the middle of a name, and one into an rdata field" {
    var name: [max_name_len]u8 = undefined;

    // The CNAME's rdata is `ntp1.glb` followed by `c0 11` — a pointer to
    // offset 17, which is **the middle of the question's name**, not its
    // start. Compression works label-wise, and a parser that only understood
    // whole-name pointers would produce `ntp1.glb` and stop.
    const cname_rdata = 43;
    try testing.expectEqualSlices(u8, "ntp1.glb.nist.gov", try readName(&nist_reply, cname_rdata, &name));

    // The second answer's own name is `c0 2b` — offset 43, which is inside the
    // *previous answer's rdata*. Names can point anywhere earlier in the
    // message, including into a length-delimited field.
    const second_answer = 54;
    try testing.expectEqual(@as(u8, 0xC0), nist_reply[second_answer] & 0xC0);
    try testing.expectEqualSlices(u8, "ntp1.glb.nist.gov", try readName(&nist_reply, second_answer, &name));
}

test "a forward pointer is refused rather than followed" {
    var msg = @as([32]u8, @splat(0));
    std.mem.writeInt(u16, msg[0..2], 0x1234, .big);
    std.mem.writeInt(u16, msg[2..4], 0x8180, .big);
    // A name at offset 12 pointing at offset 12: **`c0 0c`, four bytes, and a
    // parser without the backwards rule spins on it forever.** Reachable from
    // anyone who can send this stack a datagram.
    msg[12] = 0xC0;
    msg[13] = 0x0C;

    var name: [max_name_len]u8 = undefined;
    try testing.expectError(Error.BadPointer, readName(&msg, 12, &name));

    // Forward is refused too, for the same reason: only a strictly decreasing
    // target makes a cycle unrepresentable.
    msg[13] = 0x14; // offset 20, ahead of us
    try testing.expectError(Error.BadPointer, readName(&msg, 12, &name));
}

test "a two-step pointer cycle is refused" {
    var msg = @as([32]u8, @splat(0));
    std.mem.writeInt(u16, msg[2..4], 0x8180, .big);
    // 20 -> 16 -> 20. Each hop is backwards *from where it is written*, so a
    // parser checking only "the target is less than my current position" would
    // follow this forever. **The rule is that each target is below every
    // previous target**, which this violates on the second hop.
    msg[16] = 0xC0;
    msg[17] = 0x14; // -> 20
    msg[20] = 0xC0;
    msg[21] = 0x10; // -> 16

    var name: [max_name_len]u8 = undefined;
    try testing.expectError(Error.BadPointer, readName(&msg, 20, &name));
}

test "replies this stack must refuse" {
    var msg: [94]u8 = undefined;

    // Truncated: the answer did not fit in a datagram, so what is present may
    // not be what the server would have ranked first.
    @memcpy(&msg, &pool_reply);
    std.mem.writeInt(u16, msg[2..4], 0x8380, .big); // TC set
    try testing.expectError(Error.Truncated, parseReply(&msg));

    // A query, not a response. Answering our own question back would be a
    // strange thing to accept.
    @memcpy(&msg, &pool_reply);
    std.mem.writeInt(u16, msg[2..4], 0x0100, .big);
    try testing.expectError(Error.Malformed, parseReply(&msg));

    // An A record whose length is not four. **A record disagreeing with its
    // own type is a broken server or a deliberate one**, and reading four
    // bytes from it regardless is how a parser reads past a buffer.
    @memcpy(&msg, &pool_reply);
    std.mem.writeInt(u16, msg[41..43], 8, .big); // first answer's RDLENGTH
    try testing.expectError(Error.Malformed, parseReply(&msg));

    // Cut short mid-record.
    try testing.expectError(Error.Truncated, parseReply(pool_reply[0..40]));
}

test "NXDOMAIN is an answer, not a failure to parse" {
    var msg: [31]u8 = undefined;
    @memcpy(&msg, pool_query[0..30] ++ [_]u8{0});
    std.mem.writeInt(u16, msg[2..4], 0x8183, .big); // response, RCODE 3
    std.mem.writeInt(u16, msg[6..8], 0, .big); // no answers

    // **The server replied, and the reply is "no such name".** Reporting that
    // as a parse error would make a typo indistinguishable from a broken
    // resolver, and the caller retries the wrong thing.
    const r = try parseReply(msg[0..30]);
    try testing.expectEqual(Rcode.name_error, r.rcode);
    try testing.expectEqual(@as(?[4]u8, null), r.address);
}

test "a long backwards chain is refused, though it would terminate" {
    // **Strictly-decreasing pointers cannot loop, so this walk ends** — but it
    // ends after one hop per pointer, and a stranger picks how many there are.
    // Twenty chained hops resolve one name; a 512-byte datagram could ask for
    // hundreds. The jump limit turns that linear cost into a constant.
    //
    // Removing the limit passes every other test in this file, because every
    // other test is about *termination*, which the backwards rule handles.
    var msg = @as([64]u8, @splat(0));
    std.mem.writeInt(u16, msg[2..4], 0x8180, .big);
    var at: usize = 12;
    while (at <= 50) : (at += 2) {
        msg[at] = 0xC0;
        msg[at + 1] = @intCast(at - 2);
    }

    var name: [max_name_len]u8 = undefined;
    try testing.expectError(Error.BadPointer, readName(&msg, 50, &name));

    // A short chain is fine — the limit is a ceiling, not a ban on chaining.
    try testing.expectEqualSlices(u8, "", try readName(&msg, 16, &name));
}
