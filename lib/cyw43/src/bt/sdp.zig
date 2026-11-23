//! SDP — rung 23b, the record that makes this device an audio sink.
//!
//! A paired phone learns nothing from a device with no service record. Rung 22
//! left one connected and asking; rung 23a gave it a channel to ask on; this
//! answers. **Until it does, the phone cannot request AVDTP at all** — in a
//! capture of btstack's sink, PSM 0x0019 is asked for 530 ms after the
//! first SDP query and never before.
//!
//! ## The thing that is not obvious, and would have been built wrong
//!
//! **The phone does not search for `AudioSink`.** It searches for UUID
//! **`0x0100`, L2CAP**, which matches every service running over L2CAP, and
//! that is how it enumerates the device. A server that answers only a search
//! for `0x110B` is never found, and would present as a device that pairs and
//! then sits there — indistinguishable from having no server at all.
//!
//! That is read off the trace, not off the spec, and it is the single finding
//! that justified building the reference rig.
//!
//! ## Matching is ALL, not ANY
//!
//! A record matches a ServiceSearchPattern when **every** UUID in the pattern
//! appears somewhere in the record. Every pattern in the captured trace holds
//! exactly one UUID, so "any" and "all" cannot be told apart there — which is
//! exactly why it is worth implementing the rule rather than the observation.
//!
//! ## Data elements
//!
//! One byte of type and size, then the value. The type is the top five bits and
//! the size index the low three, which produces the constants that appear
//! throughout the trace:
//!
//!     0x09  unsigned int, 2 bytes      0x19  UUID, 2 bytes
//!     0x0A  unsigned int, 4 bytes      0x25  text string, 8-bit length
//!     0x35  sequence, 8-bit length     0x36  sequence, 16-bit length

const std = @import("std");

pub const Error = error{
    Malformed,
    BufferTooSmall,
    /// A request this server does not implement. Answered with an SDP error
    /// response rather than silence, because silence is what a broken server
    /// looks like.
    Unsupported,
};

/// PDU identifiers.
pub const pdu = struct {
    pub const error_response: u8 = 0x01;
    pub const service_search_request: u8 = 0x02;
    pub const service_search_response: u8 = 0x03;
    pub const service_attribute_request: u8 = 0x04;
    pub const service_attribute_response: u8 = 0x05;
    /// **The only one the phone actually sends**, in all three of its queries.
    pub const service_search_attribute_request: u8 = 0x06;
    pub const service_search_attribute_response: u8 = 0x07;
};

pub const error_code = struct {
    pub const unsupported_version: u16 = 0x0001;
    pub const invalid_record_handle: u16 = 0x0002;
    pub const invalid_syntax: u16 = 0x0003;
    pub const invalid_pdu_size: u16 = 0x0004;
    pub const invalid_continuation: u16 = 0x0005;
    pub const insufficient_resources: u16 = 0x0006;
};

pub const uuid = struct {
    pub const sdp: u16 = 0x0001;
    pub const l2cap: u16 = 0x0100;
    pub const avdtp: u16 = 0x0019;
    pub const public_browse_root: u16 = 0x1002;
    /// Device ID. **The phone asks for this first and stops if it gets
    /// nothing** — see the note on
    /// `device_id_uuids`.
    pub const pnp_information: u16 = 0x1200;
    pub const audio_sink: u16 = 0x110B;
    pub const advanced_audio: u16 = 0x110D;
};

pub const attr = struct {
    pub const service_record_handle: u16 = 0x0000;
    pub const service_class_id_list: u16 = 0x0001;
    pub const protocol_descriptor_list: u16 = 0x0004;
    pub const browse_group_list: u16 = 0x0005;
    pub const profile_descriptor_list: u16 = 0x0009;
    pub const service_name: u16 = 0x0100;
    pub const provider_name: u16 = 0x0102;
    pub const supported_features: u16 = 0x0311;
    // Device ID (PnPInformation).
    pub const specification_id: u16 = 0x0200;
    pub const vendor_id: u16 = 0x0201;
    pub const product_id: u16 = 0x0202;
    pub const version: u16 = 0x0203;
    pub const primary_record: u16 = 0x0204;
    pub const vendor_id_source: u16 = 0x0205;
};

/// A2DP sink feature bits. **`0x0002` is what this device should claim** — it
/// drives powered speakers. btstack's demo says `0x0001`, headphone, and that
/// is the one field in the reference record worth differing from.
pub const sink_feature = struct {
    pub const headphone: u16 = 0x0001;
    pub const speaker: u16 = 0x0002;
    pub const recorder: u16 = 0x0004;
    pub const amplifier: u16 = 0x0008;
};

/// Every UUID in the Device ID record.
///
/// **Registering this record is not optional, and that cost a flash cycle.**
/// An A2DP sink with only an AudioSink record answers the phone's first query —
/// a search for `0x1200` — with an empty list, and the phone stops there: it
/// never sends the broad L2CAP search that would find the sink, and it drops
/// the link having learned nothing. The symptom is a device that pairs
/// perfectly, exchanges SDP, and is still treated as having no services.
pub const device_id_uuids = [_]u16{
    uuid.pnp_information,
    uuid.public_browse_root,
};

/// Every UUID that appears anywhere in the record `writeAudioSinkRecord`
/// produces. Matching walks this rather than re-parsing our own output: the
/// record is built here, so its contents are known here, and a parser of our
/// own bytes would be a second place for the truth to live.
pub const audio_sink_uuids = [_]u16{
    uuid.audio_sink,
    uuid.l2cap,
    uuid.avdtp,
    uuid.public_browse_root,
    uuid.advanced_audio,
};

// ---------------------------------------------------------------------------
// Writing data elements
// ---------------------------------------------------------------------------

pub const Writer = struct {
    buf: []u8,
    len: usize = 0,

    pub fn init(buf: []u8) Writer {
        return .{ .buf = buf };
    }

    fn put(self: *Writer, bytes: []const u8) !void {
        if (self.len + bytes.len > self.buf.len) return Error.BufferTooSmall;
        @memcpy(self.buf[self.len..][0..bytes.len], bytes);
        self.len += bytes.len;
    }

    fn put1(self: *Writer, b: u8) !void {
        try self.put(&[_]u8{b});
    }

    pub fn uint16(self: *Writer, v: u16) !void {
        try self.put(&[_]u8{ 0x09, @truncate(v >> 8), @truncate(v) });
    }

    pub fn uint32(self: *Writer, v: u32) !void {
        try self.put(&[_]u8{
            0x0A,              @truncate(v >> 24), @truncate(v >> 16),
            @truncate(v >> 8), @truncate(v),
        });
    }

    pub fn uuid16(self: *Writer, v: u16) !void {
        try self.put(&[_]u8{ 0x19, @truncate(v >> 8), @truncate(v) });
    }

    /// A text string with an 8-bit length. **The NUL is not included** — SDP
    /// strings are length-counted, and appending one makes every name a byte
    /// longer than the peer expects.
    pub fn text(self: *Writer, s: []const u8) !void {
        if (s.len > 255) return Error.BufferTooSmall;
        try self.put(&[_]u8{ 0x25, @intCast(s.len) });
        try self.put(s);
    }

    /// Begin a sequence whose length is written back once its contents are
    /// known. Returns the position to hand to `endSequence`.
    ///
    /// **The width is fixed when the sequence opens**, before its size is
    /// known, so the caller says which to use. Growing an 8-bit length into a
    /// 16-bit one afterwards would move every byte already written.
    pub fn beginSequence(self: *Writer, wide: bool) !usize {
        try self.put1(if (wide) 0x36 else 0x35);
        const at = self.len;
        try self.put(if (wide) &[_]u8{ 0, 0 } else &[_]u8{0});
        return at;
    }

    pub fn endSequence(self: *Writer, at: usize) !void {
        const wide = self.buf[at - 1] == 0x36;
        const inner = self.len - at - @as(usize, if (wide) 2 else 1);
        if (wide) {
            std.mem.writeInt(u16, self.buf[at..][0..2], @intCast(inner), .big);
        } else {
            if (inner > 255) return Error.BufferTooSmall;
            self.buf[at] = @intCast(inner);
        }
    }

    pub fn boolean(self: *Writer, v: bool) !void {
        try self.put(&[_]u8{ 0x28, @intFromBool(v) });
    }

    pub fn attribute(self: *Writer, id: u16) !void {
        try self.uint16(id);
    }

    pub fn slice(self: *Writer) []u8 {
        return self.buf[0..self.len];
    }
};

// ---------------------------------------------------------------------------
// The record
// ---------------------------------------------------------------------------

/// Write the AudioSink service record, in the attribute order the reference
/// uses. **Attributes must ascend by ID**, which they do here and which a
/// reordering would silently break.
pub fn writeAudioSinkRecord(
    out: []u8,
    handle: u32,
    service_name: []const u8,
    provider_name: []const u8,
    features: u16,
) ![]u8 {
    var w = Writer.init(out);
    const rec = try w.beginSequence(true);

    try w.attribute(attr.service_record_handle);
    try w.uint32(handle);

    try w.attribute(attr.service_class_id_list);
    const cls = try w.beginSequence(true);
    try w.uuid16(uuid.audio_sink);
    try w.endSequence(cls);

    try w.attribute(attr.protocol_descriptor_list);
    const proto = try w.beginSequence(true);
    const l2 = try w.beginSequence(true);
    try w.uuid16(uuid.l2cap);
    try w.uint16(0x0019); // PSM — AVDTP
    try w.endSequence(l2);
    const av = try w.beginSequence(true);
    try w.uuid16(uuid.avdtp);
    try w.uint16(0x0103); // AVDTP 1.3
    try w.endSequence(av);
    try w.endSequence(proto);

    try w.attribute(attr.browse_group_list);
    const browse = try w.beginSequence(true);
    try w.uuid16(uuid.public_browse_root);
    try w.endSequence(browse);

    try w.attribute(attr.profile_descriptor_list);
    const profs = try w.beginSequence(true);
    const prof = try w.beginSequence(true);
    try w.uuid16(uuid.advanced_audio);
    try w.uint16(0x0104); // A2DP 1.4
    try w.endSequence(prof);
    try w.endSequence(profs);

    try w.attribute(attr.service_name);
    try w.text(service_name);

    try w.attribute(attr.provider_name);
    try w.text(provider_name);

    try w.attribute(attr.supported_features);
    try w.uint16(features);

    try w.endSequence(rec);
    return w.slice();
}

/// Vendor ID sources.
pub const vendor_source = struct {
    pub const bluetooth_sig: u16 = 0x0001;
    pub const usb_if: u16 = 0x0002;
};

/// Write the Device ID record. Attributes ascend, as they must.
pub fn writeDeviceIdRecord(
    out: []u8,
    handle: u32,
    vendor: u16,
    product: u16,
    ver: u16,
    primary: bool,
    source: u16,
) ![]u8 {
    var w = Writer.init(out);
    const rec = try w.beginSequence(true);

    try w.attribute(attr.service_record_handle);
    try w.uint32(handle);

    try w.attribute(attr.service_class_id_list);
    const cls = try w.beginSequence(true);
    try w.uuid16(uuid.pnp_information);
    try w.endSequence(cls);

    try w.attribute(attr.browse_group_list);
    const browse = try w.beginSequence(true);
    try w.uuid16(uuid.public_browse_root);
    try w.endSequence(browse);

    try w.attribute(attr.specification_id);
    try w.uint16(0x0103); // Device ID 1.3

    try w.attribute(attr.vendor_id);
    try w.uint16(vendor);

    try w.attribute(attr.product_id);
    try w.uint16(product);

    try w.attribute(attr.version);
    try w.uint16(ver);

    try w.attribute(attr.primary_record);
    try w.boolean(primary);

    try w.attribute(attr.vendor_id_source);
    try w.uint16(source);

    try w.endSequence(rec);
    return w.slice();
}

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

pub const Header = struct {
    id: u8,
    transaction: u16,
    params: []const u8,
};

pub fn parseHeader(buf: []const u8) !Header {
    if (buf.len < 5) return Error.Malformed;
    const len: usize = std.mem.readInt(u16, buf[3..5], .big);
    if (5 + len > buf.len) return Error.Malformed;
    return .{
        .id = buf[0],
        .transaction = std.mem.readInt(u16, buf[1..3], .big),
        .params = buf[5..][0..len],
    };
}

pub const SearchAttributeRequest = struct {
    /// Up to twelve UUIDs, which is the spec's maximum for a search pattern.
    pattern: [12]u16 = @splat(0),
    pattern_len: usize = 0,
    max_attribute_bytes: u16 = 0,
    /// True when the pattern held something this parser could not read — a
    /// 128-bit UUID, most likely. **Recorded rather than ignored**: treating an
    /// unreadable pattern as an empty one would match everything.
    pattern_truncated: bool = false,
    continuation: []const u8 = &.{},
};

/// Read the UUID16s out of a data-element sequence, skipping elements that are
/// not 16-bit UUIDs but noting that they were there.
fn readUuidSequence(buf: []const u8, req: anytype) !usize {
    if (buf.len < 1) return Error.Malformed;
    const kind = buf[0];
    var at: usize = 1;
    var total: usize = 0;
    switch (kind) {
        0x35 => {
            if (buf.len < 2) return Error.Malformed;
            total = buf[1];
            at = 2;
        },
        0x36 => {
            if (buf.len < 3) return Error.Malformed;
            total = std.mem.readInt(u16, buf[1..3], .big);
            at = 3;
        },
        else => return Error.Malformed,
    }
    if (at + total > buf.len) return Error.Malformed;

    const end = at + total;
    while (at < end) {
        const t = buf[at];
        if (t == 0x19 and at + 3 <= end) {
            if (req.pattern_len < req.pattern.len) {
                req.pattern[req.pattern_len] = std.mem.readInt(u16, buf[at + 1 ..][0..2], .big);
                req.pattern_len += 1;
            } else {
                req.pattern_truncated = true;
            }
            at += 3;
        } else if (t == 0x1A and at + 5 <= end) {
            // 32-bit UUID. Not used by anything here.
            req.pattern_truncated = true;
            at += 5;
        } else if (t == 0x1C and at + 17 <= end) {
            // 128-bit UUID.
            req.pattern_truncated = true;
            at += 17;
        } else {
            return Error.Malformed;
        }
    }
    return end;
}

pub fn parseSearchAttributeRequest(params: []const u8) !SearchAttributeRequest {
    var req = SearchAttributeRequest{};
    const after_pattern = try readUuidSequence(params, &req);

    if (after_pattern + 2 > params.len) return Error.Malformed;
    req.max_attribute_bytes = std.mem.readInt(u16, params[after_pattern..][0..2], .big);
    var at = after_pattern + 2;

    // The AttributeIDList. Its contents are not read: this server has one
    // record and returns all of it, which every observed request asks for
    // (0x0000-0xFFFF). Skipping it correctly still matters, because the
    // continuation state sits behind it.
    //
    // **"Every observed request" is the phrasing that cost a day**, one function
    // down, where it justified answering a single PDU. It is an honest
    // statement of what has been seen and not a claim about the protocol: a
    // peer asking for a subset would be sent attributes it did not request,
    // which is legal-ish and might not be tolerated. If a third device ever
    // fails at SDP, filter here before suspecting anything else.
    if (at >= params.len) return Error.Malformed;
    const kind = params[at];
    const hdr: usize = switch (kind) {
        0x35 => 2,
        0x36 => 3,
        else => return Error.Malformed,
    };
    if (at + hdr > params.len) return Error.Malformed;
    const alen: usize = if (kind == 0x35)
        params[at + 1]
    else
        std.mem.readInt(u16, params[at + 1 ..][0..2], .big);
    at += hdr + alen;
    if (at >= params.len) return Error.Malformed;

    const cont_len: usize = params[at];
    at += 1;
    if (at + cont_len > params.len) return Error.Malformed;
    req.continuation = params[at..][0..cont_len];
    return req;
}

/// **Every** UUID in the pattern must appear in `have`. See the header: with
/// single-UUID patterns this is indistinguishable from "any", which is why the
/// rule is implemented rather than the observation.
pub fn matches(pattern: []const u16, have: []const u16) bool {
    if (pattern.len == 0) return false;
    for (pattern) |want| {
        var found = false;
        for (have) |h| {
            if (h == want) {
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Responses
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ServiceSearch (0x02) and ServiceAttribute (0x04) — the two-step flow
// ---------------------------------------------------------------------------
//
// **macOS asks these; Android does not.** Until 2026-08-27 this server answered
// only `service_search_attribute_request` (0x06), the combined query, because
// "every observed request is this one PDU" — and every observation had been
// against one Android phone. A MacBook Air opened SDP, sent `0x02`, was told
// `invalid_syntax`, closed the channel and dropped the link. From the outside
// that is "it shows in the connecting box and won't connect".
//
// The two-step flow is: **0x02** asks which records match a pattern and gets
// back their handles, then **0x04** asks for one handle's attributes. Formats
// traced from btstack's `sdp_server.c`, which is the reference this package
// already checks its records against.

/// A `ServiceSearchRequest`. Field names match `SearchAttributeRequest` so both
/// can share `readUuidSequence`.
pub const SearchRequest = struct {
    pattern: [12]u16 = @splat(0),
    pattern_len: usize = 0,
    pattern_truncated: bool = false,
    /// The client's cap on how many handles it wants back.
    max_records: u16 = 0,
    continuation: []const u8 = &.{},
};

pub fn parseSearchRequest(params: []const u8) !SearchRequest {
    var req = SearchRequest{};
    var at = try readUuidSequence(params, &req);

    if (at + 2 > params.len) return Error.Malformed;
    req.max_records = std.mem.readInt(u16, params[at..][0..2], .big);
    at += 2;

    if (at >= params.len) return Error.Malformed;
    const cont_len: usize = params[at];
    at += 1;
    if (at + cont_len > params.len) return Error.Malformed;
    req.continuation = params[at..][0..cont_len];
    return req;
}

/// Build a `ServiceSearchResponse` carrying every matching record handle.
///
/// **No continuation, for the same reason `writeSearchAttributeResponse` has
/// none**: this device has two records, so four bytes each against any MTU a
/// peer will negotiate always fits. `BufferTooSmall` rather than a truncated
/// list if that stops being true — a short handle list is a lie the client
/// cannot detect.
pub fn writeSearchResponse(out: []u8, transaction: u16, handles: []const u32) ![]u8 {
    const total = 5 + 2 + 2 + handles.len * 4 + 1;
    if (out.len < total) return Error.BufferTooSmall;

    out[0] = pdu.service_search_response;
    std.mem.writeInt(u16, out[1..3], transaction, .big);
    std.mem.writeInt(u16, out[3..5], @intCast(2 + 2 + handles.len * 4 + 1), .big);
    // Total and current are equal because everything matching goes in one
    // response; they differ only when a continuation splits the list.
    std.mem.writeInt(u16, out[5..7], @intCast(handles.len), .big);
    std.mem.writeInt(u16, out[7..9], @intCast(handles.len), .big);
    var at: usize = 9;
    for (handles) |h| {
        std.mem.writeInt(u32, out[at..][0..4], h, .big);
        at += 4;
    }
    out[at] = 0; // continuation state: none
    return out[0..total];
}

/// A `ServiceAttributeRequest`.
///
/// **The requested attribute IDs are skipped rather than read.** This server
/// answers with the whole record, which is what `writeSearchAttributeResponse`
/// has always done for the combined query and what every peer so far has
/// accepted. Filtering is the strictly correct behaviour and would matter to a
/// client that asked for one attribute and could not tolerate the rest; none
/// has appeared, and an unused filter is an untested one.
pub const AttributeRequest = struct {
    handle: u32 = 0,
    max_attribute_bytes: u16 = 0,
    continuation: []const u8 = &.{},
};

/// Total length of a data-element sequence, header included.
fn sequenceTotalLen(buf: []const u8) !usize {
    if (buf.len < 1) return Error.Malformed;
    return switch (buf[0]) {
        0x35 => if (buf.len < 2) Error.Malformed else 2 + @as(usize, buf[1]),
        0x36 => if (buf.len < 3) Error.Malformed else 3 + @as(usize, std.mem.readInt(u16, buf[1..3], .big)),
        else => Error.Malformed,
    };
}

pub fn parseAttributeRequest(params: []const u8) !AttributeRequest {
    if (params.len < 6) return Error.Malformed;
    var req = AttributeRequest{
        .handle = std.mem.readInt(u32, params[0..4], .big),
        .max_attribute_bytes = std.mem.readInt(u16, params[4..6], .big),
    };
    var at: usize = 6;

    const id_list = try sequenceTotalLen(params[at..]);
    if (at + id_list > params.len) return Error.Malformed;
    at += id_list;

    if (at >= params.len) return Error.Malformed;
    const cont_len: usize = params[at];
    at += 1;
    if (at + cont_len > params.len) return Error.Malformed;
    req.continuation = params[at..][0..cont_len];
    return req;
}

/// Build a `ServiceAttributeResponse` carrying one record whole.
///
/// The record is already a data-element sequence, which is exactly what an
/// `AttributeList` is — so it goes in as it stands, with no extra nesting.
/// That is the difference from `writeSearchAttributeResponse`, whose
/// `AttributeLists` is a sequence *of* such lists and needs one more wrapper.
pub fn writeAttributeResponse(
    out: []u8,
    transaction: u16,
    record: []const u8,
    max_attribute_bytes: u16,
) ![]u8 {
    if (record.len > max_attribute_bytes) return Error.BufferTooSmall;
    const total = 5 + 2 + record.len + 1;
    if (out.len < total) return Error.BufferTooSmall;

    out[0] = pdu.service_attribute_response;
    std.mem.writeInt(u16, out[1..3], transaction, .big);
    std.mem.writeInt(u16, out[3..5], @intCast(2 + record.len + 1), .big);
    std.mem.writeInt(u16, out[5..7], @intCast(record.len), .big);
    @memcpy(out[7..][0..record.len], record);
    out[7 + record.len] = 0; // continuation state: none
    return out[0..total];
}

/// The handle a record declares as its own first attribute.
///
/// **Read out of the record rather than tracked beside it.** A handle held in
/// two places is a handle that can disagree with itself, and `0x04` looks a
/// record up by exactly the number `0x02` handed out. Returns null if the
/// record does not open the way `writeAudioSinkRecord` and
/// `writeDeviceIdRecord` build it — a sequence, then attribute `0x0000`, then a
/// `uint32`.
pub fn recordHandle(record: []const u8) ?u32 {
    if (record.len < 11) return null;
    if (record[0] != 0x36) return null; // wide sequence
    if (record[3] != 0x09 or record[4] != 0x00 or record[5] != 0x00) return null; // attribute 0
    if (record[6] != 0x0A) return null; // uint32
    return std.mem.readInt(u32, record[7..11], .big);
}

/// Build a ServiceSearchAttributeResponse carrying `record`, whole.
///
/// **No continuation.** One record of about 140 bytes against a
/// MaximumAttributeByteCount of 1008 always fits, and a continuation path that
/// is never taken is a path nobody has watched. It returns `BufferTooSmall`
/// rather than truncating if that ever stops being true — a truncated
/// attribute list is a malformed record, and the phone's symptom would be a
/// device that appears and then does nothing.
pub fn writeSearchAttributeResponse(
    out: []u8,
    transaction: u16,
    records: []const []const u8,
    max_attribute_bytes: u16,
) ![]u8 {
    var inner: usize = 0;
    for (records) |r| inner += r.len;

    // AttributeLists is a sequence *of* attribute lists, so the records nest
    // inside one more sequence. The reference returns three at once to a broad
    // search, so carrying several is the normal case, not an edge one.
    const lists_len = inner + 3;
    if (lists_len > max_attribute_bytes) return Error.BufferTooSmall;

    const total = 5 + 2 + lists_len + 1;
    if (out.len < total) return Error.BufferTooSmall;

    out[0] = pdu.service_search_attribute_response;
    std.mem.writeInt(u16, out[1..3], transaction, .big);
    std.mem.writeInt(u16, out[3..5], @intCast(2 + lists_len + 1), .big);
    std.mem.writeInt(u16, out[5..7], @intCast(lists_len), .big);
    out[7] = 0x36;
    std.mem.writeInt(u16, out[8..10], @intCast(inner), .big);
    var at: usize = 10;
    for (records) |r| {
        @memcpy(out[at..][0..r.len], r);
        at += r.len;
    }
    out[at] = 0; // continuation state: none
    return out[0..total];
}

/// The same response with nothing in it — the correct answer to a search for a
/// service this device does not have, and one the phone asks for every time
/// (`0x112E` in the trace).
pub fn writeEmptyResponse(out: []u8, transaction: u16) ![]u8 {
    if (out.len < 9) return Error.BufferTooSmall;
    out[0] = pdu.service_search_attribute_response;
    std.mem.writeInt(u16, out[1..3], transaction, .big);
    std.mem.writeInt(u16, out[3..5], 4, .big);
    std.mem.writeInt(u16, out[5..7], 1, .big); // AttributeListsByteCount
    out[7] = 0x35; // empty sequence
    out[8] = 0x00;
    // continuation state
    if (out.len < 10) return Error.BufferTooSmall;
    out[9] = 0;
    std.mem.writeInt(u16, out[3..5], 5, .big);
    return out[0..10];
}

pub fn writeErrorResponse(out: []u8, transaction: u16, code: u16) ![]u8 {
    if (out.len < 7) return Error.BufferTooSmall;
    out[0] = pdu.error_response;
    std.mem.writeInt(u16, out[1..3], transaction, .big);
    std.mem.writeInt(u16, out[3..5], 2, .big);
    std.mem.writeInt(u16, out[5..7], code, .big);
    return out[0..7];
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// The AudioSink record btstack's `a2dp_sink_demo` served during a capture on
/// this board, byte for byte.
const reference_record = "36008c" ++
    "0900000a00010001" ++
    "09000136000319110b" ++
    "090004360012360006190100090019360006190019090103" ++
    "090005360003191002" ++
    "09000936000936000619110d090104" ++
    "09010025194254737461636b20413244502053696e6b2053657276696365" ++
    "09010225224254737461636b20413244502053696e6b20536572766963652050726f7669646572" ++
    "090311090001";

test "our record reproduces the reference byte for byte" {
    // **The strongest test in this package**: not that the builder is
    // self-consistent, but that it emits exactly what a sink that played music
    // emitted, given the same inputs. Every element type, every nesting depth,
    // every length field.
    var expected: [256]u8 = undefined;
    const want = try std.fmt.hexToBytes(&expected, reference_record);

    var buf: [256]u8 = undefined;
    const got = try writeAudioSinkRecord(
        &buf,
        0x00010001,
        "BTstack A2DP Sink Service",
        "BTstack A2DP Sink Service Provider",
        sink_feature.headphone,
    );

    try testing.expectEqualSlices(u8, want, got);
}

test "the speaker feature bit is the one field we differ on" {
    var buf: [256]u8 = undefined;
    const rec = try writeAudioSinkRecord(&buf, 0x00010001, "zig-speaker", "zig", sink_feature.speaker);
    // SupportedFeatures is the last attribute: 09 0311 09 0002.
    try testing.expectEqualSlices(u8, &.{ 0x09, 0x03, 0x11, 0x09, 0x00, 0x02 }, rec[rec.len - 6 ..]);
    try testing.expect(sink_feature.speaker != sink_feature.headphone);
}

test "the phone's three real requests parse" {
    // Verbatim from a capture of a phone talking to btstack's sink.
    var buf: [64]u8 = undefined;

    const device_id = try std.fmt.hexToBytes(&buf, "350319120003f035050a0000ffff00");
    var r = try parseSearchAttributeRequest(device_id);
    try testing.expectEqual(@as(usize, 1), r.pattern_len);
    try testing.expectEqual(@as(u16, 0x1200), r.pattern[0]);
    try testing.expectEqual(@as(u16, 0x03F0), r.max_attribute_bytes);
    try testing.expectEqual(@as(usize, 0), r.continuation.len);

    const l2cap_search = try std.fmt.hexToBytes(&buf, "350319010003f035050a0000ffff00");
    r = try parseSearchAttributeRequest(l2cap_search);
    try testing.expectEqual(@as(u16, uuid.l2cap), r.pattern[0]);

    const pbap = try std.fmt.hexToBytes(&buf, "35031911 2e03f035050a0000ffff00"[0..0] ++ "350319112e03f035050a0000ffff00");
    r = try parseSearchAttributeRequest(pbap);
    try testing.expectEqual(@as(u16, 0x112E), r.pattern[0]);
}

test "the L2CAP search finds us and the PBAP search does not" {
    // **This is the finding the reference rig paid for.** The phone enumerates
    // by searching for L2CAP, not for AudioSink; a server that matched only
    // 0x110B would never be discovered.
    try testing.expect(matches(&.{uuid.l2cap}, &audio_sink_uuids));
    try testing.expect(matches(&.{uuid.audio_sink}, &audio_sink_uuids));
    try testing.expect(matches(&.{uuid.public_browse_root}, &audio_sink_uuids));

    try testing.expect(!matches(&.{0x112E}, &audio_sink_uuids)); // PBAP
    try testing.expect(!matches(&.{0x1200}, &audio_sink_uuids)); // DeviceID
}

test "matching requires every UUID in the pattern, not any of them" {
    // Single-UUID patterns cannot tell these apart, which is why the rule is
    // implemented rather than the observation.
    try testing.expect(matches(&.{ uuid.l2cap, uuid.audio_sink }, &audio_sink_uuids));
    try testing.expect(!matches(&.{ uuid.l2cap, 0x112E }, &audio_sink_uuids));
    try testing.expect(!matches(&.{}, &audio_sink_uuids));
}

test "a response carries the record inside one more sequence" {
    var rbuf: [256]u8 = undefined;
    const rec = try writeAudioSinkRecord(&rbuf, 0x00010001, "zig-speaker", "zig", sink_feature.speaker);

    var buf: [512]u8 = undefined;
    const rsp = try writeSearchAttributeResponse(&buf, 0x1234, &.{rec}, 0x03F0);

    try testing.expectEqual(pdu.service_search_attribute_response, rsp[0]);
    try testing.expectEqual(@as(u16, 0x1234), std.mem.readInt(u16, rsp[1..3], .big));
    // ParameterLength counts everything after the five-byte header.
    try testing.expectEqual(@as(usize, 5 + std.mem.readInt(u16, rsp[3..5], .big)), rsp.len);
    // AttributeListsByteCount, then the wrapping sequence, then our record.
    try testing.expectEqual(@as(u16, @intCast(rec.len + 3)), std.mem.readInt(u16, rsp[5..7], .big));
    try testing.expectEqual(@as(u8, 0x36), rsp[7]);
    try testing.expectEqualSlices(u8, rec, rsp[10 .. 10 + rec.len]);
    try testing.expectEqual(@as(u8, 0), rsp[rsp.len - 1]); // no continuation
}

test "a response too big for the peer's limit is refused, not truncated" {
    var rbuf: [256]u8 = undefined;
    const rec = try writeAudioSinkRecord(&rbuf, 1, "zig-speaker", "zig", sink_feature.speaker);
    var buf: [512]u8 = undefined;
    // A truncated attribute list is a malformed record, and the phone's symptom
    // would be a device that appears and then does nothing.
    try testing.expectError(
        Error.BufferTooSmall,
        writeSearchAttributeResponse(&buf, 1, &.{rec}, 32),
    );
}

test "an empty response is well formed" {
    var buf: [32]u8 = undefined;
    const rsp = try writeEmptyResponse(&buf, 0x0007);
    try testing.expectEqual(pdu.service_search_attribute_response, rsp[0]);
    try testing.expectEqual(@as(u16, 0x0007), std.mem.readInt(u16, rsp[1..3], .big));
    try testing.expectEqual(@as(usize, 5 + std.mem.readInt(u16, rsp[3..5], .big)), rsp.len);
    try testing.expectEqual(@as(u8, 0), rsp[rsp.len - 1]);
}

test "a truncated request is refused rather than read short" {
    try testing.expectError(Error.Malformed, parseSearchAttributeRequest(&.{ 0x35, 0x03 }));
    try testing.expectError(Error.Malformed, parseSearchAttributeRequest(&.{}));
    try testing.expectError(Error.Malformed, parseHeader(&.{ 0x06, 0x00 }));
    // A header whose declared length runs past the buffer.
    try testing.expectError(Error.Malformed, parseHeader(&.{ 0x06, 0, 0, 0xFF, 0xFF, 0x01 }));
}

test "a 128-bit UUID in the pattern is noticed, not silently ignored" {
    // Treating an unreadable pattern as empty would match everything.
    var req = SearchAttributeRequest{};
    var buf: [32]u8 = undefined;
    const seq = try std.fmt.hexToBytes(&buf, "351100000000000000000000000000000000"[0..0] ++
        "35111c0000111100001000800000805f9b34fb");
    _ = try readUuidSequence(seq, &req);
    try testing.expect(req.pattern_truncated);
    try testing.expectEqual(@as(usize, 0), req.pattern_len);
    try testing.expect(!matches(req.pattern[0..req.pattern_len], &audio_sink_uuids));
}

test "sequence lengths are written back at the right width" {
    var buf: [64]u8 = undefined;
    var w = Writer.init(&buf);
    const outer = try w.beginSequence(false);
    try w.uuid16(0x110B);
    try w.endSequence(outer);
    try testing.expectEqualSlices(u8, &.{ 0x35, 0x03, 0x19, 0x11, 0x0B }, w.slice());

    w = Writer.init(&buf);
    const wide = try w.beginSequence(true);
    try w.uuid16(0x110B);
    try w.endSequence(wide);
    try testing.expectEqualSlices(u8, &.{ 0x36, 0x00, 0x03, 0x19, 0x11, 0x0B }, w.slice());
}

test "a text string is length-counted and carries no NUL" {
    var buf: [32]u8 = undefined;
    var w = Writer.init(&buf);
    try w.text("zig");
    try testing.expectEqualSlices(u8, &.{ 0x25, 0x03, 'z', 'i', 'g' }, w.slice());
}

test "our Device ID record reproduces the reference byte for byte" {
    const want_hex = "36003d0900000a0001000409000136000319120009000536000319100209020009010309020109048f0902020900010902030900010902042801090205090001";
    var expected: [128]u8 = undefined;
    const want = try std.fmt.hexToBytes(&expected, want_hex);

    var buf: [128]u8 = undefined;
    const got = try writeDeviceIdRecord(&buf, 0x00010004, 0x048F, 0x0001, 0x0001, true, vendor_source.bluetooth_sig);
    try testing.expectEqualSlices(u8, want, got);
}

test "the Device ID search matches its record and not the sink's" {
    // **This is the bug rung 23b shipped with.** The phone's first query is a
    // search for 0x1200; with only an AudioSink record it matched nothing, the
    // phone stopped there, and the broad L2CAP search that would have found the
    // sink was never sent.
    try testing.expect(matches(&.{uuid.pnp_information}, &device_id_uuids));
    try testing.expect(!matches(&.{uuid.pnp_information}, &audio_sink_uuids));

    // And the broad search still finds the sink, as before.
    try testing.expect(matches(&.{uuid.l2cap}, &audio_sink_uuids));
    try testing.expect(!matches(&.{uuid.l2cap}, &device_id_uuids));

    // Both are in the public browse group, so a browse finds both.
    try testing.expect(matches(&.{uuid.public_browse_root}, &device_id_uuids));
    try testing.expect(matches(&.{uuid.public_browse_root}, &audio_sink_uuids));
}

test "a response carries several records at once" {
    // The reference answers the broad L2CAP search with three records in one
    // response, so carrying more than one is the normal case.
    var a: [128]u8 = undefined;
    var b: [256]u8 = undefined;
    const dev = try writeDeviceIdRecord(&a, 1, 1, 2, 3, true, vendor_source.bluetooth_sig);
    const sink = try writeAudioSinkRecord(&b, 2, "zig-speaker", "zig", sink_feature.speaker);

    var out: [512]u8 = undefined;
    const rsp = try writeSearchAttributeResponse(&out, 7, &.{ dev, sink }, 0x03F0);

    try testing.expectEqual(@as(u16, @intCast(dev.len + sink.len + 3)), std.mem.readInt(u16, rsp[5..7], .big));
    try testing.expectEqualSlices(u8, dev, rsp[10 .. 10 + dev.len]);
    try testing.expectEqualSlices(u8, sink, rsp[10 + dev.len .. 10 + dev.len + sink.len]);
    try testing.expectEqual(@as(u8, 0), rsp[rsp.len - 1]);
    try testing.expectEqual(@as(usize, 5 + std.mem.readInt(u16, rsp[3..5], .big)), rsp.len);
}

// -- the two-step flow macOS uses -------------------------------------------

test "a ServiceSearchRequest is read, pattern then count then continuation" {
    // DES(8) holding one UUID16 0x110B (AudioSink), max 3 records, no
    // continuation — the shape a Mac sends.
    const params = [_]u8{
        0x35, 0x03, 0x19, 0x11, 0x0B, // pattern
        0x00, 0x03, // MaximumServiceRecordCount
        0x00, // continuation state: none
    };
    const req = try parseSearchRequest(&params);
    try testing.expectEqual(@as(usize, 1), req.pattern_len);
    try testing.expectEqual(@as(u16, 0x110B), req.pattern[0]);
    try testing.expectEqual(@as(u16, 3), req.max_records);
    try testing.expectEqual(@as(usize, 0), req.continuation.len);
    try testing.expect(!req.pattern_truncated);
}

test "a ServiceSearchResponse is counts then handles then continuation" {
    var buf: [64]u8 = @splat(0);
    const rsp = try writeSearchResponse(&buf, 0x1234, &.{ 0x0001_0001, 0x0001_0004 });

    try testing.expectEqualSlices(u8, &.{
        0x03, // ServiceSearchResponse
        0x12, 0x34, // transaction
        0x00, 0x0D, // ParameterLength: 2 + 2 + 8 + 1
        0x00, 0x02, // TotalServiceRecordCount
        0x00, 0x02, // CurrentServiceRecordCount
        0x00, 0x01, 0x00, 0x01, // handle 1
        0x00, 0x01, 0x00, 0x04, // handle 2
        0x00, // continuation state
    }, rsp);
}

test "a ServiceAttributeRequest is handle, cap, id list, continuation" {
    const params = [_]u8{
        0x00, 0x01, 0x00, 0x01, // ServiceRecordHandle
        0x02, 0x00, // MaximumAttributeByteCount 512
        0x35, 0x03, 0x09, 0x00, 0x00, // AttributeIDList: DES holding attr 0x0000
        0x00, // continuation state
    };
    const req = try parseAttributeRequest(&params);
    try testing.expectEqual(@as(u32, 0x0001_0001), req.handle);
    try testing.expectEqual(@as(u16, 512), req.max_attribute_bytes);
    try testing.expectEqual(@as(usize, 0), req.continuation.len);
}

test "a ServiceAttributeResponse carries the record with no extra nesting" {
    // **The difference from the combined response**, and the easy mistake:
    // `AttributeLists` (0x07) is a sequence *of* lists and needs a wrapper;
    // `AttributeList` (0x05) is one list, and the record already is one.
    const record = [_]u8{ 0x36, 0x00, 0x02, 0xAA, 0xBB };
    var buf: [64]u8 = @splat(0);
    const rsp = try writeAttributeResponse(&buf, 0x0007, &record, 512);

    try testing.expectEqualSlices(u8, &.{
        0x05, // ServiceAttributeResponse
        0x00, 0x07, // transaction
        0x00, 0x08, // ParameterLength: 2 + 5 + 1
        0x00, 0x05, // AttributeListByteCount
        0x36, 0x00, 0x02, 0xAA, 0xBB, // the record, verbatim
        0x00, // continuation state
    }, rsp);
}

test "a record too big for the client's cap is refused, not truncated" {
    const record = [_]u8{ 0x36, 0x00, 0x02, 0xAA, 0xBB };
    var buf: [64]u8 = @splat(0);
    try testing.expectError(Error.BufferTooSmall, writeAttributeResponse(&buf, 1, &record, 4));
}

test "a record's handle is read back out of the record it was written into" {
    // **One source of truth.** `0x04` looks up exactly the number `0x02` handed
    // out, so both come from the record itself.
    var buf: [256]u8 = @splat(0);
    const rec = try writeAudioSinkRecord(&buf, 0x0001_0001, "zig-speaker", "zig", sink_feature.speaker);
    try testing.expectEqual(@as(?u32, 0x0001_0001), recordHandle(rec));

    var buf2: [256]u8 = @splat(0);
    const dev = try writeDeviceIdRecord(&buf2, 0x0001_0004, 0x048F, 0x0001, 0x0001, true, vendor_source.bluetooth_sig);
    try testing.expectEqual(@as(?u32, 0x0001_0004), recordHandle(dev));

    // Not a record.
    try testing.expectEqual(@as(?u32, null), recordHandle(&.{ 0x35, 0x01, 0x00 }));
}

test "malformed two-step requests are refused rather than half-read" {
    // Truncated everywhere a length could lie.
    try testing.expectError(Error.Malformed, parseSearchRequest(&.{0x35}));
    try testing.expectError(Error.Malformed, parseSearchRequest(&.{ 0x35, 0x03, 0x19, 0x11, 0x0B }));
    try testing.expectError(Error.Malformed, parseSearchRequest(&.{ 0x35, 0x03, 0x19, 0x11, 0x0B, 0x00, 0x03 }));
    try testing.expectError(Error.Malformed, parseAttributeRequest(&.{ 0x00, 0x01 }));
    try testing.expectError(Error.Malformed, parseAttributeRequest(&.{ 0, 1, 0, 1, 2, 0, 0x35, 0x09 }));
}
