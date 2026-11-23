//! HTTP/1.1 message syntax — RFC 9112, with semantics from RFC 9110. Rung N10.
//!
//! **Pure, like everything else below `ipstack.zig`**: functions over
//! caller-supplied bytes, no device, no state, no allocator. Enough to make a
//! request and understand the head of a response, and no more.
//!
//! ## Chunked transfer coding is refused, by name
//!
//! §6.3 rule 4 requires a client to decode chunked bodies, and this does not.
//! It returns `error.ChunkedUnsupported` rather than reading the chunk-size
//! lines as though they were content — which is the failure mode that matters,
//! because a chunked body *looks* like a body and a stack that ignored the
//! framing would deliver something almost right. "We never saw a chunked
//! response" is not a design.
//!
//! **The request asks not to get one.** `HTTP/1.0` in the request line means a
//! conforming server must not use chunked encoding at all (it is an HTTP/1.1
//! feature), so the refusal is a backstop rather than the primary defence.
//!
//! ## And the length rules are the other half
//!
//! §6.3 gives eight rules in precedence order. Four of them can reach a client
//! here, and getting the order wrong is how a response is truncated or
//! over-read:
//!
//!   1. 1xx, 204 and 304 have **no body**, whatever the headers say.
//!   3. `Transfer-Encoding` **overrides** `Content-Length` — and a message
//!      carrying both "ought to be handled as an error" because it is the
//!      shape of a request-smuggling attempt.
//!   6. A valid `Content-Length` gives the length.
//!   8. Otherwise the body runs until the server closes the connection.

const std = @import("std");

pub const max_status_line: usize = 128;

pub const Error = error{
    /// The head is not complete yet — more bytes are needed, not an error in
    /// what has arrived.
    Incomplete,
    Malformed,
    /// §6.3 rule 4. See the module comment.
    ChunkedUnsupported,
    /// §6.3 rule 3: both framings present, which is the shape of a smuggling
    /// attempt rather than a server being generous.
    ConflictingLength,
    BufferTooSmall,
};

/// How the body's end is determined — §6.3, once the rules have been applied.
pub const BodyLength = union(enum) {
    /// Rules 1 and 7: there is no body at all.
    none,
    /// Rule 6: this many octets.
    length: u64,
    /// Rule 8: read until the server closes. **Indistinguishable from a
    /// truncated transfer**, which the RFC says in as many words — so a caller
    /// that needs to know it got everything wants a server that sends
    /// `Content-Length`.
    until_close,
};

pub const Response = struct {
    status: u16,
    /// Offset just past the blank line that ends the head.
    body_at: usize,
    body: BodyLength,
};

/// Build a GET.
///
/// **`HTTP/1.0` and `Connection: close`**, deliberately. This stack has one
/// connection, no persistent-connection bookkeeping and no chunked decoder;
/// asking for HTTP/1.1 would invite both. The `Host` header is sent even
/// though 1.0 does not require it, because virtual hosts are universal and a
/// server without it answers with whatever it considers its default site.
pub fn buildGet(host: []const u8, path: []const u8, out: []u8) ![]u8 {
    var w: usize = 0;
    const parts = [_][]const u8{
        "GET ",                          path, " HTTP/1.0\r\nHost: ", host,
        "\r\nConnection: close\r\n\r\n",
    };
    for (parts) |p| {
        if (w + p.len > out.len) return Error.BufferTooSmall;
        @memcpy(out[w..][0..p.len], p);
        w += p.len;
    }
    return out[0..w];
}

/// Case-insensitive comparison, which is what §5 requires of field names.
fn eqlIgnoreCase(a: []const u8, b: []const u8) bool {
    if (a.len != b.len) return false;
    for (a, b) |x, y| {
        if (std.ascii.toLower(x) != std.ascii.toLower(y)) return false;
    }
    return true;
}

fn trim(s: []const u8) []const u8 {
    return std.mem.trim(u8, s, " \t");
}

/// Parse the head of a response.
///
/// Returns `error.Incomplete` until the blank line has arrived, so a caller can
/// feed it a growing buffer and stop when it stops asking for more.
pub fn parseResponse(buf: []const u8) !Response {
    const head_end = std.mem.indexOf(u8, buf, "\r\n\r\n") orelse return Error.Incomplete;
    const body_at = head_end + 4;

    var lines = std.mem.splitSequence(u8, buf[0..head_end], "\r\n");
    const status_line = lines.next() orelse return Error.Malformed;

    // status-line = HTTP-version SP status-code SP [ reason-phrase ]
    if (status_line.len < 12) return Error.Malformed;
    if (!std.mem.startsWith(u8, status_line, "HTTP/1.")) return Error.Malformed;
    if (status_line[8] != ' ') return Error.Malformed;
    const status = std.fmt.parseInt(u16, status_line[9..12], 10) catch return Error.Malformed;

    var content_length: ?u64 = null;
    var chunked = false;
    var has_te = false;

    while (lines.next()) |line| {
        if (line.len == 0) continue;
        const colon = std.mem.indexOfScalar(u8, line, ':') orelse return Error.Malformed;
        const name = line[0..colon];
        const value = trim(line[colon + 1 ..]);

        if (eqlIgnoreCase(name, "content-length")) {
            const v = std.fmt.parseInt(u64, value, 10) catch return Error.Malformed;
            // §6.3 rule 5: repeated fields are valid only if they agree.
            if (content_length) |prev| {
                if (prev != v) return Error.ConflictingLength;
            }
            content_length = v;
        } else if (eqlIgnoreCase(name, "transfer-encoding")) {
            has_te = true;
            // "chunked" must be the *final* coding; anything else means rule 4's
            // second clause, which is read-until-close.
            var it = std.mem.splitScalar(u8, value, ',');
            var last: []const u8 = "";
            while (it.next()) |c| last = trim(c);
            if (eqlIgnoreCase(last, "chunked")) chunked = true;
        }
    }

    // **Rule 1 first, and it overrides the headers rather than agreeing with
    // them.** A 304 carrying `Content-Length: 1234` describes the entity it is
    // telling you not to fetch; reading 1234 bytes of body would consume the
    // next response on a reused connection, and on this one would simply hang.
    if (status == 204 or status == 304 or (status >= 100 and status < 200)) {
        return .{ .status = status, .body_at = body_at, .body = .none };
    }

    // Rule 3, before rule 6 — the precedence is the point.
    if (has_te and content_length != null) return Error.ConflictingLength;
    if (chunked) return Error.ChunkedUnsupported;
    // Rule 4, second clause: a transfer-coding that is not chunked.
    if (has_te) return .{ .status = status, .body_at = body_at, .body = .until_close };
    if (content_length) |n| return .{ .status = status, .body_at = body_at, .body = .{ .length = n } };
    // Rule 8.
    return .{ .status = status, .body_at = body_at, .body = .until_close };
}

pub const Request = struct {
    method: []const u8,
    /// The request-target, as sent. **Not decoded and not normalised** — see
    /// `parseRequest`.
    target: []const u8,
    /// Offset just past the blank line ending the head.
    body_at: usize,
    body: BodyLength,
};

/// Parse the head of a request.
///
/// **The target is returned exactly as it arrived.** No percent-decoding, no
/// `..` collapsing, no case folding. A server here compares it against a fixed
/// route and serves nothing from a filesystem, so decoding would add an attack
/// surface to buy nothing — and a half-done normalisation is how path traversal
/// gets through. A consumer that ever maps a target onto a file must do that
/// work itself, deliberately, and this comment is where it should start.
pub fn parseRequest(buf: []const u8) !Request {
    const head_end = std.mem.indexOf(u8, buf, "\r\n\r\n") orelse return Error.Incomplete;
    const body_at = head_end + 4;

    var lines = std.mem.splitSequence(u8, buf[0..head_end], "\r\n");
    const line = lines.next() orelse return Error.Malformed;

    // request-line = method SP request-target SP HTTP-version
    const sp1 = std.mem.indexOfScalar(u8, line, ' ') orelse return Error.Malformed;
    const rest = line[sp1 + 1 ..];
    const sp2 = std.mem.indexOfScalar(u8, rest, ' ') orelse return Error.Malformed;
    const method = line[0..sp1];
    const target = rest[0..sp2];
    const version = rest[sp2 + 1 ..];

    if (method.len == 0 or target.len == 0) return Error.Malformed;
    if (!std.mem.startsWith(u8, version, "HTTP/1.")) return Error.Malformed;

    var content_length: ?u64 = null;
    var has_te = false;

    while (lines.next()) |h| {
        if (h.len == 0) continue;
        const colon = std.mem.indexOfScalar(u8, h, ':') orelse return Error.Malformed;
        const name = h[0..colon];
        const value = trim(h[colon + 1 ..]);
        if (eqlIgnoreCase(name, "content-length")) {
            const v = std.fmt.parseInt(u64, value, 10) catch return Error.Malformed;
            if (content_length) |prev| {
                if (prev != v) return Error.ConflictingLength;
            }
            content_length = v;
        } else if (eqlIgnoreCase(name, "transfer-encoding")) {
            has_te = true;
        }
    }

    if (has_te and content_length != null) return Error.ConflictingLength;
    // **Any transfer-coding on a request is refused, chunked or not**, and
    // this deliberately does not look at which one — unlike `parseResponse`,
    // where a non-chunked coding means rule 4's second clause, read until
    // close. A request has no such fallback: §6.3 rule 7 says a request
    // without framing has no body, so a coded body this server cannot decode
    // is a body it cannot find the end of.
    //
    // An earlier version tested `chunked` here as well and the mutation
    // harness showed the branch was dead — both arms returned the same error.
    if (has_te) return Error.ChunkedUnsupported;
    // **§6.3 rule 7** — unlike a response, which under rule 8 runs until the
    // connection closes. Getting this backwards makes a server wait forever
    // for a body a client never said it would send, which presents as a hung
    // request rather than as a parsing bug.
    const body: BodyLength = if (content_length) |n| .{ .length = n } else .none;

    return .{ .method = method, .target = target, .body_at = body_at, .body = body };
}

/// Build a response with a body of known length.
///
/// **Always `Content-Length`, never close-delimited.** §6.3 rule 8 exists for
/// HTTP/1.0 compatibility and the RFC says plainly that a close-delimited
/// message cannot be told from a truncated one. A server that can count its
/// own body has no excuse for making the client guess.
pub fn buildResponse(status: u16, reason: []const u8, content_type: []const u8, body: []const u8, out: []u8) ![]u8 {
    var len_buf: [20]u8 = undefined;
    const len_str = std.fmt.bufPrint(&len_buf, "{d}", .{body.len}) catch unreachable;
    var status_buf: [4]u8 = undefined;
    const status_str = std.fmt.bufPrint(&status_buf, "{d}", .{status}) catch return Error.Malformed;

    var w: usize = 0;
    const parts = [_][]const u8{
        // zig fmt: off
        "HTTP/1.0 ", status_str, " ", reason,
        "\r\nContent-Type: ", content_type,
        "\r\nContent-Length: ", len_str,
        "\r\nConnection: close\r\n\r\n", body,
        // zig fmt: on
    };
    for (parts) |p| {
        if (w + p.len > out.len) return Error.BufferTooSmall;
        @memcpy(out[w..][0..p.len], p);
        w += p.len;
    }
    return out[0..w];
}

// ---------------------------------------------------------------------------
// Rung N10
// ---------------------------------------------------------------------------

const testing = std.testing;

test "the request asks for what this stack can actually handle" {
    var buf: [128]u8 = undefined;
    const req = try buildGet("example.com", "/index.html", &buf);

    // **`HTTP/1.0` is a request not to be sent chunked**, which is an HTTP/1.1
    // coding — the refusal below is a backstop, not the primary defence. And
    // `Connection: close` is how a stack with one connection and no
    // persistent-connection bookkeeping says so.
    try testing.expectEqualStrings(
        "GET /index.html HTTP/1.0\r\nHost: example.com\r\nConnection: close\r\n\r\n",
        req,
    );
}

test "a head that has not finished arriving asks for more, and is not an error" {
    // A caller feeds a growing buffer; **`Incomplete` is the signal to keep
    // reading**, and treating it as malformed would fail every response that
    // arrives in more than one segment — which, at 1460 bytes an MSS, is most
    // of them.
    try testing.expectError(Error.Incomplete, parseResponse("HTTP/1.0 200 OK\r\n"));
    try testing.expectError(Error.Incomplete, parseResponse("HTTP/1.0 200 OK\r\nContent-Length: 5\r\n"));

    const r = try parseResponse("HTTP/1.0 200 OK\r\nContent-Length: 5\r\n\r\nhello");
    try testing.expectEqual(@as(u16, 200), r.status);
    try testing.expectEqual(@as(u64, 5), r.body.length);
}

test "the body length rules, in the order §6.3 gives them" {
    // Rule 6: a Content-Length.
    const a = try parseResponse("HTTP/1.0 200 OK\r\nContent-Length: 1234\r\n\r\n");
    try testing.expectEqual(@as(u64, 1234), a.body.length);

    // Rule 8: no framing at all, so the body ends when the connection does.
    const b = try parseResponse("HTTP/1.0 200 OK\r\nServer: x\r\n\r\n");
    try testing.expectEqual(BodyLength.until_close, b.body);

    // **Rule 1 beats the headers.** A 304 carrying a Content-Length is
    // describing the entity it is telling you not to fetch. Reading 1234 bytes
    // would consume whatever came next — or, on this stack's single
    // connection, simply hang.
    const c = try parseResponse("HTTP/1.0 304 Not Modified\r\nContent-Length: 1234\r\n\r\n");
    try testing.expectEqual(BodyLength.none, c.body);

    const d = try parseResponse("HTTP/1.0 204 No Content\r\nContent-Length: 99\r\n\r\n");
    try testing.expectEqual(BodyLength.none, d.body);
}

test "chunked is refused by name rather than misread" {
    // **The failure mode this exists to prevent**: a chunked body looks like a
    // body, so a stack ignoring the framing delivers the chunk-size lines as
    // content and produces something almost right.
    try testing.expectError(Error.ChunkedUnsupported, parseResponse(
        "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n5\r\nhello\r\n0\r\n\r\n",
    ));

    // Case and whitespace are not a way around it — §5 makes field names
    // case-insensitive and the value may carry a list.
    try testing.expectError(Error.ChunkedUnsupported, parseResponse(
        "HTTP/1.1 200 OK\r\nTRANSFER-ENCODING:  gzip, chunked \r\n\r\n",
    ));

    // A transfer-coding that is *not* chunked is rule 4's second clause:
    // read until close.
    const r = try parseResponse("HTTP/1.1 200 OK\r\nTransfer-Encoding: gzip\r\n\r\n");
    try testing.expectEqual(BodyLength.until_close, r.body);
}

test "both framings at once is an error, not a preference" {
    // **§6.3 rule 3**: a message with both "ought to be handled as an error",
    // because it is the shape of a request-smuggling or response-splitting
    // attempt rather than a server being helpful. Silently preferring one is
    // exactly what makes the attack work.
    try testing.expectError(Error.ConflictingLength, parseResponse(
        "HTTP/1.1 200 OK\r\nContent-Length: 5\r\nTransfer-Encoding: chunked\r\n\r\n",
    ));

    // Two Content-Lengths that disagree, rule 5.
    try testing.expectError(Error.ConflictingLength, parseResponse(
        "HTTP/1.0 200 OK\r\nContent-Length: 5\r\nContent-Length: 9\r\n\r\n",
    ));

    // Two that agree are permitted by the same rule.
    const r = try parseResponse("HTTP/1.0 200 OK\r\nContent-Length: 5\r\nContent-Length: 5\r\n\r\n");
    try testing.expectEqual(@as(u64, 5), r.body.length);
}

test "status lines this stack must refuse" {
    try testing.expectError(Error.Malformed, parseResponse("HTTP/1.0 2OK OK\r\n\r\n"));
    try testing.expectError(Error.Malformed, parseResponse("NOTHTTP/1.0 200 OK\r\n\r\n"));
    try testing.expectError(Error.Malformed, parseResponse("HTTP/1.0\r\n\r\n"));
    // A header with no colon is not a header.
    try testing.expectError(Error.Malformed, parseResponse("HTTP/1.0 200 OK\r\nrubbish\r\n\r\n"));
    // A Content-Length that is not a number is rule 5's unrecoverable error.
    try testing.expectError(Error.Malformed, parseResponse("HTTP/1.0 200 OK\r\nContent-Length: soon\r\n\r\n"));
}

test "the body starts after the blank line, not after the last header" {
    const msg = "HTTP/1.0 200 OK\r\nContent-Length: 5\r\n\r\nhello";
    const r = try parseResponse(msg);
    // **Four bytes of CRLFCRLF, not two.** Off by two here and the body starts
    // with a blank line that the caller passes on as content.
    try testing.expectEqualStrings("hello", msg[r.body_at..]);
}

// ---------------------------------------------------------------------------
// Rung N11 — the server side
// ---------------------------------------------------------------------------

test "a GET this stack built parses as the request this stack reads" {
    var buf: [128]u8 = undefined;
    const raw = try buildGet("example.com", "/status", &buf);

    // **Both halves of the same wire format**, checked against each other —
    // which is worth something here in a way it was not at N5, because a
    // client and a server on the same board really will talk to each other at
    // N12, and a shared misunderstanding would pass every test either side.
    const r = try parseRequest(raw);
    try testing.expectEqualStrings("GET", r.method);
    try testing.expectEqualStrings("/status", r.target);
    try testing.expectEqual(BodyLength.none, r.body);
    try testing.expectEqual(raw.len, r.body_at);
}

test "a request with no framing has no body — rule 7, not rule 8" {
    const r = try parseRequest("GET / HTTP/1.1\r\nHost: x\r\n\r\n");
    // **A response with no framing runs until close; a request does not.**
    // Getting this backwards makes a server wait forever for a body the client
    // never said it would send, which presents as a hung request rather than
    // as a parsing bug.
    try testing.expectEqual(BodyLength.none, r.body);

    const p = try parseRequest("POST /x HTTP/1.1\r\nContent-Length: 4\r\n\r\nabcd");
    try testing.expectEqual(@as(u64, 4), p.body.length);
    try testing.expectEqualStrings("POST", p.method);
}

test "the target is returned exactly as it arrived" {
    const r = try parseRequest("GET /a/../../etc/passwd?q=%2e%2e HTTP/1.0\r\n\r\n");

    // **No decoding and no normalisation**, deliberately. A half-done
    // normalisation is how path traversal gets through, and this server
    // compares against fixed routes rather than reaching for a filesystem.
    // Anything that ever maps a target onto a file does that work itself.
    try testing.expectEqualStrings("/a/../../etc/passwd?q=%2e%2e", r.target);
}

test "requests this server must refuse" {
    // Not complete yet — keep reading.
    try testing.expectError(Error.Incomplete, parseRequest("GET / HTTP/1.0\r\n"));
    // No version.
    try testing.expectError(Error.Malformed, parseRequest("GET /\r\n\r\n"));
    // No target.
    try testing.expectError(Error.Malformed, parseRequest("GET  HTTP/1.0\r\n\r\n"));
    // Not HTTP.
    try testing.expectError(Error.Malformed, parseRequest("GET / SPDY/1\r\n\r\n"));
    // Chunked, refused by name on the way in as well as on the way out.
    try testing.expectError(Error.ChunkedUnsupported, parseRequest(
        "POST / HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n",
    ));
    // Both framings: smuggling-shaped, and an error either direction.
    try testing.expectError(Error.ConflictingLength, parseRequest(
        "POST / HTTP/1.1\r\nContent-Length: 4\r\nTransfer-Encoding: chunked\r\n\r\n",
    ));
}

test "a response this stack builds parses as the response this stack reads" {
    var buf: [256]u8 = undefined;
    const raw = try buildResponse(200, "OK", "text/plain", "hello", &buf);

    const r = try parseResponse(raw);
    try testing.expectEqual(@as(u16, 200), r.status);
    // **Always length-delimited.** §6.3 rule 8 says a close-delimited message
    // cannot be told from a truncated one, and a server that can count its own
    // body has no excuse for making the client guess.
    try testing.expectEqual(@as(u64, 5), r.body.length);
    try testing.expectEqualStrings("hello", raw[r.body_at..]);
}

test "an empty body still declares its length" {
    var buf: [256]u8 = undefined;
    const raw = try buildResponse(404, "Not Found", "text/plain", "", &buf);
    const r = try parseResponse(raw);
    try testing.expectEqual(@as(u16, 404), r.status);
    // Zero is a length. Omitting the header would make this close-delimited
    // and leave the client reading until the socket shut.
    try testing.expectEqual(@as(u64, 0), r.body.length);
}
