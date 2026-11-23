//! Freestanding NTP (Network Time Protocol) RFC 5905 Packet Formatting & Client Helper.
//!
//! Provides pure, host-testable byte arithmetic for formatting 48-byte NTP client requests,
//! parsing server responses, converting 1900 NTP timestamps to Unix epoch seconds,
//! and generic transport wrapper logic.

const std = @import("std");

/// Default NTP port per RFC 5905.
pub const ntp_port: u16 = 123;

/// Standard length of an NTP header / message in bytes.
pub const ntp_msg_len: usize = 48;

/// Difference in seconds between NTP epoch (1900-01-01 00:00:00 UTC) and Unix epoch (1970-01-01 00:00:00 UTC).
pub const ntp_delta: u64 = 2_208_988_800;

/// Default public IPv4 NTP server fallback (time.nist.gov).
pub const default_ntp_server_ip: [4]u8 = .{ 129, 6, 15, 28 };

pub const NtpError = error{
    PayloadTooShort,
    InvalidMode,
    InvalidStratum,
    ZeroTimestamp,
};

/// Create a 48-byte NTP client request payload.
pub fn createRequest() [ntp_msg_len]u8 {
    var req: [ntp_msg_len]u8 = @as([ntp_msg_len]u8, @splat(0));
    // LI = 0 (0b00), VN = 3 (0b011), Mode = 3 client (0b011) => 0x1B
    req[0] = 0x1B;
    return req;
}

/// Parse a 48-byte NTP server response payload and return Unix epoch seconds.
pub fn parseResponse(payload: []const u8) !u64 {
    if (payload.len < ntp_msg_len) return error.PayloadTooShort;

    const mode = payload[0] & 0x07;
    const stratum = payload[1];

    if (mode != 4) return error.InvalidMode; // Mode 4 is NTP server response
    if (stratum == 0) return error.InvalidStratum; // Stratum 0 indicates Kiss-o'-Death or unsynchronized

    // Transmit Timestamp seconds field starts at byte 40 (32-bit big-endian uint)
    const secs_1900 = std.mem.readInt(u32, payload[40..][0..4], .big);

    if (secs_1900 < ntp_delta) return error.ZeroTimestamp;

    const unix_sec: u64 = secs_1900 - ntp_delta;
    return unix_sec;
}

/// Generic NTP Client wrapper parameterised over network transport.
pub fn NtpClient(comptime Transport: type) type {
    return struct {
        const Self = @This();

        transport: Transport,

        pub fn init(transport: Transport) Self {
            return Self{ .transport = transport };
        }

        pub fn queryTime(self: *Self, server_ip: [4]u8, timeout_ms: u32) !u64 {
            const req = createRequest();
            var resp_buf: [ntp_msg_len]u8 = undefined;

            const bytes_read = try self.transport.sendAndReceiveUdp(server_ip, ntp_port, &req, &resp_buf, timeout_ms);
            return try parseResponse(resp_buf[0..bytes_read]);
        }
    };
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "NTP request payload formatting" {
    const req = createRequest();
    try testing.expectEqual(@as(usize, 48), req.len);
    try testing.expectEqual(@as(u8, 0x1B), req[0]);
    for (req[1..]) |b| {
        try testing.expectEqual(@as(u8, 0), b);
    }
}

test "NTP response parsing and Unix timestamp conversion" {
    var resp: [48]u8 = @as([48]u8, @splat(0));
    resp[0] = 0x24; // LI=0, VN=4, Mode=4 (server)
    resp[1] = 2; // Stratum 2

    // 2026-08-11 07:24:04 UTC = Unix epoch 1786433044
    // NTP timestamp = 1786433044 + 2208988800 = 3995421844 (0xEDDE6694)
    const ntp_ts: u32 = 3995421844;
    std.mem.writeInt(u32, resp[40..44], ntp_ts, .big);

    const unix_sec = try parseResponse(&resp);
    try testing.expectEqual(@as(u64, 1786433044), unix_sec);
}

test "NTP response error validation" {
    var resp: [48]u8 = @as([48]u8, @splat(0));
    resp[0] = 0x24; // Mode 4
    resp[1] = 0; // Stratum 0
    try testing.expectError(error.InvalidStratum, parseResponse(&resp));

    resp[1] = 1;
    resp[0] = 0x23; // Mode 3 (Client instead of Server)
    try testing.expectError(error.InvalidMode, parseResponse(&resp));

    const short_buf = @as([12]u8, @splat(0));
    try testing.expectError(error.PayloadTooShort, parseResponse(&short_buf));
}
