//! BR/EDR — rung 21, the feasibility rung.
//!
//! Everything above the transport in this package is LE. `hci.zig`'s opcode
//! table is `HCI_Reset`, `Read_BD_ADDR` and six `0x20xx` commands; `l2cap.zig`
//! says in its own header that it never opens a channel. That is the whole of
//! what a BLE peripheral needs and none of what Bluetooth Classic does.
//!
//! This file is the first step of the other direction, and it is deliberately
//! small: **it asks the controller whether Classic is there at all**, and it
//! makes the board discoverable so a phone can say the same thing
//! independently. Nothing here connects, pairs, or carries audio.
//!
//! ## Why this rung exists before any of the rest
//!
//! A2DP is a tower — SSP, L2CAP with real channel negotiation, an SDP server,
//! AVDTP, an SBC decoder — and every floor of it is wasted if the CYW43439's
//! patchram on this board turns out to have BR/EDR compiled out. The chip is
//! advertised as Bluetooth 5.2, which includes Classic, but this package's
//! standing rule is that a datasheet is not a measurement, and
//! the combined firmware image is Infineon's build, not the spec.
//!
//! So: three reads and three writes, one flash cycle, and an answer that can be
//! **no**. That is the point of it.
//!
//! ## Provenance of the feature bits
//!
//! The bit positions below are not read off a spec from memory. They are the
//! ones btstack uses, in `src/hci.c` (as vendored in pico-sdk), and the comments there give the
//! feature number as well as the byte and bit:
//!
//! ```c
//! bool hci_classic_supported(void){
//!     // No. 37, byte 4, bit 5, = No BR/EDR Support
//!     return (hci_stack->local_supported_features[4] & (1 << 5)) == 0;
//! }
//! ```
//!
//! **Note the sense of that one**: the bit is set when Classic is *absent*, so
//! the check reads inverted and a naive `!= 0` gets the answer exactly
//! backwards. That is why it is a named function here rather than an inline
//! mask at the call site.

const std = @import("std");

pub const Error = error{
    /// A response too short to be the answer it claims to be.
    Malformed,
};

/// Opcodes, OGF and OCF already combined, matching `hci.op`'s convention.
///
/// **`read_buffer_size` is 0x1005, not 0x2002.** The LE command `hci.start`
/// already issues reports the LE controller's buffers, which on this chip are
/// 27-byte packets — see `hci.Host.acl_payload_max`. BR/EDR has its own, much
/// larger, and a media stream needs them.
pub const op = struct {
    pub const read_local_supported_commands: u16 = 0x1002;
    // -- rung 22: accepting, and pairing ------------------------------------
    //
    // **The first five answer with Command Status, not Command Complete.**
    // See `hci.parseCommandStatus`: reaching for `commandWithResult` here waits
    // out its attempts and reports `NoAnswer` for a command the controller took.
    /// **`Create_Connection` — the one command that makes this a device rather
    /// than a doorbell.** Everything else here answers a phone; this one pages
    /// it. A speaker that has been paired knows the phone's address and can
    /// reach it whenever Bluetooth is on, because **a phone stops being
    /// discoverable but never stops being connectable**: inquiry scan is on for
    /// a couple of minutes at the pairing screen, page scan is on always. That
    /// asymmetry is why commercial speakers appear to connect by themselves.
    /// `Switch_Role`. **Which end is the central is not a formality for an
    /// audio sink.** The central schedules the piconet and the peripheral
    /// transmits when it is polled, so the role decides whose timing governs
    /// delivery. A phone streaming A2DP expects to be the central — that is
    /// the topology its scheduler is written for, and the one every rung here
    /// before paging ran in, because the phone always initiated.
    ///
    /// Paging reverses it: the pager is the central. `create_connection`
    /// *permits* a switch back but does not ask for one, and a phone that does
    /// not ask is left transmitting on our controller's poll interval instead
    /// of its own.
    pub const switch_role: u16 = 0x080B;
    pub const create_connection: u16 = 0x0405;
    pub const create_connection_cancel: u16 = 0x0408;
    pub const accept_connection_request: u16 = 0x0409;
    pub const reject_connection_request: u16 = 0x040A;
    pub const authentication_requested: u16 = 0x0411;
    pub const set_connection_encryption: u16 = 0x0413;
    // -- and these answer with Command Complete -----------------------------
    pub const link_key_request_reply: u16 = 0x040B;
    pub const link_key_request_negative_reply: u16 = 0x040C;
    pub const pin_code_request_negative_reply: u16 = 0x040E;
    pub const io_capability_request_reply: u16 = 0x042B;
    pub const user_confirmation_request_reply: u16 = 0x042C;
    pub const user_confirmation_request_negative_reply: u16 = 0x042D;
    pub const read_local_supported_features: u16 = 0x1003;
    pub const read_buffer_size: u16 = 0x1005;

    pub const write_page_timeout: u16 = 0x0C18;
    pub const write_local_name: u16 = 0x0C13;
    pub const write_scan_enable: u16 = 0x0C1A;
    pub const write_class_of_device: u16 = 0x0C24;
    pub const write_simple_pairing_mode: u16 = 0x0C56;
};

// ---------------------------------------------------------------------------
// Local supported features
// ---------------------------------------------------------------------------

/// The eight bytes of the LMP feature mask, page 0.
pub const Features = struct {
    bits: [8]u8,

    /// **Inverted, and that is the whole trap**: feature No. 37 is *BR/EDR Not
    /// Supported*, so Classic is present when the bit is clear.
    pub fn classic(self: Features) bool {
        return (self.bits[4] & (1 << 5)) == 0;
    }

    /// No. 38, byte 4, bit 6 — LE Supported (Controller). Expected true here:
    /// rungs 17 through 20 have been running on it.
    pub fn le(self: Features) bool {
        return (self.bits[4] & (1 << 6)) != 0;
    }

    /// No. 51, byte 6, bit 3 — Secure Simple Pairing. This is what a phone
    /// bonds with; without it the only way in is legacy PIN pairing, which
    /// Android will not offer for an audio device.
    pub fn secureSimplePairing(self: Features) bool {
        return (self.bits[6] & (1 << 3)) != 0;
    }

    /// No. 54, byte 6, bit 6 — non-flushable packet boundary flag. Relevant
    /// later: an A2DP media channel wants its packets flushable and its
    /// signalling not.
    pub fn nonFlushablePacketBoundary(self: Features) bool {
        return (self.bits[6] & (1 << 6)) != 0;
    }
};

/// Parse the parameters of `Read_Local_Supported_Features`, which
/// `hci.Host.commandWithResult` hands back with the status byte already
/// removed.
pub fn parseFeatures(params: []const u8) !Features {
    if (params.len < 8) return Error.Malformed;
    return .{ .bits = params[0..8].* };
}

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

/// What `Read_Buffer_Size` reports. **Four fields, where the LE version has
/// two**, and in a different order — `hci.parseBufferSize` cannot be reused and
/// silently returns nonsense if it is.
pub const BufferSize = struct {
    acl_len: u16,
    sco_len: u8,
    acl_count: u16,
    sco_count: u16,
};

pub fn parseBufferSize(params: []const u8) !BufferSize {
    if (params.len < 7) return Error.Malformed;
    return .{
        .acl_len = std.mem.readInt(u16, params[0..2], .little),
        .sco_len = params[2],
        .acl_count = std.mem.readInt(u16, params[3..5], .little),
        .sco_count = std.mem.readInt(u16, params[5..7], .little),
    };
}

/// SBC joint stereo at 44.1 kHz is about 328 kbit/s, so roughly 41 KB/s of
/// payload before ACL and L2CAP overhead. This is the figure rung 21's report
/// is measured against — not a pass condition here, because a buffer count is
/// not a throughput measurement, but the number that makes the report mean
/// something.
pub const a2dp_payload_bytes_per_sec: u32 = 41_000;

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

/// The Classic events, none of which `hci.event` lists because none of them can
/// reach an LE-only host.
///
/// Rung 21 accepts none of these — it cannot, there is nothing behind them yet.
/// They are named here because **seeing one is the strongest evidence this rung
/// can produce**: a `connection_request` carrying a phone's real BD_ADDR is a
/// value that exists only because a Classic radio is scanning and a phone
/// paged it.
pub const event = struct {
    pub const inquiry_complete: u8 = 0x01;
    pub const connection_complete: u8 = 0x03;
    pub const connection_request: u8 = 0x04;
    pub const authentication_complete: u8 = 0x06;
    pub const remote_name_request_complete: u8 = 0x07;
    pub const encryption_change: u8 = 0x08;
    pub const link_key_notification: u8 = 0x18;
    pub const io_capability_request: u8 = 0x31;
    pub const io_capability_response: u8 = 0x32;
    pub const user_confirmation_request: u8 = 0x33;
    pub const simple_pairing_complete: u8 = 0x36;
    pub const pin_code_request: u8 = 0x16;
    pub const link_key_request: u8 = 0x17;
    pub const read_remote_supported_features: u8 = 0x0B;
    pub const role_change: u8 = 0x12;
    pub const max_slots_change: u8 = 0x1B;
    pub const page_scan_repetition_mode_change: u8 = 0x20;
};

/// What this device can show a user, and therefore which pairing model the two
/// ends negotiate.
///
/// **A speaker is `no_input_no_output`**, which selects "just works": no
/// six-digit number to compare, because there is nothing to compare it on. The
/// consequence is that the pairing has no MITM protection, and that is the
/// honest capability of a box with no screen and no keypad — claiming
/// `display_yes_no` to get a stronger-looking model would be a lie the security
/// model is built on.
pub const IoCapability = enum(u8) {
    display_only = 0x00,
    display_yes_no = 0x01,
    keyboard_only = 0x02,
    no_input_no_output = 0x03,
};

/// Authentication requirements, as the low three bits of one byte: MITM in bit
/// 0, and the bonding mode above it.
pub const auth = struct {
    pub const no_bonding: u8 = 0x00;
    pub const no_bonding_mitm: u8 = 0x01;
    pub const dedicated_bonding: u8 = 0x02;
    pub const dedicated_bonding_mitm: u8 = 0x03;
    /// **What a speaker asks for**: a link key kept for next time, and no MITM
    /// because `no_input_no_output` cannot provide it anyway.
    pub const general_bonding: u8 = 0x04;
    pub const general_bonding_mitm: u8 = 0x05;
};

/// `Connection_Complete` (0x03) — eleven parameters, which is what
/// `bredr-probe` was printing undecoded as `[ev 0x03, 11 params]`.
///
/// **Status 0 is the only good one**, and the interesting failure is `0x10`,
/// Connection Accept Timeout Exceeded: the controller gave up because the host
/// never answered the `Connection_Request`. That is precisely what rung 21b
/// produced, by design.
pub const ConnectionComplete = struct {
    status: u8,
    handle: u16,
    peer: [6]u8,
    link_type: LinkType,
    encryption_enabled: bool,
};

pub fn parseConnectionComplete(params: []const u8) !ConnectionComplete {
    if (params.len < 11) return Error.Malformed;
    return .{
        .status = params[0],
        .handle = std.mem.readInt(u16, params[1..3], .little) & 0x0FFF,
        .peer = params[3..9].*,
        .link_type = @fromBackingInt(@intCast(params[9])),
        .encryption_enabled = params[10] != 0,
    };
}

/// `Link_Key_Notification` (0x18): the bonding material.
///
/// **The key is a secret and must never reach a log.** `app/bt-speaker` records its
/// type and a checksum, not its bytes — a link key printed over UART is a link
/// key in a terminal scrollback, and the whole point of the pairing is that
/// only the two devices hold it.
pub const LinkKeyNotification = struct {
    peer: [6]u8,
    key: [16]u8,
    key_type: u8,
};

pub fn parseLinkKeyNotification(params: []const u8) !LinkKeyNotification {
    if (params.len < 23) return Error.Malformed;
    return .{
        .peer = params[0..6].*,
        .key = params[6..22].*,
        .key_type = params[22],
    };
}

/// `Encryption_Change` (0x08). **This is rung 22's finish line**: a status of 0
/// with `enabled` set is a link that is authenticated and encrypted.
pub const EncryptionChange = struct {
    status: u8,
    handle: u16,
    enabled: bool,
};

pub fn parseEncryptionChange(params: []const u8) !EncryptionChange {
    if (params.len < 4) return Error.Malformed;
    return .{
        .status = params[0],
        .handle = std.mem.readInt(u16, params[1..3], .little) & 0x0FFF,
        .enabled = params[3] != 0,
    };
}

/// `User_Confirmation_Request` (0x33). The numeric value is the six digits the
/// two ends would compare; with `no_input_no_output` nobody sees it and the
/// host replies positively on the user's behalf.
pub const UserConfirmationRequest = struct {
    peer: [6]u8,
    numeric_value: u32,
};

pub fn parseUserConfirmationRequest(params: []const u8) !UserConfirmationRequest {
    if (params.len < 10) return Error.Malformed;
    return .{
        .peer = params[0..6].*,
        .numeric_value = std.mem.readInt(u32, params[6..10], .little),
    };
}

/// Six bytes of peer address — the whole parameter block for
/// `User_Confirmation_Request_Reply`, `Link_Key_Request_Negative_Reply` and
/// their neighbours.
pub fn addrParams(peer: [6]u8) [6]u8 {
    return peer;
}

/// `Accept_Connection_Request`: the peer, then the role we intend to hold.
///
/// **Role 0x01 — remain the peripheral.** A sink has no reason to become
/// central, and asking for a role switch during connection setup is one more
/// thing that can fail before there is any evidence the rest works.
pub fn acceptConnectionParams(peer: [6]u8, remain_peripheral: bool) [7]u8 {
    var p: [7]u8 = @splat(0);
    @memcpy(p[0..6], &peer);
    p[6] = if (remain_peripheral) 0x01 else 0x00;
    return p;
}

/// `Create_Connection`: peer, packet types, how to page, and whether the other
/// end may take the central role.
///
/// **`allow_role_switch` is 0x01 here, the opposite of `acceptConnectionParams`,
/// and deliberately so.** Paging makes this device the central, which is the
/// reverse of every link this stack has been tested on — the phone has always
/// been the central and the sink the peripheral. Permitting the switch lets the
/// phone take the role back and puts the connection into the topology the rest
/// of the code has actually run in.
///
/// `page_scan_repetition_mode` is the target's scan behaviour, which decides how
/// long a pager must transmit each train: R0 scans continuously, R2 as little as
/// once every 2.56 s. **Guessing low is the failure that looks like the phone is
/// out of range** — the pager gives up before a slow scanner has looked. R2
/// costs time against a fast scanner and works against every scanner, which is
/// the right way round when the alternative is not connecting at all.
///
/// The clock offset is zero: bit 15 is the "valid" flag, so a plain zero means
/// *unknown*, not *zero*.
pub fn createConnectionParams(peer: [6]u8, allow_role_switch: bool) [13]u8 {
    var p: [13]u8 = @splat(0);
    @memcpy(p[0..6], &peer);
    // DM1/DH1/DM3/DH3/DM5/DH5 — every basic-rate packet type. The controller
    // picks among them; naming fewer only removes choices.
    std.mem.writeInt(u16, p[6..8], 0xCC18, .little);
    p[8] = 0x02; // R2
    p[9] = 0x00; // reserved
    std.mem.writeInt(u16, p[10..12], 0x0000, .little); // clock offset unknown
    p[12] = if (allow_role_switch) 0x01 else 0x00;
    return p;
}

/// `Switch_Role`: the peer, and the role **we** want to end up holding.
///
/// Answers with Command Status; the outcome arrives as `role_change`, which is
/// the only thing that says whether it happened. It may legitimately fail — the
/// peer can refuse, and a link with an active stream may not be switchable at
/// all — so ask early and treat a refusal as information rather than an error.
pub fn switchRoleParams(peer: [6]u8, want_peripheral: bool) [7]u8 {
    var p: [7]u8 = @splat(0);
    @memcpy(p[0..6], &peer);
    p[6] = if (want_peripheral) 0x01 else 0x00;
    return p;
}

/// `Write_Page_Timeout`, in 625 us slots.
///
/// The default is 0x2000 — 5.12 s — which is tuned for a device you have just
/// told to connect. **A speaker paging a phone that might be in a pocket in the
/// next room wants longer**, and the cost of a longer timeout is only that a
/// failed attempt takes longer to report.
pub fn pageTimeoutParams(slots: u16) [2]u8 {
    var p: [2]u8 = @splat(0);
    std.mem.writeInt(u16, &p, slots, .little);
    return p;
}

/// 10.24 s. Twice the default, and long enough for two full R2 page trains.
pub const page_timeout_generous: u16 = 0x4000;

/// `IO_Capability_Request_Reply`: peer, capability, OOB present, auth
/// requirements.
/// `Link_Key_Request_Reply`: the address being asked about, then the 16-byte
/// key.
///
/// **The address belongs to the request and the key belongs to the bond, and
/// they are the same device only if the caller checked.** This builder cannot
/// check — it is handed one key and told to format it — so the caller must, and
/// `app/bt-speaker` did not: it answered every request with the single key it
/// held, so a Mac that had never paired was sent a *phone's* key. A device
/// given a key it does not have fails authentication and gives up, which
/// presents as "it won't connect" with nothing in the log to say why.
///
/// The negative reply is what a request from an unknown device should get: it
/// is what starts pairing.
pub fn linkKeyReplyParams(peer: [6]u8, key: [16]u8) [22]u8 {
    var p: [22]u8 = @splat(0);
    @memcpy(p[0..6], &peer);
    @memcpy(p[6..22], &key);
    return p;
}

pub fn ioCapabilityReplyParams(peer: [6]u8, cap: IoCapability, auth_req: u8) [9]u8 {
    var p: [9]u8 = @splat(0);
    @memcpy(p[0..6], &peer);
    p[6] = @backingInt(cap);
    p[7] = 0x00; // no out-of-band data — there is no NFC or QR code here
    p[8] = auth_req;
    return p;
}

pub const LinkType = enum(u8) {
    sco = 0x00,
    acl = 0x01,
    esco = 0x02,
    _,
};

/// A `Connection_Request` event: who is paging us, what they claim to be, and
/// which kind of link they want. A2DP wants `.acl`.
pub const ConnectionRequest = struct {
    peer: [6]u8,
    class_of_device: u24,
    link_type: LinkType,
};

pub fn parseConnectionRequest(params: []const u8) !ConnectionRequest {
    if (params.len < 10) return Error.Malformed;
    return .{
        .peer = params[0..6].*,
        .class_of_device = @as(u24, params[6]) |
            @as(u24, params[7]) << 8 |
            @as(u24, params[8]) << 16,
        .link_type = @fromBackingInt(@intCast(params[9])),
    };
}

// ---------------------------------------------------------------------------
// L2CAP, as Classic does it
// ---------------------------------------------------------------------------

/// Fixed channel identifiers **for BR/EDR**, which are not LE's.
///
/// **Signalling is CID 1 here and CID 5 on LE.** `l2cap.zig` names `0x0005`
/// "signalling" and that is correct for the only transport it has ever carried;
/// a Classic connection request arrives on `0x0001` and a host looking at 5
/// sees silence. This is the third LE-shaped assumption to surface in this
/// package — after a 260-byte receive buffer and a 27-byte ACL payload — and
/// the pattern is worth naming: **anything `l2cap.zig` or `hci.zig` calls "the"
/// value is an LE value** until checked.
pub const l2cap_cid = struct {
    pub const signalling: u16 = 0x0001;
    pub const connectionless: u16 = 0x0002;
};

/// The protocol/service multiplexers an A2DP sink cares about.
pub const psm = struct {
    /// Service discovery. **A phone opens this first, immediately after
    /// pairing**, and a device that never answers it has no services as far as
    /// the phone is concerned.
    pub const sdp: u16 = 0x0001;
    pub const rfcomm: u16 = 0x0003;
    /// AV control — AVRCP rides here.
    pub const avctp: u16 = 0x0017;
    /// AV distribution — A2DP's signalling *and* media both use this PSM, on
    /// two separate channels.
    pub const avdtp: u16 = 0x0019;
};

/// L2CAP signalling command codes.
pub const sig = struct {
    pub const command_reject: u8 = 0x01;
    pub const connection_request: u8 = 0x02;
    pub const connection_response: u8 = 0x03;
    pub const configure_request: u8 = 0x04;
    pub const configure_response: u8 = 0x05;
    pub const disconnection_request: u8 = 0x06;
    pub const disconnection_response: u8 = 0x07;
    pub const echo_request: u8 = 0x08;
    pub const information_request: u8 = 0x0A;
    pub const information_response: u8 = 0x0B;
};

pub const Signalling = struct {
    code: u8,
    /// Echoed in the response. **A response carrying the wrong id is ignored by
    /// the peer**, which presents as a request that was never answered.
    id: u8,
    payload: []const u8,
};

pub fn parseSignalling(frame: []const u8) !Signalling {
    if (frame.len < 4) return Error.Malformed;
    const len: usize = std.mem.readInt(u16, frame[2..4], .little);
    if (4 + len > frame.len) return Error.Malformed;
    return .{ .code = frame[0], .id = frame[1], .payload = frame[4..][0..len] };
}

/// An L2CAP `Connection Request`: which service, and the channel the peer will
/// receive on.
pub const L2capConnectionRequest = struct {
    psm: u16,
    source_cid: u16,
};

pub fn parseL2capConnectionRequest(payload: []const u8) !L2capConnectionRequest {
    if (payload.len < 4) return Error.Malformed;
    return .{
        .psm = std.mem.readInt(u16, payload[0..2], .little),
        .source_cid = std.mem.readInt(u16, payload[2..4], .little),
    };
}

/// Dynamic channel identifiers start here. Below `0x0040` the numbers are
/// reserved for fixed channels.
pub const first_dynamic_cid: u16 = 0x0040;

/// The MTU this device offers. **Bounded by `hci.rx_buf_len`**, which holds one
/// full BR/EDR ACL packet: offering more than can be received is a promise the
/// receive path cannot keep, and the failure would arrive as a truncated SDP
/// response rather than as an error.
pub const local_mtu: u16 = 672;

pub const ConnectionResult = enum(u16) {
    success = 0x0000,
    pending = 0x0001,
    refused_psm_unsupported = 0x0002,
    refused_security = 0x0003,
    refused_no_resources = 0x0004,
    _,
};

pub const config_option = struct {
    pub const mtu: u8 = 0x01;
    pub const flush_timeout: u8 = 0x02;
    pub const qos: u8 = 0x03;
    pub const retransmission_and_flow_control: u8 = 0x04;
};

pub const info_type = struct {
    pub const connectionless_mtu: u16 = 0x0001;
    pub const extended_features: u16 = 0x0002;
    pub const fixed_channels: u16 = 0x0003;
};

/// Build an L2CAP signalling frame: code, identifier, length, payload. The
/// result goes inside `l2cap.build` and then `hci.buildAcl`.
pub fn buildSignalling(code: u8, id: u8, payload: []const u8, out: []u8) ![]u8 {
    if (out.len < 4 + payload.len) return error.BufferTooSmall;
    out[0] = code;
    out[1] = id;
    std.mem.writeInt(u16, out[2..4], @intCast(payload.len), .little);
    @memcpy(out[4..][0..payload.len], payload);
    return out[0 .. 4 + payload.len];
}

/// `Connection Response`. **`dcid` is ours and `scid` is theirs**, and the two
/// are easy to swap: the peer then addresses a channel that does not exist and
/// nothing further arrives, with no error from either end.
pub fn connectionResponseParams(dcid: u16, scid: u16, result: ConnectionResult) [8]u8 {
    var p: [8]u8 = @splat(0);
    std.mem.writeInt(u16, p[0..2], dcid, .little);
    std.mem.writeInt(u16, p[2..4], scid, .little);
    std.mem.writeInt(u16, p[4..6], @backingInt(result), .little);
    // Status, meaningful only when the result is `pending`.
    std.mem.writeInt(u16, p[6..8], 0, .little);
    return p;
}

/// `Connection Request` — **the outbound direction**, which this stack did not
/// have until it started paging.
///
/// Every channel up to rung 27 was opened by the phone; a sink only ever
/// answered. Once the sink is the one that brings the link up, the phone has no
/// reason to open anything, so the sink has to ask for the AVDTP channel
/// itself.
///
/// `scid` is **our** channel identifier, allocated before sending, because the
/// response names it and there is nothing else to match the response against.
pub fn connectionRequestParams(protocol: u16, scid: u16) [4]u8 {
    var p: [4]u8 = @splat(0);
    std.mem.writeInt(u16, p[0..2], protocol, .little);
    std.mem.writeInt(u16, p[2..4], scid, .little);
    return p;
}

/// The answer to `connectionRequestParams`.
///
/// **`result == 1` is not a failure**: it is *pending*, and it means the peer is
/// still deciding — typically waiting on an authorisation it will grant. A
/// second response follows. Treating pending as a refusal gives up on a channel
/// that was about to open, which is a bug that only appears against peers slow
/// enough to use it.
pub const L2capConnectionResponse = struct {
    /// Theirs — the channel we must address from now on.
    dcid: u16,
    /// Ours, echoed back.
    scid: u16,
    result: u16,
    status: u16,

    pub fn pending(self: L2capConnectionResponse) bool {
        return self.result == 1;
    }
    pub fn ok(self: L2capConnectionResponse) bool {
        return self.result == 0;
    }
};

pub fn parseL2capConnectionResponse(payload: []const u8) !L2capConnectionResponse {
    if (payload.len < 8) return Error.Malformed;
    return .{
        .dcid = std.mem.readInt(u16, payload[0..2], .little),
        .scid = std.mem.readInt(u16, payload[2..4], .little),
        .result = std.mem.readInt(u16, payload[4..6], .little),
        .status = std.mem.readInt(u16, payload[6..8], .little),
    };
}

pub const ConfigureRequest = struct {
    /// The channel being configured — **ours**, from the peer's point of view.
    dcid: u16,
    flags: u16,
    options: []const u8,
};

pub fn parseConfigureRequest(payload: []const u8) !ConfigureRequest {
    if (payload.len < 4) return Error.Malformed;
    return .{
        .dcid = std.mem.readInt(u16, payload[0..2], .little),
        .flags = std.mem.readInt(u16, payload[2..4], .little),
        .options = payload[4..],
    };
}

pub const ConfigureResponse = struct {
    scid: u16,
    flags: u16,
    result: u16,
};

pub fn parseConfigureResponse(payload: []const u8) !ConfigureResponse {
    if (payload.len < 6) return Error.Malformed;
    return .{
        .scid = std.mem.readInt(u16, payload[0..2], .little),
        .flags = std.mem.readInt(u16, payload[2..4], .little),
        .result = std.mem.readInt(u16, payload[4..6], .little),
    };
}

pub fn mtuOption(mtu: u16) [4]u8 {
    return .{ config_option.mtu, 2, @truncate(mtu), @truncate(mtu >> 8) };
}

/// Find the MTU in a configuration options block, if it names one.
///
/// **Bit 7 of the type byte is the "hint" flag**, not part of the type. An
/// option carrying it may be ignored by the receiver — but the type underneath
/// still has to be read correctly, and a host that compares the whole byte
/// misses every hinted option and walks the block at the wrong offset from
/// there on.
pub fn findMtu(options: []const u8) ?u16 {
    var i: usize = 0;
    while (i + 2 <= options.len) {
        const kind = options[i] & 0x7F;
        const len: usize = options[i + 1];
        if (i + 2 + len > options.len) return null;
        if (kind == config_option.mtu and len == 2) {
            return std.mem.readInt(u16, options[i + 2 ..][0..2], .little);
        }
        i += 2 + len;
    }
    return null;
}

// ---------------------------------------------------------------------------
// Being discoverable
// ---------------------------------------------------------------------------

/// Class of Device for a loudspeaker: service classes Audio (bit 21) and
/// Rendering (bit 18), major class Audio/Video (4), minor class Loudspeaker
/// (5, shifted into bits 7..2).
///
/// **btstack's `a2dp_sink_demo` uses `0x240400`** — the same service and major
/// classes with an uncategorised minor. If a phone shows this board with the
/// wrong icon, that value is the known-good fallback to diff against, and the
/// difference is in the minor class only.
pub const cod_loudspeaker: u24 = 0x240414;

pub const ScanEnable = enum(u8) {
    none = 0x00,
    /// Discoverable but not connectable.
    inquiry = 0x01,
    /// Connectable but not discoverable — how a bonded device is rejoined.
    page = 0x02,
    /// Both, which is what a device waiting to be paired for the first time
    /// needs.
    both = 0x03,
};

/// The three bytes of `Write_Class_of_Device`, little-endian.
pub fn classOfDeviceParams(cod: u24) [3]u8 {
    var p: [3]u8 = undefined;
    p[0] = @truncate(cod);
    p[1] = @truncate(cod >> 8);
    p[2] = @truncate(cod >> 16);
    return p;
}

/// The 248 bytes of `Write_Local_Name`.
///
/// **The field is fixed-width and null-padded, not length-prefixed**, which is
/// the same shape as `hci.advData`'s 31-byte block and the same reason it is a
/// function: a short name written as a short parameter is a malformed command,
/// and the controller rejects it with a status rather than a hint.
pub fn localNameParams(name: []const u8) ![248]u8 {
    if (name.len > 247) return error.BufferTooSmall;
    var p: [248]u8 = @splat(0);
    @memcpy(p[0..name.len], name);
    return p;
}

// ---------------------------------------------------------------------------
// Host unit tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "BR/EDR is reported by a clear bit, not a set one" {
    // All zeroes: nothing claimed, so No. 37 is clear and Classic is present.
    // This is the case that a naive `bits[4] & 0x20 != 0` gets backwards, so it
    // is asserted first.
    const present = Features{ .bits = @splat(0) };
    try testing.expect(present.classic());

    var absent = Features{ .bits = @splat(0) };
    absent.bits[4] = 1 << 5;
    try testing.expect(!absent.classic());
}

test "the LE and Classic bits are adjacent and independent" {
    // Byte 4 bit 5 is BR/EDR Not Supported, bit 6 is LE Supported. A
    // single-mode LE controller sets both; this chip should set only bit 6.
    var le_only = Features{ .bits = @splat(0) };
    le_only.bits[4] = (1 << 5) | (1 << 6);
    try testing.expect(!le_only.classic());
    try testing.expect(le_only.le());

    var dual = Features{ .bits = @splat(0) };
    dual.bits[4] = 1 << 6;
    try testing.expect(dual.classic());
    try testing.expect(dual.le());
}

test "secure simple pairing and the boundary flag live in byte 6" {
    var f = Features{ .bits = @splat(0) };
    f.bits[6] = (1 << 3) | (1 << 6);
    try testing.expect(f.secureSimplePairing());
    try testing.expect(f.nonFlushablePacketBoundary());

    f.bits[6] = 1 << 3;
    try testing.expect(f.secureSimplePairing());
    try testing.expect(!f.nonFlushablePacketBoundary());
}

test "a short feature response is refused rather than padded" {
    try testing.expectError(Error.Malformed, parseFeatures(&.{ 0, 0, 0, 0 }));
}

test "Read_Buffer_Size is four fields, not the LE command's two" {
    // ACL 1021 bytes, SCO 255, 8 ACL buffers, 1 SCO buffer — the shape a
    // Broadcom/Infineon Classic controller reports.
    const params = [_]u8{ 0xFD, 0x03, 0xFF, 0x08, 0x00, 0x01, 0x00 };
    const bs = try parseBufferSize(&params);
    try testing.expectEqual(@as(u16, 1021), bs.acl_len);
    try testing.expectEqual(@as(u8, 255), bs.sco_len);
    try testing.expectEqual(@as(u16, 8), bs.acl_count);
    try testing.expectEqual(@as(u16, 1), bs.sco_count);
}

test "the LE parser would misread a BR/EDR buffer response" {
    // Not a test of this file so much as a record of why it exists: `hci`'s
    // parser takes byte 2 as a packet *count*, where here it is the SCO packet
    // length. It succeeds, and it is wrong.
    const params = [_]u8{ 0xFD, 0x03, 0xFF, 0x08, 0x00, 0x01, 0x00 };
    const hci = @import("hci.zig");
    const wrong = try hci.parseBufferSize(&params);
    try testing.expectEqual(@as(u8, 255), wrong.packet_count);

    const right = try parseBufferSize(&params);
    try testing.expectEqual(@as(u16, 8), right.acl_count);
    try testing.expect(wrong.packet_count != right.acl_count);
}

test "a short buffer-size response is refused" {
    try testing.expectError(Error.Malformed, parseBufferSize(&.{ 0xFD, 0x03, 0xFF }));
}

test "class of device goes out little-endian" {
    const p = classOfDeviceParams(cod_loudspeaker);
    try testing.expectEqualSlices(u8, &.{ 0x14, 0x04, 0x24 }, &p);
}

test "the loudspeaker class decomposes the way the spec assigns it" {
    // Service classes in bits 23..13, major in 12..8, minor in 7..2.
    const cod: u24 = cod_loudspeaker;
    try testing.expect((cod & (1 << 21)) != 0); // Audio
    try testing.expect((cod & (1 << 18)) != 0); // Rendering
    try testing.expectEqual(@as(u24, 4), (cod >> 8) & 0x1F); // Audio/Video
    try testing.expectEqual(@as(u24, 5), (cod >> 2) & 0x3F); // Loudspeaker
    try testing.expectEqual(@as(u24, 0), cod & 0x03); // format type
}

test "a local name is padded to the full field, not truncated to its length" {
    const p = try localNameParams("zig-speaker");
    try testing.expectEqual(@as(usize, 248), p.len);
    try testing.expectEqualSlices(u8, "zig-speaker", p[0..11]);
    // Everything after the name is zero, including the byte right after it.
    for (p[11..]) |b| try testing.expectEqual(@as(u8, 0), b);
}

test "a name too long for the field is refused rather than clipped" {
    const long: [248]u8 = @splat('x');
    try testing.expectError(error.BufferTooSmall, localNameParams(&long));
}

test "a connection request carries the pager's address, class and link type" {
    // A phone paging us for an ACL link, claiming to be a smartphone
    // (0x5A020C). The address is little-endian on the wire, so it reads
    // backwards from how it is printed.
    const params = [_]u8{
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, // BD_ADDR 66:55:44:33:22:11
        0x0C, 0x02, 0x5A, // class of device
        0x01, // ACL
    };
    const req = try parseConnectionRequest(&params);
    try testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 }, &req.peer);
    try testing.expectEqual(@as(u24, 0x5A020C), req.class_of_device);
    try testing.expectEqual(LinkType.acl, req.link_type);
}

test "a truncated connection request is refused" {
    try testing.expectError(
        Error.Malformed,
        parseConnectionRequest(&.{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x0C }),
    );
}

test "an unknown link type survives as its raw value" {
    // `_` on the enum, so a controller reporting something the spec added
    // later does not turn into an illegal-value panic in a probe.
    const params = [_]u8{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x7F };
    const req = try parseConnectionRequest(&params);
    try testing.expectEqual(@as(u8, 0x7F), @backingInt(req.link_type));
}

test "scan enable 0x03 is both halves" {
    try testing.expectEqual(@as(u8, 0x03), @backingInt(ScanEnable.both));
    try testing.expectEqual(
        @as(u8, 0x03),
        @backingInt(ScanEnable.inquiry) | @backingInt(ScanEnable.page),
    );
}

test "a connection complete decodes the eleven parameters rung 21b printed raw" {
    // Status 0, handle 0x0029, the phone from rung 21b, ACL, not yet encrypted.
    const params = [_]u8{
        // zig fmt: off
        0x00, 0x29, 0x00,
        0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0,
        0x01, 0x00,
        // zig fmt: on
    };
    const cc = try parseConnectionComplete(&params);
    try testing.expectEqual(@as(u8, 0), cc.status);
    try testing.expectEqual(@as(u16, 0x0029), cc.handle);
    try testing.expectEqual(LinkType.acl, cc.link_type);
    try testing.expect(!cc.encryption_enabled);
    try testing.expectEqualSlices(u8, &.{ 0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0 }, &cc.peer);
}

test "the connection handle is twelve bits, not sixteen" {
    // The top four bits are reserved and a controller may set them. Carrying
    // them into a handle makes every later ACL packet address a connection
    // that does not exist.
    const params = [_]u8{ 0x00, 0x29, 0xF0, 0, 0, 0, 0, 0, 0, 0x01, 0x00 };
    const cc = try parseConnectionComplete(&params);
    try testing.expectEqual(@as(u16, 0x0029), cc.handle);
}

test "a connection that timed out is reported, not mistaken for success" {
    // 0x10 = Connection Accept Timeout Exceeded — what rung 21b produced every
    // time, by never answering the request.
    const params = [_]u8{ 0x10, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0x01, 0x00 };
    const cc = try parseConnectionComplete(&params);
    try testing.expectEqual(@as(u8, 0x10), cc.status);
}

test "encryption change is the finish line, and its handle is masked too" {
    const params = [_]u8{ 0x00, 0x29, 0x50, 0x01 };
    const ec = try parseEncryptionChange(&params);
    try testing.expectEqual(@as(u8, 0), ec.status);
    try testing.expectEqual(@as(u16, 0x0029), ec.handle);
    try testing.expect(ec.enabled);

    const off = [_]u8{ 0x00, 0x29, 0x00, 0x00 };
    try testing.expect(!(try parseEncryptionChange(&off)).enabled);
}

test "a link key notification carries sixteen bytes and a type" {
    var params: [23]u8 = @splat(0);
    @memcpy(params[0..6], &[_]u8{ 0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0 });
    for (params[6..22], 0..) |*b, i| b.* = @truncate(0xE0 + i);
    params[22] = 0x05; // authenticated combination key, P-192

    const n = try parseLinkKeyNotification(&params);
    try testing.expectEqual(@as(u8, 0x05), n.key_type);
    try testing.expectEqual(@as(u8, 0xE0), n.key[0]);
    try testing.expectEqual(@as(u8, 0xEF), n.key[15]);
    try testing.expectEqualSlices(u8, &.{ 0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0 }, &n.peer);
}

test "a truncated link key notification is refused rather than read short" {
    const short: [22]u8 = @splat(0);
    try testing.expectError(Error.Malformed, parseLinkKeyNotification(&short));
}

test "a user confirmation request carries six digits nobody here can show" {
    const params = [_]u8{
        0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0,
        0x40, 0xE2, 0x01, 0x00, // 123456
    };
    const r = try parseUserConfirmationRequest(&params);
    try testing.expectEqual(@as(u32, 123456), r.numeric_value);
}

test "accept-connection asks to remain the peripheral" {
    const peer = [6]u8{ 0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0 };
    const p = acceptConnectionParams(peer, true);
    try testing.expectEqualSlices(u8, &peer, p[0..6]);
    try testing.expectEqual(@as(u8, 0x01), p[6]);

    const central = acceptConnectionParams(peer, false);
    try testing.expectEqual(@as(u8, 0x00), central[6]);
}

test "a speaker reports no input and no output, and asks for general bonding" {
    const peer = [6]u8{ 0xA5, 0xF9, 0x7F, 0x6A, 0x1C, 0xC0 };
    const p = ioCapabilityReplyParams(peer, .no_input_no_output, auth.general_bonding);
    try testing.expectEqualSlices(u8, &peer, p[0..6]);
    try testing.expectEqual(@as(u8, 0x03), p[6]); // NoInputNoOutput
    try testing.expectEqual(@as(u8, 0x00), p[7]); // no OOB
    try testing.expectEqual(@as(u8, 0x04), p[8]); // general bonding, no MITM

    // MITM is the low bit, and a device with no display must not claim it.
    try testing.expectEqual(@as(u8, 0), auth.general_bonding & 0x01);
}

test "BR/EDR signalling is channel 1, and LE's 5 is a different channel" {
    // The distinction this constant exists for. A host that reuses
    // `l2cap.cid.signalling` here waits on a channel Classic never uses.
    const l2cap = @import("l2cap.zig");
    try testing.expectEqual(@as(u16, 0x0001), l2cap_cid.signalling);
    try testing.expectEqual(@as(u16, 0x0005), l2cap.cid.signalling);
    try testing.expect(l2cap_cid.signalling != l2cap.cid.signalling);
}

test "a signalling header keeps its id, which a response must echo" {
    // Connection Request for SDP, id 0x07, four bytes of payload.
    const frame = [_]u8{ 0x02, 0x07, 0x04, 0x00, 0x01, 0x00, 0x41, 0x00 };
    const s = try parseSignalling(&frame);
    try testing.expectEqual(sig.connection_request, s.code);
    try testing.expectEqual(@as(u8, 0x07), s.id);

    const req = try parseL2capConnectionRequest(s.payload);
    try testing.expectEqual(psm.sdp, req.psm);
    try testing.expectEqual(@as(u16, 0x0041), req.source_cid);
}

test "an AVDTP connection request is recognised by its PSM" {
    const frame = [_]u8{ 0x02, 0x09, 0x04, 0x00, 0x19, 0x00, 0x45, 0x00 };
    const s = try parseSignalling(&frame);
    const req = try parseL2capConnectionRequest(s.payload);
    try testing.expectEqual(psm.avdtp, req.psm);
    try testing.expectEqual(@as(u16, 0x0019), req.psm);
}

test "a signalling frame shorter than its own length field is refused" {
    const frame = [_]u8{ 0x02, 0x07, 0x08, 0x00, 0x01, 0x00 };
    try testing.expectError(Error.Malformed, parseSignalling(&frame));
    try testing.expectError(Error.Malformed, parseL2capConnectionRequest(&.{ 0x01, 0x00 }));
}

test "a signalling frame carries its own length, not the whole PDU's" {
    var buf: [32]u8 = undefined;
    const f = try buildSignalling(sig.connection_response, 0x07, &[_]u8{ 1, 2, 3, 4, 5, 6, 7, 8 }, &buf);
    try testing.expectEqual(@as(usize, 12), f.len);
    try testing.expectEqual(sig.connection_response, f[0]);
    try testing.expectEqual(@as(u8, 0x07), f[1]);
    try testing.expectEqual(@as(u16, 8), std.mem.readInt(u16, f[2..4], .little));

    // And it round-trips through the parser it will meet at the other end.
    const back = try parseSignalling(f);
    try testing.expectEqual(@as(u8, 0x07), back.id);
    try testing.expectEqual(@as(usize, 8), back.payload.len);
}

test "a connection response names our channel first and theirs second" {
    const p = connectionResponseParams(0x0040, 0x0041, .success);
    try testing.expectEqual(@as(u16, 0x0040), std.mem.readInt(u16, p[0..2], .little));
    try testing.expectEqual(@as(u16, 0x0041), std.mem.readInt(u16, p[2..4], .little));
    try testing.expectEqual(@as(u16, 0), std.mem.readInt(u16, p[4..6], .little));

    const refused = connectionResponseParams(0, 0x0041, .refused_psm_unsupported);
    try testing.expectEqual(@as(u16, 2), std.mem.readInt(u16, refused[4..6], .little));
}

test "the MTU option is found even when it carries the hint bit" {
    const plain = [_]u8{ 0x01, 0x02, 0xA0, 0x02 }; // MTU 672
    try testing.expectEqual(@as(?u16, 672), findMtu(&plain));

    // **The same option with bit 7 set.** A host comparing the whole type byte
    // sees 0x81, matches nothing, and misses an MTU the peer did specify.
    const hinted = [_]u8{ 0x81, 0x02, 0xA0, 0x02 };
    try testing.expectEqual(@as(?u16, 672), findMtu(&hinted));
}

test "the MTU is found after an option that is not the MTU" {
    // Flush timeout first, then MTU — the walk has to step over the first by
    // its length rather than assume a fixed stride.
    const options = [_]u8{ 0x02, 0x02, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0x04 };
    try testing.expectEqual(@as(?u16, 1024), findMtu(&options));
}

test "an options block with no MTU, or a truncated one, yields nothing" {
    try testing.expectEqual(@as(?u16, null), findMtu(&.{ 0x02, 0x02, 0xFF, 0xFF }));
    try testing.expectEqual(@as(?u16, null), findMtu(&.{ 0x01, 0x02, 0xA0 }));
    try testing.expectEqual(@as(?u16, null), findMtu(&.{}));
}

test "configure request and response keep their channel fields straight" {
    const req = try parseConfigureRequest(&[_]u8{ 0x40, 0x00, 0x00, 0x00, 0x01, 0x02, 0xA0, 0x02 });
    try testing.expectEqual(@as(u16, 0x0040), req.dcid);
    try testing.expectEqual(@as(u16, 0), req.flags);
    try testing.expectEqual(@as(?u16, 672), findMtu(req.options));

    const rsp = try parseConfigureResponse(&[_]u8{ 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 });
    try testing.expectEqual(@as(u16, 0x0041), rsp.scid);
    try testing.expectEqual(@as(u16, 0), rsp.result);

    try testing.expectError(Error.Malformed, parseConfigureRequest(&.{ 0x40, 0x00 }));
    try testing.expectError(Error.Malformed, parseConfigureResponse(&.{ 0x41, 0x00, 0x00, 0x00 }));
}

test "the offered MTU fits in one receive buffer" {
    // Offering more than the receive path can hold is a promise it cannot keep,
    // and the failure arrives as a truncated response rather than an error.
    const hci = @import("hci.zig");
    try testing.expect(local_mtu <= hci.bredr_acl_data_max);
    try testing.expect(first_dynamic_cid == 0x0040);
}

test "connection request names the psm and our own cid" {
    const p = connectionRequestParams(psm.avdtp, 0x0040);
    try testing.expectEqualSlices(u8, &.{ 0x19, 0x00, 0x40, 0x00 }, &p);
}

test "connection response separates pending from success and refusal" {
    const ok = try parseL2capConnectionResponse(&.{ 0x41, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 });
    try testing.expectEqual(@as(u16, 0x0041), ok.dcid);
    try testing.expectEqual(@as(u16, 0x0040), ok.scid);
    try testing.expect(ok.ok());
    try testing.expect(!ok.pending());

    // Pending carries no usable dcid yet, and must not be read as a refusal.
    const pend = try parseL2capConnectionResponse(&.{ 0x00, 0x00, 0x40, 0x00, 0x01, 0x00, 0x00, 0x00 });
    try testing.expect(pend.pending());
    try testing.expect(!pend.ok());

    const refused = try parseL2capConnectionResponse(&.{ 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00 });
    try testing.expect(!refused.ok());
    try testing.expect(!refused.pending());

    try testing.expectError(Error.Malformed, parseL2capConnectionResponse(&.{ 0, 0, 0 }));
}

test "switch role names the peer and the role we want" {
    const p = switchRoleParams(.{ 1, 2, 3, 4, 5, 6 }, true);
    try testing.expectEqualSlices(u8, &.{ 1, 2, 3, 4, 5, 6, 0x01 }, &p);
    const central = switchRoleParams(.{ 1, 2, 3, 4, 5, 6 }, false);
    try testing.expectEqual(@as(u8, 0x00), central[6]);
}

test "a link key reply is the peer then the key, in that order" {
    // **Bytes on the wire**, which is what this file asserts rather than that a
    // function returns something. A swapped order here would authenticate
    // nothing and look like a peer problem.
    const peer = [6]u8{ 0xC0, 0x1C, 0x6A, 0x7F, 0xF9, 0xA5 };
    var key: [16]u8 = undefined;
    for (&key, 0..) |*b, i| b.* = @intCast(0x10 + i);

    const p = linkKeyReplyParams(peer, key);
    try testing.expectEqual(@as(usize, 22), p.len);
    try testing.expectEqualSlices(u8, &peer, p[0..6]);
    try testing.expectEqualSlices(u8, &key, p[6..22]);
}
