//! Infineon CYW43439 Wi-Fi 4 / Bluetooth 5.2 driver for the Raspberry Pi Pico 2 W.
//!
//! Layered over the gSPI PIO transport in `gspi_pio.zig`, which owns the wire
//! format. This file owns the chip: the bus handshake, the core reset and
//! firmware download, and the SDPCM control frame that carries an IOCTL.
//!
//! ## What works, and what does not
//!
//! Proven on hardware: `busInit`, `loadFirmware`, `setGpio`, `sendIoctl`,
//! `loadClm`, `wifiOn` and `joinWifi` — SDPCM credits refreshed from
//! `pollDevice` so the IOCTL loop sustains, responses matched to requests by
//! id, the chip's CDC status word checked, async events parsed off channel 1,
//! and a WPA2 association completed against a real AP. With `sendEthFrame` and
//! `pollEthFrame` below, that is rungs 1–14 of the bring-up ladder.
//!
//! `sendEthFrame` and `pollEthFrame` are rung 14, proven by an ARP round-trip:
//! the board asked which host held the gateway's address and the gateway
//! answered, to this station's own MAC.
//!
//! **Above the frame is not this package.** `lib/ipnet` takes the DHCP lease,
//! defends it by ARP and carries UDP — rungs 15, 16 and 18, all proven on
//! hardware while that code lived here. It moved out on 2026-08-31 because an
//! IPv4 stack has no business knowing what radio is under it, and because it
//! already imported nothing from this file. Nothing here imports it either:
//! `ipnet.IpStack(Dev)` is generic over a device, and an application is what
//! joins the two.
//!
//! `bt/` is the exception to all of this: it is reachable from here, it has
//! never been run, and nothing in the package exercises it. Rung 17.
//!
//! The reason for the care: `sendIoctl` was once an empty body that returned
//! success, which made `joinWifi` appear to associate — it issued nothing, then
//! set `link_status = .joined` unconditionally, and everything above it
//! inherited that fiction.

const std = @import("std");

/// Build-time selections. Credentials for the examples, and **which firmware
/// image this build embeds** — see `Image` below.
const wifi_config = @import("wifi_config");

/// Submodules.
pub const gspi = @import("bus/gspi_pio.zig");
pub const btbus = @import("bt/btbus.zig");
pub const hci = @import("bt/hci.zig");
pub const l2cap = @import("bt/l2cap.zig");
pub const att = @import("bt/att.zig");
pub const bredr = @import("bt/bredr.zig");
pub const sdp = @import("bt/sdp.zig");
pub const avdtp = @import("bt/avdtp.zig");
pub const sbc = @import("bt/sbc.zig");
pub const sbc_decode = @import("bt/sbc_decode.zig");
pub const BtHost = hci.Host;
pub const BtBus = btbus.BtBus;
/// The Bluetooth patchram, downloaded over the shared bus after the WLAN side
/// is running. Not part of either firmware image.
pub const fw_btfw = @embedFile("btfw_43439.bin");

pub const PicogSpiBus = gspi.GspiBus;
pub const BoardPins = gspi.BoardPins;

/// Raspberry Pi Pico 2 W default hardware pin layout.
pub const default_pins = BoardPins{};

/// The wire format lives with the transport; these are re-exports so callers
/// need not know which file it came from. There is exactly one packer.
pub const Function = gspi.Function;
pub const Header = gspi.Header;
pub const makeCmd = gspi.makeCmd;

// -- embedded firmware ------------------------------------------------------
//
// **Two images, and which one a build uses is the build's choice.** Infineon
// ships a WiFi-only firmware and a WiFi+BT one, and they are *different builds
// of the WiFi microcode* rather than the same bytes with Bluetooth appended:
// 224,190 against 231,077. Bluetooth needs the combined one, because it is what
// carries the shared-memory mailbox `cybt_shared_bus` talks to.
//
// Rungs 1–16 were proven on the WiFi-only image, so that stays the default and
// a BT build opts in. `-Dfw=combined` selects it; `Image` is the type and
// `image` the selection. Only the chosen blob is referenced, so only it is
// embedded.

/// The layout both images share: WiFi microcode, then the CLM blob at a
/// 512-aligned offset, and nothing else.
///
/// **Downloading the whole file writes CLM bytes past the end of the
/// microcode.** The canonical driver's trace says `writing 224192 bytes to 0x0`
/// — 224,190 rounded up to 4 — not 225,240.
pub const Image = struct {
    blob: []const u8,
    /// `CYW43_WIFI_FW_LEN` for this image.
    fw_len: usize,
    /// `CYW43_CLM_LEN`. The same 984 in both.
    clm_len: usize = 984,

    /// The microcode alone — **this is what `loadFirmware` wants.**
    pub fn microcode(comptime self: Image) []const u8 {
        return self.blob[0..self.fw_len];
    }

    /// The CLM blob alone.
    pub fn clm(comptime self: Image) []const u8 {
        return self.blob[std.mem.alignForward(usize, self.fw_len, 512)..][0..self.clm_len];
    }

    /// The two together account for the whole file, or the lengths are wrong.
    pub fn check(comptime self: Image) void {
        const end = std.mem.alignForward(usize, self.fw_len, 512) + self.clm_len;
        if (end != self.blob.len) @compileError("firmware image length does not match its layout");
    }
};

pub const wifi_image = Image{
    .blob = @embedFile("w43439A0_combined.bin"),
    .fw_len = 224190,
};

/// WiFi+BT. The firmware array in `wb43439A0_7_95_49_00_combined.h` is 231,077
/// bytes, and 231,424 + 984 = 232,408, which is the whole blob.
pub const combined_image = Image{
    .blob = @embedFile("wb43439A0_combined.bin"),
    .fw_len = 231077,
};

/// The image this build uses. `-Dfw=combined` for Bluetooth.
pub const image: Image = if (wifi_config.fw_combined) combined_image else wifi_image;

comptime {
    wifi_image.check();
    combined_image.check();
}

/// The selected image's parts. Everything below the firmware download uses
/// these, so an example changes one build flag rather than every call site.
pub const fw_wifi: []const u8 = image.microcode();
pub const fw_clm: []const u8 = image.clm();
pub const wifi_fw_len: usize = image.fw_len;
pub const clm_len: usize = image.clm_len;
pub const clm_offset: usize = std.mem.alignForward(usize, image.fw_len, 512);
/// The whole selected file, for tests that check the layout.
pub const fw_wifi_only = image.blob;
/// WiFi NVRAM configuration string.
pub const fw_nvram = @embedFile("wifi_nvram_43439.bin");

// ---------------------------------------------------------------------------
// Chip addresses and constants, from cyw43_ll.c
// ---------------------------------------------------------------------------

const chipcommon_base: u32 = 0x1800_0000;

/// chipcommon's chip-ID register on a CYW43439, measured.
///
/// The low 16 bits are `0xA9AF` — **43439 decimal**, the part number — with
/// revision 5, package 4, five cores and backplane type 1 above it. This is the
/// value that confirms a windowed backplane read actually worked, and it is
/// worth knowing by sight: `0x08000009` in its place is not data at all but a
/// status word with `DATA_UNAVAILABLE` set.
pub const chipcommon_chip_id: u32 = 0x1545_A9AF;
const sdio_base: u32 = 0x1800_2000;
const wlan_armcm3_base: u32 = 0x1800_3000;
const socsram_base: u32 = 0x1800_4000;
const wrapper_offset: u32 = 0x0010_0000;

const socsram_bankx_index: u32 = socsram_base + 0x10;
const socsram_bankx_pda: u32 = socsram_base + 0x44;
const sdio_int_host_mask: u32 = sdio_base + 0x24;

const ai_ioctrl_offset: u32 = 0x408;
const ai_resetctrl_offset: u32 = 0x800;

const sicf_clock_en: u8 = 0x01;
const sicf_fgc: u8 = 0x02;
const sicf_cpuhalt: u8 = 0x20;
const airc_reset: u8 = 0x01;

/// Backplane (Function 1) registers, addressed directly rather than through the
/// window.
const sdio_function2_watermark: u17 = 0x10008;
const sdio_chip_clock_csr: u17 = 0x1000E;

const sbsdio_alp_avail_req: u8 = 0x08;
const sbsdio_alp_avail: u8 = 0x40;
const sbsdio_ht_avail: u8 = 0x80;

const spi_f2_watermark: u8 = 32;
const i_hmb_sw_mask: u32 = 0x0000_00F0;

/// 512 KB of on-chip RAM; the NVRAM length footer sits in its top word.
const ram_size: u32 = 512 * 1024;

/// Writes are padded to this boundary.
///
/// **Four, not sixty-four — and this file said sixty-four for a while.**
/// `CYW43_WRITE_BYTES_PAD` is `ALIGN(len, 4)` **on SPI** and `ALIGN(len, 64)`
/// on SDIO, which is the opposite of how it was read here first. Confirmed
/// against a live trace of the canonical driver on this board: it downloads
/// 224,192 bytes (224,190 aligned to 4) and places 744 bytes of NVRAM (742
/// aligned to 4). Neither is a multiple of 64.
const sdpcm_align: usize = 4;

const Core = enum { wlan_arm, socram };

fn coreAddress(core: Core) u32 {
    return switch (core) {
        .wlan_arm => wlan_armcm3_base + wrapper_offset,
        .socram => socsram_base + wrapper_offset,
    };
}

// ---------------------------------------------------------------------------
// SDPCM / CDC
// ---------------------------------------------------------------------------

/// SDPCM header length — `struct sdpcm_header_t`, twelve bytes.
const sdpcm_header_len: usize = 12;
/// CDC IOCTL header length — cmd, len, flags, status.
const cdc_header_len: usize = 16;

const control_header: u8 = 0;
/// Channel 1 — the chip's asynchronous events.
const asyncevent_header: u8 = 1;
/// Channel 2 — 802.3 frames, in both directions. Rung 14.
const data_header: u8 = 2;
/// `BDC_HEADER_LEN`: flags, priority, flags2, data_offset.
const bdc_header_len: usize = 4;
/// The event structure's own length, from `cyw43_async_event_t`.
const async_event_len: usize = 48;

/// **Two bytes of padding sit between the SDPCM and BDC headers, on the data
/// channel only.** `cyw43_ll_send_ethernet` writes its BDC header at
/// `spid_buf[SDPCM_HEADER_LEN + 2]` and hands `6 + len` to
/// `cyw43_sdpcm_send_common`, which sets `header_length = SDPCM_HEADER_LEN +
/// (kind == DATA_HEADER ? 2 : 0)`. Control frames have no such gap, which is
/// why nothing below rung 14 needed this.
///
/// It is not free padding: `header_length` is the field the chip — and our own
/// receive path — uses to find the BDC header, so getting it wrong displaces
/// the whole frame by two bytes rather than producing an error.
const sdpcm_data_pad: usize = 2;

/// A full 802.3 frame: 14-byte header plus a 1500-byte MTU. No FCS — the chip
/// appends it.
pub const eth_max_frame: usize = 1514;

/// How many received frames the driver will hold. **Two, and it was four until
/// hardware said four was three too many.**
///
/// The original reasoning was that a peer sends back-to-back segments — which
/// it does, 26 µs apart in a captured TCP session — and that the
/// ring had to hold them. **That was wrong about where they queue.** The chip's
/// own F2 FIFO holds a burst; `pollDevice` reads one packet per call and
/// `pollEthFrame` pops it immediately, so the receive path is one-in-one-out
/// and cannot reach depth 2 however hard it is driven.
///
/// **The path that does accumulate is `awaitCredit`**, inside every
/// `sendEthFrame`: it drains Function 2 while waiting for the chip's credit,
/// and frames latched there have nothing popping them until the transmit
/// finishes. One slot dropped the *second* such frame, and closing that hole is
/// the whole of what this depth buys.
///
/// **Measured across three runs — 15,519 frames and 183 transmits — the ring
/// never exceeded depth 1.** Credit is almost always available, so the window
/// during which frames can pile up is very short. Four slots were carrying
/// 4,542 bytes that no traffic on this bench could reach.
///
/// Two, then: enough for the documented hole, and **provisional**. A stack
/// running TCP transmits an ACK per segment, so the credit window is open far
/// more often than anything measured here — revisit this with `rx_high_water`
/// from a real TCP run rather than assuming either way.
///
/// Costs `3 * 1514` = 4,542 bytes of RAM — one slot more than the depth, see
/// below.
pub const rx_ring_depth: usize = 2;

/// **One more slot than the depth, because an empty ring and a full one are
/// otherwise the same state.** With `head == tail` meaning empty, a ring of N
/// slots holds N-1 frames — so sizing the array to `rx_ring_depth` would have
/// made `rx_ring_depth` a lie by one, and quietly dropped the fourth frame of
/// a four-frame burst. It did exactly that for one run of the tests below,
/// which is the only reason this comment can be written with confidence.
const rx_ring_slots: usize = rx_ring_depth + 1;

/// `bdc_flags` on a transmitted frame.
///
/// The reference sets 0x20 and defines no constant for it. Linux's `brcmfmac`
/// names the same field `BDC_FLAG_VER_SHIFT = 4` over `BDC_PROTO_VER = 2`,
/// which would make this "protocol version 2" — plausible, and **not verified
/// here**. What is verified is the byte the chip accepts.
const bdc_flags_tx: u8 = 0x20;

/// The largest data-channel frame this driver builds or reads.
const max_data_frame: usize = sdpcm_header_len + sdpcm_data_pad +
    bdc_header_len + eth_max_frame;

/// Largest Function 2 packet this driver will read.
///
/// **A control response echoes the request's length, so this cannot be smaller
/// than the largest frame we send.** It was 256 while the biggest request was
/// 44 bytes, and the first CLM chunk deadlocked against it: the chip reported a
/// 1036-byte packet waiting, `pollDevice` declined to read anything that large,
/// the packet was never drained, and the status word sat at `0x81928` —
/// packet-available, length 1036 — until the IOCTL timed out. Nothing was wrong
/// with the chunk.
///
/// The `oversize` census counter existed precisely so this would be visible
/// rather than mysterious, and it is what the status word confirmed.
///
/// **Rung 14 is what makes the data channel the larger of the two.** This
/// tracked the control path alone while nothing above the link was read, at
/// 1076 bytes — smaller than a full frame, so any traffic above that size was
/// counted in `oversize` and skipped rather than read.
/// **The largest packet the chip can announce, not the largest we expect.**
///
/// `gspi.packetLength` reads an 11-bit field, so 2047 is the ceiling and 2048
/// holds anything the status word is capable of describing. That is a bound on
/// the *instrument* rather than on the traffic, and it is the only kind that
/// cannot be four bytes short.
///
/// It was `@max(max_control_frame, max_data_frame)` — 1532, from 12 + 2 + 4 +
/// 1514. Correct-looking arithmetic, and wrong: the chip announced **1536** for
/// a full-size Ethernet frame. The extra four bytes are not explained by
/// anything in the datasheet that I could find, and the reference does not
/// derive a bound at all — `cyw43_internal.h` reserves a flat `spid_buf[2048]`.
///
/// The consequence was not a dropped frame. **A packet refused for size is
/// never drained**, so it stays pending and every later poll refuses the same
/// one: 7436 refusals of one frame, no ARP, no TCP, and a receive path wedged
/// from the first full-size frame onward — which above the link looks exactly
/// like a bug in the TCP stack. The same shape as the CLM chunk deadlock in
/// `max_control_frame`'s note, which should have been the warning.
///
/// Found by `lib/ipnet`'s net-check fetching 1500 bytes over HTTP: the body
/// arrived in two segments and the second was never acknowledged, through the
/// original and eight retransmissions.
const max_rx_packet: usize = 1 << 11;

comptime {
    // The refusal branch in `pollDevice` is unreachable at this size. Keep it
    // anyway — it costs a compare — but keep this assertion with it, so that
    // shrinking the buffer fails here rather than on a bench in six months.
    if (max_rx_packet <= gspi.max_packet_length) @compileError("rx_buf cannot hold the largest announceable packet");
    if (max_rx_packet < max_control_frame or max_rx_packet < max_data_frame) @compileError("rx_buf smaller than a frame this driver builds");
}

const sdpcm_get: u32 = 0;
const sdpcm_set: u32 = 2;
const cdcf_ioc_id_shift: u5 = 16;
const cdcf_ioc_if_shift: u5 = 12;

// -- CLM download -----------------------------------------------------------

/// `CLM_CHUNK_LEN` **on SPI**. The SDIO build uses 1024+512; this is the branch
/// that applies to us. Our whole blob is 984 bytes, so it goes in one chunk —
/// the loop is kept because the constant is the chip's, not the blob's.
pub const clm_chunk_len: usize = 1024;

/// The `clmload` IOVAR's own header, ahead of the blob bytes: the name and its
/// terminator (8), flag (2), type (2), length (4), CRC (4).
const clm_hdr_len: usize = 20;

const dload_handler_ver: u16 = 1 << 12;
const dload_flag_begin: u16 = 2;
const dload_flag_end: u16 = 4;
/// `DL_TYPE_CLM`.
const dload_type_clm: u16 = 2;

/// **A CLM chunk's IOCTL length is padded to 8, not to 4.** `cyw43_clm_load`
/// passes `ALIGN_UINT(20 + len, 8)` as the length to `cyw43_do_ioctl`, so the
/// CDC header itself declares the padded length and the trailing bytes are
/// junk the chip ignores. That is a different padding from `sdpcm_align`, which
/// rounds the *transfer* rather than the declared payload.
const clm_payload_align: usize = 8;

/// `clm_status` before any `clmload_status` read has produced a word. Distinct
/// from 0, which is the chip saying the download was accepted.
pub const no_clm_status: u32 = 0xFFFF_FFFF;

/// Largest control frame this driver builds.
///
/// **Sized by the CLM download, which is far larger than anything else on the
/// control channel.** A chunk is the 20-byte `clmload` header plus up to
/// `clm_chunk_len` bytes of blob, the pair padded to 8, with the SDPCM and CDC
/// headers in front. This was 128 while `gpioout` was the only IOCTL; a CLM
/// chunk would have been refused as `PayloadTooLarge` rather than truncated,
/// which is the right failure, but it is still why the number moved.
const max_control_frame: usize = sdpcm_header_len + cdc_header_len +
    std.mem.alignForward(usize, clm_hdr_len + clm_chunk_len, clm_payload_align);

/// CYW43 IOCTL commands, as `cyw43_ll.c` defines them and the canonical join
/// trace names them on the wire.
///
/// **`set_var` is 263 and `get_var` is 262.** They were 264 and 263 here, both
/// one too high — an off-by-one that would have made every IOVAR a different
/// command than intended, silently.
///
/// **Three more were wrong the same way, and were found by the join trace
/// rather than by reading:** `set_wsec` was 92 and is 134, and there was a
/// `join = 267` that does not exist — a join is `WLC_SET_SSID`, which is 26.
/// The 26 had been attached to a `get_link_status` that has no IOCTL at all;
/// link state is reported through async events. None of the three was reachable
/// from any code path, which is why nothing caught them, and all three sit in
/// the middle of the rung-12 sequence, and had been recorded as already right.
pub const IoctlCmd = enum(u32) {
    get_version = 1,
    up = 2,
    set_infra = 20,
    set_auth = 22,
    get_bssid = 23,
    get_ssid = 25,
    /// A join *is* this command: the SSID structure is what starts association.
    set_ssid = 26,
    set_channel = 30,
    disassoc = 52,
    set_antdiv = 64,
    set_pm = 86,
    set_gmode = 110,
    set_wsec = 134,
    set_band = 142,
    set_wpa_auth = 165,
    get_var = 262,
    set_var = 263,
    /// `WLC_SET_WSEC_PMK` — the passphrase, in a 68-byte `wsec_pmk_t`.
    set_passphrase = 268,
};

/// Link connection state.
///
/// `joined` means associated *and* keyed — see `JoinState`. There is no IP
/// address in this driver, so it has no state above that.
pub const LinkStatus = enum(u32) {
    down = 0,
    joining = 1,
    joined = 2,
    /// The join failed for a reason that is not the passphrase.
    fail = 3,
    /// No AP with that SSID answered: out of range, or not up.
    no_net = 4,
    bad_auth = 5,
};

/// The three things that must all happen for a join to be complete, as the
/// reference's `WIFI_JOIN_STATE_*` bits.
///
/// **A join is not done when `WLC_SET_SSID` returns.** It is done when
/// authentication, link-up and key exchange have each been reported by a
/// separate async event, in no guaranteed order. `WIFI_JOIN_STATE_ALL` is
/// `0x0E01` — all three, plus "active".
pub const JoinState = packed struct(u16) {
    kind: u4 = 0,
    _pad: u5 = 0,
    /// `CYW43_EV_AUTH` succeeded.
    auth: bool = false,
    /// `CYW43_EV_LINK` reported the link up.
    link: bool = false,
    /// `CYW43_EV_PSK_SUP` reported `WLC_SUP_KEYED`.
    keyed: bool = false,
    _rest: u4 = 0,

    const kind_active: u4 = 1;
    const kind_fail: u4 = 2;
    const kind_nonet: u4 = 3;
    const kind_badauth: u4 = 4;

    fn complete(self: JoinState) bool {
        return self.kind == kind_active and self.auth and self.link and self.keyed;
    }
};

/// Async event types (`CYW43_EV_*`).
pub const EventType = enum(u32) {
    set_ssid = 0,
    join = 1,
    auth = 3,
    deauth = 5,
    deauth_ind = 6,
    assoc = 7,
    disassoc = 11,
    disassoc_ind = 12,
    link = 16,
    prune = 23,
    psk_sup = 46,
    icv_error = 49,
    escan_result = 69,
    csa_complete_ind = 80,
    assoc_req_ie = 87,
    assoc_resp_ie = 88,
    _,
};

/// `WLC_SUP_KEYED` — the `psk_sup` status that means the key exchange finished.
const sup_keyed: u32 = 6;

/// One parsed async event.
pub const AsyncEvent = struct {
    event_type: u32 = 0,
    status: u32 = 0,
    reason: u32 = 0,
    flags: u16 = 0,
    interface: u8 = 0,
};

/// WPA/WPA2/WPA3 authentication mode.
///
/// **These are packed values, not an index.** They were 0–4 here, which is what
/// they look like if you assume an enumeration. The low byte goes straight to
/// `WLC_SET_WSEC` — the join trace's `Setting wsec=0x4` is the low byte of
/// `wpa2_aes` — and the upper bits select the `WLC_SET_WPA_AUTH` value. A
/// `wpa2_aes` of 2 would have set `wsec=0x2` and picked the WPA1 auth mode.
pub const AuthType = enum(u32) {
    open = 0x0000_0000,
    wpa_tkip = 0x0020_0002,
    wpa2_aes = 0x0040_0004,
    wpa2_mix = 0x0040_0006,
    wpa3_sae = 0x0100_0004,
    wpa3_wpa2 = 0x0140_0004,

    /// The `wsec` value: the low byte, and nothing else.
    pub fn wsec(self: AuthType) u32 {
        return @backingInt(self) & 0xFF;
    }

    /// The `WLC_SET_WPA_AUTH` value this mode implies.
    pub fn wpaAuth(self: AuthType) u32 {
        return switch (self) {
            .open => 0,
            .wpa_tkip => wpa_auth_psk,
            .wpa2_aes, .wpa2_mix => wpa2_auth_psk,
            .wpa3_sae, .wpa3_wpa2 => wpa3_auth_sae_psk,
        };
    }
};

const wpa_auth_psk: u32 = 0x0004;
const wpa2_auth_psk: u32 = 0x0080;
const wpa3_auth_sae_psk: u32 = 0x4_0000;

/// How many IOCTLs `wifiOn` issues, and the width of `wifi_on_status`.
pub const wifi_on_steps: usize = 20;

/// How many IOCTLs `joinWifi` issues, and the width of `join_status`.
pub const join_steps: usize = 11;

/// `CYW43_PM2_POWERSAVE_MODE` — saves power only after a quiet period, and is
/// what `CYW43_PERFORMANCE_PM` selects.
const pm2_powersave_mode: u32 = 2;

const auth_type_open: u32 = 0;
const auth_type_sae: u32 = 3;
const mfp_none: u32 = 0;
const mfp_capable: u32 = 1;
/// `CYW_EAPOL_KEY_TIMEOUT`, in milliseconds.
const eapol_key_timeout: u32 = 5000;
/// `CYW43_WPA_MAX_PASSWORD_LEN`. The `wsec_pmk_t` is always sent full-length.
const wpa_max_password_len: usize = 64;

/// A two-letter country code, as the `country` IOVAR wants it.
pub const Country = enum(u32) {
    gb = code("GB"),
    us = code("US"),
    de = code("DE"),
    /// `XX` — the worldwide locale, and the SDK's default.
    worldwide = code("XX"),

    fn code(comptime s: *const [2]u8) u32 {
        return @as(u32, s[0]) | (@as(u32, s[1]) << 8);
    }
};

pub const Error = error{
    NotPowered,
    FirmwareNotLoaded,
    /// The test register never returned `0xFEEDBEAD` — the transport is not
    /// talking to the chip at all.
    ChipNotResponding,
    /// The handshake was sent but `SPI_BUS_CONTROL` did not read back.
    BusControlMismatch,
    /// The backplane's ALP clock never became available.
    AlpTimeout,
    /// The WLAN ARM core did not bring its HT clock up after firmware release.
    HtClockTimeout,
    /// Firmware was released but Function 2 never signalled ready.
    F2NotReady,
    /// A core was expected to be in reset and was not.
    CoreNotInReset,
    /// The chip never returned an SDPCM credit.
    CreditTimeout,
    /// An SDPCM header's length and its complement disagreed.
    SdpcmHeaderMismatch,
    /// The chip answered the IOCTL with a non-zero CDC status.
    IoctlRejected,
    /// `wifiOn` steps whose failure makes everything after them meaningless.
    CountryRejected,
    EventMaskRejected,
    WifiUpRejected,
    /// A join was attempted before `wifiOn`.
    WifiNotOn,
    BadSsid,
    BadPassphrase,
    /// The chip refused `WLC_SET_SSID` outright, before any event.
    JoinRejected,
    /// The join neither completed nor failed within the time allowed.
    JoinTimeout,
    /// An async event was too short, or was not an event at all.
    EventTooSmall,
    NotAnEvent,
    NotBroadcomOui,
    /// No response to the outstanding IOCTL arrived in time.
    IoctlTimeout,
    NotImplemented,
    PayloadTooLarge,
    /// A frame was sent before the link was up. The chip does not report this;
    /// it accepts the frame and drops it.
    LinkDown,
    /// A received frame is longer than the buffer offered for it.
    BufferTooSmall,
    /// The CLM blob was sent but `clmload_status` reported a non-zero result,
    /// or answered with no result at all.
    ClmLoadFailed,
};

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// Generic CYW43 driver, parameterised over a gSPI transport.
pub fn Device(comptime Bus: type) type {
    return struct {
        const Self = @This();

        bus: Bus,
        pins: BoardPins,
        powered: bool = false,
        bus_ready: bool = false,
        firmware_loaded: bool = false,
        link_status: LinkStatus = .down,
        /// Last value of the SDPCM header's wireless flow-control byte.
        wlan_flow_control: u8 = 0,
        /// The IOCTL awaiting a response, and where the response lands.
        pending_ioctl_id: u16 = 0,
        response: IoctlResponse = .{},

        /// What `pollDevice` has actually seen. Diagnostic only — the question
        /// it answers is whether the chip ever returns a control *response*,
        /// or whether every packet it sends is a flow-control update with no
        /// payload. The reference drops the latter explicitly
        /// (`header->size == SDPCM_HEADER_LEN`), so they are expected to be
        /// common; what matters is whether anything else arrives.
        census: RxCensus = .{},

        /// The word `clmload_status` returned, kept whether or not it was zero.
        /// `loadClm` turns a non-zero value into an error, and an error that
        /// carries no number is one that cannot be diagnosed over SWD.
        /// `no_clm_status` means the read never produced one.
        clm_status: u32 = no_clm_status,

        /// The station MAC, read from `cur_etheraddr` during `wifiOn`. All
        /// zeroes until then.
        mac: [6]u8 = @splat(0),
        /// The CDC status of each `wifiOn` step, indexed as in the §5 table.
        /// Kept whether zero or not: some are non-zero in a working bring-up,
        /// and a driver that discards them cannot tell which.
        wifi_on_status: [wifi_on_steps]u32 = @splat(0),
        /// The most recent parsed event, kept for diagnosis over SWD.
        last_event: AsyncEvent = .{},
        /// The join's progress, assembled from events.
        join_state: JoinState = .{},
        /// The CDC status of each `joinWifi` step, same reasoning as
        /// `wifi_on_status`.
        join_status: [join_steps]u32 = @splat(0),
        /// Set once `wifiOn` has completed. A join before it is an error rather
        /// than a sequence of IOCTLs the chip is not ready for.
        wifi_on: bool = false,

        /// SDPCM transmit sequence number, and the last credit the chip issued.
        ///
        /// **The credit starts at 1 while the sequence starts at 0**, because
        /// the two being equal *is* the stall condition — starting both at zero
        /// stalls before the first packet is ever sent. `cyw43_ll.c:306` does
        /// the same and its comment ("we get an immediate stall if this isn't
        /// done?") is the only explanation upstream offers.
        sdpcm_seq: u8 = 0,
        sdpcm_credit: u8 = 1,
        ioctl_id: u16 = 0,
        /// How many control frames have been sent. A rejected IOCTL in the
        /// middle of a twenty-step sequence is unreadable without it: the CDC
        /// status says a command failed, and this says *which* command.
        ioctl_count: u32 = 0,

        /// Frame buffers — rung 14.
        ///
        /// **Fields, not locals.** These were stack buffers while the largest
        /// frame was a 1036-byte CLM chunk drained by a function that only ever
        /// ran from `main`. A full 802.3 frame takes them past 1.5 KB each, and
        /// `pollDevice` runs from inside `awaitCredit` and `awaitResponse` — so
        /// on the stack they would land underneath whatever the caller was
        /// already holding.
        rx_buf: [max_rx_packet]u8 = @splat(0),
        tx_buf: [max_data_frame]u8 = @splat(0),

        /// Received 802.3 frames waiting to be collected.
        ///
        /// **Buffered here because `pollDevice` is not only called by
        /// `pollEthFrame`.** Every IOCTL drains Function 2 while it waits for
        /// its response, and the chip interleaves data frames with that freely:
        /// a receive path that only kept what arrived during its own call would
        /// drop frames in a pattern set by whatever else the caller happened to
        /// be doing.
        ///
        /// **This was one slot until rung N3**, and the reason the depth matters
        /// is not the one first written here. It is *not* that a burst
        /// overflows the receive path: `pollEthFrame` pops immediately after
        /// `pollDevice` latches, so that path is one-in-one-out and never
        /// reaches depth 2 at any rate — measured under a flood, `rx_high_water`
        /// stayed at 1.
        ///
        /// **It is `awaitCredit`.** Every `sendEthFrame` waits for the chip's
        /// credit and drains Function 2 while it waits, and frames latched
        /// there have nothing popping them until the transmit completes. One
        /// slot dropped the *second* such frame. A stack running TCP transmits
        /// an ACK per segment, so that window is open constantly — which is
        /// what makes the depth a TCP prerequisite, by a different route than
        /// the back-to-back segments in a captured TCP session.
        rx_ring: [rx_ring_slots][eth_max_frame]u8 = @splat(@splat(0)),
        rx_ring_len: [rx_ring_slots]usize = @splat(0),
        rx_ring_head: usize = 0,
        rx_ring_tail: usize = 0,
        /// Deepest the ring has ever been. **Read this before believing a
        /// `data_dropped` of zero**: a high-water mark that never leaves 1 says
        /// the depth was never needed, and one that touches `rx_ring_depth`
        /// says the next burst will overflow.
        rx_high_water: u32 = 0,
        rx_eth_itf: u8 = 0,
        /// Frames handed to the chip, and frames handed to the caller.
        eth_tx_count: u32 = 0,
        eth_rx_count: u32 = 0,

        pub fn init(bus: Bus, pins: BoardPins) Self {
            return .{ .bus = bus, .pins = pins };
        }

        // -- power ----------------------------------------------------------

        pub fn powerUp(self: *Self) !void {
            self.bus.powerOn();
            self.powered = true;
            self.bus_ready = false;
            self.firmware_loaded = false;
        }

        pub fn powerDown(self: *Self) !void {
            self.bus.powerOff();
            self.powered = false;
            self.bus_ready = false;
            self.firmware_loaded = false;
            self.link_status = .down;
        }

        // -- bus bring-up ---------------------------------------------------

        /// Read the gSPI test register in the chip's boot-time 16-bit mode.
        ///
        /// **This is the probe, and it happens before the handshake.** The
        /// analysis that preceded this rewrite had the order the other way
        /// round, which cannot work: the chip will not answer a 32-bit command
        /// until it has been told to expect one, and it is told that by a
        /// command it can only receive in 16-bit mode.
        pub fn readTestRegister(self: *Self) !u32 {
            if (!self.powered) try self.powerUp();
            return self.bus.readReg32Swap(.bus, gspi.reg.read_test);
        }

        /// Is a CYW43 out there and answering?
        pub fn probe(self: *Self) !bool {
            var attempts: u32 = 10;
            while (attempts > 0) : (attempts -= 1) {
                const v = self.readTestRegister() catch 0;
                if (v == gspi.test_pattern) return true;
                self.bus.delayMs(1);
            }
            return false;
        }

        /// `cyw43_ll_bus_init`: probe in 16-bit mode, switch to 32-bit, set the
        /// backplane read padding, and arm the interrupt mask.
        pub fn busInit(self: *Self) !void {
            if (!self.powered) try self.powerUp();

            if (!try self.probe()) return error.ChipNotResponding;

            // Leave 16-bit little-endian boot mode. Everything after this point
            // is a plain 32-bit command.
            const control_word = gspi.busControlWord(self.bus.high_speed);
            try self.bus.writeReg32Swap(.bus, gspi.reg.bus_control, control_word);

            // **Check the mode byte, not the whole word.** The chip does not
            // read back the response-delay byte at bits 8..15: writing
            // 0x000204B3 reads 0x000200B3, measured. An equality check here
            // therefore rejects a handshake that worked perfectly — which it
            // did, until the readback value was put on the wire and looked at.
            // The SDK reads this register and only logs it, never comparing.
            const readback = try self.bus.readReg32(.bus, gspi.reg.bus_control);
            if ((readback & 0xFF) != (control_word & 0xFF)) {
                return error.BusControlMismatch;
            }

            // Sixteen bytes of Function 1 response delay, and pad reads to
            // match — exactly as the reference does.
            //
            // This spent a long time set to 4, "measured" on hardware, because
            // the write above was being silently dropped: the register kept its
            // power-on 0x04 and every measurement faithfully confirmed it. The
            // measurements were right; what they were measuring was a chip that
            // had never been configured. See the stall-flag note in
            // `bus/gspi_pio.zig`'s `transfer`.
            try self.bus.writeReg8(.bus, gspi.reg.resp_delay_f1, gspi.backplane_read_pad_bytes);
            self.bus.resp_delay_bytes = gspi.backplane_read_pad_bytes;

            // Clear the error bits the boot window may have set, then enable
            // the interrupts worth hearing about.
            try self.bus.writeReg8(.bus, gspi.reg.interrupt, gspi.intr_clear_at_init);
            try self.bus.writeReg16(.bus, gspi.reg.interrupt_enable, gspi.intr_enable_wlan);

            self.bus_ready = true;
        }

        // -- backplane cores ------------------------------------------------

        pub fn disableCore(self: *Self, core: Core) !void {
            const base = coreAddress(core);
            // Read twice: the first read after a window change returns stale
            // data on this backplane, which the SDK works around the same way.
            _ = try self.bus.readBackplaneReg8(base + ai_resetctrl_offset);
            const reg = try self.bus.readBackplaneReg8(base + ai_resetctrl_offset);
            if ((reg & airc_reset) == 0) return error.CoreNotInReset;
        }

        pub fn resetCore(self: *Self, core: Core, halt: bool) !void {
            self.disableCore(core) catch {};

            const base = coreAddress(core);
            const halt_bit: u8 = if (halt) sicf_cpuhalt else 0;

            try self.bus.writeBackplaneReg8(base + ai_ioctrl_offset, sicf_fgc | sicf_clock_en | halt_bit);
            _ = try self.bus.readBackplaneReg8(base + ai_ioctrl_offset);
            try self.bus.writeBackplaneReg8(base + ai_resetctrl_offset, 0);
            self.bus.delayMs(1);
            try self.bus.writeBackplaneReg8(base + ai_ioctrl_offset, sicf_clock_en | halt_bit);
            _ = try self.bus.readBackplaneReg8(base + ai_ioctrl_offset);
            self.bus.delayMs(1);
        }

        // -- firmware -------------------------------------------------------

        /// Bring the backplane's ALP clock up, then let it go again.
        /// Bring the backplane's ALP clock up, then release the request.
        ///
        /// **Nothing behind the backplane window answers until this has run.**
        /// Direct Function 1 registers (`0x10000`+, including the window
        /// registers themselves) live in the SDIO function block and respond
        /// without a backplane clock; anything reached *through* the window does
        /// not, and a read issued too early comes back with `DATA_UNAVAILABLE`
        /// rather than an error. That reads as a plausible constant and cost a
        /// session's worth of chasing.
        pub fn requestAlp(self: *Self) !void {
            try self.bus.writeReg8(.backplane, sdio_chip_clock_csr, sbsdio_alp_avail_req);

            var attempts: u32 = 10;
            while (attempts > 0) : (attempts -= 1) {
                const reg = self.bus.readReg8(.backplane, sdio_chip_clock_csr) catch 0;
                if ((reg & sbsdio_alp_avail) != 0) {
                    // Release the request once granted, as the SDK does.
                    // Holding it asserted was tried and changes nothing: the
                    // clock is not what is going away.
                    try self.bus.writeReg8(.backplane, sdio_chip_clock_csr, 0);
                    return;
                }
                self.bus.delayMs(1);
            }
            return error.AlpTimeout;
        }

        /// Write a pattern into chip RAM and read it back. Returns the number of
        /// bytes that differ, so 0 is a healthy backplane.
        ///
        /// **Run this before the firmware download, not instead of checking it.**
        /// The download is 225 KB through the same path; if that path is wrong,
        /// the only symptom is a core that will not start thirty seconds later,
        /// which says nothing about why. Chip RAM at address 0 is free to
        /// scribble on — the firmware is about to overwrite it.
        ///
        /// Requires `requestAlp` to have run.
        /// Put both cores down, cycle SOCRAM, and disable the SRAM_3 remap.
        ///
        /// **Chip RAM is not writable until this has run.** SOCRAM is the core
        /// that owns the RAM the firmware is about to be written into, and while
        /// it sits in reset a write to address 0 is accepted and discarded — the
        /// read that follows returns the old contents with no error anywhere.
        /// A RAM round-trip attempted before this point measures nothing, which
        /// cost a full round of chasing a write path that was never broken.
        pub fn prepareCores(self: *Self) !void {
            self.disableCore(.wlan_arm) catch {};
            self.disableCore(.socram) catch {};
            try self.resetCore(.socram, false);
            try self.disableSram3Remap();
        }

        /// Does a plain 32-bit-mode Function 0 write reach the chip at all?
        ///
        /// **The handshake write is byte-swapped and demonstrably works; every
        /// later write is not, and none has ever been verified.** If ordinary
        /// writes are silently dropped then the backplane *window* registers
        /// were never written either — which would explain why every windowed
        /// read returns the same value whatever address is asked for, why
        /// SOCRAM never leaves reset, and why the download goes nowhere.
        ///
        /// `RESP_DELAY_F2` (0x1E) is the scratch register: the canonical driver
        /// reads it as 0x00 and never uses it, so writing it disturbs nothing.
        pub fn probeWritePath(self: *Self) struct {
            delay_before: u8,
            delay_after: u8,
            scratch_before: u8,
            scratch_after: u8,
        } {
            const d0 = self.bus.readReg8(.bus, gspi.reg.resp_delay_f1) catch 0xFF;
            self.bus.writeReg8(.bus, gspi.reg.resp_delay_f1, 16) catch {};
            const d1 = self.bus.readReg8(.bus, gspi.reg.resp_delay_f1) catch 0xFF;

            const s0 = self.bus.readReg8(.bus, 0x1E) catch 0xFF;
            self.bus.writeReg8(.bus, 0x1E, 0x55) catch {};
            const s1 = self.bus.readReg8(.bus, 0x1E) catch 0xFF;
            self.bus.writeReg8(.bus, 0x1E, 0x00) catch {};

            return .{ .delay_before = d0, .delay_after = d1, .scratch_before = s0, .scratch_after = s1 };
        }

        /// Read one known register at every plausible response delay and hand
        /// back all the answers.
        ///
        /// **This is the honest version of a measurement that has been botched
        /// three times.** Each read uses a receive length correct for *its own*
        /// padding, so nothing is over-clocked; the only variable is how many
        /// words are skipped. `SDIO_CHIP_CLOCK_CSR` is the target because its
        /// value is known independently — after `requestAlp` it must have
        /// `ALP_AVAIL` (0x40) set, and once the core is up, `HT_AVAIL` (0x80)
        /// too. Whichever slot shows that is the real delay.
        pub fn measureRespDelay(self: *Self) [5]u32 {
            const saved = self.bus.resp_delay_bytes;
            var out: [5]u32 = @splat(0xFFFF_FFFF);
            for (0..5) |p| {
                self.bus.resp_delay_bytes = p * 4;
                out[p] = self.bus.readReg8(.backplane, sdio_chip_clock_csr) catch 0xEE;
            }
            self.bus.resp_delay_bytes = saved;
            return out;
        }

        /// Re-assert `SPI_RESP_DELAY_F1`, then measure again.
        ///
        /// Isolates whether the register write simply does not take, or takes
        /// and is later undone by something else in the sequence. Also reports
        /// what the register itself reads back — a Function 0 read, so it is
        /// unaffected by the Function 1 padding under investigation.
        pub fn remeasureAfterDelayWrite(self: *Self, val: u8) struct { readback: u8, slots: [5]u32 } {
            self.bus.writeReg8(.bus, gspi.reg.resp_delay_f1, val) catch {};
            const rb = self.bus.readReg8(.bus, gspi.reg.resp_delay_f1) catch 0xFF;
            return .{ .readback = rb, .slots = self.measureRespDelay() };
        }

        /// Read chipcommon's chip-ID register — a read-only, known-constant
        /// backplane location. The instrument for "is windowed access still
        /// working at this point in the sequence".
        pub fn probeBackplane(self: *Self) !u32 {
            return self.bus.readBackplaneReg32(chipcommon_base);
        }

        /// How many of `tries` reads of chipcommon return the true chip ID.
        ///
        /// **Do not measure this by over-clocking a dump.** An earlier version
        /// requested eight words of response for a command whose length field
        /// said four bytes, and never saw a good read in twenty attempts while
        /// ordinary reads were succeeding intermittently in the same run — the
        /// instrument was corrupting the transaction it was measuring. Vary
        /// nothing but the repeat count.
        pub fn backplaneReadSuccessRate(self: *Self, tries: u32) !u32 {
            var good: u32 = 0;
            for (0..tries) |_| {
                const v = self.probeBackplane() catch 0;
                if (v == chipcommon_chip_id) good += 1;
            }
            return good;
        }

        /// A core's wrapper status: `AI_IOCTRL` in the low byte, `AI_RESETCTRL`
        /// in the next. `resetctrl & 1` set means the core is still held in
        /// reset — and a core in reset accepts writes to the RAM it owns and
        /// discards them.
        pub fn coreStatus(self: *Self, core: Core) !u32 {
            const base = coreAddress(core);
            const ioctrl = try self.bus.readBackplaneReg8(base + ai_ioctrl_offset);
            const resetctrl = try self.bus.readBackplaneReg8(base + ai_resetctrl_offset);
            return (@as(u32, resetctrl) << 8) | ioctrl;
        }

        /// The SRAM_3 remap disable, on its own so it can be bisected.
        pub fn disableSram3Remap(self: *Self) !void {
            try self.bus.writeBackplaneReg32(socsram_bankx_index, 0x3);
            try self.bus.writeBackplaneReg32(socsram_bankx_pda, 0);
        }

        pub const BackplaneReport = struct {
            /// chipcommon's chip-ID register: read-only, known non-zero, and
            /// needs no prior write. Isolates the read path on its own.
            chip_id: u32,
            /// A single 32-bit write followed by a 32-bit read.
            word_ok: bool,
            /// Bytes wrong out of 64 in a block write followed by a block read.
            block_wrong: u32,
            /// The three window registers, read back from the chip. These are
            /// direct F1 registers and are known to work, so they say whether
            /// the window is where we think it is.
            window_readback: u32,
            /// SPI_STATUS_REGISTER sampled straight after a windowed read.
            status: u32,
            /// chipcommon's chip ID assembled from four **1-byte** windowed
            /// reads instead of one 4-byte read. The ALP poll proves 1-byte F1
            /// reads work, so if this differs from `chip_id` the fault is the
            /// transfer size, not the window.
            chip_id_by_bytes: u32,
        };

        pub fn backplaneSelfTest(self: *Self) !BackplaneReport {
            // 1. Read-only, so a failure here is unambiguously the read path.
            const chip_id = try self.bus.readBackplaneReg32(chipcommon_base);
            const status = try self.bus.readReg32(.bus, gspi.reg.status);

            // Did the window actually move? These are direct F1 registers, which
            // respond correctly, so they settle "stale window" versus "refused
            // read" — two faults that look identical from the data alone.
            var by_bytes: u32 = 0;
            for (0..4) |i| {
                const b = try self.bus.readBackplaneReg8(chipcommon_base + @as(u32, @intCast(i)));
                by_bytes |= @as(u32, b) << @intCast(i * 8);
            }

            const w_lo = try self.bus.readReg8(.backplane, 0x1000A);
            const w_mid = try self.bus.readReg8(.backplane, 0x1000B);
            const w_hi = try self.bus.readReg8(.backplane, 0x1000C);
            const window_readback = (@as(u32, w_hi) << 16) | (@as(u32, w_mid) << 8) | w_lo;

            // 2. One word, to separate small access from block handling.
            const sentinel: u32 = 0xA5A5_1234;
            try self.bus.writeBackplaneReg32(0x0000_0000, sentinel);
            const back = try self.bus.readBackplaneReg32(0x0000_0000);

            // 3. A full 64-byte block through the chunked path the firmware
            //    download actually uses.
            const pattern: [64]u8 = @bitCast(@as([8][8]u8, @splat(.{ 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67 })));
            var readback: [pattern.len]u8 = @splat(0);
            try self.bus.writeBackplaneChunked(0x0000_0040, &pattern);
            try self.bus.readBackplaneChunked(0x0000_0040, &readback);

            var wrong: u32 = 0;
            for (pattern, readback) |a, b| {
                if (a != b) wrong += 1;
            }

            return .{
                .chip_id = chip_id,
                .word_ok = back == sentinel,
                .block_wrong = wrong,
                .window_readback = window_readback,
                .status = status,
                .chip_id_by_bytes = by_bytes,
            };
        }

        /// Read back the first bytes of the downloaded microcode and compare.
        ///
        /// A download that silently drops or misplaces blocks produces a core
        /// that never raises its HT clock, which is indistinguishable from a
        /// wrong reset sequence. This tells the two apart.
        pub fn verifyFirmware(self: *Self, microcode: []const u8, sample: []u8) !u32 {
            const n = @min(sample.len, microcode.len);
            try self.bus.readBackplaneChunked(0x0000_0000, sample[0..n]);

            var wrong: u32 = 0;
            for (microcode[0..n], sample[0..n]) |a, b| {
                if (a != b) wrong += 1;
            }
            return wrong;
        }

        /// Download microcode and NVRAM, and bring the WLAN ARM core up.
        ///
        /// The predecessor wrote `0x00000003` to `0x18002000` and called that a
        /// core reset. `0x18002000` is `SDIO_BASE_ADDRESS`, not a core control
        /// register — the real ones live behind the wrapper offset at
        /// `0x18103000` and `0x18104000`, and the sequence needs SOCRAM reset
        /// and the SRAM_3 remap disable before any firmware is written.
        pub fn loadFirmware(self: *Self, microcode: []const u8, nvram: []const u8) !void {
            if (!self.bus_ready) try self.busInit();

            try self.requestAlp();
            try self.prepareCores();

            // Microcode at the bottom of RAM.
            try self.bus.writeBackplaneChunked(0x0000_0000, microcode);

            // NVRAM just below the top word, with a length footer above it: the
            // word count in the low half, its complement in the high half, so
            // the firmware can tell a real footer from uninitialised RAM.
            // 742 -> 744, which puts the NVRAM at 0x7FD14 and makes the footer
            // word count 186. Both match the canonical driver's trace exactly;
            // padding to 64 instead put it at 0x7FCFC with a count of 192.
            const nvram_len = std.mem.alignForward(usize, nvram.len, sdpcm_align);
            const nvram_addr: u32 = ram_size - 4 - @as(u32, @intCast(nvram_len));
            try self.bus.writeBackplaneChunked(nvram_addr, nvram);

            const words: u32 = @intCast(nvram_len / 4);
            const footer: u32 = ((~words & 0xFFFF) << 16) | (words & 0xFFFF);
            try self.bus.writeBackplaneReg32(ram_size - 4, footer);

            // Release the ARM core and wait for it to raise its HT clock —
            // about 29 ms in practice.
            try self.resetCore(.wlan_arm, false);

            var attempts: u32 = 1000;
            while (attempts > 0) : (attempts -= 1) {
                const reg = self.bus.readReg8(.backplane, sdio_chip_clock_csr) catch 0;
                if ((reg & sbsdio_ht_avail) != 0) break;
                self.bus.delayMs(1);
            } else return error.HtClockTimeout;

            try self.bus.writeBackplaneReg32(sdio_int_host_mask, i_hmb_sw_mask);
            try self.bus.writeReg8(.backplane, sdio_function2_watermark, spi_f2_watermark);

            // **The gate the predecessor skipped entirely.** Function 2 is the
            // WLAN engine; sending it an SDPCM frame before it says it is ready
            // is a dropped packet with no error anywhere.
            try self.waitF2Ready();

            self.firmware_loaded = true;
        }

        /// Poll `SPI_STATUS_REGISTER` for `STATUS_F2_RX_READY`.
        pub fn waitF2Ready(self: *Self) !void {
            var attempts: u32 = 1000;
            while (attempts > 0) : (attempts -= 1) {
                const reg = self.bus.readReg32(.bus, gspi.reg.status) catch 0;
                if ((reg & gspi.status_f2_rx_ready) != 0) return;
                self.bus.delayMs(1);
            }
            return error.F2NotReady;
        }

        // -- SDPCM ----------------------------------------------------------

        /// Send one SDPCM control frame carrying a CDC IOCTL.
        ///
        /// Two things here are what the predecessor was missing, and either one
        /// alone is enough for the chip to drop the packet in silence:
        ///
        ///  1. **The write is padded to a 64-byte boundary.** The length fields
        ///     inside the header still describe the true length; it is the
        ///     transfer that is rounded up.
        ///  2. **A bus data credit is required before transmitting.** Credits
        ///     roll over as 8-bit values and the stall condition is equality,
        ///     not exhaustion.
        fn sendControlFrame(self: *Self, cmd: IoctlCmd, kind: u32, payload: []const u8) !void {
            if (!self.firmware_loaded) return error.FirmwareNotLoaded;

            const body_len = cdc_header_len + payload.len;
            const size = sdpcm_header_len + body_len;
            if (size > max_control_frame) return error.PayloadTooLarge;

            try self.awaitCredit();

            var frame: [max_control_frame]u8 = @splat(0);

            // SDPCM header (12 bytes).
            std.mem.writeInt(u16, frame[0..2], @intCast(size), .little);
            std.mem.writeInt(u16, frame[2..4], @intCast(~size & 0xFFFF), .little);
            frame[4] = self.sdpcm_seq;
            frame[5] = control_header;
            frame[6] = 0; // next_length
            frame[7] = sdpcm_header_len; // header_length
            frame[8] = 0; // wireless_flow_control
            frame[9] = 0; // bus_data_credit
            frame[10] = 0;
            frame[11] = 0;

            // CDC IOCTL header (16 bytes).
            self.ioctl_id +%= 1;
            self.pending_ioctl_id = self.ioctl_id;
            self.response = .{};
            const flags: u32 = (@as(u32, self.ioctl_id) << cdcf_ioc_id_shift) |
                kind |
                (0 << cdcf_ioc_if_shift); // interface 0 = STA
            std.mem.writeInt(u32, frame[12..16], @backingInt(cmd), .little);
            std.mem.writeInt(u32, frame[16..20], @intCast(payload.len), .little);
            std.mem.writeInt(u32, frame[20..24], flags, .little);
            std.mem.writeInt(u32, frame[24..28], 0, .little); // status

            @memcpy(frame[28..][0..payload.len], payload);

            self.sdpcm_seq +%= 1;
            self.ioctl_count += 1;

            const padded = std.mem.alignForward(usize, size, sdpcm_align);
            try self.bus.writeBytes(.wlan_or_bt, 0, frame[0..padded]);
        }

        /// Drain one waiting Function 2 packet, updating the transmit credit.
        ///
        /// Returns true if a packet was read. Takes the credit byte, then
        /// classifies by channel: a control response is matched to the
        /// outstanding request by id and its CDC status kept (channel 0), an
        /// async event is folded into the join state (channel 1), an 802.3
        /// frame is latched for `pollEthFrame` (channel 2), and anything else
        /// is counted in the census.
        ///
        /// **Every one of those three paths runs from inside the others' waits.**
        /// `awaitResponse`, `awaitCredit` and `awaitJoin` all call this, so a
        /// frame or an event can arrive while the driver is nominally doing
        /// something else. That is why nothing here returns its packet to the
        /// caller — the caller is usually not the one that wanted it.
        pub fn pollDevice(self: *Self) !bool {
            const status = try self.bus.readReg32(.bus, gspi.reg.status);
            if ((status & gspi.status_f2_pkt_available) == 0) return false;

            const len = gspi.packetLength(status);
            if (len == 0) return false;
            if (len > max_rx_packet) {
                // Recorded rather than silently skipped: a response larger than
                // the buffer is a blind spot, and a census that hid it would be
                // one more instrument that measures nothing.
                self.census.oversize += 1;
                self.census.largest_seen = @max(self.census.largest_seen, len);
                return false;
            }
            self.census.largest_seen = @max(self.census.largest_seen, len);

            const buf = &self.rx_buf;
            try self.bus.readBytes(.wlan_or_bt, 0, buf[0..len]);

            // The SDPCM header opens with a length and its complement. Both
            // zero means no packet; a failed complement means a torn read.
            const size = std.mem.readInt(u16, buf[0..2], .little);
            const size_com = std.mem.readInt(u16, buf[2..4], .little);
            if (size == 0 and size_com == 0) return false;
            if ((size ^ size_com) != 0xFFFF) return error.SdpcmHeaderMismatch;
            if (len < sdpcm_header_len) return false;

            // Credits roll over as 8-bit values. The reference only accepts a
            // forward jump of 20 or less, which rejects a garbage byte without
            // needing to know what the real value should be.
            // Mirrors sdpcm_process_rx_packet's order: flow control, then
            // credit for channels below 3, then classify.
            self.wlan_flow_control = buf[8];

            const channel: u8 = buf[5] & 0x0F;
            if (channel < 3) {
                const advertised = buf[9];
                const delta = advertised -% self.sdpcm_credit;
                if (delta <= 20) self.sdpcm_credit = advertised;
            }

            self.census.total += 1;

            // A packet whose size is exactly the header is a flow-control
            // update carrying no payload. The reference ignores these.
            if (size == sdpcm_header_len) {
                self.census.flow_control_only += 1;
                return true;
            }

            switch (channel) {
                control_header => {
                    self.census.control += 1;
                    const hl: usize = buf[7];
                    if (size >= sdpcm_header_len + cdc_header_len and hl + cdc_header_len <= len) {
                        const cmd = std.mem.readInt(u32, buf[hl..][0..4], .little);
                        const plen = std.mem.readInt(u32, buf[hl + 4 ..][0..4], .little);
                        const flags = std.mem.readInt(u32, buf[hl + 8 ..][0..4], .little);
                        const cstatus = std.mem.readInt(u32, buf[hl + 12 ..][0..4], .little);
                        const id: u16 = @truncate(flags >> cdcf_ioc_id_shift);

                        if (!self.census.saw_control) {
                            self.census.saw_control = true;
                            self.census.ctrl_cmd = cmd;
                            self.census.ctrl_len = plen;
                            self.census.ctrl_flags = flags;
                            self.census.ctrl_status = cstatus;
                            self.census.ctrl_id = id;
                            self.census.ctrl_id_expected = self.pending_ioctl_id;
                        }

                        // Only the response to the request still outstanding is
                        // ours. The reference drops any other id rather than
                        // guessing, and so does this: a stale response matched
                        // to a new request reports someone else's status.
                        if (id == self.pending_ioctl_id) {
                            const body = hl + cdc_header_len;
                            const n = @min(@as(usize, @intCast(plen)), @min(max_ioctl_payload, len -| body));
                            self.response = .{
                                .ready = true,
                                .status = cstatus,
                                .cmd = cmd,
                                .len = n,
                            };
                            @memcpy(self.response.payload[0..n], buf[body..][0..n]);
                        } else {
                            self.census.id_mismatch += 1;
                        }
                    } else {
                        self.census.control_too_small += 1;
                    }
                },
                asyncevent_header => {
                    self.census.event += 1;
                    if (parseAsyncEvent(buf[0..len])) |ev| {
                        self.last_event = ev;
                        self.census.event_parsed += 1;
                        self.processEvent(ev);
                    } else |_| {
                        // Counted, not silenced. The reference logs these too:
                        // packets on this channel that are not events do arrive
                        // during startup, and it calls that expected.
                        self.census.event_unparsed += 1;
                    }
                },
                data_header => {
                    self.census.data += 1;
                    self.latchEthFrame(buf[0..len], size);
                },
                else => self.census.other += 1,
            }

            return true;
        }

        /// Pull an async event out of an SDPCM packet — rung 10.
        ///
        /// The event is buried four layers down: SDPCM header, then a BDC
        /// header whose `data_offset` is **in words**, then an Ethernet frame
        /// of type `0x886C` carrying the Broadcom OUI, and only then the event
        /// itself, 24 bytes into the Ethernet payload.
        ///
        /// **Its fields are big-endian**, alone among everything else in this
        /// driver. Reading them little-endian yields an `event_type` of
        /// 0x10000000 for `CYW43_EV_LINK`, which matches nothing and looks like
        /// an unknown event rather than a byte-order mistake.
        fn parseAsyncEvent(pkt: []const u8) !AsyncEvent {
            if (pkt.len <= sdpcm_header_len + bdc_header_len) return error.EventTooSmall;

            const hl: usize = pkt[7];
            if (hl + bdc_header_len > pkt.len) return error.EventTooSmall;

            // BDC `data_offset` counts 32-bit words, not bytes.
            const data_offset: usize = @as(usize, pkt[hl + 3]) << 2;
            const payload = hl + bdc_header_len + data_offset;
            if (payload + 24 + async_event_len > pkt.len) return error.EventTooSmall;

            const eth = pkt[payload..];

            // The Ethernet type must be 0x886C. Packets that are not events do
            // arrive on this channel during startup — the reference sees them
            // too and treats them as expected.
            if (eth[12] != 0x88 or eth[13] != 0x6C) return error.NotAnEvent;
            // Broadcom's OUI, 00:10:18.
            if (eth[19] != 0x00 or eth[20] != 0x10 or eth[21] != 0x18) return error.NotBroadcomOui;

            const ev = eth[24..];
            return .{
                .flags = std.mem.readInt(u16, ev[2..4], .big),
                .event_type = std.mem.readInt(u32, ev[4..8], .big),
                .status = std.mem.readInt(u32, ev[8..12], .big),
                .reason = std.mem.readInt(u32, ev[12..16], .big),
                .interface = ev[46],
            };
        }

        /// Take the 802.3 frame out of a data-channel packet and latch it —
        /// rung 14.
        ///
        /// The frame sits behind the same two headers an event does, and is
        /// found the same way: BDC header at `header_length` (which is 14 on
        /// this channel, not 12), then `data_offset` **32-bit words** of
        /// whatever the firmware put in front of the payload.
        ///
        /// **The SDPCM `size` field bounds the frame, not the gSPI transfer
        /// length.** The transfer is padded and the padding is junk; measuring
        /// the frame by `pkt.len` would hand the caller those bytes as though
        /// they were part of it.
        /// How many frames are waiting.
        pub fn rxQueued(self: *Self) usize {
            return (self.rx_ring_tail +% rx_ring_slots -% self.rx_ring_head) % rx_ring_slots;
        }

        fn latchEthFrame(self: *Self, pkt: []const u8, size: u16) void {
            if (size <= sdpcm_header_len + bdc_header_len) {
                self.census.data_too_small += 1;
                return;
            }

            const hl: usize = pkt[7];
            const bdc = hl;
            if (bdc + bdc_header_len > pkt.len) {
                self.census.data_too_small += 1;
                return;
            }

            const start = bdc + bdc_header_len + (@as(usize, pkt[bdc + 3]) << 2);
            if (start >= size or start > pkt.len) {
                self.census.data_too_small += 1;
                return;
            }
            const frame_len = @as(usize, size) - start;
            if (frame_len > eth_max_frame or start + frame_len > pkt.len) {
                self.census.data_too_small += 1;
                return;
            }

            // **A full ring drops the newest, not the oldest.** The caller
            // asked for the older frame first, and silently replacing it is how
            // a receive path starts reordering traffic — which is far harder to
            // diagnose than a loss that is counted.
            const next = (self.rx_ring_tail + 1) % rx_ring_slots;
            if (next == self.rx_ring_head) {
                self.census.data_dropped += 1;
                return;
            }

            @memcpy(self.rx_ring[self.rx_ring_tail][0..frame_len], pkt[start..][0..frame_len]);
            self.rx_ring_len[self.rx_ring_tail] = frame_len;
            self.rx_ring_tail = next;
            self.rx_eth_itf = pkt[bdc + 2];

            const queued: u32 = @intCast(self.rxQueued());
            if (queued > self.rx_high_water) self.rx_high_water = queued;
        }

        /// Fold one event into the join state, as `cyw43_cb_process_async_event`
        /// does. Only the events a join depends on are acted on; the rest are
        /// recorded by the census and dropped.
        fn processEvent(self: *Self, ev: AsyncEvent) void {
            switch (@as(EventType, @fromBackingInt(@intCast(ev.event_type)))) {
                .set_ssid => {
                    if (ev.status == 0) {
                        // The SSID was accepted. Not a completed join.
                    } else if (ev.status == 3 and ev.reason == 0) {
                        self.join_state = .{ .kind = JoinState.kind_nonet };
                    } else {
                        self.join_state = .{ .kind = JoinState.kind_fail };
                    }
                },
                .auth => {
                    if (ev.status == 0) {
                        // A good auth after a bad one puts the join back on
                        // track rather than leaving it stuck failed.
                        if (self.join_state.kind == JoinState.kind_badauth) {
                            self.join_state.kind = JoinState.kind_active;
                        }
                        self.join_state.auth = true;
                    } else if (ev.status == 2 or ev.status == 5 or ev.status == 6) {
                        // Timeout, no ack, or an unsolicited auth packet: the
                        // firmware keeps trying, so none of these fails the join.
                        // Ignored, as the reference driver does from v2.0.0.
                    } else {
                        self.join_state = .{ .kind = JoinState.kind_badauth };
                    }
                },
                .link => {
                    if (ev.status == 0) {
                        if (ev.flags & 1 != 0) {
                            self.join_state.link = true;
                        } else {
                            self.join_state.link = false;
                        }
                    }
                },
                .psk_sup => {
                    if (ev.status == sup_keyed) {
                        self.join_state.keyed = true;
                    } else if ((ev.status == 4 or ev.status == 8 or ev.status == 10) and ev.reason == 15) {
                        // Key-exchange timeout. The reference retries the join;
                        // we record it and let the caller decide.
                        self.census.key_timeout += 1;
                    } else {
                        self.join_state = .{ .kind = JoinState.kind_badauth };
                    }
                },
                .disassoc => {
                    self.join_state = .{};
                    self.link_status = .down;
                },
                else => {},
            }

            self.link_status = if (self.join_state.complete())
                .joined
            else switch (self.join_state.kind) {
                JoinState.kind_active => .joining,
                JoinState.kind_fail => .fail,
                JoinState.kind_nonet => .no_net,
                JoinState.kind_badauth => .bad_auth,
                else => .down,
            };
        }

        /// The chip's answer to an IOCTL.
        ///
        /// `status` is the CDC header's status word and is the **only** thing
        /// that says whether the command was accepted. A `gpioout` that the
        /// firmware rejects still produces a response; it just carries a
        /// non-zero status, and a driver that does not read it cannot tell the
        /// two apart. Measured `status=0` for the LED write.
        pub const IoctlResponse = struct {
            ready: bool = false,
            status: u32 = 0,
            cmd: u32 = 0,
            len: usize = 0,
            payload: [max_ioctl_payload]u8 = @splat(0),
        };

        pub const max_ioctl_payload: usize = 128;

        pub const RxCensus = struct {
            total: u32 = 0,
            flow_control_only: u32 = 0,
            control: u32 = 0,
            event: u32 = 0,
            /// Events that parsed as events, and those that did not. Packets on
            /// channel 1 that are not `0x886C` frames arrive during startup and
            /// the reference calls that expected — so they are counted apart.
            event_parsed: u32 = 0,
            event_unparsed: u32 = 0,
            /// `psk_sup` key-exchange timeouts. The reference retries the join
            /// on these; a non-zero count is the edge-of-cell symptom.
            key_timeout: u32 = 0,
            /// Data-channel packets seen, those whose BDC header did not
            /// describe a frame, and those dropped because the receive latch
            /// was still full. **The last of these is the one that matters**: a
            /// driver with a single slot loses frames under load, and a count
            /// is the difference between knowing that and guessing it.
            data: u32 = 0,
            data_too_small: u32 = 0,
            data_dropped: u32 = 0,
            other: u32 = 0,
            oversize: u32 = 0,
            control_too_small: u32 = 0,
            id_mismatch: u32 = 0,
            largest_seen: u32 = 0,
            /// The first control packet's CDC header, verbatim.
            saw_control: bool = false,
            ctrl_cmd: u32 = 0,
            ctrl_len: u32 = 0,
            ctrl_flags: u32 = 0,
            ctrl_status: u32 = 0,
            ctrl_id: u16 = 0,
            ctrl_id_expected: u16 = 0,
        };

        /// Wait until the chip has issued a transmit credit.
        ///
        /// Credits are 8-bit and roll over, so the stall condition is the two
        /// being **equal**, not exhausted. The chip advertises credit in the
        /// header of packets it sends us, so the only way to get one is to read
        /// what it has waiting — which is why this polls rather than sleeping.
        ///
        /// **The wireless flow-control byte stalls transmission too**, and this
        /// ignored it until rung 14. `cyw43_sdpcm_send_common` waits on
        /// `self->wlan_flow_control || credit == sequence`, the two together. It
        /// never mattered on the control path, where the frames are few and the
        /// chip has no reason to push back; a data path that ignored it would
        /// transmit into a full queue.
        fn awaitCredit(self: *Self) !void {
            var attempts: u32 = 200;
            while (attempts > 0) : (attempts -= 1) {
                if (self.wlan_flow_control == 0 and self.sdpcm_credit != self.sdpcm_seq) return;
                _ = self.pollDevice() catch false;
                self.bus.delayMs(1);
            }
            return error.CreditTimeout;
        }

        /// Set one of the CYW43's own GPIOs. GPIO 0 is the onboard LED.
        pub fn setGpio(self: *Self, gpio_n: u4, val: bool) !void {
            // "gpioout\0" then two little-endian words: a mask and a value.
            var payload: [16]u8 = @splat(0);
            @memcpy(payload[0..8], "gpioout\x00");
            const mask: u32 = @as(u32, 1) << gpio_n;
            std.mem.writeInt(u32, payload[8..12], mask, .little);
            std.mem.writeInt(u32, payload[12..16], if (val) mask else 0, .little);

            // Through `sendIoctl`, so the chip's status is checked rather than
            // the write being fire-and-forget. Measured status 0 for this one.
            try self.sendIoctl(.set_var, &payload);
        }

        /// Convenience: the onboard LED is CYW43 GPIO 0.
        pub fn setLed(self: *Self, on: bool) !void {
            return self.setGpio(0, on);
        }

        // -- CLM --------------------------------------------------------------

        /// Download the CLM blob — rung 11.
        ///
        /// The CLM (Country Locale Matrix) carries the per-region channel and
        /// power limits. **The radio will not transmit legally without it**, so
        /// this runs before any association, and its failure is an error rather
        /// than a warning: the reference only `CYW43_WARN`s here, which on a
        /// driver with this package's history is exactly the shape of a step
        /// that looks like it worked.
        ///
        /// Sent as `clmload` chunks, then confirmed with a `clmload_status`
        /// read. The status GET is the whole point — the chunk IOCTLs return
        /// CDC status 0 for a blob the handler went on to reject.
        pub fn loadClm(self: *Self, clm: []const u8) !void {
            var buf: [clm_hdr_len + clm_chunk_len + clm_payload_align]u8 = @splat(0);

            var off: usize = 0;
            while (off < clm.len) {
                const len = @min(clm_chunk_len, clm.len - off);

                // `DLOAD_HANDLER_VER` is always set; BEGIN and END mark the
                // ends of the blob and are both set when it fits in one chunk,
                // which ours does.
                var flag: u16 = dload_handler_ver;
                if (off == 0) flag |= dload_flag_begin;
                if (off + len >= clm.len) flag |= dload_flag_end;

                @memcpy(buf[0..8], "clmload\x00");
                std.mem.writeInt(u16, buf[8..10], flag, .little);
                std.mem.writeInt(u16, buf[10..12], dload_type_clm, .little);
                std.mem.writeInt(u32, buf[12..16], @intCast(len), .little);
                std.mem.writeInt(u32, buf[16..20], 0, .little); // CRC, unchecked
                @memcpy(buf[clm_hdr_len..][0..len], clm[off..][0..len]);

                // Padded to 8 in the *declared* length, as the reference does.
                // The bytes between the blob's end and there are junk to the
                // chip; they are zeroed here so a frame dump is readable.
                const declared = std.mem.alignForward(usize, clm_hdr_len + len, clm_payload_align);
                @memset(buf[clm_hdr_len + len .. declared], 0);

                try self.sendIoctl(.set_var, buf[0..declared]);
                off += len;
            }

            // "clmload_status" + NUL is 15 bytes; the reference asks for 19, the
            // extra four being room for the u32 the chip writes back.
            var status_req: [19]u8 = @splat(0);
            @memcpy(status_req[0..15], "clmload_status\x00");
            try self.doIoctl(.get_var, sdpcm_get, &status_req);

            if (self.response.len < 4) return error.ClmLoadFailed;
            const result = std.mem.readInt(u32, self.response.payload[0..4], .little);
            self.clm_status = result;
            if (result != 0) return error.ClmLoadFailed;
        }

        /// `loadClm` over the blob embedded in this package.
        pub fn loadClmDefault(self: *Self) !void {
            return self.loadClm(fw_clm);
        }

        // -- IOCTLs -----------------------------------------------------------

        /// Send an IOCTL and wait for the chip's answer.
        ///
        /// **Returns an error unless the chip reported success.** The CDC
        /// status word is the only thing that distinguishes an accepted command
        /// from a rejected one — a rejected IOCTL still produces a response —
        /// so a version of this that ignored it would be the old stub with
        /// extra steps.
        ///
        /// The response payload is left in `self.response` for callers that
        /// want it (a `get_var` reader will).
        pub fn sendIoctl(self: *Self, cmd: IoctlCmd, payload: []const u8) !void {
            return self.doIoctl(cmd, sdpcm_set, payload);
        }

        /// `sendIoctl`, with the SDPCM kind exposed: `sdpcm_set` or `sdpcm_get`.
        pub fn doIoctl(self: *Self, cmd: IoctlCmd, kind: u32, payload: []const u8) !void {
            if (try self.ioctlStatus(cmd, kind, payload) != 0) return error.IoctlRejected;
        }

        /// Send an IOCTL and **return** the chip's CDC status instead of turning
        /// a non-zero one into an error.
        ///
        /// Not a way to ignore the status — the caller still has it, and the
        /// callers here record it. It exists because a working bring-up
        /// contains rejected commands (see `wifiOn`), so "any rejection is
        /// fatal" is a rule the chip does not actually follow. Still errors on
        /// a timeout: no answer at all is a different thing from a "no".
        pub fn ioctlStatus(self: *Self, cmd: IoctlCmd, kind: u32, payload: []const u8) !u32 {
            try self.sendControlFrame(cmd, kind, payload);
            try self.awaitResponse();
            return self.response.status;
        }

        /// Poll until the response to the outstanding request arrives.
        ///
        /// Bounded. A silent timeout here would put us back where the stub was:
        /// a command that appears to have worked because nothing contradicted
        /// it.
        fn awaitResponse(self: *Self) !void {
            var attempts: u32 = 200;
            while (attempts > 0) : (attempts -= 1) {
                if (self.response.ready) return;
                _ = self.pollDevice() catch false;
                self.bus.delayMs(1);
            }
            return error.IoctlTimeout;
        }

        // -- IOVARs -----------------------------------------------------------

        /// Largest IOVAR name plus body this builds. `bsscfg:event_msgs` is the
        /// big one at 41 bytes.
        const max_iovar: usize = 64;

        /// `name\0` followed by one little-endian u32.
        ///
        /// **Returns the chip's status rather than erroring on it.** The
        /// reference's `cyw43_write_iovar_u32` returns `void` and discards it,
        /// and some of these writes are rejected in a working bring-up — see
        /// `wifiOn`. Returning it lets the caller decide, which is the whole
        /// difference between knowing and not knowing.
        fn iovarU32(self: *Self, name: []const u8, val: u32) !u32 {
            var buf: [max_iovar]u8 = @splat(0);
            const n = name.len + 1;
            @memcpy(buf[0..name.len], name);
            std.mem.writeInt(u32, buf[n..][0..4], val, .little);
            return self.ioctlStatus(.set_var, sdpcm_set, buf[0 .. n + 4]);
        }

        /// `name\0` followed by two little-endian u32s. This is the shape every
        /// `bsscfg:` IOVAR takes: the first word is the bsscfg index.
        fn iovarU32U32(self: *Self, name: []const u8, a: u32, b: u32) !u32 {
            var buf: [max_iovar]u8 = @splat(0);
            const n = name.len + 1;
            @memcpy(buf[0..name.len], name);
            std.mem.writeInt(u32, buf[n..][0..4], a, .little);
            std.mem.writeInt(u32, buf[n + 4 ..][0..4], b, .little);
            return self.ioctlStatus(.set_var, sdpcm_set, buf[0 .. n + 8]);
        }

        /// An IOCTL whose whole payload is one little-endian u32.
        fn ioctlU32(self: *Self, cmd: IoctlCmd, val: u32) !u32 {
            var buf: [4]u8 = undefined;
            std.mem.writeInt(u32, &buf, val, .little);
            return self.ioctlStatus(cmd, sdpcm_set, &buf);
        }

        /// Read an IOVAR. `room` is the total request length, which is how the
        /// chip is told how much space it has to answer in — so it is the
        /// *answer's* size that matters here, not the name's.
        fn readIovar(self: *Self, name: []const u8, room: usize) !u32 {
            var buf: [max_iovar]u8 = @splat(0);
            @memcpy(buf[0..name.len], name);
            return self.ioctlStatus(.get_var, sdpcm_get, buf[0..room]);
        }

        // -- wifi_on ----------------------------------------------------------

        /// Bring the Wi-Fi side up — rung 12's first half.
        ///
        /// **The order and the payload lengths come from a trace of the SDK's own
        /// join, not from reading `cyw43_ll.c`.** Reading it produced a shorter list
        /// than this: the power-management block, `WLC_SET_GMODE`, `WLC_SET_BAND`
        /// and the repeats of `bus:txglom`, `apsta` and `ampdu_ba_wsize` were all
        /// missing, and a `clmver` that a Release build never issues was present.
        ///
        /// Spans three reference functions — the tail of `cyw43_ll_bus_init`,
        /// then `cyw43_ll_wifi_on`, then `cyw43_ll_wifi_pm` — which is why some
        /// IOVARs are written twice. They are kept in wire order rather than
        /// tidied, because wire order is the thing that was measured.
        pub fn wifiOn(self: *Self, country: Country) !void {
            const s = &self.wifi_on_status;

            // -- tail of cyw43_ll_bus_init --
            s[0] = try self.iovarU32("bus:txglom", 0); // tx glomming off
            s[1] = try self.iovarU32("apsta", 1);

            // Twice, as the reference does: once to test whether the OTP holds
            // a MAC, once to read it. Kept as a pair so the trace matches.
            s[2] = try self.readIovar("cur_etheraddr\x00", 20);
            s[3] = try self.readIovar("cur_etheraddr\x00", 20);
            if (self.response.len >= 6) @memcpy(self.mac[0..6], self.response.payload[0..6]);

            // -- cyw43_ll_wifi_on --
            //
            // The country IOVAR takes the code twice, with a revision between:
            // -1 asks for the default revision. Takes about 32 ms.
            {
                var buf: [20]u8 = @splat(0);
                @memcpy(buf[0..8], "country\x00");
                const c = @backingInt(country);
                std.mem.writeInt(u32, buf[8..12], c, .little);
                std.mem.writeInt(u32, buf[12..16], 0xFFFF_FFFF, .little); // rev -1
                std.mem.writeInt(u32, buf[16..20], c, .little);
                s[4] = try self.ioctlStatus(.set_var, sdpcm_set, &buf);
            }
            self.bus.delayMs(50);

            // No `clmver` GET here. The reference's is inside `#ifndef NDEBUG`,
            // and the Release build that produced the trace does not issue it.

            s[5] = try self.ioctlU32(.set_antdiv, 0); // chip antenna

            s[6] = try self.iovarU32("bus:txglom", 0); // again
            s[7] = try self.iovarU32("apsta", 1); // again
            s[8] = try self.iovarU32("ampdu_ba_wsize", 8);
            s[9] = try self.iovarU32("ampdu_mpdu", 4);
            s[10] = try self.iovarU32("ampdu_rx_factor", 0);

            // The reference waits here until 150 ms have passed since bus
            // init started. **We have no clock, and do not need one**: the
            // firmware download above takes seconds, so the gate is already
            // satisfied by the time control reaches here. Faking a timestamp to
            // reproduce a wait that cannot fire would be an instrument that
            // measures nothing.
            s[11] = try self.setEventMask();
            self.bus.delayMs(50);

            s[12] = try self.ioctlStatus(.up, sdpcm_set, &.{});
            self.bus.delayMs(50);

            // -- cyw43_ll_wifi_pm, at CYW43_PERFORMANCE_PM --
            //
            // pm2_sleep_ret is in ms and is written as the clamped value times
            // ten: 200 becomes 2000.
            s[13] = try self.iovarU32("pm2_sleep_ret", 2000);
            s[14] = try self.iovarU32("bcn_li_bcn", 1);
            s[15] = try self.iovarU32("bcn_li_dtim", 1);
            s[16] = try self.iovarU32("assoc_listen", 10);
            s[17] = try self.ioctlU32(.set_pm, pm2_powersave_mode);
            s[18] = try self.ioctlU32(.set_gmode, 1); // GMODE_AUTO
            s[19] = try self.ioctlU32(.set_band, 0); // any

            // **Only the steps that matter are allowed to fail the function.**
            // The chip rejects some of these in a working bring-up and the
            // reference never notices, because every one of its helpers here
            // returns `void`. Erroring on all of them stops the sequence dead;
            // ignoring all of them is how a driver claims to work. So: record
            // everything, insist on the three whose failure would make the rest
            // meaningless, and leave the rest visible in `wifi_on_status`.
            if (s[4] != 0) return error.CountryRejected;
            if (s[11] != 0) return error.EventMaskRejected;
            if (s[12] != 0) return error.WifiUpRejected;

            self.wifi_on = true;
        }

        /// The async-event mask — 19 bytes, all events on but six.
        ///
        /// **Nothing arrives on channel 1 that is not enabled here**, so this is
        /// what rung 10 depends on. The six cleared are the noisy ones the
        /// reference drops; leaving them on floods the control path.
        fn setEventMask(self: *Self) !u32 {
            const name = "bsscfg:event_msgs";
            var buf: [18 + 4 + 19]u8 = @splat(0);
            @memcpy(buf[0..name.len], name);
            // buf[17] is the name's NUL; buf[18..22] is the bsscfg index, 0.
            @memset(buf[22..], 0xFF);

            const clear = [_]u8{
                19, // roam attempt occurred
                20, // tx fail
                40, // radio
                44, // probe request
                54, // interface change
                71, // probe response
            };
            for (clear) |ev| buf[22 + ev / 8] &= ~(@as(u8, 1) << @intCast(ev % 8));

            return self.ioctlStatus(.set_var, sdpcm_set, &buf);
        }

        // -- the join ---------------------------------------------------------

        /// Start associating with an AP — rung 12b.
        ///
        /// **Returns as soon as `WLC_SET_SSID` is accepted, which is not a
        /// join.** Association completes asynchronously: authentication,
        /// link-up and key exchange each arrive as a separate event, in no
        /// guaranteed order. Call `pollJoin` until it says otherwise. The
        /// predecessor of this function set `link_status = .joined` and
        /// returned, which is exactly the shape of the mistake being avoided.
        pub fn joinWifi(self: *Self, ssid: []const u8, passphrase: []const u8, auth: AuthType) !void {
            if (!self.wifi_on) return error.WifiNotOn;
            if (ssid.len == 0 or ssid.len > 32) return error.BadSsid;
            if (auth != .open and passphrase.len > wpa_max_password_len) return error.BadPassphrase;

            const s = &self.join_status;

            s[0] = try self.iovarU32("ampdu_ba_wsize", 8);
            s[1] = try self.ioctlU32(.set_wsec, auth.wsec());

            // Every `bsscfg:` IOVAR takes the bsscfg index first, then the
            // value — hence the pair of words.
            s[2] = try self.iovarU32U32("bsscfg:sup_wpa", 0, if (auth == .open) 0 else 1);
            // -1 means "whatever EAPOL version the AP uses".
            s[3] = try self.iovarU32U32("bsscfg:sup_wpa2_eapver", 0, 0xFFFF_FFFF);
            s[4] = try self.iovarU32U32("bsscfg:sup_wpa_tmo", 0, eapol_key_timeout);

            if (auth != .open) {
                // **The 2 ms wait is load-bearing.** The reference's comment
                // says the radio firmware is not ready to receive the PMK
                // without it and that the failure is intermittent — which is
                // the worst kind to inherit, so it is kept.
                self.bus.delayMs(2);

                // `wsec_pmk_t`: length, flags, then the key in a fixed 64-byte
                // field. Always sent full-length — 4 + 64 = 68.
                var pmk: [4 + wpa_max_password_len]u8 = @splat(0);
                std.mem.writeInt(u16, pmk[0..2], @intCast(passphrase.len), .little);
                std.mem.writeInt(u16, pmk[2..4], 1, .little);
                @memcpy(pmk[4..][0..passphrase.len], passphrase);
                s[5] = try self.ioctlStatus(.set_passphrase, sdpcm_set, &pmk);
            }

            s[6] = try self.ioctlU32(.set_infra, 1);
            s[7] = try self.ioctlU32(.set_auth, auth_type_open);
            s[8] = try self.iovarU32("mfp", switch (auth) {
                .wpa2_aes, .wpa2_mix, .wpa3_sae, .wpa3_wpa2 => mfp_capable,
                else => mfp_none,
            });
            s[9] = try self.ioctlU32(.set_wpa_auth, auth.wpaAuth());

            // The join itself: a length and a 32-byte SSID field.
            var ssid_buf: [36]u8 = @splat(0);
            std.mem.writeInt(u32, ssid_buf[0..4], @intCast(ssid.len), .little);
            @memcpy(ssid_buf[4..][0..ssid.len], ssid);

            self.join_state = .{ .kind = JoinState.kind_active };
            self.link_status = .joining;
            s[10] = try self.ioctlStatus(.set_ssid, sdpcm_set, &ssid_buf);
            if (s[10] != 0) {
                self.join_state = .{ .kind = JoinState.kind_fail };
                self.link_status = .fail;
                return error.JoinRejected;
            }
        }

        /// Drive the join forward. Returns the current link state.
        ///
        /// The events only arrive if something reads them, so this polls the
        /// device rather than sleeping — the same reason `awaitCredit` does.
        pub fn pollJoin(self: *Self) !LinkStatus {
            _ = self.pollDevice() catch false;
            return self.link_status;
        }

        /// Poll until the join settles or `timeout_ms` passes.
        pub fn awaitJoin(self: *Self, timeout_ms: u32) !LinkStatus {
            var left = timeout_ms;
            while (left > 0) : (left -= 1) {
                switch (try self.pollJoin()) {
                    .joining => {},
                    else => |st| return st,
                }
                self.bus.delayMs(1);
            }
            return error.JoinTimeout;
        }

        /// Disassociate. The chip reports the result through a `disassoc`
        /// event, as with the join.
        pub fn disconnectWifi(self: *Self) !void {
            if (!self.wifi_on) return error.WifiNotOn;
            _ = try self.ioctlStatus(.disassoc, sdpcm_set, &.{});
            self.join_state = .{};
            self.link_status = .down;
        }

        // -- the data path, rung 14 -----------------------------------------

        /// Send one 802.3 frame — rung 14.
        ///
        /// The frame goes out on channel 2 behind an SDPCM header and a BDC
        /// header, with **two bytes of padding between them that the control
        /// channel does not have** (`sdpcm_data_pad`). `header_length` says 14
        /// rather than 12 to describe that gap, and the SDPCM `size` counts it:
        /// 18 bytes of header for a frame the caller measures at 14 + payload.
        ///
        /// The predecessor wrote the bare frame to backplane address 0x2000
        /// with no header of any kind and no credit accounting. There is no
        /// interpretation under which the chip could have treated that as a
        /// packet, and nothing in the code said so.
        pub fn sendEthFrame(self: *Self, frame: []const u8) !void {
            return self.sendEthFrameItf(frame, 0);
        }

        /// `sendEthFrame`, with the interface number exposed. 0 is the station
        /// interface; the AP interface would be 1, and this driver never brings
        /// one up.
        pub fn sendEthFrameItf(self: *Self, frame: []const u8, itf: u8) !void {
            if (!self.firmware_loaded) return error.FirmwareNotLoaded;
            // A frame sent before the link is up is not an error the chip
            // reports — it is accepted, credited and discarded — so it is one
            // here. Nothing else in this driver would notice.
            if (self.link_status != .joined) return error.LinkDown;

            const head = sdpcm_header_len + sdpcm_data_pad + bdc_header_len;
            const size = head + frame.len;
            if (size > max_data_frame) return error.PayloadTooLarge;

            try self.awaitCredit();

            const buf = &self.tx_buf;
            @memset(buf[0..head], 0);

            std.mem.writeInt(u16, buf[0..2], @intCast(size), .little);
            std.mem.writeInt(u16, buf[2..4], @intCast(~size & 0xFFFF), .little);
            buf[4] = self.sdpcm_seq;
            buf[5] = data_header;
            buf[6] = 0; // next_length
            buf[7] = @intCast(sdpcm_header_len + sdpcm_data_pad); // header_length
            buf[8] = 0; // wireless_flow_control
            buf[9] = 0; // bus_data_credit

            // BDC header, after the two pad bytes.
            const bdc = sdpcm_header_len + sdpcm_data_pad;
            buf[bdc + 0] = bdc_flags_tx;
            buf[bdc + 1] = 0; // priority
            buf[bdc + 2] = itf; // flags2 carries the interface number
            buf[bdc + 3] = 0; // data_offset: nothing between here and the frame

            @memcpy(buf[head..][0..frame.len], frame);

            self.sdpcm_seq +%= 1;
            self.eth_tx_count += 1;

            const padded = std.mem.alignForward(usize, size, sdpcm_align);
            // The padding is whatever the buffer already held, as it is in the
            // reference — the length fields describe the frame, so the chip
            // never looks at it.
            try self.bus.writeBytes(.wlan_or_bt, 0, buf[0..padded]);
        }

        /// Collect one received 802.3 frame, or 0 if none is waiting — rung 14.
        ///
        /// Drains Function 2 first, so a caller that only ever calls this still
        /// makes progress. Frames latched while some *other* call was draining
        /// the bus — every IOCTL does — are returned here just the same, which
        /// is the reason for the latch.
        ///
        /// The predecessor called into the transport, which on a host build
        /// synthesised an entire Ethernet/IPv4/UDP NTP datagram and returned it
        /// as though the radio had received one. That is why the `ntp-client`
        /// example printed plausible times while doing nothing at all.
        pub fn pollEthFrame(self: *Self, buf: []u8) !usize {
            if (self.rxQueued() == 0) {
                // Bounded: one packet per call, so a caller in a tight loop
                // cannot be held here by a busy network.
                _ = self.pollDevice() catch false;
            }
            if (self.rxQueued() == 0) return 0;

            const h = self.rx_ring_head;
            const len = self.rx_ring_len[h];
            if (buf.len < len) return error.BufferTooSmall;

            @memcpy(buf[0..len], self.rx_ring[h][0..len]);
            self.rx_ring_len[h] = 0;
            self.rx_ring_head = (h + 1) % rx_ring_slots;
            self.eth_rx_count += 1;
            return len;
        }

        /// Poll until a frame arrives or `timeout_ms` passes. Returns 0 on
        /// timeout rather than erroring: nothing arriving is an ordinary
        /// outcome on a quiet network, and the caller knows what it was waiting
        /// for.
        pub fn awaitEthFrame(self: *Self, buf: []u8, timeout_ms: u32) !usize {
            var left = timeout_ms;
            while (left > 0) : (left -= 1) {
                const n = try self.pollEthFrame(buf);
                if (n != 0) return n;
                self.bus.delayMs(1);
            }
            return 0;
        }

        // -- the backplane, forwarded for rung 17 ---------------------------
        //
        // **Bluetooth needs no new transport.** `cybt_shared_bus_driver.c`
        // reaches the chip through exactly these four calls, which are the ones
        // that have carried the WLAN microcode since rung 6 — so `bt/btbus.zig`
        // takes a device and uses them, rather than knowing anything about
        // gSPI. Forwarders rather than direct `dev.bus` access so the BT layer
        // depends on `Device` and not on the bus type behind it.

        pub fn backplaneRead32(self: *Self, addr: u32) !u32 {
            return self.bus.readBackplaneReg32(addr);
        }

        pub fn backplaneWrite32(self: *Self, addr: u32, val: u32) !void {
            return self.bus.writeBackplaneReg32(addr, val);
        }

        pub fn backplaneReadMem(self: *Self, addr: u32, buf: []u8) !void {
            return self.bus.readBackplaneChunked(addr, buf);
        }

        pub fn backplaneWriteMem(self: *Self, addr: u32, data: []const u8) !void {
            return self.bus.writeBackplaneChunked(addr, data);
        }

        /// The delay seam, forwarded.
        ///
        /// `ipstack.zig` needs to wait for a reply and must not reach for
        /// `rp2350.timer` to do it — the same reason the bus carries `delayMs`
        /// rather than calling the timer itself. One seam, one place to fake.
        pub fn delayMs(self: *Self, ms: u32) void {
            self.bus.delayMs(ms);
        }

        /// The clock seam, forwarded — the other half of `delayMs`, and added
        /// for `lib/ipnet` rung N1. A stack that can only sleep cannot tell
        /// how long anything took.
        pub fn nowMs(self: *Self) u64 {
            return self.bus.nowMs();
        }

        // -- host wake, forwarded -------------------------------------------
        //
        // The chip's interrupt shares the data pin, so all of this belongs to
        // the bus; the driver only passes it through. `lib/ipnet` rung N2 is
        // what it is for, and the contract is the vendor's: **the handler
        // masks and records, and never touches this bus.**

        pub fn enableHostWake(self: *Self) void {
            self.bus.enableHostWake();
        }

        pub fn disableHostWake(self: *Self) void {
            self.bus.disableHostWake();
        }

        /// Call from the application's `io_bank0` handler, and nothing else.
        pub fn onHostWakeIrq(self: *Self) void {
            self.bus.onHostWakeIrq();
        }

        /// True if the chip has asked for attention since the last call.
        pub fn takeHostWake(self: *Self) bool {
            return self.bus.takeHostWake();
        }

        /// Re-arm once whatever the chip wanted has been collected.
        pub fn rearmHostWake(self: *Self) void {
            self.bus.rearmHostWake();
        }

        pub fn getLinkStatus(self: *Self) !LinkStatus {
            return self.link_status;
        }
    };
}

// ---------------------------------------------------------------------------
// Host unit tests
//
// The mock asserts what the driver put on the bus. Its predecessor accepted
// any address, any function and any header, and returned a constant — so every
// test passed against a driver whose command headers were malformed, whose
// probe read the wrong register, and whose frames were unpadded.
// ---------------------------------------------------------------------------

const testing = std.testing;

// **Pull the submodules in, or their tests do not run.**
//
// A `pub const x = @import(...)` at the root is not enough: Zig analyses a file
// only when something references it, and `zig build test` collects tests only
// from files it analysed. `gspi_pio.zig` and `net.zig` were reached through
// code the driver actually calls, so their tests ran — and `ntp.zig`'s three
// tests **had never run once**, silently, while the suite reported a count that
// looked complete. Nothing failed; there was simply less there than the number
// suggested.
test {
    _ = gspi;
    _ = btbus;
    _ = hci;
    _ = l2cap;
    _ = att;
    _ = bredr;
    _ = sdp;
    _ = avdtp;
    _ = sbc;
    _ = sbc_decode;
}

/// One request as the canonical trace records it.
const LoggedIoctl = struct {
    cmd: u32 = 0,
    len: u32 = 0,
    is_get: bool = false,
    /// The IOVAR name, up to its NUL. Empty for a plain IOCTL.
    name: [24]u8 = @splat(0),
    name_len: usize = 0,

    fn iovar(self: LoggedIoctl) []const u8 {
        return self.name[0..self.name_len];
    }
};

/// A gSPI bus that checks its arguments and records what it saw.
/// How many frames the mock can stage. Larger than the driver's ring on
/// purpose, so a test can overrun the driver and watch it count the loss.
const mock_rx_depth = 8;

const AssertingBus = struct {
    // Host-wake state; the behaviour is below with the rest of the mock.
    host_wake_pending: bool = false,
    host_wake_enabled: bool = false,
    /// Whether the interrupt is currently unmasked. A test reads this during a
    /// transfer to prove it was masked.
    irq_armed: bool = false,
    /// How many times the mask has been taken away and given back — a transfer
    /// that forgot to restore shows up here as a count that stops moving.
    irq_rearms: u32 = 0,

    powered: bool = false,
    window: u32 = 0,

    /// Clear this to simulate an absent or dead chip: the test register reads
    /// back as an idle bus rather than as the test pattern.
    chip_present: bool = true,

    delay_total_ms: u32 = 0,

    /// Mirrors the real bus field the driver writes after reading the chip's
    /// accepted response delay back.
    resp_delay_bytes: usize = 4,

    /// Mirrors the real bus field selecting the chip's output edge, so the
    /// mock can check the driver sends the handshake word it was configured
    /// for rather than the historical constant.
    high_speed: bool = true,

    /// Phase of the bring-up, so the mock can insist the handshake happens in
    /// the right order rather than merely happening.
    mode: enum { boot_16bit, mode_32bit } = .boot_16bit,

    saw_test_register_read: bool = false,
    saw_handshake: bool = false,
    resp_delay_set: ?u8 = null,

    f2_writes: u32 = 0,
    last_f2_len: usize = 0,
    /// Big enough for the largest frame the driver can build. It was 128, and a
    /// CLM chunk overran it — in a mock, which is the cheap place to find out.
    /// Rung 14 moved it again, to a full 802.3 frame.
    last_f2: [max_rx_packet]u8 = @splat(0),
    /// The **first** control frame of the run, kept separately. A multi-frame
    /// sequence like the CLM download otherwise leaves only its last frame
    /// visible, which is the status read rather than the thing under test.
    first_f2_len: usize = 0,
    first_f2: [max_rx_packet]u8 = @splat(0),

    /// Every request, as the canonical trace records them: the command, the
    /// declared payload length, and the IOVAR name where there is one. A
    /// sequence like `wifiOn` is 20 frames whose *order* is the thing under
    /// test, and neither the first nor the last of them shows that.
    log: [64]LoggedIoctl = @splat(.{}),
    log_len: usize = 0,

    /// The chip answers every control frame. Modelling that is not optional:
    /// without it the mock asserts against a driver that cannot complete an
    /// IOCTL, which is a different driver from the one that ships.
    response_pending: bool = false,
    response_id: u16 = 0,
    response_status: u32 = 0,
    /// Echoed back in the response, as the chip echoes the request's command.
    response_cmd: u32 = 0,
    /// Set when the outstanding request was an `SDPCM_GET`.
    response_is_get: bool = false,
    /// **The response body echoes the request's declared length**, measured on
    /// hardware: a 1008-byte `clmload` chunk is answered with a 1036-byte
    /// packet. Modelling it is what makes an undersized receive buffer a failing
    /// test here instead of a deadlock on the board.
    response_body_len: usize = 0,
    /// The body a GET answers with. `clmload_status` reads this as its u32
    /// result, so a test can make the chip reject a CLM download.
    get_result: u32 = 0,
    credit: u8 = 1,

    /// 802.3 frames the chip is holding for us, and the BDC `data_offset` each
    /// will be sent behind — in **words**, so a non-zero value here is what
    /// catches a receive path that reads that field as bytes.
    ///
    /// **A queue rather than a slot, since rung N3.** A real peer sends
    /// back-to-back segments — a captured TCP session has two
    /// 26 µs apart — and a mock that can only stage one frame cannot express
    /// that, so the driver's ring could not have been tested against the thing
    /// it exists for.
    rx_q: [mock_rx_depth][eth_max_frame]u8 = @splat(@splat(0)),
    rx_q_len: [mock_rx_depth]usize = @splat(0),
    rx_q_offset: [mock_rx_depth]u8 = @splat(0),
    rx_q_head: usize = 0,
    rx_q_tail: usize = 0,
    rx_eth_itf: u8 = 0,
    /// Serve the waiting frame ahead of a pending control response, so a test
    /// can put a data frame in front of an IOCTL's answer — which is what the
    /// chip does, and what the receive latch exists for.
    data_first: bool = false,

    /// Data-channel writes, kept apart from control ones. Counting them
    /// together would let a test pass on the wrong channel entirely.
    data_writes: u32 = 0,
    last_data_len: usize = 0,
    last_data: [max_data_frame]u8 = @splat(0),

    fault: ?[]const u8 = null,

    fn fail(self: *AssertingBus, msg: []const u8) void {
        if (self.fault == null) self.fault = msg;
    }

    pub fn powerOn(self: *AssertingBus) void {
        self.powered = true;
        self.mode = .boot_16bit;
        self.window = 0;
    }

    pub fn powerOff(self: *AssertingBus) void {
        self.powered = false;
    }

    /// Counted rather than slept, so a retry loop that never terminates shows
    /// up as a number instead of as a hung test run.
    pub fn delayMs(self: *AssertingBus, ms: u32) void {
        self.delay_total_ms += ms;
    }

    /// The mock's clock: the sleep is the only thing that advances it, so a
    /// driver that spins without sleeping hangs the test rather than passing.
    pub fn nowMs(self: *AssertingBus) u64 {
        return self.delay_total_ms;
    }

    // -- host wake ----------------------------------------------------------
    //
    // Modelled rather than stubbed, because the property worth asserting is a
    // *sequence*: masked while a transfer runs, restored afterwards, and the
    // flag surviving from the handler to whoever collects it. `irq_armed`
    // records the mask so a test can watch it move.

    pub fn enableHostWake(self: *AssertingBus) void {
        self.host_wake_enabled = true;
        self.host_wake_pending = false;
        self.irq_armed = true;
    }

    pub fn disableHostWake(self: *AssertingBus) void {
        self.host_wake_enabled = false;
        self.irq_armed = false;
    }

    pub fn onHostWakeIrq(self: *AssertingBus) void {
        self.irq_armed = false;
        self.host_wake_pending = true;
    }

    pub fn takeHostWake(self: *AssertingBus) bool {
        const was = self.host_wake_pending;
        self.host_wake_pending = false;
        return was;
    }

    pub fn rearmHostWake(self: *AssertingBus) void {
        if (self.host_wake_enabled) {
            self.irq_armed = true;
            self.irq_rearms += 1;
        }
    }

    pub fn readReg32Swap(self: *AssertingBus, fn_num: Function, addr: u17) !u32 {
        if (!self.powered) return error.NotPowered;
        if (self.mode != .boot_16bit) self.fail("swapped read after the 32-bit handshake");
        if (fn_num == .backplane) self.fail("swapped read on the backplane");
        if (addr != gspi.reg.read_test) self.fail("probed something other than 0x0014");
        self.saw_test_register_read = true;
        return if (self.chip_present) gspi.test_pattern else 0xFFFF_FFFF;
    }

    pub fn writeReg32Swap(self: *AssertingBus, fn_num: Function, addr: u17, val: u32) !void {
        if (!self.powered) return error.NotPowered;
        if (!self.saw_test_register_read) self.fail("handshake sent before the chip was probed");
        if (fn_num != .bus) self.fail("handshake on the wrong function");
        if (addr != gspi.reg.bus_control) self.fail("handshake to the wrong register");
        if (val != gspi.busControlWord(self.high_speed)) self.fail("handshake value does not match the configured edge mode");
        self.saw_handshake = true;
        self.mode = .mode_32bit;
    }

    pub fn readReg32(self: *AssertingBus, fn_num: Function, addr: u17) !u32 {
        if (!self.powered) return error.NotPowered;
        if (self.mode != .mode_32bit) self.fail("32-bit read before the handshake");
        if (fn_num == .bus and addr == gspi.reg.bus_control) {
            // The chip does not read the response-delay byte back; the mock
            // reproduces that so the driver's readback check stays honest.
            return gspi.busControlWord(self.high_speed) & ~@as(u32, 0xFF00);
        }
        if (fn_num == .bus and addr == gspi.reg.status) {
            var v: u32 = gspi.status_f2_rx_ready;
            // A control response outranks a waiting data frame unless a test
            // says otherwise: the driver's IOCTL wait drains the same queue, so
            // a mock that always handed it the frame first would deadlock every
            // IOCTL in the suite.
            if (self.stagedLen() != 0 and (self.data_first or !self.response_pending)) {
                v |= gspi.status_f2_pkt_available;
                // **The status word reports the padded transfer**, as the chip
                // does — which is longer than the SDPCM size field. A receive
                // path that measured the frame by this number would hand the
                // caller the padding as payload.
                v |= @as(u32, @intCast(std.mem.alignForward(usize, self.dataPacketLen(), sdpcm_align))) << 9;
            } else if (self.response_pending) {
                v |= gspi.status_f2_pkt_available;
                v |= @as(u32, @intCast(self.responseLen())) << 9;
            }
            return v;
        }
        return 0;
    }

    pub fn readReg8(self: *AssertingBus, fn_num: Function, addr: u17) !u8 {
        if (!self.powered) return error.NotPowered;
        if (self.mode != .mode_32bit) self.fail("32-bit read before the handshake");
        if (fn_num == .backplane and addr == sdio_chip_clock_csr) {
            return sbsdio_alp_avail | sbsdio_ht_avail;
        }
        // Cores read back as held in reset, which is what the sequence expects.
        return airc_reset;
    }

    pub fn readReg16(_: *AssertingBus, _: Function, _: u17) !u16 {
        return 0;
    }

    pub fn writeReg8(self: *AssertingBus, fn_num: Function, addr: u17, val: u8) !void {
        if (!self.powered) return error.NotPowered;
        if (self.mode != .mode_32bit) self.fail("32-bit write before the handshake");
        if (fn_num == .bus and addr == gspi.reg.resp_delay_f1) self.resp_delay_set = val;
    }

    pub fn writeReg16(self: *AssertingBus, _: Function, _: u17, _: u16) !void {
        if (!self.powered) return error.NotPowered;
    }

    pub fn writeReg32(self: *AssertingBus, _: Function, _: u17, _: u32) !void {
        if (!self.powered) return error.NotPowered;
    }

    pub fn writeBytes(self: *AssertingBus, fn_num: Function, addr: u17, data: []const u8) !void {
        if (!self.powered) return error.NotPowered;
        if (fn_num == .wlan_or_bt) {
            if (addr != 0) self.fail("SDPCM frame sent to a non-zero address");
            if (data.len % sdpcm_align != 0) self.fail("SDPCM frame not padded to 4 bytes");
            self.f2_writes += 1;
            self.last_f2_len = data.len;
            @memcpy(self.last_f2[0..data.len], data);
            if (self.f2_writes == 1) {
                self.first_f2_len = data.len;
                @memcpy(self.first_f2[0..data.len], data);
            }

            // **Channel first.** This branched on length alone until rung 14,
            // which was safe only while every F2 write was an IOCTL: a data
            // frame would have been read as a CDC header, answered with a
            // fabricated response, and counted as a control frame — so a
            // `sendEthFrame` writing to the wrong channel would have passed.
            const channel: u8 = if (data.len > 5) data[5] & 0x0F else 0xFF;
            if (channel == data_header) {
                self.checkDataFrame(data);
                self.data_writes += 1;
                self.last_data_len = data.len;
                @memcpy(self.last_data[0..data.len], data);
                self.credit +%= 1;
                return;
            }
            if (channel != control_header) self.fail("F2 write on an unknown SDPCM channel");

            // Queue the answer, echoing the request's id and command back as
            // the chip does.
            if (data.len >= 24) {
                const flags = std.mem.readInt(u32, data[20..24], .little);
                self.response_id = @truncate(flags >> cdcf_ioc_id_shift);
                self.response_cmd = std.mem.readInt(u32, data[12..16], .little);
                self.response_is_get = (flags & sdpcm_set) == 0;
                self.response_body_len = std.mem.readInt(u32, data[16..20], .little);
                self.response_pending = true;
                self.credit +%= 1;

                if (self.log_len < self.log.len) {
                    var entry = LoggedIoctl{
                        .cmd = self.response_cmd,
                        .len = @intCast(self.response_body_len),
                        .is_get = self.response_is_get,
                    };
                    // An IOVAR request opens with its NUL-terminated name.
                    if (self.response_cmd == @backingInt(IoctlCmd.set_var) or
                        self.response_cmd == @backingInt(IoctlCmd.get_var))
                    {
                        const body = data[28..];
                        const end = std.mem.indexOfScalar(u8, body[0..@min(body.len, entry.name.len)], 0) orelse 0;
                        entry.name_len = end;
                        @memcpy(entry.name[0..end], body[0..end]);
                    }
                    self.log[self.log_len] = entry;
                    self.log_len += 1;
                }
            }
        }
    }

    /// The contract for a transmitted 802.3 frame — the part of rung 14 the
    /// chip checks and never comments on.
    fn checkDataFrame(self: *AssertingBus, data: []const u8) void {
        if (data.len < sdpcm_header_len + sdpcm_data_pad + bdc_header_len) {
            self.fail("data frame shorter than its own headers");
            return;
        }
        const size = std.mem.readInt(u16, data[0..2], .little);
        const size_com = std.mem.readInt(u16, data[2..4], .little);
        if ((size ^ size_com) != 0xFFFF) self.fail("data frame size complement is wrong");
        if (size > data.len) self.fail("data frame declares more than was written");

        // The two pad bytes are the whole reason this channel has a different
        // header length, and a 12 here would displace the frame by two bytes
        // rather than failing.
        if (data[7] != sdpcm_header_len + sdpcm_data_pad) {
            self.fail("data frame header_length is not 14");
        }
        const bdc = sdpcm_header_len + sdpcm_data_pad;
        if (data[bdc] != bdc_flags_tx) self.fail("BDC flags are not 0x20");
        if (data[bdc + 3] != 0) self.fail("BDC data_offset is not zero on transmit");
    }

    /// Length of the queued response: the headers plus a body as long as the
    /// request's declared payload, which is what the chip does.
    fn responseLen(self: *AssertingBus) usize {
        return sdpcm_header_len + cdc_header_len + self.response_body_len;
    }

    /// The gSPI packet the chip would hand us for the waiting 802.3 frame:
    /// SDPCM header, two pad bytes, BDC header, `data_offset` words of filler,
    /// then the frame.
    fn stagedLen(self: *AssertingBus) usize {
        if (self.rx_q_head == self.rx_q_tail) return 0;
        return self.rx_q_len[self.rx_q_head];
    }

    fn dataPacketLen(self: *AssertingBus) usize {
        return sdpcm_header_len + sdpcm_data_pad + bdc_header_len +
            (@as(usize, self.rx_q_offset[self.rx_q_head]) << 2) + self.stagedLen();
    }

    /// Hand the driver a frame to receive. `offset_words` is the BDC
    /// `data_offset` the chip will declare.
    fn queueEthFrame(self: *AssertingBus, frame: []const u8, offset_words: u8) void {
        const next = (self.rx_q_tail + 1) % mock_rx_depth;
        if (next == self.rx_q_head) @panic("mock rx queue full — raise mock_rx_depth");
        @memcpy(self.rx_q[self.rx_q_tail][0..frame.len], frame);
        self.rx_q_len[self.rx_q_tail] = frame.len;
        self.rx_q_offset[self.rx_q_tail] = offset_words;
        self.rx_q_tail = next;
    }

    pub fn readBytes(self: *AssertingBus, fn_num: Function, _: u17, buf: []u8) !void {
        if (!self.powered) return error.NotPowered;
        @memset(buf, 0);
        if (fn_num != .wlan_or_bt) return;

        if (self.stagedLen() != 0 and (self.data_first or !self.response_pending)) {
            // A data-channel packet, built the way the chip builds one.
            const h = self.rx_q_head;
            const flen = self.rx_q_len[h];
            const off = self.rx_q_offset[h];
            const size: u16 = @intCast(self.dataPacketLen());
            if (buf.len < size) return;
            std.mem.writeInt(u16, buf[0..2], size, .little);
            std.mem.writeInt(u16, buf[2..4], ~size, .little);
            buf[5] = data_header;
            buf[7] = @intCast(sdpcm_header_len + sdpcm_data_pad);
            buf[9] = self.credit;
            const bdc = sdpcm_header_len + sdpcm_data_pad;
            buf[bdc] = bdc_flags_tx;
            buf[bdc + 2] = self.rx_eth_itf;
            buf[bdc + 3] = off;
            const start = bdc + bdc_header_len + (@as(usize, off) << 2);
            @memcpy(buf[start..][0..flen], self.rx_q[h][0..flen]);
            self.rx_q_len[h] = 0;
            self.rx_q_head = (h + 1) % mock_rx_depth;
            return;
        }

        // SDPCM header, then the CDC header: the shape the driver must parse.
        const size: u16 = @intCast(self.responseLen());
        if (buf.len < size) return;
        std.mem.writeInt(u16, buf[0..2], size, .little);
        std.mem.writeInt(u16, buf[2..4], ~size, .little);
        buf[5] = control_header;
        buf[7] = @intCast(sdpcm_header_len);
        buf[9] = self.credit;
        std.mem.writeInt(u32, buf[12..16], self.response_cmd, .little);
        std.mem.writeInt(u32, buf[16..20], @intCast(size - sdpcm_header_len - cdc_header_len), .little);
        std.mem.writeInt(u32, buf[20..24], @as(u32, self.response_id) << cdcf_ioc_id_shift, .little);
        std.mem.writeInt(u32, buf[24..28], self.response_status, .little);
        if (self.response_is_get and self.response_body_len >= 4) {
            std.mem.writeInt(u32, buf[28..32], self.get_result, .little);
        }
        self.response_pending = false;
    }

    pub fn setBackplaneWindow(self: *AssertingBus, addr: u32) !void {
        if (!self.powered) return error.NotPowered;
        self.window = addr & ~@as(u32, 0x7FFF);
    }

    /// The windowed accessors are where a raw backplane address would otherwise
    /// slip through as a register address, so the mock insists the window was
    /// actually moved to cover what is being addressed.
    fn checkWindowed(self: *AssertingBus, addr: u32) void {
        if (self.window != (addr & ~@as(u32, 0x7FFF))) {
            self.fail("backplane access outside the current window");
        }
    }

    pub fn readBackplaneReg8(self: *AssertingBus, addr: u32) !u8 {
        try self.setBackplaneWindow(addr);
        self.checkWindowed(addr);
        if (addr == chipcommon_base + 0) return 0;
        return airc_reset;
    }

    pub fn readBackplaneReg32(self: *AssertingBus, addr: u32) !u32 {
        try self.setBackplaneWindow(addr);
        self.checkWindowed(addr);
        return 0;
    }

    pub fn writeBackplaneReg8(self: *AssertingBus, addr: u32, _: u8) !void {
        try self.setBackplaneWindow(addr);
        self.checkWindowed(addr);
    }

    pub fn writeBackplaneReg32(self: *AssertingBus, addr: u32, _: u32) !void {
        try self.setBackplaneWindow(addr);
        self.checkWindowed(addr);
    }

    pub fn readBackplaneChunked(self: *AssertingBus, addr: u32, buf: []u8) !void {
        if (!self.powered) return error.NotPowered;
        _ = addr;
        @memset(buf, 0);
    }

    pub fn writeBackplaneChunked(self: *AssertingBus, addr: u32, data: []const u8) !void {
        if (!self.powered) return error.NotPowered;
        _ = addr;
        _ = data;
    }
};

fn expectNoFault(bus: *AssertingBus) !void {
    if (bus.fault) |m| {
        std.debug.print("bus contract violated: {s}\n", .{m});
        return error.BusContractViolated;
    }
}

test "busInit probes in 16-bit mode, then hands over to 32-bit" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);

    try dev.busInit();
    try expectNoFault(&bus);

    try testing.expect(bus.saw_test_register_read);
    try testing.expect(bus.saw_handshake);
    try testing.expect(dev.bus_ready);
    // Sixteen, requested and used. The register is deliberately not read back:
    // that read is issued with the very padding it would be used to determine.
    // Sixteen, written and used — the reference's value, once writes actually
    // reach the chip.
    try testing.expectEqual(@as(?u8, 16), bus.resp_delay_set);
    try testing.expectEqual(@as(usize, 16), bus.resp_delay_bytes);
}

test "a chip that never answers is reported, not assumed present" {
    var bus = AssertingBus{ .chip_present = false };
    var dev = Device(*AssertingBus).init(&bus, default_pins);

    try testing.expectError(error.ChipNotResponding, dev.busInit());
    try expectNoFault(&bus);

    // It must give up before the handshake, not send one into the void.
    try testing.expect(bus.saw_test_register_read);
    try testing.expect(!bus.saw_handshake);
    try testing.expect(!dev.bus_ready);
}

test "setGpio emits a 44-byte control frame, padded to 4" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);

    try dev.busInit();
    dev.firmware_loaded = true;

    try dev.setLed(true);
    try expectNoFault(&bus);

    try testing.expectEqual(@as(u32, 1), bus.f2_writes);
    // 12 SDPCM + 16 CDC + 16 payload = 44, which is already 4-aligned.
    // **Not 64** — that is the SDIO padding, and it was wrong here.
    try testing.expectEqual(@as(usize, 44), bus.last_f2_len);
}

test "the control frame is correct byte for byte" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    try dev.setLed(true);
    try expectNoFault(&bus);

    const f = bus.last_f2[0..bus.last_f2_len];

    // SDPCM header: size and its complement describe 44, not the padded 64.
    try testing.expectEqual(@as(u16, 44), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(@as(u16, ~@as(u16, 44)), std.mem.readInt(u16, f[2..4], .little));
    try testing.expectEqual(@as(u8, 0), f[4]); // first sequence number
    try testing.expectEqual(@as(u8, 0), f[5]); // CONTROL_HEADER
    try testing.expectEqual(@as(u8, 12), f[7]); // header_length

    // CDC header: WLC_SET_VAR is 263. It was 264 here, and 264 is a different
    // command entirely.
    try testing.expectEqual(@as(u32, 263), std.mem.readInt(u32, f[12..16], .little));
    try testing.expectEqual(@as(u32, 16), std.mem.readInt(u32, f[16..20], .little));
    // flags = ioctl_id << 16 | SDPCM_SET
    try testing.expectEqual(@as(u32, (1 << 16) | 2), std.mem.readInt(u32, f[20..24], .little));
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, f[24..28], .little));

    // Payload: the IOVAR name, then mask and value.
    try testing.expectEqualSlices(u8, "gpioout\x00", f[28..36]);
    try testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, f[36..40], .little));
    try testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, f[40..44], .little));

    // 44 bytes exactly: nothing trails the payload at this alignment.
    try testing.expectEqual(@as(usize, 44), f.len);
}

test "setGpio false clears the value but keeps the mask" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    try dev.setGpio(0, false);
    const f = bus.last_f2[0..bus.last_f2_len];
    try testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, f[36..40], .little));
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, f[40..44], .little));
}

test "the CLM blob is the tail of the image, not part of the microcode" {
    // **Both images, whichever one this build selected.** The numbers differ —
    // the WiFi-only microcode is 224,190 bytes and the WiFi+BT one is 231,077 —
    // and asserting only the selected image's would let `-Dfw=combined` pass on
    // a layout nobody had checked. If either stops holding, `loadFirmware` is
    // writing CLM bytes past the end of the microcode again.
    try testing.expectEqual(@as(usize, 224190), wifi_image.fw_len);
    try testing.expectEqual(@as(usize, 224256), std.mem.alignForward(usize, wifi_image.fw_len, 512));
    try testing.expectEqual(@as(usize, 225240), wifi_image.blob.len);

    try testing.expectEqual(@as(usize, 231077), combined_image.fw_len);
    try testing.expectEqual(@as(usize, 231424), std.mem.alignForward(usize, combined_image.fw_len, 512));
    try testing.expectEqual(@as(usize, 232408), combined_image.blob.len);

    // The same 984-byte CLM sits at the tail of each, and the two parts account
    // for the whole file in both. `Image.check` asserts that at compile time;
    // this says the same thing where a failure names the number.
    inline for (.{ wifi_image, combined_image }) |img| {
        try testing.expectEqual(@as(usize, 984), img.clm_len);
        try testing.expectEqual(img.blob.len, std.mem.alignForward(usize, img.fw_len, 512) + img.clm_len);
        try testing.expectEqual(img.clm_len, img.clm().len);
        try testing.expectEqual(img.fw_len, img.microcode().len);
    }

    // And the selection actually selects.
    try testing.expectEqual(clm_len, fw_clm.len);
    try testing.expectEqual(clm_offset + clm_len, fw_wifi_only.len);
    try testing.expectEqual(if (wifi_config.fw_combined) combined_image.fw_len else wifi_image.fw_len, wifi_fw_len);
}

test "a clmload chunk is correct byte for byte" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // Five bytes, so BEGIN and END land in one chunk — which is what the real
    // 984-byte blob does too — and so the padding to 8 is visible: 20 + 5 = 25
    // declared as 32.
    const blob = [_]u8{ 0x01, 0x02, 0x03, 0x04, 0x05 };
    try dev.loadClm(&blob);
    try expectNoFault(&bus);

    // Two frames: the chunk, then the status GET.
    try testing.expectEqual(@as(u32, 2), bus.f2_writes);

    const f = bus.first_f2[0..bus.first_f2_len];

    // SDPCM + CDC + 32 declared payload = 60.
    try testing.expectEqual(@as(u16, 60), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(@as(u32, 263), std.mem.readInt(u32, f[12..16], .little)); // WLC_SET_VAR
    try testing.expectEqual(@as(u32, 32), std.mem.readInt(u32, f[16..20], .little));

    // The IOVAR header the download handler parses.
    try testing.expectEqualSlices(u8, "clmload\x00", f[28..36]);
    // DLOAD_HANDLER_VER | DL_BEGIN | DL_END — 0x1006 for a single-chunk blob.
    try testing.expectEqual(@as(u16, 0x1006), std.mem.readInt(u16, f[36..38], .little));
    try testing.expectEqual(@as(u16, 2), std.mem.readInt(u16, f[38..40], .little)); // DL_TYPE_CLM
    // The *true* length, not the padded one: the padding is declared in the CDC
    // header, and the handler reads the real length from here.
    try testing.expectEqual(@as(u32, 5), std.mem.readInt(u32, f[40..44], .little));
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, f[44..48], .little)); // CRC
    try testing.expectEqualSlices(u8, &blob, f[48..53]);
}

test "the clmload_status read is a GET, and asks for room to answer in" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    const blob = [_]u8{ 0xAA, 0xBB, 0xCC };
    try dev.loadClm(&blob);
    try expectNoFault(&bus);

    const g = bus.last_f2[0..bus.last_f2_len];
    try testing.expectEqual(@as(u32, 262), std.mem.readInt(u32, g[12..16], .little)); // WLC_GET_VAR
    // Nineteen: the name and its NUL are fifteen, and the chip needs four bytes
    // of room to write the result into.
    try testing.expectEqual(@as(u32, 19), std.mem.readInt(u32, g[16..20], .little));
    // A GET is the *absence* of the SET bit, not a bit of its own.
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, g[20..24], .little) & sdpcm_set);
    try testing.expectEqualSlices(u8, "clmload_status\x00", g[28..43]);
}

test "the real CLM blob loads, at the sizes the chip actually sees" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // The exact configuration that deadlocked on hardware: 984 bytes of blob,
    // declared as 1008, in a 1036-byte frame — and answered by a 1036-byte
    // response the receive buffer has to be able to hold.
    try dev.loadClmDefault();
    try expectNoFault(&bus);

    const f = bus.first_f2[0..bus.first_f2_len];
    try testing.expectEqual(@as(usize, 1036), bus.first_f2_len);
    try testing.expectEqual(@as(u16, 1036), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(@as(u32, 1008), std.mem.readInt(u32, f[16..20], .little));
    try testing.expectEqual(@as(u32, 984), std.mem.readInt(u32, f[40..44], .little));
    // One chunk, then the status read — and no packet declined as oversize.
    try testing.expectEqual(@as(u32, 2), bus.f2_writes);
    try testing.expectEqual(@as(u32, 0), dev.census.oversize);
    try testing.expectEqual(@as(u32, 0), dev.clm_status);
}

test "a blob longer than one chunk is split, with BEGIN and END on the ends" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // Ours fits in one chunk, so the loop is otherwise never exercised — and an
    // untested loop that runs once is indistinguishable from no loop at all.
    var blob: [clm_chunk_len + 16]u8 = @splat(0x5A);
    try dev.loadClm(&blob);
    try expectNoFault(&bus);

    // Two chunks, then the status GET.
    try testing.expectEqual(@as(u32, 3), bus.f2_writes);

    // First chunk: BEGIN set, END clear, full length.
    const f = bus.first_f2[0..bus.first_f2_len];
    try testing.expectEqual(@as(u16, 0x1002), std.mem.readInt(u16, f[36..38], .little));
    try testing.expectEqual(@as(u32, clm_chunk_len), std.mem.readInt(u32, f[40..44], .little));
}

test "a CLM download the chip rejects is an error, not a warning" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // Every chunk IOCTL returns CDC status 0; the handler's own verdict comes
    // back only in `clmload_status`. The reference merely warns here, which is
    // indistinguishable from success to anything downstream.
    bus.get_result = 1;
    const blob = [_]u8{ 0xAA, 0xBB, 0xCC };
    try testing.expectError(error.ClmLoadFailed, dev.loadClm(&blob));
}

test "an IOCTL before firmware is an error, not a silent no-op" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();

    try testing.expectError(error.FirmwareNotLoaded, dev.setLed(true));
    try testing.expectEqual(@as(u32, 0), bus.f2_writes);
}

test "a join before wifiOn is refused, and does not touch the link state" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // The sequence in `joinWifi` is meaningless until the interface is up, so
    // it refuses rather than issuing eleven IOCTLs the chip is not ready for.
    try testing.expectError(error.WifiNotOn, dev.joinWifi("net", "pass", .wpa2_aes));
    try testing.expectError(error.WifiNotOn, dev.disconnectWifi());
    try testing.expectEqual(@as(u32, 0), bus.f2_writes);
    try testing.expectEqual(LinkStatus.down, try dev.getLinkStatus());
}

test "joinWifi issues the traced sequence and does not claim to have joined" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;
    dev.wifi_on = true;

    try dev.joinWifi("HFC", "secretpass", .wpa2_aes);
    try expectNoFault(&bus);

    const Expected = struct { cmd: IoctlCmd, len: u32, name: []const u8 = "" };
    const expected = [_]Expected{
        .{ .cmd = .set_var, .len = 19, .name = "ampdu_ba_wsize" },
        .{ .cmd = .set_wsec, .len = 4 },
        .{ .cmd = .set_var, .len = 23, .name = "bsscfg:sup_wpa" },
        .{ .cmd = .set_var, .len = 31, .name = "bsscfg:sup_wpa2_eapver" },
        .{ .cmd = .set_var, .len = 27, .name = "bsscfg:sup_wpa_tmo" },
        .{ .cmd = .set_passphrase, .len = 68 },
        .{ .cmd = .set_infra, .len = 4 },
        .{ .cmd = .set_auth, .len = 4 },
        .{ .cmd = .set_var, .len = 8, .name = "mfp" },
        .{ .cmd = .set_wpa_auth, .len = 4 },
        .{ .cmd = .set_ssid, .len = 36 },
    };

    try testing.expectEqual(expected.len, bus.log_len);
    for (expected, bus.log[0..bus.log_len]) |want, got| {
        try testing.expectEqual(@backingInt(want.cmd), got.cmd);
        try testing.expectEqual(want.len, got.len);
        try testing.expectEqualStrings(want.name, got.iovar());
    }

    // **The whole point.** Every IOCTL was accepted and the chip has said
    // nothing about the link, so the state is `joining` and not `joined`.
    try testing.expectEqual(LinkStatus.joining, try dev.getLinkStatus());
    try testing.expect(!dev.join_state.complete());
}

test "the passphrase goes in a fixed 68-byte wsec_pmk, length-prefixed" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;
    dev.wifi_on = true;

    try dev.joinWifi("HFC", "secretpass", .wpa2_aes);

    // The sixth frame is the PMK. Its payload is always 68 bytes whatever the
    // key length — the field is fixed and the length says how much is real.
    const f = bus.last_f2[0..bus.last_f2_len];
    _ = f;
    try testing.expectEqual(@as(u32, 68), bus.log[5].len);
    try testing.expectEqual(@backingInt(IoctlCmd.set_passphrase), bus.log[5].cmd);

    // And wsec is the auth type's low byte: 4 for WPA2-AES, which is what the
    // join trace shows as `Setting wsec=0x4`.
    try testing.expectEqual(@as(u32, 4), AuthType.wpa2_aes.wsec());
    try testing.expectEqual(@as(u32, 0x80), AuthType.wpa2_aes.wpaAuth());
}

test "an open network sends no passphrase at all" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;
    dev.wifi_on = true;

    try dev.joinWifi("open-net", "", .open);

    // Ten frames, not eleven: `WLC_SET_WSEC_PMK` is skipped entirely rather
    // than sent empty.
    try testing.expectEqual(@as(usize, 10), bus.log_len);
    for (bus.log[0..bus.log_len]) |e| {
        try testing.expect(e.cmd != @backingInt(IoctlCmd.set_passphrase));
    }
    try testing.expectEqual(@as(u32, 0), AuthType.open.wsec());
}

test "credits refresh from responses, so the IOCTL loop sustains" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // The seed lets the first frame out; every one after it depends on the
    // chip's response having been read and its credit byte taken. Ten in a row
    // is the behaviour that replaced a hard stall after one.
    for (0..10) |_| try dev.setLed(true);
    try expectNoFault(&bus);
    try testing.expectEqual(@as(u32, 10), bus.f2_writes);
}

test "an IOCTL the chip rejects is an error, not a success" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // A rejected IOCTL still produces a response; only the CDC status word
    // distinguishes it. Ignoring that word is what made the old stub look like
    // it worked.
    bus.response_status = 0xFFFF_FFFF;
    try testing.expectError(error.IoctlRejected, dev.setLed(true));
}

test "a response carrying someone else's id is not accepted" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    // Force the mock to answer with an id that cannot match the request.
    bus.response_pending = true;
    bus.response_id = 0xBEEF;
    dev.pending_ioctl_id = 1;
    dev.response = .{};
    _ = dev.pollDevice() catch {};

    try testing.expect(!dev.response.ready);
    try testing.expect(dev.census.id_mismatch > 0);
}

test "wifiOn issues the canonical join trace's sequence, in order and at its lengths" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    try dev.wifiOn(.gb);
    try expectNoFault(&bus);

    // Transcribed from a trace of the SDK's own join, steps 3-22, including the
    // repeats. The lengths are the chip's own view of each payload, and every
    // one of them is a line in that file — a length here that the trace does
    // not show is a length nobody has checked.
    const Expected = struct { cmd: IoctlCmd, len: u32, name: []const u8 = "", is_get: bool = false };
    const expected = [_]Expected{
        .{ .cmd = .set_var, .len = 15, .name = "bus:txglom" },
        .{ .cmd = .set_var, .len = 10, .name = "apsta" },
        .{ .cmd = .get_var, .len = 20, .name = "cur_etheraddr", .is_get = true },
        .{ .cmd = .get_var, .len = 20, .name = "cur_etheraddr", .is_get = true },
        .{ .cmd = .set_var, .len = 20, .name = "country" },
        .{ .cmd = .set_antdiv, .len = 4 },
        .{ .cmd = .set_var, .len = 15, .name = "bus:txglom" },
        .{ .cmd = .set_var, .len = 10, .name = "apsta" },
        .{ .cmd = .set_var, .len = 19, .name = "ampdu_ba_wsize" },
        .{ .cmd = .set_var, .len = 15, .name = "ampdu_mpdu" },
        .{ .cmd = .set_var, .len = 20, .name = "ampdu_rx_factor" },
        .{ .cmd = .set_var, .len = 41, .name = "bsscfg:event_msgs" },
        .{ .cmd = .up, .len = 0 },
        .{ .cmd = .set_var, .len = 18, .name = "pm2_sleep_ret" },
        .{ .cmd = .set_var, .len = 15, .name = "bcn_li_bcn" },
        .{ .cmd = .set_var, .len = 16, .name = "bcn_li_dtim" },
        .{ .cmd = .set_var, .len = 17, .name = "assoc_listen" },
        .{ .cmd = .set_pm, .len = 4 },
        .{ .cmd = .set_gmode, .len = 4 },
        .{ .cmd = .set_band, .len = 4 },
    };

    try testing.expectEqual(expected.len, bus.log_len);
    for (expected, bus.log[0..bus.log_len]) |want, got| {
        try testing.expectEqual(@backingInt(want.cmd), got.cmd);
        try testing.expectEqual(want.len, got.len);
        try testing.expectEqual(want.is_get, got.is_get);
        try testing.expectEqualStrings(want.name, got.iovar());
    }

    try testing.expect(dev.wifi_on);
}

test "the country IOVAR carries the code twice, with a default revision between" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    try dev.wifiOn(.gb);

    // GB is 'G' | 'B' << 8. The trace shows this IOVAR's length but not its
    // value, so the layout is checked against `cyw43_ll_wifi_on` and the code
    // against ASCII — the two independent things available.
    try testing.expectEqual(@as(u32, 0x4247), @backingInt(Country.gb));
}

test "the event mask enables everything except the six noisy events" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    try testing.expectEqual(@as(u32, 0), try dev.setEventMask());
    const f = bus.first_f2[0..bus.first_f2_len];

    // 12 SDPCM + 16 CDC + 41 = 69, padded to 72 on the wire.
    try testing.expectEqual(@as(u32, 41), std.mem.readInt(u32, f[16..20], .little));
    try testing.expectEqualSlices(u8, "bsscfg:event_msgs\x00", f[28..46]);
    // The bsscfg index, then the mask.
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, f[46..50], .little));

    const mask = f[50..69];
    const off = [_]u8{ 19, 20, 40, 44, 54, 71 };
    for (off) |ev| {
        const bit = mask[ev / 8] & (@as(u8, 1) << @intCast(ev % 8));
        try testing.expectEqual(@as(u8, 0), bit);
    }
    // EV_LINK (16) and EV_PSK_SUP (46) are the ones a join depends on; they
    // must survive the clearing.
    try testing.expect(mask[16 / 8] & (1 << (16 % 8)) != 0);
    try testing.expect(mask[46 / 8] & (1 << (46 % 8)) != 0);
}

/// Build an SDPCM async-event packet the way the chip does, so the parser is
/// tested against the layout rather than against itself.
fn makeEventPacket(buf: []u8, data_offset_words: u8, event_type: u32, status: u32, flags: u16) []u8 {
    @memset(buf, 0);
    const hl: usize = sdpcm_header_len;
    const payload = hl + bdc_header_len + (@as(usize, data_offset_words) << 2);
    const size = payload + 24 + async_event_len;

    std.mem.writeInt(u16, buf[0..2], @intCast(size), .little);
    std.mem.writeInt(u16, buf[2..4], @intCast(~size & 0xFFFF), .little);
    buf[5] = asyncevent_header;
    buf[7] = @intCast(hl);
    buf[hl + 3] = data_offset_words; // BDC data_offset, in words

    const eth = buf[payload..];
    eth[12] = 0x88; // event ethertype
    eth[13] = 0x6C;
    eth[19] = 0x00; // Broadcom OUI
    eth[20] = 0x10;
    eth[21] = 0x18;

    const ev = eth[24..];
    // **Big-endian**, unlike everything else in this driver.
    std.mem.writeInt(u16, ev[2..4], flags, .big);
    std.mem.writeInt(u32, ev[4..8], event_type, .big);
    std.mem.writeInt(u32, ev[8..12], status, .big);
    return buf[0..size];
}

test "an async event is parsed from four layers down, big-endian" {
    var raw: [256]u8 = undefined;
    // data_offset of 2 words: the parser must multiply by four, not add two.
    const pkt = makeEventPacket(&raw, 2, @backingInt(EventType.link), 0, 1);

    const D = Device(*AssertingBus);
    const ev = try D.parseAsyncEvent(pkt);

    try testing.expectEqual(@as(u32, 16), ev.event_type); // CYW43_EV_LINK
    try testing.expectEqual(@as(u32, 0), ev.status);
    try testing.expectEqual(@as(u16, 1), ev.flags);
}

test "a channel-1 packet that is not an event is rejected, not misparsed" {
    var raw: [256]u8 = undefined;
    var pkt = makeEventPacket(&raw, 0, @backingInt(EventType.link), 0, 1);

    // The startup packets the reference calls expected: right channel, wrong
    // ethertype. Misreading one produces a plausible event from noise.
    pkt[sdpcm_header_len + bdc_header_len + 12] = 0x05;
    pkt[sdpcm_header_len + bdc_header_len + 13] = 0x10;

    const D = Device(*AssertingBus);
    try testing.expectError(error.NotAnEvent, D.parseAsyncEvent(pkt));
}

test "a join completes only when auth, link and keyed have all arrived" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    dev.wifi_on = true;
    dev.join_state = .{ .kind = JoinState.kind_active };
    dev.link_status = .joining;

    // Each event alone leaves the join incomplete. This is the invariant the
    // old `joinWifi` violated by declaring success without any of them.
    dev.processEvent(.{ .event_type = @backingInt(EventType.auth), .status = 0 });
    try testing.expectEqual(LinkStatus.joining, dev.link_status);

    dev.processEvent(.{ .event_type = @backingInt(EventType.link), .status = 0, .flags = 1 });
    try testing.expectEqual(LinkStatus.joining, dev.link_status);

    dev.processEvent(.{ .event_type = @backingInt(EventType.psk_sup), .status = sup_keyed });
    try testing.expectEqual(LinkStatus.joined, dev.link_status);
    try testing.expect(dev.join_state.complete());
}

test "a wrong passphrase reports bad auth rather than timing out" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    dev.wifi_on = true;
    dev.join_state = .{ .kind = JoinState.kind_active };
    dev.link_status = .joining;

    // A supplicant failure that is not one of the three retryable timeouts.
    dev.processEvent(.{ .event_type = @backingInt(EventType.psk_sup), .status = 9 });
    try testing.expectEqual(LinkStatus.bad_auth, dev.link_status);
}

test "an auth timeout or missing ack does not fail the join" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    dev.wifi_on = true;

    // Status 2 is a timeout and 5 a missing ack: both are radio conditions the
    // firmware retries, and reporting them as bad auth reads as a wrong password.
    for ([_]u32{ 2, 5 }) |status| {
        dev.join_state = .{ .kind = JoinState.kind_active };
        dev.link_status = .joining;

        dev.processEvent(.{ .event_type = @backingInt(EventType.auth), .status = status });
        try testing.expectEqual(LinkStatus.joining, dev.link_status);

        dev.processEvent(.{ .event_type = @backingInt(EventType.auth), .status = 0 });
        dev.processEvent(.{ .event_type = @backingInt(EventType.link), .status = 0, .flags = 1 });
        dev.processEvent(.{ .event_type = @backingInt(EventType.psk_sup), .status = sup_keyed });
        try testing.expectEqual(LinkStatus.joined, dev.link_status);
    }

    // Any other non-zero auth status is still a failure.
    dev.join_state = .{ .kind = JoinState.kind_active };
    dev.processEvent(.{ .event_type = @backingInt(EventType.auth), .status = 1 });
    try testing.expectEqual(LinkStatus.bad_auth, dev.link_status);
}

test "an SSID nobody answers is reported as no_net, not as a generic failure" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    dev.wifi_on = true;
    dev.join_state = .{ .kind = JoinState.kind_active };

    // status 3, reason 0 is specifically "no matching SSID" — worth its own
    // state, because it means something the user can act on.
    dev.processEvent(.{ .event_type = @backingInt(EventType.set_ssid), .status = 3, .reason = 0 });
    try testing.expectEqual(LinkStatus.no_net, dev.link_status);

    dev.join_state = .{ .kind = JoinState.kind_active };
    dev.processEvent(.{ .event_type = @backingInt(EventType.set_ssid), .status = 1 });
    try testing.expectEqual(LinkStatus.fail, dev.link_status);
}

// ---------------------------------------------------------------------------
// Rung 14 — the 802.3 data path
// ---------------------------------------------------------------------------

/// A device the data path will talk to: bus up, firmware loaded, link joined.
/// The link check in `sendEthFrameItf` is deliberate, so every send test has to
/// say out loud that it is pretending to be associated.
fn joinedDevice(bus: *AssertingBus) Device(*AssertingBus) {
    var dev = Device(*AssertingBus).init(bus, default_pins);
    dev.busInit() catch unreachable;
    dev.firmware_loaded = true;
    dev.wifi_on = true;
    dev.link_status = .joined;
    return dev;
}

test "an 802.3 frame goes out on channel 2, behind eighteen bytes of header" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    const frame = [_]u8{
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // destination: broadcast
        0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F, // source
        0x08, 0x06, // ARP
        0xDE, 0xAD,
        0xBE, 0xEF,
    };
    try dev.sendEthFrame(&frame);
    try expectNoFault(&bus);

    try testing.expectEqual(@as(u32, 1), bus.data_writes);
    const f = bus.last_data[0..bus.last_data_len];

    // 12 SDPCM + 2 pad + 4 BDC + 18 frame = 36.
    const size = 12 + 2 + 4 + frame.len;
    try testing.expectEqual(@as(u16, size), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(@as(u16, ~@as(u16, size)), std.mem.readInt(u16, f[2..4], .little));
    try testing.expectEqual(@as(u8, 0), f[4]); // sequence 0, the first frame sent
    try testing.expectEqual(@as(u8, 2), f[5]); // DATA_HEADER — not 0
    try testing.expectEqual(@as(u8, 0), f[6]); // next_length

    // **14, not 12.** This is the whole difference between the two channels,
    // and a 12 here displaces the frame by two bytes rather than erroring.
    try testing.expectEqual(@as(u8, 14), f[7]);

    // BDC header sits after the two pad bytes, which are zero.
    try testing.expectEqualSlices(u8, &.{ 0, 0 }, f[12..14]);
    try testing.expectEqual(@as(u8, 0x20), f[14]); // flags
    try testing.expectEqual(@as(u8, 0), f[15]); // priority
    try testing.expectEqual(@as(u8, 0), f[16]); // flags2 = interface 0, the station
    try testing.expectEqual(@as(u8, 0), f[17]); // data_offset

    try testing.expectEqualSlices(u8, &frame, f[18..][0..frame.len]);
    // The transfer pads to 4, as every SDPCM write does: 36 is already aligned.
    try testing.expectEqual(@as(usize, 36), f.len);
}

test "a frame whose length is not a multiple of four still pads the transfer" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // 42 bytes is an ARP request, and 18 + 42 = 60, which is aligned. 43 is
    // the case that is not.
    const frame: [43]u8 = @splat(0x5A);
    try dev.sendEthFrame(&frame);
    try expectNoFault(&bus);

    const f = bus.last_data[0..bus.last_data_len];
    // The header still declares the true length; only the transfer is rounded.
    try testing.expectEqual(@as(u16, 61), std.mem.readInt(u16, f[0..2], .little));
    try testing.expectEqual(@as(usize, 64), f.len);
}

test "a frame sent before the link is up is refused here, because the chip will not refuse it" {
    var bus = AssertingBus{};
    var dev = Device(*AssertingBus).init(&bus, default_pins);
    try dev.busInit();
    dev.firmware_loaded = true;

    const frame: [42]u8 = @splat(0);
    try testing.expectError(error.LinkDown, dev.sendEthFrame(&frame));
    // Nothing reached the bus: a frame the chip would have credited and dropped
    // is worse than an error, because the credit makes it look sent.
    try testing.expectEqual(@as(u32, 0), bus.data_writes);
}

test "a received frame is found behind a BDC data_offset counted in words" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    const frame = [_]u8{
        0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
        0x08, 0x00, 0x45, 0x00, 0x01, 0x02,
    };
    // data_offset 2 means eight bytes of firmware header between the BDC header
    // and the frame. Reading that field as bytes would start the frame four
    // bytes early — inside the filler, and still returning a plausible length.
    bus.queueEthFrame(&frame, 2);

    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try expectNoFault(&bus);

    try testing.expectEqual(frame.len, n);
    try testing.expectEqualSlices(u8, &frame, out[0..n]);
    try testing.expectEqual(@as(u32, 1), dev.census.data);
    try testing.expectEqual(@as(u32, 0), dev.census.data_too_small);
}

test "a received frame is measured by the SDPCM size, not the padded transfer" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // 12 + 2 + 4 + 15 = 33, which the chip transfers as 36. The three padding
    // bytes are junk; a receive path that trusted the status word's length
    // would hand them to the caller as part of the frame.
    const frame: [15]u8 = @splat(0xC3);
    bus.queueEthFrame(&frame, 0);

    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try testing.expectEqual(@as(usize, 15), n);
}

test "a frame arriving while an IOCTL is in flight is kept, not dropped" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    const frame: [64]u8 = @splat(0x7E);
    bus.queueEthFrame(&frame, 0);
    // The chip hands over the data frame first, then the IOCTL's answer — which
    // it is free to do, and which is the case a receive path built around
    // "poll and return what I saw" loses entirely.
    bus.data_first = true;

    try dev.setLed(true);
    try expectNoFault(&bus);

    // The frame was latched by the IOCTL's own drain, and is still collectable.
    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try testing.expectEqual(frame.len, n);
    try testing.expectEqualSlices(u8, &frame, out[0..n]);
}

test "a burst arrives intact and in order" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // **The case a quiet LAN never produced.** A 2048-byte write leaves a real
    // peer as two segments 26 µs apart — see `tcp-session1.pcap` — and the
    // single latch this replaced kept the first and counted the second as a
    // drop. Four here, because four is what the ring is for.
    var sent: [rx_ring_depth][32]u8 = undefined;
    for (&sent, 0..) |*f, i| {
        f.* = @splat(@intCast(0xA0 + i));
        bus.queueEthFrame(f, 0);
    }

    // All of them arrive before anything collects, which is the point: the
    // chip does not wait for the host.
    for (0..rx_ring_depth) |_| try testing.expect(try dev.pollDevice());

    try testing.expectEqual(@as(u32, 0), dev.census.data_dropped);
    try testing.expectEqual(@as(usize, rx_ring_depth), dev.rxQueued());
    try testing.expectEqual(@as(u32, rx_ring_depth), dev.rx_high_water);

    // **In order.** A ring that mixed head and tail would still return four
    // frames and would return them wrong.
    for (sent) |expected| {
        var out: [eth_max_frame]u8 = undefined;
        const n = try dev.pollEthFrame(&out);
        try testing.expectEqualSlices(u8, &expected, out[0..n]);
    }
    try testing.expectEqual(@as(usize, 0), dev.rxQueued());
}

test "a full ring drops the newest and keeps the queue intact" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    for (0..rx_ring_depth) |i| {
        const f: [32]u8 = @splat(@intCast(0xA0 + i));
        bus.queueEthFrame(&f, 0);
        try testing.expect(try dev.pollDevice());
    }
    try testing.expectEqual(@as(u32, 0), dev.census.data_dropped);

    // One too many.
    const overflow: [32]u8 = @splat(0xFF);
    bus.queueEthFrame(&overflow, 0);
    try testing.expect(try dev.pollDevice());

    // **Counted, not silent** — and the *newest* is the casualty. Dropping the
    // oldest instead would reorder the caller's traffic, which is far harder to
    // diagnose than a loss with a number against it.
    try testing.expectEqual(@as(u32, 1), dev.census.data_dropped);
    try testing.expectEqual(@as(usize, rx_ring_depth), dev.rxQueued());

    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try testing.expectEqual(@as(u8, 0xA0), out[0]);
    try testing.expectEqual(@as(usize, 32), n);
}

test "the high-water mark records the deepest the ring ever got" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // **This is the field that makes `data_dropped = 0` mean something.** Zero
    // drops with a high-water mark of 1 says the depth was never exercised;
    // zero drops at the full depth says the next burst overflows. The two read
    // identically without it, and the first is what every quiet-network run in
    // `example/wake-check` reported.
    const a: [32]u8 = @splat(0xA1);
    bus.queueEthFrame(&a, 0);
    try testing.expect(try dev.pollDevice());
    try testing.expectEqual(@as(u32, 1), dev.rx_high_water);

    const b: [32]u8 = @splat(0xB2);
    bus.queueEthFrame(&b, 0);
    try testing.expect(try dev.pollDevice());
    try testing.expectEqual(@as(u32, 2), dev.rx_high_water);

    // Draining does not lower it: it is a mark, not a gauge.
    var out: [eth_max_frame]u8 = undefined;
    _ = try dev.pollEthFrame(&out);
    _ = try dev.pollEthFrame(&out);
    try testing.expectEqual(@as(usize, 0), dev.rxQueued());
    try testing.expectEqual(@as(u32, 2), dev.rx_high_water);
}

test "the receive buffer holds a full-size frame, which the control path's did not" {
    // 1076 bytes was enough for a CLM chunk and is not enough for an 802.3
    // frame. A frame over the old limit was counted in `oversize` and skipped,
    // which is a receive path that works only on small packets.
    try testing.expect(max_rx_packet >= max_data_frame);
    try testing.expectEqual(@as(usize, 1532), max_data_frame);

    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    const frame: [eth_max_frame]u8 = @splat(0x99);
    bus.queueEthFrame(&frame, 0);

    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try testing.expectEqual(@as(usize, 1514), n);
    try testing.expectEqual(@as(u32, 0), dev.census.oversize);
}

test "an empty poll is zero, not an error and not a stale frame" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    var out: [eth_max_frame]u8 = undefined;
    try testing.expectEqual(@as(usize, 0), try dev.pollEthFrame(&out));

    const frame: [40]u8 = @splat(0x44);
    bus.queueEthFrame(&frame, 0);
    try testing.expectEqual(frame.len, try dev.pollEthFrame(&out));
    // Collected once. A latch that did not clear would return it forever.
    try testing.expectEqual(@as(usize, 0), try dev.pollEthFrame(&out));
}

test "a chip asserting flow control stalls the transmitter" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // The reference stalls on `wlan_flow_control || credit == sequence`. This
    // driver checked only the credit until rung 14, so this frame would have
    // gone out into a queue the chip had just said was full.
    dev.wlan_flow_control = 1;
    const frame: [42]u8 = @splat(0);
    try testing.expectError(error.CreditTimeout, dev.sendEthFrame(&frame));
    try testing.expectEqual(@as(u32, 0), bus.data_writes);
}

test "host wake: the handler masks and records, and never touches the bus" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);
    dev.enableHostWake();
    try testing.expect(bus.irq_armed);

    const writes_before = bus.data_writes;
    dev.onHostWakeIrq();

    // **Masked**, because a level-high interrupt whose pin is still high
    // re-fires the instant the handler returns and the only way to quiet it is
    // to service the chip.
    try testing.expect(!bus.irq_armed);
    // **And nothing was transferred.** The bus may be mid-use by the
    // application; an ISR that took it would be racing the transfer it
    // interrupted.
    try testing.expectEqual(writes_before, bus.data_writes);

    // The flag survives to whoever collects it, and collecting is destructive.
    try testing.expect(dev.takeHostWake());
    try testing.expect(!dev.takeHostWake());
}

test "host wake: enabling clears any flag left from a previous session" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);
    dev.onHostWakeIrq();
    dev.enableHostWake();

    // Otherwise the first `takeHostWake` after a re-enable reports a wake that
    // happened before anybody was listening, and the first poll of a fresh
    // session chases nothing.
    try testing.expect(!dev.takeHostWake());
    try testing.expect(bus.irq_armed);
}

test "host wake: re-arming is a no-op while disabled" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // Never enabled: `rearmHostWake` must not be the thing that switches the
    // interrupt on. A driver that armed here would deliver interrupts to an
    // application that has no handler for them, which on this chip is a fault
    // rather than a curiosity.
    dev.rearmHostWake();
    try testing.expect(!bus.irq_armed);
    try testing.expectEqual(@as(u32, 0), bus.irq_rearms);

    dev.enableHostWake();
    dev.onHostWakeIrq();
    dev.rearmHostWake();
    try testing.expect(bus.irq_armed);
    try testing.expectEqual(@as(u32, 1), bus.irq_rearms);

    // And disabling stays disabled through a re-arm.
    dev.disableHostWake();
    dev.rearmHostWake();
    try testing.expect(!bus.irq_armed);
}

test "an ARP request survives the round trip through both headers" {
    var bus = AssertingBus{};
    var dev = joinedDevice(&bus);

    // **Literal bytes, not a builder's output.** This test used to call
    // `net.buildArpRequest` for its payload, which made it agree with the
    // frame builder rather than with the wire — and when the IP stack moved to
    // `lib/ipnet` on 2026-08-31 the compiler said so, because an L1 driver
    // cannot import an L2 library. Spelling the 42 bytes out is what this
    // test was always meant to assert: **what this driver does to a frame is
    // wrap it in eighteen bytes and hand it back unaltered.** The payload
    // could be anything; that it is a valid ARP request is only so a failure
    // is readable in a capture.
    const mac = [6]u8{ 0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F };
    const gw_mac = [6]u8{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
    const arp_frame_len = 42;

    const req = [arp_frame_len]u8{
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // dest: broadcast
        0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F, // src: us
        0x08, 0x06, // ethertype: ARP
        0x00, 0x01, // hardware type: Ethernet
        0x08, 0x00, // protocol type: IPv4
        0x06, 0x04, // hardware / protocol address lengths
        0x00, 0x01, // opcode: request
        0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F, // sender hardware address
        192, 168, 86, 200, // sender protocol address
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // target hardware address: unknown
        192, 168, 86, 1, // target protocol address: the gateway
    };

    try dev.sendEthFrame(&req);
    try expectNoFault(&bus);

    // What the chip received, read back out from behind the headers.
    try testing.expectEqualSlices(u8, &req, bus.last_data[18..][0..arp_frame_len]);

    // And the reply the gateway would send, coming back the other way.
    const reply = [arp_frame_len]u8{
        0x88, 0xA2, 0x9E, 0x00, 0xDF, 0x8F, // dest: us
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, // src: the gateway
        0x08, 0x06, 0x00, 0x01, 0x08, 0x00,
        0x06, 0x04,
        0x00, 0x02, // opcode: reply
        0x11, 0x22,
        0x33, 0x44,
        0x55, 0x66,
        192,  168,
        86,   1,
        0x88, 0xA2,
        0x9E, 0x00,
        0xDF, 0x8F,
        192,  168,
        86,   200,
    };
    bus.queueEthFrame(&reply, 0);

    var out: [eth_max_frame]u8 = undefined;
    const n = try dev.pollEthFrame(&out);
    try testing.expectEqual(@as(usize, arp_frame_len), n);
    try testing.expectEqualSlices(u8, &reply, out[0..n]);
    // Spelled out again at the offsets that matter, so a failure names the
    // field rather than dumping two arrays.
    try testing.expectEqualSlices(u8, &gw_mac, out[22..28]);
    try testing.expectEqualSlices(u8, &[_]u8{ 192, 168, 86, 1 }, out[28..32]);
    try testing.expectEqualSlices(u8, &mac, out[32..38]);
}

test "a completed JoinState is bit-for-bit the SDK's WIFI_JOIN_STATE_ALL" {
    // 0x0E01 = ACTIVE | AUTH | LINK | KEYED. Asserting the packed layout against
    // the reference's own constant is what makes the bit positions checkable
    // rather than merely plausible — and the hardware publishes this word, so
    // the same number is readable over SWD after a real join.
    const all = JoinState{
        .kind = JoinState.kind_active,
        .auth = true,
        .link = true,
        .keyed = true,
    };
    try testing.expectEqual(@as(u16, 0x0E01), @as(u16, @bitCast(all)));
    try testing.expect(all.complete());
}

test "IOCTL command numbers match the SDK" {
    // Every one of these appears by name in a trace of the SDK's own join, which
    // is where three of them were found to be wrong. A number here that no
    // trace line corroborates is a number nobody has checked.
    try testing.expectEqual(@as(u32, 2), @backingInt(IoctlCmd.up));
    try testing.expectEqual(@as(u32, 20), @backingInt(IoctlCmd.set_infra));
    try testing.expectEqual(@as(u32, 22), @backingInt(IoctlCmd.set_auth));
    try testing.expectEqual(@as(u32, 26), @backingInt(IoctlCmd.set_ssid));
    try testing.expectEqual(@as(u32, 64), @backingInt(IoctlCmd.set_antdiv));
    try testing.expectEqual(@as(u32, 86), @backingInt(IoctlCmd.set_pm));
    try testing.expectEqual(@as(u32, 110), @backingInt(IoctlCmd.set_gmode));
    try testing.expectEqual(@as(u32, 134), @backingInt(IoctlCmd.set_wsec));
    try testing.expectEqual(@as(u32, 142), @backingInt(IoctlCmd.set_band));
    try testing.expectEqual(@as(u32, 165), @backingInt(IoctlCmd.set_wpa_auth));
    try testing.expectEqual(@as(u32, 262), @backingInt(IoctlCmd.get_var));
    try testing.expectEqual(@as(u32, 263), @backingInt(IoctlCmd.set_var));
    try testing.expectEqual(@as(u32, 268), @backingInt(IoctlCmd.set_passphrase));
}

test "core wrapper addresses sit behind the wrapper offset" {
    // 0x18002000 is SDIO_BASE_ADDRESS and controls no core at all — writing a
    // reset value there was the predecessor's entire core-reset sequence.
    try testing.expectEqual(@as(u32, 0x1810_3000), coreAddress(.wlan_arm));
    try testing.expectEqual(@as(u32, 0x1810_4000), coreAddress(.socram));
    try testing.expect(coreAddress(.wlan_arm) != sdio_base);
}
