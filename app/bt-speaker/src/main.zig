//! bt-speaker — a Bluetooth A2DP sink: pair, negotiate, decode, play.
//!
//! **An app rather than an example because it needs two libraries.**
//! `lib/cyw43` carries the Bluetooth stack and `lib/pcm5100` the I2S engine;
//! neither may depend on the other, so this is where they meet. It began life
//! as `lib/cyw43/example/bredr-pair` and moved here when it grew a DAC.
//!
//! Build it at 150 MHz — the default. At the crystal's 12 MHz an SBC decoder
//! does not fit, and raising clk_sys without `GspiBus.setSysHz` takes the gSPI
//! bus with it.
//!
//! Build with `-Dfw=combined`. `-Dsys-mhz` is accepted and irrelevant here —
//! pairing is not throughput — but it is wired through so this runs in the same
//! clock configuration an A2DP application will (rung 21d).
//!
//! ## What this adds to rung 21
//!
//! `bredr-probe` made the board discoverable and watched a phone page it, then
//! deliberately did nothing: the `Connection_Request` went unanswered and the
//! controller timed it out. This answers it.
//!
//! ```
//! Connection_Request        -> Accept_Connection_Request   (Command Status!)
//! Connection_Complete
//! Link_Key_Request          -> Link_Key_Request_Negative_Reply, if we have none
//! IO_Capability_Request     -> IO_Capability_Request_Reply (NoInputNoOutput)
//! IO_Capability_Response
//! User_Confirmation_Request -> User_Confirmation_Request_Reply
//! Simple_Pairing_Complete
//! Link_Key_Notification
//! Authentication_Complete
//! Encryption_Change                                        <- the finish line
//! ```
//!
//! **The evidence is the phone saying Paired**, next to `Encryption_Change`
//! with status 0 and encryption on. Either alone is weaker: the board's own
//! view could be a misparse, and a phone's dialog could be optimism.
//!
//! ## Why this is one event loop and not a sequence of calls
//!
//! **`Accept_Connection_Request` answers with Command Status, not Command
//! Complete**, as do `Authentication_Requested` and
//! `Set_Connection_Encryption`. `hci.Host.commandWithResult` matches only
//! Command Complete, so calling it here waits out its four hundred attempts and
//! reports `NoAnswer` for a command the controller accepted — a failure that
//! looks exactly like a controller ignoring us.
//!
//! Worse, that model is wrong in shape. Pairing is not request-and-answer: the
//! controller raises what it needs, in an order that depends on what the *phone*
//! does, and several of these events can arrive while a nested wait is looking
//! for something else — which would drop them. So commands are written directly
//! and everything arrives in one loop. That is also the shape the L2CAP and
//! AVDTP work above this will need.
//!
//! ## What this does not do
//!
//! **The link key is held in RAM and never written to flash**, so a power cycle
//! forgets the bond and the phone must be told to forget it too. Persistence is
//! a deliberate non-goal for this rung — it needs a flash-backed store and a
//! policy for eviction, and neither belongs in the rung that proves pairing
//! works at all.
//!
//! ## Rung 23a — the channel
//!
//! Rung 22 left the phone paired and talking to nobody: it opened L2CAP to SDP
//! twice, got silence, and terminated the link after about three minutes with
//! reason `0x13`. **That is what this answers.**
//!
//! ```
//! Information Request   -> Information Response   (basic mode, honestly)
//! Connection Request    -> Connection Response    + our Configure Request
//! Configure Request     -> Configure Response
//! Configure Response                              <- both directions done
//! ```
//!
//! **The pass condition is data arriving on the channel we opened**, which is
//! the phone speaking SDP to us rather than trying to reach it. `data_frames`
//! going non-zero is the whole claim; a channel that opens and then carries
//! nothing is not one.
//!
//! **Both directions configure independently and either side may go first**, so
//! our Configure Request goes out immediately after the Connection Response
//! rather than waiting for theirs. Waiting is a deadlock whenever the peer is
//! doing the same.
//!
//! ## Rung 23b — the record
//!
//! An SDP server answering `ServiceSearchAttributeRequest` with one AudioSink
//! record, built against a capture of btstack's
//! own sink on this board, taken during a run that ended in music.
//!
//! **The phone searches for UUID `0x0100`, L2CAP — not for AudioSink.** That
//! matches every service running over L2CAP and is how it enumerates the
//! device. A server answering only a search for `0x110B` is never found, and
//! would look exactly like no server at all. That is read off the trace, and it
//! is the single finding that paid for building the reference.
//!
//! **The pass condition is the phone asking for PSM `0x0019`** — `psms` high
//! half reading `0x0019`. It cannot do that until a record tells it this device
//! is a sink, so nothing weaker will do.
//!
//! Six channels now, not one. Rung 23a held a single channel and argued a table
//! was structure ahead of evidence; the first real peer opened a second SDP
//! channel two hundred milliseconds after the first and silently overwrote it.
//!
//! ## Rung 24 — the stream
//!
//! ```
//! DISCOVER              -> one endpoint: SEID 1, audio, sink
//! GET_ALL_CAPABILITIES  -> media transport, a media codec, delay reporting
//! SET_CONFIGURATION     -> the phone chooses; accept and record it
//! OPEN                  -> accept; the phone then opens a *second* 0x0019
//! START                 -> accept, and media begins
//! ```
//!
//! **Both AVDTP channels are on PSM `0x0019`**, and the only thing telling them
//! apart is order: the first is signalling, the second — opened after OPEN is
//! accepted — is media. `Channel.role` records which, so a media packet is
//! never handed to the signalling parser.
//!
//! **The pass condition is media arriving on that second channel**, counted
//! rather than decoded. A negotiation that completes and then carries nothing
//! is not a stream, for the same reason rung 23a claimed on data rather than on
//! a channel opening.
//!
//! **Every response echoes the command's transaction label.** A response
//! carrying its own is discarded by the peer and looks exactly like a command
//! nobody answered — the same failure shape as L2CAP's signalling identifier.
//!
//! ## What this still does not do
//!
//! **Nothing decodes SBC**, so nothing reaches the DAC. Rung 25. The stream's
//! shape is recorded from SET_CONFIGURATION, which is the phone stating what
//! the decoder will have to meet.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const config = @import("throughput_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const pcm5100 = @import("pcm5100");
const bond = @import("bond.zig");
const bredr = cyw43.bredr;
const hci = cyw43.hci;
const Dev = cyw43.Device(cyw43.PicogSpiBus);

const local_name = "zig-speaker";

const plan: ?clocks.Plan = if (config.sys_mhz == 0)
    board.clock_plan
else
    clocks.Plan.require(config.sys_mhz * 1_000_000);

const sys_hz: u32 = if (plan) |p| p.sys_hz else board.sys_hz;

const Watch = extern struct {
    magic: u32,
    /// Idle passes, written every loop. A heartbeat that ticks once every
    /// thirty seconds cannot tell idling from wedged; this one can.
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// **Repurposed while streaming**, because paging means no
    /// `connection_request` ever arrives and this stayed zero: the **worst gap
    /// between `serviceAudio` calls** in microseconds, low half, and **how many
    /// gaps exceeded one buffer period** in the high half. The phase that
    /// preceded the worst one is in `stream_bitpool`'s top byte.
    peer_lo: u32,
    conn_handle: u32,
    /// **The pairing ladder, one bit each.** A full pass reads `0xBF`, not
    /// `0xFF`: bit 6 is `Authentication_Complete`, which a responder never
    /// sees.
    ///
    ///     0 request  1 connected  2 io cap  3 user confirm
    ///     4 pairing complete  5 link key  6 auth complete  7 encrypted
    steps: u32,
    /// connection | pairing << 8 | encryption << 16. All zero is the pass.
    statuses: u32,
    events: u32,
    /// Disconnects | the last reason << 16. `0x13` is the phone choosing to
    /// leave; `0x08` would be a supervision timeout and mean something else.
    disconnects: u32,
    /// ACL packets | those no L2CAP frame could be read out of << 16.
    ///
    /// **The high half has sat at 4-5 every run** and is not explained. The
    /// description that names it lives in the `acl_fail_*` module variables
    /// rather than in a field here — this block is full at 0x80 by design, and
    /// that boundary is a workspace convention rather than this application's
    /// to spend. `report()` prints them.
    acl: u32,
    /// L2CAP signalling frames | connection requests << 16.
    sig_frames: u32,
    /// PSMs asked for: first | last << 16. `0x0019` in the high half is the
    /// phone asking for AVDTP.
    psms: u32,
    /// **The L2CAP ladder** in the low half: 0 info answered, 1 connect
    /// answered, 2 their config, 3 our config, 4 open. `0x1F` is a working
    /// channel. Channels currently open in the high half.
    l2cap_state: u32,
    /// **The shortest and longest gap between detected buffer flips**, in
    /// microseconds: shortest | longest << 16.
    ///
    /// A buffer is 256 frames at 44.1 kHz, so a real flip is 5,805 us after the
    /// last one, always. Two readings decide what is happening when the fill
    /// rate is wrong: **a shortest gap near 5,805 means every detection is a
    /// real flip and the DAC is simply consuming faster than it should; a
    /// shortest gap of tens of microseconds means `playingIndex` is reporting
    /// changes that did not happen**, and the fills are landing in a buffer the
    /// DMA is reading. The counters cannot tell those apart; this can.
    flip_us: u32,
    /// Signalling frames sent | refused << 16. **A non-zero high half means a
    /// reply was built and never went out**, which is how rung 23b's second bug
    /// was found.
    sig_sent: u32,
    /// `bt.read` failures | last error code << 16.
    read_errors: u32,

    // -- rung 24 -----------------------------------------------------------

    /// **The AVDTP ladder**, one bit each: 0 discover, 1 capabilities,
    /// 2 configured, 3 open, 4 started, 5 closed. `0x1F` is a running stream.
    /// Bit 29 a rejected configuration, 30 a fragmented message, 31 a header
    /// that would not parse.
    avdtp_state: u32,
    /// The negotiated stream, from SET_CONFIGURATION. **This is the phone
    /// telling us what the decoder has to meet**, not what we asked for.
    stream_rate: u32,
    /// channel mode | blocks << 8 | subbands << 16 | allocation << 24.
    stream_shape: u32,
    /// min bitpool | max << 8.
    stream_bitpool: u32,
    /// **Rung 24's pass condition.** Media packets arriving on the second
    /// `0x0019` channel — counted, not decoded; a decoder is rung 25.
    media_frames: u32,
    /// **Repurposed, and the old value was derivable anyway** — it was always
    /// `media_frames` times a near-constant payload. Now **missed buffer
    /// flips** in the low half and **worst microseconds inside `bt.read`** in
    /// the high half. The second decides whether the long passes are the bus
    /// or our own processing, which is the fork the fix turns on.
    media_bytes: u32,

    // -- rung 26: audio out ------------------------------------------------

    /// 0 I2S started, 1 amplifier on, 2 first buffer filled from the stream,
    /// 3 the ring has run dry at least once.
    audio_state: u32,
    /// PCM frames decoded, in stereo pairs.
    decoded: u32,
    /// Ring occupancy now | its high-water mark << 16, in stereo frames.
    /// **The high-water mark is the jitter the link actually delivered**, and
    /// the ring has to be bigger than it.
    ring_level: u32,
    /// **Worst microseconds for one loop pass | worst for one packet decode
    /// << 16.** Added because the stream stopped after twelve packets and every
    /// explanation for that was a guess. A buffer turns over every 5.8 ms and a
    /// packet arrives every 14.5 ms; either number crossing its budget says
    /// which.
    timing_us: u32,
    /// Buffers handed to the DMA.
    ///
    /// **A whole word each, and that is the point.** These were packed two to a
    /// word, and a buffer turns over 172 times a second — so sixteen bits wrap
    /// in about six minutes. Over one album they wrapped ten times and reported
    /// that 66 minutes of audio had produced 2.8 minutes of playback, which was
    /// arithmetic on rubbish. **Size a counter to how often it counts.**
    audio_fills: u32,
    /// **Frames** padded with silence because the ring was short, not buffers.
    /// The DAC ran ahead of the stream by this much, and counting frames rather
    /// than buffers makes it directly comparable with `audio_dropped` and
    /// directly convertible to drift: 44,100 frames is one second.
    audio_silent: u32,
    /// Frames the ring refused because it was full. The stream ran ahead of the
    /// DAC. **Previously uncounted**, which made a full ring and an empty one
    /// indistinguishable after the fact.
    audio_dropped: u32,

    // -- rung 25a ----------------------------------------------------------

    /// **Packets the framing did not explain**, one byte each:
    /// length mismatch | bad syncword << 8 | not RTP << 16 | fragmented << 24.
    /// All zero is rung 25a's pass — the frames tile every packet exactly.
    sbc_errors: u32,
    clk_sys_khz: u32,
};

pub const watch_magic: u32 = 0x4254_3232; // 'BT22'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "FAILED at rung {d}: {s}\n", .{ rung, @errorName(err) });
    watch.magic = 0xDEAD_4254;
    watch.rung = rung;
    watch.err_code = switch (err) {
        error.ChipNotResponding => 2,
        error.AlpTimeout => 4,
        error.CoreNotInReset => 5,
        error.HtClockTimeout => 6,
        error.F2NotReady => 7,
        error.BackplaneUnreliable => 13,
        error.BtNotReady => 40,
        error.QueueFull => 41,
        error.PacketTooLarge => 42,
        error.BadPatchram => 43,
        error.NoHeaderRoom => 44,
        error.NoHciAnswer => 45,
        error.CommandFailed => 46,
        error.NoAnswer => 47,
        error.WrongOpcode => 48,
        error.Malformed => 49,
        error.BufferTooSmall => 51,
        error.ClassicNotSupported => 60,
        else => 99,
    };
    while (true) timer.delayMs(1000);
}

/// **Six channels, because the reference trace shows a phone opening six**:
/// three to SDP, two to AVDTP and one to AVCTP. Rung 23a held a single channel
/// and argued a table was structure ahead of evidence; the first real peer
/// opened a second SDP channel two hundred milliseconds after the first and
/// silently overwrote it. The evidence arrived, so the table is here now.
/// What a channel on PSM 0x0019 turned out to be. **The first is signalling
/// and the second is media** — AVDTP puts both on the same PSM, and the only
/// thing distinguishing them is that the media channel is the one the phone
/// opens *after* OPEN is accepted.
const Role = enum { other, avdtp_signalling, avdtp_media };

const Channel = struct {
    local_cid: u16 = 0,
    remote_cid: u16 = 0,
    psm: u16 = 0,
    role: Role = .other,
    in_configured: bool = false,
    out_configured: bool = false,
    open: bool = false,
};

var chans: [6]Channel = @splat(.{});

fn channelByLocal(cid: u16) ?*Channel {
    for (&chans) |*c| {
        if (c.local_cid == cid and c.local_cid != 0) return c;
    }
    return null;
}

fn channelAlloc() ?*Channel {
    for (&chans) |*c| {
        if (c.local_cid == 0) return c;
    }
    return null;
}

fn channelByRole(r: Role) ?*Channel {
    for (&chans) |*c| {
        if (c.role == r and c.local_cid != 0) return c;
    }
    return null;
}

fn countOpen() u32 {
    var n: u32 = 0;
    for (chans) |c| {
        if (c.open) n += 1;
    }
    return n;
}

var next_local_cid: u16 = cyw43.bredr.first_dynamic_cid;

/// Our signalling identifier. **Every request we originate needs a fresh one**,
/// and the response echoes it; a response carrying an id the peer never sent is
/// discarded, which presents as a request that was never answered.
var next_sig_id: u8 = 1;

/// The controller's ACL buffers, which are not ours to overfill. `start` reads
/// the count and `Number_Of_Completed_Packets` gives them back — the example
/// has to do that itself, because it drives the bus directly rather than
/// through `Host`'s command path.
var acl_credits: u16 = 0;

/// The AVDTP ladder, one bit each. `0x1F` is a configured, open, started
/// stream.
const st = struct {
    const discover: u32 = 1 << 0;
    const capabilities: u32 = 1 << 1;
    const configured: u32 = 1 << 2;
    const opened: u32 = 1 << 3;
    const started: u32 = 1 << 4;
    const closed: u32 = 1 << 5;
};

const l2 = struct {
    const info_answered: u32 = 1 << 0;
    const conn_answered: u32 = 1 << 1;
    const in_configured: u32 = 1 << 2;
    const out_configured: u32 = 1 << 3;
    const open: u32 = 1 << 4;
    const all: u32 = 0x1F;
};

const step = struct {
    const request: u32 = 1 << 0;
    const connected: u32 = 1 << 1;
    const io_cap: u32 = 1 << 2;
    const user_confirm: u32 = 1 << 3;
    const pairing_complete: u32 = 1 << 4;
    const link_key: u32 = 1 << 5;
    const auth_complete: u32 = 1 << 6;
    const encrypted: u32 = 1 << 7;
    /// Everything a responder can see. See `Watch.steps`.
    const responder_pass: u32 = 0xBF;

    // -- rung 27: paging the phone -----------------------------------------
    //
    // **The high half of `steps`, because the watch block is full.** `Watch` is
    // exactly 0x80 bytes and `.faultinfo` begins at 0x20000080; a
    // thirty-third field would overrun it and the `comptime` guard below would
    // say so. These bits have been free since rung 22.
    const paged: u32 = 1 << 8;
    const page_answered: u32 = 1 << 9;
    const page_gave_up: u32 = 1 << 10;
    const we_authenticated: u32 = 1 << 11;
    /// Attempts made, in bits 16..19; the last page failure status in 20..27.
    fn recordPageFailure(status: u8) void {
        watch.steps = (watch.steps & ~@as(u32, 0x0FF0_0000)) |
            (@as(u32, status) << 20);
    }
};

// ---------------------------------------------------------------------------
// Paging: connecting to the phone rather than waiting for it
// ---------------------------------------------------------------------------

/// **Why this exists at all.** Every rung up to here made this device a
/// responder: it sits in page scan and answers whatever the phone starts. That
/// works, and it means the phone has to be told to connect — you go into the
/// saved-devices list and tap. A commercial speaker does not need that, and the
/// reason is not a cleverer profile, it is a reversed direction: **on power-up
/// it pages the phone at the address it stored when it paired**.
///
/// It can, because a phone stays *connectable* whenever Bluetooth is on even
/// though it is only *discoverable* for a couple of minutes at the pairing
/// screen. Page scan never stops. Anything holding the address can reach it.
///
/// **The retry policy is a decision, not a detail.** Paging a phone that is not
/// there costs power and airtime for as long as it goes on, so this tries hard
/// for about a minute after boot and then stops for good and goes back to
/// waiting. That is what a speaker does: the case it is optimising is "the
/// phone is in the room", and forever-retry buys nothing for the case where it
/// is not.
const paging = struct {
    /// About a minute all told, at a 10.24 s page timeout plus the gap.
    const max_attempts: u32 = 4;
    const gap_us: u64 = 3_000_000;

    var attempts: u32 = 0;
    var in_flight: bool = false;
    var next_us: u64 = 0;
    var done: bool = false;
    var peer: [6]u8 = @splat(0);
};

/// Stop paging, for good, for the given reason. Called when a connection exists
/// however it was made — **including one the phone started**, which is the race
/// this has to lose gracefully: both ends may reach for the link at once, and
/// two connections between the same pair is not a thing the controller allows.
fn stopPaging(why: []const u8) void {
    if (paging.done and !paging.in_flight) return;
    paging.done = true;
    paging.in_flight = false;
    uart.print(.uart0, "   paging stopped: {s}\n", .{why});
}

// -- roadmap 2.6: what a gSPI transfer actually costs -----------------------
//
// Reads that returned no packet, and the microseconds they took. An empty read
// is one index fetch and nothing else, so these two give the per-fetch cost
// directly. Packet reads are kept beside them because their difference is the
// payload cost, which is what a bus-divisor sweep (2.5) should move most.
//
// **Reset with the stream**, like every other audio number, so a report
// describes one stream rather than a boot.
// **`u64`, and not for elegance.** At 3 MHz these summed to 3.49e9 ns on a
// 110-second stream -- 81% of the way to overflowing `u32`, which a three-minute
// track would have crossed silently and reported as a fast bus.
var empty_reads: u32 = 0;
var empty_read_us: u64 = 0;
var packet_reads: u32 = 0;
var packet_read_us: u64 = 0;

/// **True only for the link we are on**, and the fix for a bug that read as a
/// board which stopped accepting its own phone.
///
/// Three decisions turn on "did *we* page this phone?", and all three used to
/// read `watch.steps & step.we_authenticated`. **`watch.steps` is a cumulative
/// record and is never cleared** — it exists to be read over SWD after the
/// fact, and every write to it is an `|=`. So the bit latched on the first
/// paged connection of a boot and stayed set for every connection afterwards,
/// including ones the phone initiated.
///
/// The visible symptom was the phone connecting and dropping back to "saved"
/// after about three seconds, on every reconnect after the first, cleared only
/// by a reflash — because a reflash is the only thing that clears RAM. On a
/// phone-initiated link the phone opens AVDTP itself; we opened one too, on the
/// strength of a stale bit, and the phone abandoned a link with two signalling
/// channels on PSM 0x0019.
///
/// **One variable was doing two jobs** — recording history and steering the
/// current link — and only the first survives a disconnect. They are separate
/// now.
var we_paged_this_link: bool = false;

// -- what the source actually encoded ---------------------------------------
//
// **The report was printing the negotiated ceiling, not the encoding.** Both a
// phone and a MacBook negotiate `bitpool 2-53`, identical in every field of
// `SET_CONFIGURATION` — and a source is then free to encode anywhere inside
// that range. The actual per-frame value was captured in `stream_bitpool`'s
// third byte and never printed, so "bitpool 53" in every report was the
// agreement, not the audio.
//
// **A range, not a sample.** `lib/cyw43/src/bt/sbc.zig` notes that a source may
// lower its bitpool mid-stream under interference, so one reading says almost
// nothing; the pair says whether the encoder is holding station or backing off.
//
// **Low bitpool in SBC is specifically treble loss** — fewer bits to allocate,
// so the top subbands get none — which is what "muffled, but no distortion and
// no burbling" sounds like. Channel mode is here for the same reason: joint
// stereo against plain stereo is a real difference at a given bitpool, and it
// is another field two host stacks can legitimately disagree on.
var sbc_bitpool_min: u8 = 0xFF;
var sbc_bitpool_max: u8 = 0;
var sbc_mode: ?cyw43.sbc.ChannelMode = null;

/// A bond that has been agreed but not yet committed to flash.
///
/// **Deferred deliberately**: writing it masks interrupts for tens of
/// milliseconds, which is long enough for the CYW43's Bluetooth firmware to
/// reset itself. Written on disconnect instead, where a stall costs nothing.
var pending_bond: ?struct { peer: [6]u8, key_type: u8, key: [16]u8 } = null;

/// **The controller's firmware was up last time we looked.** Edge-triggered so
/// the transition prints once rather than forty times a second.
var fw_was_ready: bool = true;
var last_fw_check_us: u64 = 0;
/// Backplane reads of `bt_ctrl` that errored. **Counted separately from the
/// bit's value**, because "the bus would not answer" and "the firmware says it
/// is down" are different faults.
var fw_reads_failed: u32 = 0;

/// Consecutive three-second idle prints with no new event. **The wedge
/// condition**: the loop running, `read` returning nothing and not erroring,
/// and the controller silent.
var quiet_passes: u32 = 0;
var last_traffic: u32 = 0;

/// Ten passes, so about thirty seconds — long enough that an ordinary quiet
/// link does not produce a wall of diagnostics, short enough that a wedge is
/// named while whoever caused it is still at the bench.
const quiet_passes_before_dump: u32 = 10;

/// **The health report reads the command-credit counters through this.**
/// `report()` lives in the `phase` struct and has no `host` in scope; a
/// pointer set once at start-up is cheaper than threading one through.
var host_ptr: ?*cyw43.BtHost(Dev) = null;
var rx: [hci.rx_buf_len]u8 = @splat(0);

/// The link key. **Loaded from flash at boot and written back when the
/// controller hands us a new one**, so a power cycle no longer forgets the
/// bond — see `bond.zig` for where it lives and why not at the end of flash.
var link_key: [16]u8 = @splat(0);
var link_key_type: u8 = 0;
var link_key_peer: [6]u8 = @splat(0);
var have_link_key: bool = false;

/// Page the phone, if it is time to and there is any point.
///
/// **Called only from the idle branch**, which is exactly right and worth
/// saying why: if the loop has traffic to handle then something is connected,
/// and paging while connected is at best wasted and at worst a second link the
/// controller will refuse.
fn servicePaging(host: *cyw43.BtHost(Dev)) void {
    if (paging.done or paging.in_flight) return;
    // **A link already exists, whatever the last attempt reported.** Run two
    // recorded attempts 1 and 2 with a failure status of `0x0B` — ACL
    // Connection Already Exists — so the retry raced a link that had come up.
    // Harmless, but it is a command sent into a state that cannot accept it.
    if ((watch.steps & step.connected) != 0) {
        stopPaging("a connection already exists");
        return;
    }
    if (timer.now64() < paging.next_us) return;

    if (paging.attempts >= paging.max_attempts) {
        watch.steps |= step.page_gave_up;
        stopPaging("no answer after every attempt -- waiting to be connected to instead");
        return;
    }

    paging.attempts += 1;
    paging.in_flight = true;
    watch.steps |= step.paged;
    watch.steps = (watch.steps & ~@as(u32, 0x000F_0000)) |
        ((paging.attempts & 0xF) << 16);

    uart.print(.uart0, "\n>> PAGING ", .{});
    addrOf(paging.peer);
    uart.print(.uart0, " (attempt {d} of {d})\n", .{ paging.attempts, paging.max_attempts });

    // Command Status, not Command Complete — `Connection_Complete` is the
    // answer and it arrives in the loop like everything else.
    const p = bredr.createConnectionParams(paging.peer, true);
    send(host, bredr.op.create_connection, &p);
}

/// Write a command and do **not** wait for its answer — whatever it is arrives
/// in the loop like everything else.
/// **Posts rather than writes, since 2026-08-27.** This used to build a packet
/// and put it straight on the bus, which ignored `Num_HCI_Command_Packets`
/// entirely — the allowance the controller states in every Command Complete
/// and Command Status, and which the spec says starts at **one**. The paging
/// path posts three commands back to back, so two of them were routinely sent
/// past that allowance. `postCommand` queues what will not fit and
/// `creditsFrom` releases it.
///
/// Failures are counted in `host.cmd_dropped` rather than printed: this is
/// called from paths where a UART burst is the last thing wanted, and the
/// health report prints the count.
fn send(host: *cyw43.BtHost(Dev), opcode: u16, params: []const u8) void {
    host.postCommand(opcode, params);
}

/// Fire-and-forget, like `send`, and for the same reason: both call sites are
/// inside the event loop, where a blocking `host.command` would wait on a
/// completion that this loop is the one responsible for reading. A refusal is
/// not lost — the `command_complete` arm prints any non-zero status with its
/// opcode.
///
/// Idempotent against what was **asked for**, not against what was confirmed:
/// re-sending because a completion has not landed yet would send two.  The
/// disconnect path also runs for links that never streamed, so the `both` there
/// is usually a no-op.
fn setScanEnable(host: *cyw43.BtHost(Dev), mode: bredr.ScanEnable) void {
    if (scan_requested == mode) return;
    scan_requested = mode;
    send(host, bredr.op.write_scan_enable, &.{@backingInt(mode)});
    uart.print(.uart0, "   scan -> {s}\n", .{@tagName(mode)});
}

/// The largest payload this example ever hands to L2CAP — an SDP response
/// carrying every record that matched.
const max_payload = 512;

/// **Sized to the largest thing that can be sent, not to the largest seen.**
/// These were 112 and 128, which is ample for signalling and one byte short of
/// an SDP response: the reply to the phone's L2CAP search is about 109 bytes,
/// L2CAP adds four, and `l2cap.build` returned `BufferTooSmall`. The response
/// was never sent, the phone waited for a record it never got, and the only
/// trace of it was a `1` in the high half of `sig_sent`.
///
/// **That is the third buffer in this package sized to the traffic in front of
/// the author** — after `hci.Host.buf` at 64 bytes and `hci.Host.rx` at 260 —
/// and all three were invisible until a peer sent something bigger. The rule
/// that keeps coming back: a buffer's size is a property of the protocol, not
/// of the messages that have happened to arrive so far.
var sig_buf: [max_payload]u8 = @splat(0);
var l2_buf: [max_payload + 8]u8 = @splat(0);
var acl_buf: [max_payload + 16]u8 = @splat(0);

/// Wrap a signalling payload in an L2CAP frame and an ACL packet, and write it.
///
/// **Three headers, innermost first**: signalling (code, id, length), then
/// L2CAP (length, CID), then HCI ACL (handle, boundary, length). Each length
/// counts only what follows it, which is the rule `l2cap.zig` exists to state.
fn sendSig(host: *cyw43.BtHost(Dev), code: u8, id: u8, payload: []const u8) void {
    phase.set(phase.signalling);
    if (acl_credits == 0) {
        // Better to drop and count than to overrun the controller's buffers,
        // which fails in a way that looks like a radio problem.
        watch.sig_sent +%= 1 << 16;
        return;
    }
    const frame = cyw43.bredr.buildSignalling(code, id, payload, &sig_buf) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    const l2f = cyw43.l2cap.build(cyw43.bredr.l2cap_cid.signalling, frame, &l2_buf) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    const pkt = hci.buildAcl(@truncate(watch.conn_handle), .first_flushable, l2f, &acl_buf) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    host.bt.write(pkt) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    acl_credits -= 1;
    watch.sig_sent +%= 1;
}

/// Look inside an ACL packet far enough to say what the phone is asking for.
///
/// **Reconnaissance, not a protocol.** Nothing here answers anything; the point
/// is to turn "it disconnected" into "the phone asked for these PSMs and got
/// silence", which is the measurement rung 23 needs before a line of L2CAP is
/// written.
/// **Two records, and the second one is not optional.** The phone's first
/// query is a search for Device ID (`0x1200`); answering it with an empty list
/// stops discovery dead, and the broad L2CAP search that finds the sink is
/// never sent. Rung 23b shipped without it and paired perfectly while remaining
/// a device with no services.
var sdp_sink: [200]u8 = @splat(0);
var sdp_sink_len: usize = 0;
var sdp_devid: [128]u8 = @splat(0);
var sdp_devid_len: usize = 0;
var sdp_rsp: [max_payload]u8 = @splat(0);
var sdp_hit: [2][]const u8 = undefined;

/// Send an L2CAP data frame on an open channel.
fn sendData(host: *cyw43.BtHost(Dev), cid: u16, payload: []const u8) void {
    if (acl_credits == 0) {
        watch.sig_sent +%= 1 << 16;
        return;
    }
    const l2f = cyw43.l2cap.build(cid, payload, &l2_buf) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    const p = hci.buildAcl(@truncate(watch.conn_handle), .first_flushable, l2f, &acl_buf) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    host.bt.write(p) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    acl_credits -= 1;
}

/// Answer one SDP PDU.
///
/// **The record is built once at start-up, not per request.** It cannot change,
/// and rebuilding it inside the response path would put a failure that can only
/// happen once into a place that runs many times.
fn handleSdp(host: *cyw43.BtHost(Dev), cid: u16, payload: []const u8) void {
    phase.set(phase.sdp);
    const sdp = cyw43.sdp;

    const hdr = sdp.parseHeader(payload) catch {
        uart.print(.uart0, "   ! malformed SDP PDU\n", .{});
        return;
    };

    // **Three PDUs, because two devices ask differently.** Android sends the
    // combined `0x06`; macOS sends the two-step `0x02` then `0x04`. Answering
    // only `0x06` meant a Mac opened SDP, was told `invalid_syntax`, closed the
    // channel and dropped the link -- which presents as "it appears in the
    // connecting box and will not connect". Found 2026-08-27.
    switch (hdr.id) {
        sdp.pdu.service_search_request => return handleSdpSearch(host, cid, hdr),
        sdp.pdu.service_attribute_request => return handleSdpAttribute(host, cid, hdr),
        sdp.pdu.service_search_attribute_request => {},
        else => {
            // Answered rather than ignored: silence is what a broken server
            // looks like.
            uart.print(.uart0, "   SDP pdu 0x{X:0>2} unsupported -- error response\n", .{hdr.id});
            const r = sdp.writeErrorResponse(&sdp_rsp, hdr.transaction, sdp.error_code.invalid_syntax) catch return;
            sendData(host, cid, r);
            return;
        },
    }

    const req = sdp.parseSearchAttributeRequest(hdr.params) catch {
        uart.print(.uart0, "   ! malformed search request\n", .{});
        const r = sdp.writeErrorResponse(&sdp_rsp, hdr.transaction, sdp.error_code.invalid_syntax) catch return;
        sendData(host, cid, r);
        return;
    };

    // Every record whose UUIDs satisfy the pattern, in one response — which is
    // what the reference does for a broad search.
    var n: usize = 0;
    const pattern = req.pattern[0..req.pattern_len];
    if (!req.pattern_truncated) {
        if (sdp.matches(pattern, &sdp.device_id_uuids)) {
            sdp_hit[n] = sdp_devid[0..sdp_devid_len];
            n += 1;
        }
        if (sdp.matches(pattern, &sdp.audio_sink_uuids)) {
            sdp_hit[n] = sdp_sink[0..sdp_sink_len];
            n += 1;
        }
    }

    // Record what was asked before deciding anything about it.

    const rsp = if (n > 0)
        sdp.writeSearchAttributeResponse(&sdp_rsp, hdr.transaction, sdp_hit[0..n], req.max_attribute_bytes) catch {
            uart.print(.uart0, "   ! response would not fit\n", .{});
            return;
        }
    else
        sdp.writeEmptyResponse(&sdp_rsp, hdr.transaction) catch return;

    uart.print(.uart0, "   SDP search 0x{X:0>4} -> {d} record(s), {d} bytes\n", .{
        if (req.pattern_len > 0) req.pattern[0] else 0, n, rsp.len,
    });
    sendData(host, cid, rsp);
    if (n > 0) watch.rung = 0x23B;
}

/// `ServiceSearchRequest` (0x02): which records match, by handle.
///
/// The handles come out of the records themselves, so the number handed out
/// here is exactly the one `handleSdpAttribute` will look up.
fn handleSdpSearch(host: *cyw43.BtHost(Dev), cid: u16, hdr: cyw43.sdp.Header) void {
    const sdp = cyw43.sdp;
    const req = sdp.parseSearchRequest(hdr.params) catch {
        uart.print(.uart0, "   ! malformed service search\n", .{});
        const r = sdp.writeErrorResponse(&sdp_rsp, hdr.transaction, sdp.error_code.invalid_syntax) catch return;
        sendData(host, cid, r);
        return;
    };

    var handles: [2]u32 = undefined;
    var n: usize = 0;
    const pattern = req.pattern[0..req.pattern_len];
    if (!req.pattern_truncated) {
        if (sdp.matches(pattern, &sdp.device_id_uuids)) {
            if (sdp.recordHandle(sdp_devid[0..sdp_devid_len])) |h| {
                handles[n] = h;
                n += 1;
            }
        }
        if (sdp.matches(pattern, &sdp.audio_sink_uuids)) {
            if (sdp.recordHandle(sdp_sink[0..sdp_sink_len])) |h| {
                handles[n] = h;
                n += 1;
            }
        }
    }
    // **The client's cap is an upper bound, not a request for that many.**
    if (req.max_records != 0 and n > req.max_records) n = req.max_records;

    const rsp = sdp.writeSearchResponse(&sdp_rsp, hdr.transaction, handles[0..n]) catch {
        uart.print(.uart0, "   ! search response would not fit\n", .{});
        return;
    };
    uart.print(.uart0, "   SDP search 0x{X:0>4} -> {d} handle(s)\n", .{
        if (req.pattern_len > 0) req.pattern[0] else 0, n,
    });
    sendData(host, cid, rsp);
    if (n > 0) watch.rung = 0x23B;
}

/// `ServiceAttributeRequest` (0x04): one record, by the handle `0x02` gave out.
fn handleSdpAttribute(host: *cyw43.BtHost(Dev), cid: u16, hdr: cyw43.sdp.Header) void {
    const sdp = cyw43.sdp;
    const req = sdp.parseAttributeRequest(hdr.params) catch {
        uart.print(.uart0, "   ! malformed attribute request\n", .{});
        const r = sdp.writeErrorResponse(&sdp_rsp, hdr.transaction, sdp.error_code.invalid_syntax) catch return;
        sendData(host, cid, r);
        return;
    };

    const record: ?[]const u8 = blk: {
        const sink = sdp_sink[0..sdp_sink_len];
        const dev = sdp_devid[0..sdp_devid_len];
        if (sdp.recordHandle(sink)) |h| if (h == req.handle) break :blk sink;
        if (sdp.recordHandle(dev)) |h| if (h == req.handle) break :blk dev;
        break :blk null;
    };

    const r = record orelse {
        // **A specific error, not the generic one.** "No such handle" tells a
        // client its handle is stale; `invalid_syntax` tells it we are broken.
        uart.print(.uart0, "   SDP attributes for unknown handle 0x{X:0>8}\n", .{req.handle});
        const e = sdp.writeErrorResponse(&sdp_rsp, hdr.transaction, sdp.error_code.invalid_record_handle) catch return;
        sendData(host, cid, e);
        return;
    };

    const rsp = sdp.writeAttributeResponse(&sdp_rsp, hdr.transaction, r, req.max_attribute_bytes) catch {
        uart.print(.uart0, "   ! attribute response would not fit ({d} bytes, cap {d})\n", .{
            r.len, req.max_attribute_bytes,
        });
        return;
    };
    uart.print(.uart0, "   SDP attributes for 0x{X:0>8} -> {d} bytes\n", .{ req.handle, rsp.len });
    sendData(host, cid, rsp);
    watch.rung = 0x23B;
}

/// Everything the controller must be told that it forgets on a restart.
///
/// **Factored out because the firmware really does restart.** `bond.save`'s
/// flash erase masks interrupts for tens of milliseconds; the host stops
/// servicing the gSPI interface for that long and the CYW43's Bluetooth
/// firmware resets itself. Measured 2026-08-27: `fw_ready` drops in the poll
/// immediately after the erase, with both rings drained and command credits
/// healthy — so the stall, not any packet. A disconnect that writes no bond
/// does not do it; the same disconnect with a bond write does. That is the
/// whole experiment.
///
/// A restarted firmware has forgotten its buffer sizes, its name, its class,
/// simple-pairing mode, its page timeout and its scan state, and it no longer
/// agrees with our ring indices. Everything here is what boot sets; calling it
/// twice is the point.
fn configureController(host: *cyw43.BtHost(Dev)) !void {
    const bsp = try host.commandWithResult(bredr.op.read_buffer_size, &.{});
    const bs = try bredr.parseBufferSize(bsp);
    _ = host.useClassicBuffers(bs.acl_len, bs.acl_count);
    acl_credits = bs.acl_count;

    const cod = bredr.classOfDeviceParams(bredr.cod_loudspeaker);
    try host.command(bredr.op.write_class_of_device, &cod);
    const name_params = try bredr.localNameParams(local_name);
    try host.command(bredr.op.write_local_name, &name_params);
    try host.command(bredr.op.write_simple_pairing_mode, &.{0x01});
    try host.command(
        bredr.op.write_page_timeout,
        &bredr.pageTimeoutParams(bredr.page_timeout_generous),
    );
    try host.command(
        bredr.op.write_scan_enable,
        &.{@backingInt(bredr.ScanEnable.both)},
    );
    // `host.command` blocks on the completion and checks its status, so these
    // are confirmed by having returned at all.
    scan_requested = .both;
    scan_mode = .both;
    return;
}

/// Write a pending bond, then put the controller back together.
///
/// **The erase is known to stun the controller, so the restart is handled
/// rather than hoped against.** Deferring the write out of pairing (where it
/// broke the session outright) only moved the damage to the disconnect, where
/// it left the board deaf until a power cycle. This is the half that was
/// missing: bring the stack back up and re-apply everything a restarted
/// firmware has forgotten.
///
/// Called only from `disconnection_complete`, where there is no link to lose
/// and no stream to interrupt.
fn commitPendingBond(host: *cyw43.BtHost(Dev)) void {
    const b = pending_bond orelse return;
    pending_bond = null;

    if (bond.save(b.peer, b.key_type, b.key)) {
        watch.audio_state |= bond_saved;
        uart.print(.uart0, "   bond written to flash\n", .{});
    } else |err| {
        uart.print(.uart0, "   ! bond flash write failed: {s}\n", .{@errorName(err)});
    }

    // **Reboot, because the controller cannot be restarted in place.**
    //
    // The erase runs before any failure is known, so the controller has been
    // stunned either way, and it does not come back on its own. Restarting it
    // from here was tried and does not work: `bt.start` succeeds — patchram
    // re-downloaded, indices zeroed, `fw_ready` set, `bt_ctrl 01000100` — and
    // then `HCI_Reset` returns `NoAnswer`, six attempts over 500 ms included.
    //
    // The reason it cannot work is that `bt2wlan_pwrup_wake` *powers up* the
    // BT core, which is a no-op on a core already running. There is no
    // operation available that makes it re-execute from the start, which is
    // also why the vendor's `cybt_shared_bus_driver.c` has no restart path at
    // all. Re-downloading patchram underneath a running core is not a restart.
    //
    // So: take the one path known to bring this chip up correctly. The bond is
    // on flash by now, so the board comes back, loads it, and pages the peer.
    //
    // **This only ever happens after a first pairing** — `pending_bond` is null
    // on every ordinary disconnect — and it happens at the moment the user has
    // just walked away.
    _ = host;
    uart.print(.uart0, "   rebooting: the erase stops the controller and it cannot be restarted\n", .{});
    uart.print(.uart0, "   (this happens once, after a first pairing)\n\n", .{});
    // Let the UART drain before the reset takes the world away.
    timer.delayMs(50);
    rp2350.watchdog.trigger();
}

var avdtp_buf: [64]u8 = @splat(0);

/// The negotiated stream, once SET_CONFIGURATION has named it.
var stream: ?cyw43.avdtp.SbcConfig = null;

/// Answer one AVDTP signalling message.
///
/// **Every response echoes the command's transaction label**, which is why each
/// branch passes `h.transaction` rather than a counter of its own. A response
/// carrying the wrong label is discarded by the peer and looks exactly like a
/// command that was never answered.
fn handleAvdtp(host: *cyw43.BtHost(Dev), cid: u16, payload: []const u8) void {
    const av = cyw43.avdtp;

    const h = av.parseHeader(payload) catch |err| {
        watch.avdtp_state |= if (err == av.Error.Fragmented) @as(u32, 1) << 30 else @as(u32, 1) << 31;
        uart.print(.uart0, "   ! avdtp header: {s}\n", .{@errorName(err)});
        return;
    };
    if (h.message_type != av.message.command) return;

    switch (h.signal) {
        av.sig.discover => {
            const info = av.endpointInfo(av.local_seid, stream != null, true);
            reply(host, cid, h.transaction, h.signal, true, &info);
            watch.avdtp_state |= st.discover;
            // **The in-use flag is printed because its absence hid a bug.** A
            // reconnect showed `DISCOVER -> seid 1, audio sink` and the phone
            // leaving immediately after, with nothing to say the endpoint had
            // been advertised as busy.
            uart.print(.uart0, "   AVDTP DISCOVER -> seid {d}, audio sink, {s}\n", .{
                av.local_seid,
                if (stream != null) "IN USE" else "free",
            });
        },

        av.sig.get_capabilities, av.sig.get_all_capabilities => {
            var caps: [32]u8 = @splat(0);
            const c = av.writeCapabilities(&caps) catch return;
            reply(host, cid, h.transaction, h.signal, true, c);
            watch.avdtp_state |= st.capabilities;
            uart.print(.uart0, "   AVDTP GET_CAPABILITIES -> {d} bytes, 44.1 kHz only\n", .{c.len});
        },

        av.sig.set_configuration => {
            // Two SEIDs first — ours, then theirs — and the configuration
            // block after.
            if (h.payload.len < 2) return;
            const cfg = av.findSbcConfig(h.payload[2..]) catch |err| {
                // **Rejected, not ignored.** A reject names the category that
                // was unacceptable; silence would stall the phone until it
                // gave up, and look like a device that had crashed.
                watch.avdtp_state |= 1 << 29;
                uart.print(.uart0, "   ! AVDTP config rejected: {s}\n", .{@errorName(err)});
                reply(host, cid, h.transaction, h.signal, false, &.{ cyw43.avdtp.category.media_codec, 0x00 });
                return;
            };
            stream = cfg;
            watch.avdtp_state |= st.configured;
            watch.stream_rate = cfg.sample_rate;
            watch.stream_shape = @as(u32, cfg.channel_mode) |
                (@as(u32, cfg.blocks) << 8) |
                (@as(u32, cfg.subbands) << 16) |
                (@as(u32, cfg.allocation) << 24);
            watch.stream_bitpool = @as(u32, cfg.min_bitpool) | (@as(u32, cfg.max_bitpool) << 8);
            reply(host, cid, h.transaction, h.signal, true, &.{});
            uart.print(.uart0, "   AVDTP SET_CONFIGURATION -> {d} Hz, {d} ch, {d} blocks, {d} subbands, bitpool {d}-{d}\n", .{
                cfg.sample_rate, cfg.channels(), cfg.blocks, cfg.subbands, cfg.min_bitpool, cfg.max_bitpool,
            });
        },

        av.sig.get_configuration => {
            var caps: [32]u8 = @splat(0);
            const c = av.writeCapabilities(&caps) catch return;
            reply(host, cid, h.transaction, h.signal, true, c);
        },

        av.sig.open => {
            reply(host, cid, h.transaction, h.signal, true, &.{});
            watch.avdtp_state |= st.opened;
            uart.print(.uart0, "   AVDTP OPEN -> accepted; expecting a media channel\n", .{});
        },

        av.sig.start => {
            reply(host, cid, h.transaction, h.signal, true, &.{});
            watch.avdtp_state |= st.started;
            // Enable the amplifier now, while the buffers still hold zeros,
            // and ramp up behind it. See `gate`.
            gate.unmute();

            // **"This stream" has to mean this stream.** `media_frames` is only
            // ever incremented, so the reset it gates -- `media_frames == 1`,
            // which zeroes every audio counter and the phase table -- fired
            // once per *boot*, not once per stream. Every report after the
            // first therefore described everything since the first media
            // packet ever seen, silently folding in the idle time between
            // streams. It is why a report taken after a reconnect blip carried
            // a `worst media gap` of 27.7 seconds: the blip and the listening
            // were one stream as far as the counters were concerned.
            //
            // Zeroing it here makes the next media packet re-run that reset, so
            // the existing block stays the single place that knows what "a
            // stream" resets -- and START is exactly the phone saying a new one
            // begins.
            watch.media_frames = 0;
            uart.print(.uart0, "   AVDTP START -> accepted; media should follow\n", .{});
        },

        av.sig.close, av.sig.abort => {
            reply(host, cid, h.transaction, h.signal, true, &.{});
            gate.mute();
            stream = null;
            watch.avdtp_state |= st.closed;
            uart.print(.uart0, "   AVDTP {s}\n", .{if (h.signal == av.sig.close) "CLOSE" else "ABORT"});
        },

        av.sig.suspend_ => {
            reply(host, cid, h.transaction, h.signal, true, &.{});
            // **The hook this was always waiting for.** SUSPEND is the phone
            // saying the music has stopped, which is exactly the moment the
            // stream would otherwise end mid-waveform against silence.
            gate.mute();
            uart.print(.uart0, "   AVDTP SUSPEND -- fading out\n", .{});
            phase.report();
        },

        av.sig.delay_report => {
            reply(host, cid, h.transaction, h.signal, true, &.{});
        },

        else => {
            // **Answered, not dropped.** An unanswered command stalls the
            // phone; a reject tells it to move on.
            uart.print(.uart0, "   AVDTP signal 0x{X:0>2} -- rejected\n", .{h.signal});
            reply(host, cid, h.transaction, h.signal, false, &.{0x00});
        },
    }
}

fn reply(
    host: *cyw43.BtHost(Dev),
    cid: u16,
    transaction: u8,
    signal: u8,
    accept: bool,
    payload: []const u8,
) void {
    const msg = cyw43.avdtp.buildResponse(&avdtp_buf, transaction, signal, accept, payload) catch {
        watch.sig_sent +%= 1 << 16;
        return;
    };
    sendData(host, cid, msg);
}

// ---------------------------------------------------------------------------
// Rung 26 — the audio path
// ---------------------------------------------------------------------------
//
// **The two rates do not divide.** A media packet carries 640 stereo frames and
// arrives about every 14.5 ms; the I2S ping-pong consumes 256 at a time, every
// 5.8 ms. Nothing lines up, and Bluetooth delivers in bursts besides — so a
// ring sits between them, and its high-water mark is the jitter the link
// actually produced rather than the jitter it was assumed to have.
//
// 8192 frames is about 186 ms. Generous on a 512 KB part, and generosity here
// is cheap next to the alternative: a ring that runs dry pads with silence, and
// silence in the middle of a track is the most audible failure this project
// can produce.

const ring_frames = 8192;

var ring: struct {
    data: [ring_frames]u32 = @splat(0),
    head: usize = 0,
    tail: usize = 0,

    fn level(self: *const @This()) usize {
        return (self.head -% self.tail) % ring_frames;
    }

    fn push(self: *@This(), v: u32) void {
        const next = (self.head + 1) % ring_frames;
        // **Drop the newest, not the oldest.** Overwriting the tail would hand
        // the DMA a discontinuity it is already reading through. Counted, so a
        // full ring and an empty one are told apart afterwards.
        if (next == self.tail) {
            watch.audio_dropped +%= 1;
            // **A burst, not a frame.** 6,913 dropped frames is one glitch if
            // they went in a single run and hundreds of glitches if they did
            // not, and those are different faults with different fixes. The
            // frame count cannot tell them apart; this can.
            if (!dropping) {
                dropping = true;
                const n = (watch.flip_us >> 16) +% 1;
                const capped: u32 = @min(n, 0xFFFF);
                watch.flip_us = (watch.flip_us & 0xFFFF) | (capped << 16);
            }
            return;
        }
        dropping = false;
        self.data[self.head] = v;
        self.head = next;
    }

    fn pop(self: *@This()) ?u32 {
        if (self.tail == self.head) return null;
        const v = self.data[self.tail];
        self.tail = (self.tail + 1) % ring_frames;
        return v;
    }
} = .{};

var audio: pcm5100.PicoAudio = undefined;
var decoder = cyw43.sbc_decode.Decoder{};
var last_playing: usize = 0;
var last_flip: u64 = 0;

/// When the last media packet arrived, for the inter-arrival gap.
var last_media_us: u64 = 0;

/// Worst microseconds between two **fills**. `peer_lo` holds the worst gap
/// between **calls**; a call that finds no change does not fill, so the two
/// differ and only the first can explain a missed flip.
var worst_fill_gap_us: u32 = 0;

// -- the ACL parse failures, answered 2026-08-27 ---------------------------
//
// `watch.acl`'s high half sat at 4-5 every run since rung 23a and nothing
// explained it. The field added by roadmap 0.7 named it in one trace:
//
//     acl parse fail: hci 0, l2cap 4
//     first: boundary 0b01, declared 0, actual 0
//
// **Empty ACL continuation fragments.** `hci 0` says `parseAcl` never failed;
// all four were rejected at the L2CAP stage, correctly, because a continuation
// carries no L2CAP header — and these carried no payload at all, so there was
// nothing there to parse under any reading. (`declared 0` is not independent
// evidence: `acl_fail_declared` is written 0 whenever `payload.len < 2`, which
// an empty payload always is. `actual` is the datum.)
//
// **The same four are the "four unaccounted packets"** of the cyw43
// bring-up. `watch.acl`'s low half is incremented for every ACL
// packet before `handleAcl` runs, so a packet that yields no L2CAP frame is
// counted as ACL and never as signalling: rung 22's `6 ACL, 2 sig_frames` is
// `6 = 2 + 4`. Two counters, one set of packets.
//
// **These should now read zero, and that is the point.** Both shapes they were
// written to describe are handled upstream by `reasm`: a continuation never
// reaches `l2cap.parse` at all, and `declared > actual` on a first fragment is
// ordinary fragmentation rather than the "truncated frame, a different fault"
// this comment used to call it. That reading was wrong — see the reassembly
// note below.
//
// So if `acl_fail_l2cap` moves again it is something genuinely new: a frame
// that was whole, parsed as ACL, and still made no sense as L2CAP. Read
// `reasm.orphans` and `reasm.overflows` first — a fault there explains a
// failure here, and the reverse is not true.
//
// **Deliberately not in the watch block**: that is full at 0x80 by design, and
// `mdw 0x20000080` is a workspace-wide convention rather than this
// application's to spend. These are investigative, not permanent.
var acl_fail_seen: bool = false;
var acl_fail_boundary: hci.Boundary = .first_non_flushable;
var acl_fail_declared: u16 = 0;
var acl_fail_actual: u16 = 0;
var acl_fail_hci: u32 = 0;
var acl_fail_l2cap: u32 = 0;

// -- ACL reassembly ---------------------------------------------------------
//
// **The counters that replaced this said to write it, within the hour.** The
// first version merely counted continuations and dropped them, on the reading
// that they were all empty and nothing fragmented. The first real soak said
// otherwise: 669 continuations carrying 196,546 bytes, 665 first fragments
// declaring 608 bytes while holding 83, and 435,328 frames of silence padded
// into the stream — which is within 2% of the media those lost PDUs would have
// decoded to. That was the burbling.
//
// **Fragmentation only appears when the link degrades**, which is why every
// earlier run looked clean: the same phone on a good link sends every media
// packet whole. A defect that hides while the radio is healthy and destroys
// audio when it is not is exactly the shape that survives casual testing.
//
// Sized from the MTU this device advertises, not from any packet seen.
var reasm: hci.Reassembler(bredr.local_mtu + cyw43.l2cap.header_len) = .{};

// -- inquiry scan while streaming -------------------------------------------
//
// `ScanEnable.both` was written once at boot and never changed, so the
// controller ran **inquiry scan** for the whole of every stream — periodically
// leaving the audio link to listen for devices wanting to pair. That is a
// known cause of A2DP stuttering, and once a bond exists it buys nothing:
// there is nobody left to be discovered by.
//
// So `page` while connected — still reachable, in particular by a phone coming
// back after an ungraceful drop, which leaves no `disconnection_complete`
// behind — and `both` again on disconnect, so an unfamiliar phone can still
// find this board.
//
// **This is the one candidate raised in the pops investigation that was never
// tried**, and the only one that would explain the run-to-run variance: what an
// inquiry scan costs depends on what else is on 2.4 GHz.
//
// **Two variables, because the report has to be able to be believed.** Nothing
// here honours `Num_HCI_Command_Packets` — `hci.Host` tracks ACL credits and
// not command ones — so a command sent from the event loop can in principle go
// nowhere, and the paging path now sends three in a row. An optimistic
// `scan_mode` would then name an arm the controller never entered, and the one
// thing this change exists to produce is a trustworthy A/B. So `requested` is
// what was asked for and `scan_mode` is what `command_complete` confirmed; the
// two differ only if the write was refused or lost, and `report()` says so.
var scan_requested: bredr.ScanEnable = .none;
var scan_mode: bredr.ScanEnable = .none;

/// True while `ring.push` is refusing frames, so a run of refusals counts once.
var dropping: bool = false;

/// Set once a bond has been written to flash this session.
const bond_saved: u32 = 1 << 4;

const audio_state = struct {
    const started: u32 = 1 << 0;
    const amp_on: u32 = 1 << 1;
    const streaming: u32 = 1 << 2;
    const ran_dry: u32 = 1 << 3;
    // bit 4 is `bond_saved`, declared above.

    /// **Ring-dry *events*, bits 8..19, saturating at 4095.** `audio_silent`
    /// counts frames, which cannot tell one long gap from a thousand short
    /// ones — and those sound completely different: a gap is a dropout, a
    /// thousand short ones is a burble. Two numbers are needed and there was
    /// only ever one.
    const dry_shift: u5 = 8;
    /// **Late fills, bits 20..31, saturating at 4095.** The documented
    /// detector from `lib/pcm5100`: after filling a buffer, ask whether the DMA
    /// has already reached it. If it has, the fill was late and what played was
    /// half-stale — **and nothing else registers that**, because the DMA always
    /// has a buffer to read and the FIFO never stalls.
    const late_shift: u5 = 20;
    const counter_max: u32 = 0xFFF;

    fn bump(shift: u5) void {
        const cur = (watch.audio_state >> shift) & counter_max;
        if (cur == counter_max) return;
        watch.audio_state = (watch.audio_state & ~(counter_max << shift)) |
            ((cur + 1) << shift);
    }
};

// ---------------------------------------------------------------------------
// The gate: fading, then muting
// ---------------------------------------------------------------------------

/// **A click is a step, so the fix is a slope.** The amplifier has been enabled
/// once at boot and never touched since rung 26, and the audible cost is at the
/// ends of a stream: audio stops mid-waveform at whatever sample it had reached
/// and the padding that follows is zero, which is a step discontinuity — and a
/// step is a click no matter how quiet the material was.
///
/// **Switching the amplifier off is not on its own a fix, it is a second
/// click.** The PAM8908 popping as its enable line drops is the same problem
/// one stage later. The order that works is: ramp the signal to zero, and only
/// once it is actually zero drop the enable line. Coming back, the reverse —
/// enable first, while the buffers are still silent, then ramp up.
///
/// **Q15, ramped per frame rather than per buffer.** A buffer is 256 frames,
/// 5.8 ms; stepping the gain once per buffer would be 172 small steps a second,
/// which is a 172 Hz buzz on top of the fade rather than a fade. The ramp is
/// `ramp_frames` long — about 23 ms, slow enough to be inaudible and short
/// enough that a pause feels immediate.
const gate = struct {
    const full: i32 = 1 << 15;
    const ramp_frames: i32 = 1024;
    const ramp_step: i32 = full / ramp_frames;

    var gain: i32 = full;
    var target: i32 = full;
    /// Set when the ramp reaches zero and the enable line has been dropped, so
    /// the work is done once rather than every buffer.
    var muted: bool = false;
    /// Buffers handed over since the ramp reached zero. **Two are needed, not
    /// none**: the buffer just filled still holds the tail of the fade and has
    /// not been played yet, and the DMA is reading the other one. Cutting the
    /// amplifier the instant the gain variable reaches zero silences the fade
    /// partway through, which is the click again.
    var settled: u32 = 0;

    /// Silence, from now, over `ramp_frames`.
    fn mute() void {
        target = 0;
    }

    /// Resume only from a *completed* mute.
    ///
    /// **The distinction matters and the obvious version is wrong.** Media
    /// keeps arriving for a packet or two after a SUSPEND, and an unmute on
    /// every media packet would pull the target back to full mid-fade — the
    /// fade would never finish and the click it exists to remove would still
    /// be there, now with no visible cause.
    fn resumeIfMuted() void {
        if (muted) unmute();
    }

    /// Sound again. **The enable line goes high here, not when the ramp
    /// finishes**, because the buffers are still carrying zeros at this instant
    /// and that is the only moment when enabling is silent.
    fn unmute() void {
        settled = 0;
        if (muted) {
            audio.enableAmplifier();
            watch.audio_state |= audio_state.amp_on;
            muted = false;
        }
        target = full;
    }

    /// Scale one buffer in place, advancing the ramp.
    ///
    /// **Returns immediately in the common case** — full gain, nothing to do —
    /// so a running stream pays one comparison per buffer rather than 256
    /// multiplies.
    fn apply(buf: []u32) void {
        if (gain == full and target == full) return;

        for (buf) |*f| {
            if (gain < target) {
                gain = @min(gain + ramp_step, target);
            } else if (gain > target) {
                gain = @max(gain - ramp_step, target);
            }
            const s = pcm5100.unpackStereoFrame(f.*);
            const l: i32 = (@as(i32, s.left) * gain) >> 15;
            const r: i32 = (@as(i32, s.right) * gain) >> 15;
            f.* = pcm5100.packStereoFrame(@intCast(l), @intCast(r));
        }

        if (gain != 0 or target != 0) {
            settled = 0;
            return;
        }
        settled +%= 1;
        if (settled >= 2 and !muted) {
            audio.disableAmplifier();
            watch.audio_state &= ~audio_state.amp_on;
            muted = true;
            // Whatever the phone sent before it paused is stale now, and
            // playing it on resume would be a 50 ms echo of the moment before
            // the pause.
            ring = .{};
            decoder.reset();
            uart.print(.uart0, "   faded out, amplifier off\n", .{});
        }
    }
};

/// Hand the DMA the next buffer, if it has moved on to the other one.
///
/// **A late fill does not present as an underrun.** The DMA always has a buffer
/// to read, so missing the deadline plays half-stale audio with the stall flag
/// still clear — `lib/pcm5100`'s own notes say so. Watching `playingIndex`
/// change is the detector that works, and it is why this is called from the
/// main loop rather than left to a timer nobody wrote.
/// Where the loop was when it last set this, so the longest gap between
/// `serviceAudio` calls can be attributed rather than guessed at.
///
/// **Every previous attempt at this reasoned from a number that bounded the
/// gap instead of measuring it** — worst `bt.read`, worst decode, worst pass —
/// and they do not add up to what the missed-flip counter sees. Something
/// between them is blocking and none of those three can name it.
const phase = struct {
    const idle: u32 = 1;
    const after_read: u32 = 2;
    const acl: u32 = 3;
    const sdp: u32 = 4;
    const signalling: u32 = 5;
    const decode: u32 = 6;
    const event: u32 = 7;

    const names = [_][]const u8{
        "unset", "idle", "bt.read", "acl/l2cap", "sdp", "signalling", "decode", "hci event",
    };

    var at: u32 = 0;
    var since: u64 = 0;
    /// **Microseconds spent in each phase, not tags at instants.** The tag
    /// version attributed every gap to `after bt.read`, and had to: that tag is
    /// set immediately before the loop-top `serviceAudio`, so it overwrites
    /// whatever actually blocked. Worst `bt.read` was 4,638 us against a gap of
    /// 21,875 — the read was never the cause, only the last thing to happen
    /// before the measurement. Time spent cannot be misattributed that way.
    var spent: [names.len]u64 = @splat(0);
    /// **Overruns per phase, not just the worst one's phase.** A single worst
    /// gap names one event; what decides where the fix goes is which phase
    /// blows the deadline *repeatedly*. The watch block had no room and the
    /// one byte it did have came back zero, which is its own argument for
    /// counting all of them and printing the table.
    var overruns: [names.len]u32 = @splat(0);

    fn set(p: u32) void {
        const t = timer.now64();
        if (since != 0) spent[@min(at, names.len - 1)] +%= t -% since;
        since = t;
        at = p;
    }

    /// Safe to call **only once the stream has stopped** — it blocks the loop
    /// for as long as the UART takes, which is the very fault being measured.
    fn report() void {
        uart.print(.uart0, "\n   -- audio health, this stream --\n", .{});
        uart.print(.uart0, "   missed flips {d}, dropped {d} frames, silent {d}\n", .{
            watch.media_bytes & 0xFFFF, watch.audio_dropped, watch.audio_silent,
        });
        uart.print(.uart0, "   ring high-water {d} of {d}, worst bt.read {d} us\n", .{
            watch.ring_level >> 16, ring_frames, watch.media_bytes >> 16,
        });
        uart.print(.uart0, "   worst gap: calls {d} us ({d} over {d}), fills {d} us\n", .{
            watch.peer_lo & 0xFFFF,   watch.peer_lo >> 16,
            pcm5100.buffer_period_us, worst_fill_gap_us,
        });
        // **Two short lines rather than one long one.** The bench UART is a
        // kludge and drops characters; a line that runs off the end takes its
        // numbers with it. The watch block holds all of this regardless.
        uart.print(.uart0, "   stream {d} Hz, {d}x{d}, bitpool {d}\n", .{
            watch.stream_rate & 0xFFFF,
            (watch.stream_shape >> 8) & 0xFF,
            (watch.stream_shape >> 16) & 0xFF,
            (watch.stream_bitpool >> 8) & 0xFF,
        });
        uart.print(.uart0, "   worst media gap {d} ms\n", .{watch.stream_rate >> 16});
        // **What the source encoded, against what it was allowed to.** The
        // line above carries the negotiated ceiling; this one carries the
        // audio. A `used` well below `max` is a source choosing to send less
        // than this device would accept.
        if (sbc_mode) |m| {
            uart.print(.uart0, "   sbc {s}, bitpool used {d}-{d} of {d} allowed\n", .{
                @tagName(m),
                sbc_bitpool_min,
                sbc_bitpool_max,
                (watch.stream_bitpool >> 8) & 0xFF,
            });
        }
        // The arm this stream actually ran under. `scan_mode` is the confirmed
        // one; a disagreement means the write never took, and the numbers above
        // belong to the other arm.
        if (scan_mode == scan_requested) {
            uart.print(.uart0, "   scan {s}\n", .{@tagName(scan_mode)});
            // **Nanoseconds, because a fetch is a few microseconds** and integer
            // division to whole us would round the answer away.
            if (empty_reads != 0) {
                uart.print(.uart0, "   fetch {d} ns x{d} empty; pkt {d} ns x{d}\n", .{
                    (empty_read_us * 1000) / empty_reads,
                    empty_reads,
                    if (packet_reads != 0) (packet_read_us * 1000) / packet_reads else 0,
                    packet_reads,
                });
                // **The phase bucket printed as `bt.read` is not time in `bt.read`.**
                // `phase.set(phase.after_read)` runs *before* the call and the next
                // transition is after `serviceAudio`, so that bucket carries the
                // audio fill work too -- 25,082 ms against 7,514 ms actually inside
                // the read, on the first clean arm. Every claim that "bt.read is a
                // quarter of wall time" was reading the bucket. This line is the
                // call itself.
                const read_ms = (empty_read_us + packet_read_us) / 1000;
                uart.print(.uart0, "   in bt.read {d} ms; 0.4 saved ~{d} ms of it\n", .{
                    read_ms,
                    (empty_read_us / @max(empty_reads, 1)) * packet_reads / 1000,
                });
            }
        } else {
            uart.print(.uart0, "   scan {s} (asked {s} -- NOT taken)\n", .{
                @tagName(scan_mode), @tagName(scan_requested),
            });
        }

        // **The unexplained ACL parse failures, described rather than counted.**
        // See the module vars above for how to read the boundary flag. Printed
        // only when there is something to say, so a clean run stays quiet.
        if (acl_fail_seen or acl_fail_hci != 0) {
            uart.print(.uart0, "   acl parse fail: hci {d}, l2cap {d}\n", .{
                acl_fail_hci, acl_fail_l2cap,
            });
            if (acl_fail_seen) {
                uart.print(.uart0, "   first: boundary {s}, declared {d}, actual {d}\n", .{
                    @tagName(acl_fail_boundary), acl_fail_declared, acl_fail_actual,
                });
            }
        }
        // **`assembled` is the arm of the experiment, not a curiosity.** It is
        // how much of this stream arrived fragmented, and therefore how much
        // would have been silence before reassembly existed. `orphans` and
        // `overflows` are faults: the first means fragments are being lost
        // before the reassembler sees them, the second that the buffer is
        // sized wrong.
        // **`delayed` is the count that says the violation was real.** Every
        // one of these is a command that used to go out past the controller's
        // stated allowance. `dropped` is a fault: a burst outgrew the queue.
        if (host_ptr) |h| {
            if (h.cmd_delayed != 0 or h.cmd_dropped != 0) {
                uart.print(.uart0, "   cmds delayed {d}, dropped {d}\n", .{
                    h.cmd_delayed, h.cmd_dropped,
                });
            }
        }
        if (reasm.assembled != 0 or reasm.orphans != 0 or reasm.overflows != 0) {
            uart.print(.uart0, "   reassembled {d} pdus, orphans {d}, overflows {d}\n", .{
                reasm.assembled, reasm.orphans, reasm.overflows,
            });
        }
        uart.print(.uart0, "   where the time went, ms:\n", .{});
        defer {
            // **Do not measure the measurement.** This prints roughly ten
            // lines at 115200 baud — tens of milliseconds with the loop
            // blocked — and `gate.muted` is still false here, because the fade
            // has only just been asked for. Every run so far reported a worst
            // gap of 17-21 ms against a worst `bt.read` of 3-5 ms, and this is
            // at least part of that difference. No audio is lost (the stream
            // has stopped, which is why it is safe to print at all) but the
            // gap it opens is not a fault and must not be counted as one.
            last_service_us = 0;
            since = 0;
        }
        for (spent, 0..) |us, i| {
            if (us != 0) uart.print(.uart0, "     {d:>7} ms  {s}  ({d} overruns)\n", .{
                us / 1000, names[i], overruns[i],
            });
        }
    }
};

var last_service_us: u64 = 0;

fn serviceAudio() void {
    // **The gap this call is closing, whatever it does next.** Measured at
    // entry, so it covers everything since the last call including the calls
    // that returned early.
    const entry = timer.now64();
    if (last_service_us != 0 and !gate.muted) {
        const gap: u32 = @intCast(@min(entry -% last_service_us, 0xFFFF));
        if (gap > (watch.peer_lo & 0xFFFF)) {
            watch.peer_lo = (watch.peer_lo & 0xFFFF_0000) | gap;
            // The phase that preceded the worst gap, in the spare top byte of
            // `stream_bitpool` — bitpool is two bytes and never needed four.
            watch.stream_bitpool = (watch.stream_bitpool & 0x00FF_FFFF) |
                (phase.at << 24);
        }
        if (gap > pcm5100.buffer_period_us) {
            const n = (watch.peer_lo >> 16) +% 1;
            const capped: u32 = @min(n, 0xFFFF);
            watch.peer_lo = (watch.peer_lo & 0xFFFF) | (capped << 16);
            phase.overruns[@min(phase.at, phase.names.len - 1)] +%= 1;
        }
    }
    last_service_us = entry;

    // **`playingIndex`, which is what played a 28-minute album cleanly at the
    // correct 172 fills a second.** Two replacements were written for it while
    // chasing silence that turned out to be a blown fuse in the speaker, and
    // both measured *worse* — `playingBuffer` at 1587 fills a second, the
    // transfer-count edge at 705. Neither was ever shown to fix anything.
    const now = audio.playingIndex();
    if (now == last_playing) return;
    last_playing = now;

    // **Missed flips, which nothing so far could see.** `playingIndex` is 0 or
    // 1, so a pass longer than *two* buffer periods leaves it reading what it
    // read before: this function returns early and the miss never reaches the
    // late-fill check below. Each one replays a stale buffer — the pop — and
    // strands 256 frames in the ring, which is why the ring ratchets up and
    // eventually overflows. Timing the gap is the only way to catch it,
    // because the index has already lied.
    const t_now = timer.now64();
    if (last_flip != 0 and !gate.muted) {
        const gap = t_now -% last_flip;
        // **The gap between fills, which is not the gap between calls.** The
        // report conflated them and so did I: 8 missed flips were reported
        // beside a "worst gap" of 7,212 us, when a miss needs 11.6 ms between
        // fills. Two different quantities, one label. Recorded here so the two
        // can be compared instead of confused.
        const g32: u32 = @intCast(@min(gap, 0xFFFF_FFFF));
        if (g32 > worst_fill_gap_us) worst_fill_gap_us = g32;
        if (gap > (pcm5100.buffer_period_us * 3) / 2) {
            const spans: u64 = @min(gap / pcm5100.buffer_period_us, 0xFF);
            const missed: u32 = @intCast(spans -| 1);
            const cur = watch.media_bytes & 0xFFFF;
            watch.media_bytes = (watch.media_bytes & 0xFFFF_0000) |
                @min(cur + missed, 0xFFFF);
        }
    }
    last_flip = t_now;

    // **Ring low-water, which decides between the two explanations.** The
    // ring reached 7,935 of 8,192 and threw frames away. Either the source is
    // faster than the DAC and the ring never drains again — low-water stays
    // high — or delivery is bursty and it sawtooths to nothing between bursts.
    // The high-water mark cannot distinguish those and the gap timing this
    // replaced is superseded by the late-fill counter, which reads 2.
    if (watch.audio_dropped > 0 and !gate.muted) {
        const lvl16: u32 = @intCast(@min(ring.level(), 0xFFFF));
        const lo = watch.flip_us & 0xFFFF;
        if (lo == 0 or lvl16 < lo) watch.flip_us = (watch.flip_us & 0xFFFF_0000) | lvl16;
    }

    const buf = audio.idleBuffer();
    var i: usize = 0;
    while (i < pcm5100.buffer_sample_count) : (i += 1) {
        buf[i] = ring.pop() orelse break;
    }
    if (i < pcm5100.buffer_sample_count) {
        // Pad rather than repeat. A repeated buffer is a 5.8 ms loop, which is
        // a tone; silence is at least honestly nothing.
        @memset(buf[i..], 0);
        // **Not while muted, and the last two readings were wrong for want of
        // this.** A pause clears the ring and stops the stream, so every
        // buffer after it is empty — and counted, that reads as starvation.
        // Across a few play/pause cycles it inflated `audio_silent` to 18% of
        // playback and saturated the dry counter, and I read those as
        // dropouts inside the music. Silence the device chose is not silence
        // it suffered.
        if (!gate.muted) {
            watch.audio_silent +%= @intCast(pcm5100.buffer_sample_count - i);
            watch.audio_state |= audio_state.ran_dry;
            audio_state.bump(audio_state.dry_shift);
        }
    }
    gate.apply(buf);
    watch.audio_fills +%= 1;

    // **Was that fill in time?** `lib/pcm5100` is explicit that a late one
    // leaves no other trace: the DMA always has a buffer to read, so the FIFO
    // never stalls and `underrunSeen` stays false while half-stale audio
    // plays. The test it gives is this one — if the DMA has reached the buffer
    // we were just writing, we missed the deadline.
    if (audio.playingBuffer()) |p| {
        if (p == 1 - now) audio_state.bump(audio_state.late_shift);
    }

    const lvl: u32 = @intCast(ring.level());
    const high = watch.ring_level >> 16;
    watch.ring_level = lvl | (@max(high, lvl) << 16);
}

/// Decode one media payload's SBC frames into the ring.
fn decodeToRing(payload: []const u8, p: cyw43.sbc.Packet) void {
    var pcm: [16 * 8 * 2]i16 = undefined;
    var at = p.rtp.payload_at + 1;

    for (0..p.frames_walked) |_| {
        // **Service the DAC between frames, not just between packets.**
        //
        // A buffer turns over every 5.8 ms and only one can be handed over per
        // turn, so a pass longer than that misses a flip — and a missed flip
        // adds 256 frames to the ring permanently, because with two buffers
        // there is no way to drain faster than the DAC consumes. The ring then
        // fills monotonically and starts dropping, which is what 15% of a track
        // going missing sounded like.
        //
        // Decoding five frames took 5.7 ms; decoding one takes about 1.1 ms.
        // Spreading the work is the fix, and it stays the fix whatever the
        // decoder later costs.
        serviceAudio();

        if (at + p.frame_len > payload.len) break;
        phase.set(phase.decode);
        const n = decoder.decodeFrame(payload[at..][0..p.frame_len], &pcm) catch break;
        at += p.frame_len;

        var i: usize = 0;
        while (i + 1 < n) : (i += 2) {
            ring.push(pcm5100.packStereoFrame(pcm[i], pcm[i + 1]));
        }
        watch.decoded +%= @intCast(n / 2);
    }

    // Recorded, not printed, and for the same reason.
    watch.audio_state |= audio_state.streaming;
}

/// SBC frames lifted off the wire, for rung 25b's oracle.
///
/// **Real frames from a real phone**, concatenated exactly as they arrive after
/// the RTP and payload headers are stripped — which is precisely the input a
/// decoder is handed. Lift with OpenOCD's `dump_image` at the `sbc_capture`
/// symbol; `tools/sbc_reference.c` decodes them with bluedroid to produce the
/// expected PCM, and the pair becomes a host test vector.
///
/// Synthetic frames would have done for exercising a decoder, but not for
/// this: the whole point is that these are the bytes this phone sends in the
/// configuration this board negotiated.
const capture_bytes = 8 * 1024;

/// **Plain `.bss`, not a named section.** `rp2350.ld` defines `.selftest` and
/// `.faultinfo` and nothing else at fixed addresses; a `linksection(".noinit")`
/// here produced an eight-kilobyte *loaded* section that overlapped
/// `.faultinfo`, and the flash write failed verification. The symbol is found
/// by name for `dump_image`, so it needs no fixed address of its own.
export var sbc_capture: extern struct {
    /// 'SBCC', so a dump that captured nothing is distinguishable from a dump
    /// that missed the symbol.
    magic: u32,
    used: u32,
    frames: u32,
    frame_len: u32,
    data: [capture_bytes]u8,
} = undefined;

var capture_ready = false;

fn captureFrames(payload: []const u8, frame_len: usize, frames: usize) void {
    if (!capture_ready) {
        sbc_capture.magic = 0x53424343; // 'SBCC'
        sbc_capture.used = 0;
        sbc_capture.frames = 0;
        sbc_capture.frame_len = @intCast(frame_len);
        capture_ready = true;
    }
    // **Stop when full rather than wrap.** A wrapped capture reads as a stream
    // that restarted, and the decoder built against it would be validated on a
    // discontinuity.
    if (sbc_capture.used + payload.len > capture_bytes) return;
    @memcpy(sbc_capture.data[sbc_capture.used..][0..payload.len], payload);
    sbc_capture.used += @intCast(payload.len);
    sbc_capture.frames += @intCast(frames);
}

/// Prove the media stream is understood, without decoding it.
///
/// **Every packet is walked, not a sample of them.** A framing error that
/// happens once in a thousand packets is exactly the kind that would later be
/// blamed on the decoder, and walking is cheap next to what decoding will cost.
fn walkMedia(payload: []const u8) void {
    // **Not every resume arrives as a START.** A phone that suspended and then
    // simply resumes sending media would otherwise play into a muted
    // amplifier, which is a silent failure and the worst kind. Media arriving
    // is itself proof that sound is wanted.
    gate.resumeIfMuted();

    const p = cyw43.sbc.walk(payload) catch |err| {
        const shift: u5 = switch (err) {
            cyw43.sbc.Error.LengthMismatch => 0,
            cyw43.sbc.Error.BadSyncword => 8,
            cyw43.sbc.Error.NotRtp => 16,
            cyw43.sbc.Error.Fragmented => 24,
            else => 0,
        };
        // Saturating per byte, so one busy field cannot roll into its
        // neighbour and read as a different fault.
        const cur = (watch.sbc_errors >> shift) & 0xFF;
        if (cur < 0xFF) watch.sbc_errors += @as(u32, 1) << shift;
        if (cur == 0) {
            uart.print(.uart0, "   ! media framing: {s}, {d} bytes\n", .{
                @errorName(err), payload.len,
            });
        }
        return;
    };

    // The bitpool rides in `stream_bitpool`'s spare byte: it is the one thing
    // that moves mid-stream, and `decoded` already counts frames.
    const before = watch.decoded;
    watch.stream_bitpool = (watch.stream_bitpool & 0x0000_FFFF) |
        (@as(u32, p.frame.bitpool) << 16);
    if (p.frame.bitpool < sbc_bitpool_min) sbc_bitpool_min = p.frame.bitpool;
    if (p.frame.bitpool > sbc_bitpool_max) sbc_bitpool_max = p.frame.bitpool;
    sbc_mode = p.frame.channel_mode;

    // The frames sit immediately after the RTP and payload headers.
    captureFrames(payload[p.rtp.payload_at + 1 ..], p.frame_len, p.frames_walked);
    const t0 = timer.now64();
    decodeToRing(payload, p);
    const took: u32 = @intCast(@min(timer.now64() - t0, 0xFFFF));
    if (took > (watch.timing_us >> 16)) {
        watch.timing_us = (watch.timing_us & 0xFFFF) | (took << 16);
    }

    // **No printing from the media path.** This block described the framing in
    // six lines, and `before == 0` became true every session once the audio
    // counters started being reset at the first media packet — so it blocked
    // the loop for about 25 ms at the exact moment the ring is empty and
    // filling. That is the 17.7 ms worst gap every run has reported, both of
    // its overruns, and probably its missed flips: a diagnostic manufacturing
    // the fault it was there to help find. The same facts are in the health
    // report, which prints when the stream has stopped and costs nothing.
    if (before == 0) watch.rung = 0x25A;
}

fn handleAcl(host: *cyw43.BtHost(Dev), pkt: []const u8) void {
    const a = hci.parseAcl(pkt) catch {
        watch.acl +%= 1 << 16;
        acl_fail_hci +%= 1;
        return;
    };
    // **Reassemble first.** A continuation carries no L2CAP header and a first
    // fragment declares a length it does not hold, so neither can be parsed
    // alone — handing either to `l2cap.parse` is what `acl_parse_fail` counted
    // for every run since rung 23a. `null` means the PDU is still arriving, or
    // that the fragment was unusable and the reassembler has counted why.
    //
    // The slice is valid only until the next `push`, which is why it is parsed
    // on the next line rather than stored.
    const assembled = reasm.push(a) orelse return;

    const frame = cyw43.l2cap.parse(assembled) catch {
        watch.acl +%= 1 << 16;
        acl_fail_l2cap +%= 1;
        // **Describe the first one rather than only counting it.** Only the
        // first: the count says how many, and if they are all the same
        // recurring packet — which a constant 4-5 says they are — then one
        // description is the whole answer.
        if (!acl_fail_seen) {
            acl_fail_seen = true;
            acl_fail_boundary = a.boundary;
            acl_fail_actual = @intCast(@min(a.payload.len, 0xFFFF));
            acl_fail_declared = if (a.payload.len >= 2)
                std.mem.readInt(u16, a.payload[0..2], .little)
            else
                0;
        }
        return;
    };

    // **Data on the channel we opened is the pass condition.** Once SDP traffic
    // arrives here, rung 23a has done its job: the phone is talking a protocol
    // to us rather than trying to reach one.
    if (channelByLocal(frame.cid)) |c| {
        if (c.open) {
            switch (c.role) {
                .avdtp_signalling => handleAvdtp(host, c.remote_cid, frame.payload),
                .avdtp_media => {
                    // **The pass condition for rung 24.** Not decoded yet —
                    // counted, because a stream that arrives is the claim and a
                    // decoder is rung 25.
                    watch.media_frames +%= 1;

                    // **The worst gap between media packets, in `stream_rate`'s
                    // spare high half** — 44,100 is 0xAC44 and never needed it.
                    //
                    // This is the measurement that separates the two remaining
                    // explanations, and nothing recorded so far can. A packet
                    // carries 640 frames, 14.5 ms of audio, so packets must
                    // arrive about that often. **If the worst gap is ~15 ms the
                    // link is delivering evenly and the fault is ours**, in
                    // whatever makes a loop pass take 25.7 ms. If it is 150 ms
                    // the phone or the link is batching, the ring drains and
                    // then takes a burst it cannot hold, and no amount of
                    // servicing faster would help.
                    const t_media = timer.now64();
                    if (last_media_us != 0) {
                        // **Milliseconds, because microseconds saturated.** A
                        // track break is longer than the 65 ms a u16 of
                        // microseconds can hold, and a capped reading says
                        // nothing about how long the phone was quiet.
                        const gap: u32 = @intCast(@min((t_media -% last_media_us) / 1000, 0xFFFF));
                        if (gap > (watch.stream_rate >> 16)) {
                            watch.stream_rate = (watch.stream_rate & 0xFFFF) | (gap << 16);
                        }
                    }
                    last_media_us = t_media;
                    if (watch.media_frames == 1) {
                        watch.rung = 24;
                        // **Zero the audio counters here, not at boot.**
                        // This had been recorded as already done and it was
                        // not, which is why 8.1 s of silence could not be told
                        // apart from 8.1 s of idle before the stream started —
                        // and the flattering reading was the one taken. From
                        // this packet on, every audio number describes the
                        // stream.
                        watch.decoded = 0;
                        watch.audio_fills = 0;
                        watch.audio_silent = 0;
                        watch.audio_dropped = 0;
                        watch.ring_level = 0;
                        watch.flip_us = 0;
                        watch.stream_rate &= 0xFFFF;
                        watch.peer_lo = 0;
                        watch.stream_bitpool &= 0x00FF_FFFF;
                        watch.media_bytes = 0;
                        last_service_us = 0;
                        phase.overruns = @splat(0);
                        phase.spent = @splat(0);
                        sbc_bitpool_min = 0xFF;
                        sbc_bitpool_max = 0;
                        sbc_mode = null;
                        empty_reads = 0;
                        empty_read_us = 0;
                        packet_reads = 0;
                        packet_read_us = 0;
                        phase.since = 0;
                        worst_fill_gap_us = 0;
                        watch.audio_state &= 0xFF; // keep the flags, drop the counters
                        watch.audio_state &= ~audio_state.ran_dry;
                        last_flip = 0;
                        uart.print(.uart0, "\n   >> MEDIA on cid 0x{X:0>4}: {d} bytes\n", .{
                            frame.cid, frame.payload.len,
                        });
                    }
                    walkMedia(frame.payload);
                },
                .other => if (c.psm == cyw43.bredr.psm.sdp)
                    handleSdp(host, c.remote_cid, frame.payload)
                else
                    uart.print(.uart0, "   >> DATA psm 0x{X:0>4}: {d} bytes\n", .{ c.psm, frame.payload.len }),
            }
            return;
        }
    }

    // BR/EDR signalling is CID 1. LE's is 5, and looking there sees nothing.
    if (frame.cid != bredr.l2cap_cid.signalling) {
        return;
    }
    watch.sig_frames = ((watch.sig_frames & 0xFFFF) +% 1) | (watch.sig_frames & 0xFFFF_0000);

    const sg = bredr.parseSignalling(frame.payload) catch {
        watch.acl +%= 1 << 16;
        return;
    };

    switch (sg.code) {
        bredr.sig.information_request => {
            if (sg.payload.len < 2) return;
            const kind = std.mem.readInt(u16, sg.payload[0..2], .little);
            // **Answered honestly with an empty feature set.** This device does
            // basic mode and nothing else; claiming enhanced retransmission
            // would invite a configuration it cannot perform.
            var p: [12]u8 = @splat(0);
            std.mem.writeInt(u16, p[0..2], kind, .little);
            std.mem.writeInt(u16, p[2..4], 0, .little); // result: success
            const len: usize = switch (kind) {
                bredr.info_type.extended_features => 8, // 4 bytes of zero mask
                bredr.info_type.fixed_channels => 12, // 8 bytes, signalling only
                else => 4,
            };
            if (kind == bredr.info_type.fixed_channels) p[4] = 0x02; // CID 1
            sendSig(host, bredr.sig.information_response, sg.id, p[0..len]);
            watch.l2cap_state |= l2.info_answered;
            uart.print(.uart0, "   info request type {d} -- answered\n", .{kind});
        },

        bredr.sig.connection_request => {
            const req = bredr.parseL2capConnectionRequest(sg.payload) catch return;
            watch.sig_frames +%= 1 << 16;
            if (watch.psms & 0xFFFF == 0) watch.psms |= req.psm;
            watch.psms = (watch.psms & 0xFFFF) | (@as(u32, req.psm) << 16);

            const c = channelAlloc() orelse {
                // Refused rather than silently dropped: the peer learns it, and
                // "no resources" is the truthful reason.
                uart.print(.uart0, "   !! no free channel for psm 0x{X:0>4}\n", .{req.psm});
                const rp = bredr.connectionResponseParams(0, req.source_cid, .refused_no_resources);
                sendSig(host, bredr.sig.connection_response, sg.id, &rp);
                return;
            };
            c.local_cid = next_local_cid;
            next_local_cid += 1;
            c.remote_cid = req.source_cid;
            c.psm = req.psm;
            c.role = if (req.psm != bredr.psm.avdtp)
                .other
            else if (channelByRole(.avdtp_signalling) == null)
                .avdtp_signalling
            else
                .avdtp_media;
            c.in_configured = false;
            c.out_configured = false;
            c.open = false;

            uart.print(.uart0, "   >> L2CAP CONNECT to PSM 0x{X:0>4}, scid 0x{X:0>4} -- accepting as 0x{X:0>4}\n", .{
                req.psm, req.source_cid, c.local_cid,
            });

            const rp = bredr.connectionResponseParams(c.local_cid, c.remote_cid, .success);
            sendSig(host, bredr.sig.connection_response, sg.id, &rp);
            watch.l2cap_state |= l2.conn_answered;

            // **Our half of the configuration, sent without waiting for
            // theirs.** Both directions are configured independently and either
            // side may go first; waiting for the peer here is a deadlock
            // whenever the peer is doing the same.
            var cr: [8]u8 = @splat(0);
            std.mem.writeInt(u16, cr[0..2], c.remote_cid, .little);
            std.mem.writeInt(u16, cr[2..4], 0, .little);
            @memcpy(cr[4..8], &bredr.mtuOption(bredr.local_mtu));
            next_sig_id +%= 1;
            sendSig(host, bredr.sig.configure_request, next_sig_id, &cr);
        },

        bredr.sig.connection_response => {
            const rsp = bredr.parseL2capConnectionResponse(sg.payload) catch return;
            const c = channelByLocal(rsp.scid) orelse return;
            if (rsp.pending()) {
                // Not a refusal. A second response follows.
                uart.print(.uart0, "   our connect is pending, waiting\n", .{});
                return;
            }
            if (!rsp.ok()) {
                uart.print(.uart0, "   ! our connect refused, result {d}\n", .{rsp.result});
                c.* = .{};
                return;
            }
            c.remote_cid = rsp.dcid;
            uart.print(.uart0, "   our AVDTP connect accepted, dcid 0x{X:0>4}\n", .{rsp.dcid});
            watch.l2cap_state |= l2.conn_answered;

            // Our half of the configuration, as in the inbound path and for
            // the same reason: both directions configure independently.
            var cr: [8]u8 = @splat(0);
            std.mem.writeInt(u16, cr[0..2], c.remote_cid, .little);
            std.mem.writeInt(u16, cr[2..4], 0, .little);
            @memcpy(cr[4..8], &bredr.mtuOption(bredr.local_mtu));
            next_sig_id +%= 1;
            sendSig(host, bredr.sig.configure_request, next_sig_id, &cr);
        },

        bredr.sig.configure_request => {
            const req = bredr.parseConfigureRequest(sg.payload) catch return;
            const mtu = bredr.findMtu(req.options) orelse 0;
            // **Configuration names the channel it configures.** Answering on
            // whichever channel happened to be most recent is how a second SDP
            // channel ends up configuring the first.
            const c = channelByLocal(req.dcid) orelse return;

            // Accept whatever was proposed, echoing the options back. A sink
            // with no constraints of its own has nothing to renegotiate, and
            // an empty response would leave the peer's MTU unconfirmed.
            var rp: [64]u8 = @splat(0);
            std.mem.writeInt(u16, rp[0..2], c.remote_cid, .little);
            std.mem.writeInt(u16, rp[2..4], 0, .little);
            std.mem.writeInt(u16, rp[4..6], 0, .little); // result: success
            const n = @min(req.options.len, rp.len - 6);
            @memcpy(rp[6..][0..n], req.options[0..n]);
            sendSig(host, bredr.sig.configure_response, sg.id, rp[0 .. 6 + n]);

            c.in_configured = true;
            watch.l2cap_state |= l2.in_configured;
            uart.print(.uart0, "   their config accepted (mtu {d})\n", .{mtu});
            noteOpen(c);
        },

        bredr.sig.configure_response => {
            const rsp = bredr.parseConfigureResponse(sg.payload) catch return;
            // The response names *our* channel as its scid.
            const c = channelByLocal(rsp.scid) orelse return;
            if (rsp.result == 0) {
                c.out_configured = true;
                watch.l2cap_state |= l2.out_configured;
                uart.print(.uart0, "   our config accepted\n", .{});
                noteOpen(c);
            } else {
                uart.print(.uart0, "   ! our config refused, result {d}\n", .{rsp.result});
            }
        },

        bredr.sig.disconnection_request => {
            if (sg.payload.len < 4) return;
            sendSig(host, bredr.sig.disconnection_response, sg.id, sg.payload[0..4]);
            const dcid = std.mem.readInt(u16, sg.payload[0..2], .little);
            if (channelByLocal(dcid)) |c| c.* = .{};
            watch.l2cap_state = (watch.l2cap_state & 0xFFFF) | (countOpen() << 16);
            uart.print(.uart0, "   channel 0x{X:0>4} closed by peer\n", .{dcid});
        },

        else => {
            _ = sg.code;
            uart.print(.uart0, "   [l2cap sig code 0x{X:0>2}, id {d}]\n", .{ sg.code, sg.id });
        },
    }
}

/// Ask the phone for an AVDTP signalling channel.
///
/// **The channel this stack has always had opened for it.** Everything in
/// `handleSignalling` answers; this is the one place that asks, and it exists
/// because paging leaves the phone with no reason to ask first.
///
/// What happens after the channel opens is the open question of rung 28: the
/// phone may take over and run `DISCOVER` itself, in which case the existing
/// responder code handles the whole stream, or it may wait for us to drive
/// AVDTP as well. The channel is needed either way, so it is worth finding out
/// which before writing an initiator that might be redundant.
fn openAvdtpSignalling(host: *cyw43.BtHost(Dev)) void {
    if (channelByRole(.avdtp_signalling) != null) return;
    const c = channelAlloc() orelse {
        uart.print(.uart0, "   !! no free channel to open AVDTP with\n", .{});
        return;
    };
    c.local_cid = next_local_cid;
    next_local_cid += 1;
    c.remote_cid = 0; // not known until the response
    c.psm = bredr.psm.avdtp;
    c.role = .avdtp_signalling;

    uart.print(.uart0, "   << opening AVDTP ourselves, scid 0x{X:0>4}\n", .{c.local_cid});
    const rp = bredr.connectionRequestParams(bredr.psm.avdtp, c.local_cid);
    next_sig_id +%= 1;
    sendSig(host, bredr.sig.connection_request, next_sig_id, &rp);
}

fn noteOpen(c: *Channel) void {
    if (c.in_configured and c.out_configured and !c.open) {
        c.open = true;
        watch.l2cap_state |= l2.open;
        watch.l2cap_state = (watch.l2cap_state & 0xFFFF) | (countOpen() << 16);
        uart.print(.uart0, "   L2CAP OPEN: local 0x{X:0>4}, remote 0x{X:0>4}, psm 0x{X:0>4}\n", .{
            c.local_cid, c.remote_cid, c.psm,
        });
    }
}

fn addrOf(a: [6]u8) void {
    uart.print(.uart0, "{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}", .{
        a[5], a[4], a[3], a[2], a[1], a[0],
    });
}

pub fn main() noreturn {
    clocks.init(plan);
    gpio.init();
    uart.init(.uart0, .{
        .inst = .uart0,
        .tx = board.uart_tx,
        .rx = board.uart_rx,
        .clk_hz = sys_hz,
    });

    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;

    uart.print(.uart0, "\n=== BR/EDR pairing (rung 22) ===\nboard {s}\n", .{board.name});
    watch.clk_sys_khz = clocks.measureKhz(rp2350.hw.Clocks.fc0_src_clk_sys) orelse 0;
    uart.print(.uart0, "clk_sys {d} kHz\n", .{watch.clk_sys_khz});

    if (cyw43.fw_wifi.len != 231077) {
        uart.print(.uart0, "\nThis needs -Dfw=combined.\n", .{});
        die(error.BadPatchram, 22);
    }

    const pins = cyw43.default_pins;
    var bus = cyw43.PicogSpiBus.init(pins);

    // **The bus rate, and the PIO program that has to go with it.**
    // `-Dbus-div=0` is the historical behaviour: hold the bus at its proven
    // 3 MHz however fast clk_sys runs. Anything else is a raw divisor, and the
    // program follows the resulting rate per `lib/cyw43` rung A's measured
    // crossover — change one without the other and bring-up dies at rung 1
    // with `ChipNotResponding`, which is what rung 21d did.
    if (config.bus_div == 0) bus.setSysHz(sys_hz) else bus.setBusDiv(config.bus_div);
    const bus_hz = bus.busHz(sys_hz);
    bus.setVariant(cyw43.gspi.Variant.forBusHz(bus_hz));
    uart.print(.uart0, "gSPI {d} kHz (div {d}), program {s}\n", .{
        bus_hz / 1000,
        bus.clock_div,
        @tagName(bus.variant),
    });

    var dev = Dev.init(bus, pins);

    uart.print(.uart0, "1. Bus and firmware...\n", .{});
    dev.powerUp() catch |err| die(err, 0);

    var tr: u32 = 0;
    var tries: u32 = 10;
    while (tries > 0) : (tries -= 1) {
        tr = dev.readTestRegister() catch 0;
        if (tr == cyw43.gspi.test_pattern) break;
        timer.delayMs(1);
    }
    if (tr != cyw43.gspi.test_pattern) die(error.ChipNotResponding, 1);

    dev.busInit() catch |err| die(err, 2);
    dev.requestAlp() catch |err| die(err, 3);
    dev.disableCore(.wlan_arm) catch {};
    dev.disableCore(.socram) catch {};
    dev.resetCore(.socram, false) catch |err| die(err, 4);
    dev.disableSram3Remap() catch |err| die(err, 4);

    const bp = dev.backplaneSelfTest() catch |err| die(err, 5);
    if (bp.block_wrong != 0) die(error.BackplaneUnreliable, 5);

    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| die(err, 6);
    dev.waitF2Ready() catch |err| die(err, 7);
    dev.loadClmDefault() catch |err| die(err, 11);
    dev.wifiOn(.gb) catch |err| die(err, 12);

    // **Name the reboot we asked for**, so it is never mistaken for a crash.
    // `commitPendingBond` triggers one after writing a bond; anything else
    // reporting `forced` here would be a fault.
    if (rp2350.watchdog.resetReason() != .power_on) {
        uart.print(.uart0, "   (came back from a deliberate reboot after writing the bond)\n", .{});
    }

    uart.print(.uart0, "2. Starting the Bluetooth controller...\n", .{});
    var host = cyw43.BtHost(Dev).init(&dev);
    host_ptr = &host;
    host.start(cyw43.fw_btfw, 2000) catch |err| die(err, 17);

    const bd = host.commandWithResult(hci.op.read_bd_addr, &.{}) catch |err| die(err, 17);
    if (bd.len >= 6) {
        uart.print(.uart0, "   BD_ADDR ", .{});
        addrOf(bd[0..6].*);
        uart.print(.uart0, "\n", .{});
    }

    const bsp0 = host.commandWithResult(bredr.op.read_buffer_size, &.{}) catch |err| die(err, 21);
    const bs0 = bredr.parseBufferSize(bsp0) catch |err| die(err, 21);
    uart.print(.uart0, "   BR/EDR ACL {d} x {d}\n", .{ bs0.acl_len, bs0.acl_count });

    // -- the service record, built once -------------------------------------
    //
    // **`SupportedFeatures` is `speaker`, not `headphone`.** That is the one
    // field this record deliberately differs from the reference's in: btstack's
    // demo says headphone, and this device drives powered speakers.
    const rec = cyw43.sdp.writeAudioSinkRecord(
        &sdp_sink,
        0x0001_0001,
        "zig-speaker",
        "zig",
        cyw43.sdp.sink_feature.speaker,
    ) catch |err| die(err, 23);
    sdp_sink_len = rec.len;

    // **The vendor and product ids are placeholders.** Claiming another
    // vendor's assigned id would be a lie told to every device this ever pairs
    // with; these say "unassigned" as loudly as the field allows, and want
    // replacing before anything ships.
    const devid = cyw43.sdp.writeDeviceIdRecord(
        &sdp_devid,
        0x0001_0002,
        0xFFFF, // vendor: unassigned
        0x0001, // product
        0x0001, // version
        true, // primary record
        cyw43.sdp.vendor_source.bluetooth_sig,
    ) catch |err| die(err, 23);
    sdp_devid_len = devid.len;

    uart.print(.uart0, "   SDP records built: sink {d} bytes, device id {d} bytes\n", .{
        sdp_sink_len, sdp_devid_len,
    });

    // -- the bond, if this board has been paired before ---------------------
    //
    // **`verifyRamResident` is checked, not assumed.** `flash.zig` records that
    // an earlier version of itself claimed `.ramfunc` placement and did not get
    // it, with nothing in the build or the tests noticing — and calling it in
    // that state faults mid-erase. One branch at start-up against a fault that
    // would happen during pairing and look like a Bluetooth problem.
    if (!rp2350.flash.verifyRamResident()) {
        uart.print(.uart0, "   !! flash routine is not in RAM -- refusing to write bonds\n", .{});
    } else if (bond.load()) |b| {
        link_key = b.key;
        link_key_type = b.key_type;
        link_key_peer = b.peer;
        have_link_key = true;
        uart.print(.uart0, "   bond loaded from flash: {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}, key type {d}\n", .{
            b.peer[5], b.peer[4], b.peer[3], b.peer[2], b.peer[1], b.peer[0], b.key_type,
        });
    } else {
        uart.print(.uart0, "   no bond in flash -- the first phone to pair claims the slot\n", .{});
    }

    // -- the DAC ------------------------------------------------------------
    //
    // Started before anything connects, so it is streaming silence by the time
    // the first packet arrives: bringing I2S up underneath a running stream
    // would put the gap on the speakers.
    audio = pcm5100.PicoAudio.init(pcm5100.default_pins, pcm5100.default_sample_rate, sys_hz);
    audio.start() catch |err| die(err, 26);
    watch.audio_state |= audio_state.started;

    // **The amplifier last, and only after `start` returned.** Enabling it
    // before BCLK is stable puts a click through whatever is plugged in;
    // `lib/pcm5100` says so in its own header and it is not our click to spend.
    audio.enableAmplifier();
    watch.audio_state |= audio_state.amp_on;
    last_playing = audio.playingIndex();

    const i2s_plan = pcm5100.ClockPlan.compute(sys_hz, pcm5100.default_sample_rate);
    uart.print(.uart0, "   I2S running at {d} Hz ({d} ppm), amp on\n", .{
        pcm5100.default_sample_rate,
        i2s_plan.errorPpm(pcm5100.default_sample_rate),
    });

    // -- discoverable, as rung 21b left it ----------------------------------
    //
    // **The same call the controller gets after a restart.** It carries the
    // page timeout too — longer than the 0x2000 default, for the reason
    // `pageTimeoutParams` gives: the default is tuned for a device you have
    // just told to connect, not for one guessing where a phone is.
    uart.print(.uart0, "3. Becoming discoverable as \"{s}\"...\n", .{local_name});
    configureController(&host) catch |err| die(err, 21);
    watch.rung = 0x21B;
    watch.statuses |= 0xFF00_0000; // no role reported yet

    if (have_link_key) {
        paging.peer = link_key_peer;
        paging.next_us = timer.now64();
        uart.print(.uart0, "   a bond is held -- will page that phone rather than wait for it\n", .{});
    } else {
        paging.done = true;
    }

    if (have_link_key) {
        uart.print(.uart0,
            \\
            \\   A phone is already bonded, so nothing needs doing on it: this
            \\   board is paging that phone now and it should connect on its own
            \\   within about ten seconds. Four attempts, then it gives up and
            \\   waits to be connected to instead.
            \\
        , .{});
    } else {
        uart.print(.uart0,
            \\
            \\   On the phone: Settings > Connected devices > Pair new device,
            \\   then tap "{s}".
            \\
            \\   The bond is written to flash, so a power cycle keeps it -- and
            \\   from then on this board pages the phone itself rather than
            \\   waiting. One slot: the last phone to pair owns it.
            \\
        , .{local_name});
    }

    // -- one loop, everything through it ------------------------------------
    var idle: u32 = 0;
    const boot_us = timer.now64();
    var pass_start = boot_us;
    while (true) {
        const read_start = timer.now64();
        phase.set(phase.after_read);
        const n = host.bt.read(&rx) catch |err| {
            const code: u32 = switch (err) {
                error.QueueFull => 1,
                error.PacketTooLarge => 2,
                error.NoHeaderRoom => 3,
                error.BtNotReady => 4,
                else => 9,
            };
            watch.read_errors = ((watch.read_errors & 0xFFFF) +% 1) | (code << 16);
            uart.print(.uart0, "[bt] read: {s}\n", .{@errorName(err)});
            timer.delayMs(50);
            continue;
        };
        // **How long the bus itself took**, in `media_bytes`'s high half.
        // The loop's worst pass is 25.7 ms — four buffer periods — and the fix
        // is a different one depending on where that goes. If it is mostly
        // here, the gSPI read is blocking through the deadline and the audio
        // has to be serviced from somewhere that is not this loop. If it is
        // not, it is our own processing and can be broken up.
        const read_us: u32 = @intCast(@min(timer.now64() -% read_start, 0xFFFF));
        if (read_us > (watch.media_bytes >> 16)) {
            watch.media_bytes = (watch.media_bytes & 0xFFFF) | (read_us << 16);
        }

        // **Split empty reads from packet reads, and roadmap 2.6 becomes a
        // measurement rather than an A/B.** A read that returns nothing did
        // exactly one thing: fetch the two receive-ring indices in a single
        // 8-byte transfer. So its average *is* the cost of one index fetch, on
        // this bus, at this divisor, measured rather than modelled.
        //
        // That is the whole of what 0.4 removed — it used to fetch the indices
        // a second time before reading the payload — so the saving is
        // `packets x this average`, with no second binary and no second run to
        // hold conditions equal between.
        if (n == 0) {
            empty_reads +%= 1;
            empty_read_us +%= read_us;
        } else {
            packet_reads +%= 1;
            packet_read_us +%= read_us;
        }

        // **Every pass, before anything else.** The DMA does not wait for
        // Bluetooth to be quiet, and a fill that happens only on idle
        // iterations is a fill that stops happening exactly when the stream is
        // busiest.
        serviceAudio();

        // **Catch the controller's firmware stopping, at the moment it stops.**
        //
        // `fw_ready` low means the CYW43's Bluetooth firmware has died. Every
        // downstream symptom follows from it and none of them names it: the
        // h2b ring stops being consumed, so no command ever completes, so
        // command credits never come back, so `Accept_Connection_Request`
        // never runs and the peer sees `accept timeout` -- which reads as a
        // board that answers pages and will not connect.
        //
        // It was found in a 30-second wedge dump, which localises it to
        // "somewhere in the last half minute" -- useless for pinning a cause.
        // One backplane read every 50 ms costs about 0.04% of the loop and
        // pins it to the packet that preceded it, which is printed alongside.
        const now_fw = timer.now64();
        if (now_fw -% last_fw_check_us >= 50_000) {
            last_fw_check_us = now_fw;
            // **The raw register and the read's own success, separately.** A
            // failed backplane read and a cleared `fw_ready` are different
            // faults wanting opposite investigations, and conflating them
            // produced 322 "firmware stopped" transitions in 16 seconds.
            const ctrl: ?u32 = host.bt.btControl() catch |err| blk: {
                if (fw_reads_failed == 0) {
                    uart.print(.uart0, "!! bt_ctrl read failed: {s}\n", .{@errorName(err)});
                }
                fw_reads_failed +%= 1;
                break :blk null;
            };

            if (ctrl) |v| {
                const ready = (v & 0x0100_0000) != 0; // btbus.bit.fw_ready
                if (!ready and fw_was_ready) {
                    fw_was_ready = false;
                    const w = host.bt.last_write;
                    const d = host.bt.diagnostics() catch null;
                    uart.print(.uart0, "\n!! fw_ready CLEAR at {d} ms, bt_ctrl {X:0>8}, read fails {d}\n", .{
                        (now_fw -% boot_us) / 1000, v, fw_reads_failed,
                    });
                    uart.print(.uart0, "   last write {d} bytes: {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{
                        host.bt.last_write_len, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
                    });
                    if (d) |dd| {
                        uart.print(.uart0, "   b2h {d}/{d}, h2b {d}/{d}, pkts {d}\n", .{
                            dd.b2h_in, dd.b2h_out, dd.h2b_in, dd.h2b_out, dd.packets_read,
                        });
                    }
                    uart.print(.uart0, "   rung 0x{X}, events {d}, credits {d}, queued {d}\n", .{
                        watch.rung, watch.events, host.cmd_credits, host.cmd_queued,
                    });
                } else if (ready and !fw_was_ready) {
                    fw_was_ready = true;
                    uart.print(.uart0, "!! fw_ready set again, bt_ctrl {X:0>8}\n", .{v});
                }
            }
        }

        const pass_us: u32 = @intCast(@min(timer.now64() -% pass_start, 0xFFFF));
        if (pass_us > (watch.timing_us & 0xFFFF)) {
            watch.timing_us = (watch.timing_us & 0xFFFF_0000) | pass_us;
        }
        pass_start = timer.now64();

        if (n == 0) {
            phase.set(phase.idle);
            timer.delayMs(1);
            idle +%= 1;
            // **Written every pass, not every three thousand.** The coarse
            // version could not distinguish "idling normally" from "wedged",
            // which is the exact question that cost two flash cycles: a
            // heartbeat that only ticks every thirty seconds is not a
            // heartbeat.
            watch.ticks = idle;
            servicePaging(&host);
            // **Elapsed time, measured rather than counted.** The label used
            // to be `idle / 100`, from when the delay here was 10 ms; it stayed
            // behind when the delay became 1 ms and every "[600s]" was really
            // about a minute. A tick count is not a clock as soon as the loop
            // body changes.
            if (idle % 3000 == 0) {
                const secs = (timer.now64() -% boot_us) / 1_000_000;
                uart.print(.uart0, "   [{d}s] waiting, {d} events\n", .{ secs, watch.events });

                // **A quiet bus and a wedged one look identical from here**,
                // and telling them apart by guesswork has now cost two flash
                // cycles and two plausible fixes that were right about
                // something else. If nothing has arrived for a while, say what
                // the chip actually holds.
                //
                // The reading that matters is `b2h in` against `out`: **equal
                // means the controller has produced nothing** and the fault is
                // on its side of the bus; unequal, with nothing coming out of
                // `read`, means it is on ours. Everything tried so far assumed
                // one of those without checking which.
                // **Events alone are the wrong signal, and the range test
                // proved it**: a healthy stream produces media, which is ACL
                // data and not an event, so `watch.events` sits still for
                // minutes while everything is fine. The first run of this
                // printed "quiet 30s" twice in the middle of perfect playback
                // -- with `pkts` climbing 6,144 to 9,918 in the dump itself,
                // which is the diagnostic contradicting its own headline.
                //
                // Packets read is the honest measure of "anything arriving at
                // all", and it covers both.
                const traffic = watch.events +% watch.acl;
                if (traffic == last_traffic) {
                    quiet_passes +%= 1;
                } else {
                    quiet_passes = 0;
                    last_traffic = traffic;
                }
                if (quiet_passes >= quiet_passes_before_dump and
                    quiet_passes % quiet_passes_before_dump == 0)
                {
                    if (host.bt.diagnostics()) |d| {
                        uart.print(.uart0, "   ! quiet {d}s: b2h {d}/{d}, h2b {d}/{d}\n", .{
                            quiet_passes * 3, d.b2h_in, d.b2h_out, d.h2b_in, d.h2b_out,
                        });
                        uart.print(.uart0, "   ! bt_ctrl {X:0>8}, pkts {d}, short {d}, big {d}\n", .{
                            d.bt_ctrl, d.packets_read, d.packets_short, d.packets_too_large,
                        });
                        uart.print(.uart0, "   ! cmds credits {d}, queued {d}, delayed {d}, dropped {d}\n", .{
                            host.cmd_credits, host.cmd_queued, host.cmd_delayed, host.cmd_dropped,
                        });
                    } else |err| {
                        uart.print(.uart0, "   ! quiet {d}s: bus read failed: {s}\n", .{
                            quiet_passes * 3, @errorName(err),
                        });
                    }
                }
            }
            continue;
        }

        // **ACL first, because it is not an event and the first version of this
        // loop lost it.** `parseEvent` refuses a non-event packet and the
        // `catch continue` below discarded the error, so every L2CAP frame the
        // phone sent vanished without a counter moving. The phone opens SDP
        // immediately after pairing; that traffic is exactly what was being
        // dropped, and the disconnect that followed was the phone concluding
        // this device has no services.
        if (rx[3] == 0x02) {
            phase.set(phase.acl);
            watch.acl = ((watch.acl & 0xFFFF) +% 1) | (watch.acl & 0xFFFF_0000);
            handleAcl(&host, rx[0..n]);
            continue;
        }

        phase.set(phase.event);
        const ev = hci.parseEvent(rx[0..n]) catch continue;
        watch.events += 1;

        switch (ev.code) {
            bredr.event.connection_request => {
                const req = bredr.parseConnectionRequest(ev.params) catch continue;
                watch.steps |= step.request;
                // The phone got there first. Both ends may reach for the link
                // at once after a power cycle; whoever wins, there is exactly
                // one connection and no reason to keep paging.
                stopPaging("the phone connected to us first");
                watch.peer_lo = @as(u32, req.peer[0]) | @as(u32, req.peer[1]) << 8 |
                    @as(u32, req.peer[2]) << 16 | @as(u32, req.peer[3]) << 24;

                uart.print(.uart0, "\n>> CONNECTION_REQUEST from ", .{});
                addrOf(req.peer);
                uart.print(.uart0, " -- accepting\n", .{});

                // Remain the peripheral: a sink has no reason to be central,
                // and a role switch is one more thing to fail before there is
                // any evidence the rest works.
                const p = bredr.acceptConnectionParams(req.peer, true);
                send(&host, bredr.op.accept_connection_request, &p);
            },

            bredr.event.connection_complete => {
                const cc = bredr.parseConnectionComplete(ev.params) catch continue;
                watch.statuses |= @as(u32, cc.status);
                const was_paging = paging.in_flight;
                paging.in_flight = false;
                if (cc.status == 0) {
                    watch.steps |= step.connected;
                    watch.conn_handle = cc.handle;
                    uart.print(.uart0, "   CONNECTED, handle 0x{X:0>4}\n", .{cc.handle});

                    // Off inquiry scan for the duration of the link.
                    setScanEnable(&host, .page);
                    if (was_paging) {
                        watch.steps |= step.page_answered;
                        stopPaging("the phone answered");

                        // **The initiator has to ask.** In every rung before
                        // this one the phone was the initiator and it drove
                        // authentication, so this command has never been sent
                        // from here. Paging reverses the roles, and a link
                        // that is never authenticated is never encrypted --
                        // and A2DP will not run unencrypted. What comes back
                        // is `Link_Key_Request`, which is already handled and
                        // will be answered out of flash.
                        watch.steps |= step.we_authenticated;
                        we_paged_this_link = true;
                        var hp: [2]u8 = undefined;
                        std.mem.writeInt(u16, &hp, cc.handle, .little);
                        uart.print(.uart0, "   we paged, so we ask for authentication\n", .{});
                        send(&host, bredr.op.authentication_requested, &hp);

                        // **And ask to be the peripheral again.** Paging made
                        // us the central, and the central schedules the
                        // piconet — so the role decides whose timing governs
                        // delivery. Every rung before paging ran as the
                        // peripheral because the phone always initiated, and
                        // that is the topology an album played through with
                        // zero dropped frames. Asked here, before a stream
                        // exists, because a link carrying one may not be
                        // switchable at all.
                        const sr = bredr.switchRoleParams(cc.peer, true);
                        send(&host, bredr.op.switch_role, &sr);
                    }
                } else {
                    // 0x10 is the accept timeout. 0x04 is the page timeout,
                    // which is what a phone switched off or out of range looks
                    // like -- the common failure now that we page.
                    uart.print(.uart0, "   connection failed, status 0x{X:0>2}{s}\n", .{
                        cc.status,
                        if (cc.status == 0x10)
                            " (accept timeout)"
                        else if (cc.status == 0x04)
                            " (page timeout -- phone not reachable)"
                        else
                            "",
                    });
                    if (was_paging) {
                        step.recordPageFailure(cc.status);
                        paging.next_us = timer.now64() +% paging.gap_us;
                    }
                }
            },

            bredr.event.link_key_request => {
                if (ev.params.len < 6) continue;
                const peer = ev.params[0..6].*;

                // **The bond answers for one device, and this used to answer
                // for all of them.** `have_link_key` was the whole test, so the
                // single key held was handed to whoever asked -- and a device
                // given a key it has never seen fails authentication and gives
                // up. With one phone that is invisible; the first time a second
                // device tries, it simply "won't connect", with the log
                // cheerfully reporting that a key was supplied.
                //
                // `link_key_peer` was already loaded from flash and already
                // used to decide who to *page*. It was never consulted here.
                const ours = have_link_key and std.mem.eql(u8, &peer, &link_key_peer);

                if (ours) {
                    uart.print(.uart0, "   link key requested -- supplying the one we hold\n", .{});
                    const p = bredr.linkKeyReplyParams(peer, link_key);
                    send(&host, bredr.op.link_key_request_reply, &p);
                } else {
                    // **The negative reply is what starts pairing.** Silence
                    // here stalls the phone until it gives up.
                    //
                    // Naming which case this is, because they mean different
                    // things: no bond at all is a fresh board, whereas a bond
                    // for someone else means **this device is about to displace
                    // the phone** -- one slot, by design, and the last to pair
                    // owns it.
                    uart.print(.uart0, "   link key requested -- {s}, pairing\n", .{
                        if (have_link_key) "our bond is another device's" else "we have none",
                    });
                    send(&host, bredr.op.link_key_request_negative_reply, &peer);
                }
            },

            bredr.event.io_capability_request => {
                if (ev.params.len < 6) continue;
                watch.steps |= step.io_cap;
                uart.print(.uart0, "   IO capability asked: NoInputNoOutput, general bonding\n", .{});
                const p = bredr.ioCapabilityReplyParams(
                    ev.params[0..6].*,
                    .no_input_no_output,
                    bredr.auth.general_bonding,
                );
                send(&host, bredr.op.io_capability_request_reply, &p);
            },

            bredr.event.io_capability_response => {
                if (ev.params.len >= 9) {
                    uart.print(.uart0, "   peer capability {d}, auth {d}\n", .{
                        ev.params[6], ev.params[8],
                    });
                }
            },

            bredr.event.user_confirmation_request => {
                const r = bredr.parseUserConfirmationRequest(ev.params) catch continue;
                watch.steps |= step.user_confirm;
                // Just works: there is no display to show the six digits on and
                // no button to confirm them with, so the host answers for the
                // user. Printing the value is diagnostic, not a comparison.
                uart.print(.uart0, "   user confirmation ({d:0>6}) -- accepting automatically\n", .{
                    r.numeric_value % 1_000_000,
                });
                send(&host, bredr.op.user_confirmation_request_reply, &r.peer);
            },

            bredr.event.simple_pairing_complete => {
                if (ev.params.len < 1) continue;
                watch.statuses |= @as(u32, ev.params[0]) << 8;
                watch.steps |= step.pairing_complete;
                uart.print(.uart0, "   simple pairing complete, status 0x{X:0>2}\n", .{ev.params[0]});
            },

            bredr.event.link_key_notification => {
                const nk = bredr.parseLinkKeyNotification(ev.params) catch continue;
                link_key = nk.key;
                link_key_type = nk.key_type;
                link_key_peer = nk.peer;
                have_link_key = true;
                watch.steps |= step.link_key;

                var sum: u32 = 0;
                for (nk.key) |b| sum +%= b;
                // Type and checksum only. Never the key.
                //
                // **Held, not written — the write happens on disconnect.**
                // `bond.save` masks interrupts and holds the CPU for tens of
                // milliseconds. `bond.zig`'s header argues that is safe here
                // because pairing has no stream to interrupt, and it reasons
                // about the audio DMA. **It does not reason about the
                // Bluetooth controller**, and that is the case that bites: the
                // host stops servicing the gSPI interface for the whole erase,
                // and the CYW43's Bluetooth firmware resets itself.
                //
                // Measured 2026-08-27 pairing a MacBook: `fw_ready` went low
                // in the poll immediately after this line, with the rings
                // drained and command credits healthy — so the stall, not the
                // traffic. A restarted firmware no longer agrees with our
                // ring state, so nothing is consumed from `h2b` again, no
                // command ever completes, credits never return, and every
                // later connection dies at `accept timeout`.
                //
                // Nothing needs the key on flash before the link ends. The
                // exposure this trades for is a power loss between pairing and
                // disconnect, which costs a re-pair.
                pending_bond = .{ .peer = nk.peer, .key_type = nk.key_type, .key = nk.key };
                uart.print(.uart0, "   link key stored (type {d}, checksum 0x{X:0>4}) -- flash write deferred\n", .{
                    nk.key_type, sum,
                });
            },

            bredr.event.authentication_complete => {
                if (ev.params.len < 1) continue;
                watch.steps |= step.auth_complete;
                uart.print(.uart0, "   authentication complete, status 0x{X:0>2}\n", .{ev.params[0]});

                // **And now we have to ask for encryption too.** The same
                // reversal as `Authentication_Requested`: as a responder the
                // phone drove this and the link came up encrypted without this
                // code doing anything, so the omission was invisible until we
                // paged. An authenticated but unencrypted link is one A2DP
                // will not start on -- the phone connects, engages at L2CAP,
                // and then simply never asks for a stream, which reads as
                // "connected, playing out of the phone's own speaker".
                if (ev.params[0] == 0 and ev.params.len >= 3 and we_paged_this_link) {
                    const handle = std.mem.readInt(u16, ev.params[1..3], .little) & 0x0FFF;
                    var p: [3]u8 = undefined;
                    std.mem.writeInt(u16, p[0..2], handle, .little);
                    p[2] = 0x01; // on
                    uart.print(.uart0, "   we paged, so we ask for encryption\n", .{});
                    send(&host, bredr.op.set_connection_encryption, &p);
                }
            },

            bredr.event.encryption_change => {
                const ec = bredr.parseEncryptionChange(ev.params) catch continue;
                watch.statuses |= @as(u32, ec.status) << 16;
                if (ec.status == 0 and ec.enabled) {
                    watch.steps |= step.encrypted;
                    watch.rung = 22;
                    uart.print(.uart0, "\n   ENCRYPTED.\n", .{});

                    // **If we paged, nothing else will happen unless we make
                    // it happen.** A phone that initiated the link goes on to
                    // open AVDTP itself; a phone that merely answered a page
                    // does not, and it does not complain either -- it sits on
                    // a working encrypted link, shows "connected", and keeps
                    // playing out of its own speaker. Measured, not assumed:
                    // rung 27 reached exactly this point and stopped.
                    if (we_paged_this_link) openAvdtpSignalling(&host);
                } else {
                    uart.print(.uart0, "   encryption change: status 0x{X:0>2}, enabled {}\n", .{
                        ec.status, ec.enabled,
                    });
                }
            },

            bredr.event.pin_code_request => {
                // Legacy pairing. With SSP on both ends this should never
                // arrive; refusing is better than a PIN this device cannot ask
                // anyone for.
                if (ev.params.len < 6) continue;
                uart.print(.uart0, "   PIN requested -- refusing, this device has no keypad\n", .{});
                send(&host, bredr.op.pin_code_request_negative_reply, ev.params[0..6]);
            },

            hci.event.disconnection_complete => {
                watch.disconnects = ((watch.disconnects & 0xFFFF) +% 1);
                const reason: u8 = if (ev.params.len >= 4) ev.params[3] else 0;
                watch.disconnects = (watch.disconnects & 0xFFFF) | (@as(u32, reason) << 16);
                chans = @splat(.{});
                next_local_cid = cyw43.bredr.first_dynamic_cid;
                watch.l2cap_state &= 0xFFFF;
                acl_credits = host.acl_max;

                // **The stream must be forgotten, and this is what broke
                // reconnection.** `DISCOVER` reports the endpoint's in-use flag
                // as `stream != null`, so a configuration left over from the
                // previous session advertises the only endpoint this device has
                // as busy. The phone reconnects, encrypts, asks what is
                // available, is told the one thing it wants is in use, and
                // leaves — which reads as "briefly connected, then Saved" and
                // looks nothing like a stale bit of state.
                stream = null;
                watch.avdtp_state = 0;

                // **The deferred bond write, now that no link can be hurt by
                // it.** See `link_key_notification` for why it is not done
                // where the key arrives.
                commitPendingBond(&host);

                // A half-assembled PDU belongs to the link that just ended;
                // carrying it forward would splice it onto the next one.
                reasm.discard();

                // **And so does "we paged this one".** Leaving it set made the
                // next connection -- which the phone initiates, and for which
                // it opens AVDTP itself -- open a second signalling channel on
                // PSM 0x0019. The phone left rather than choose between them.
                we_paged_this_link = false;

                // Discoverable again: whatever this link was, it is over, and a
                // phone that has never seen this board has to be able to find
                // it. Paired with the `page` at connect.
                setScanEnable(&host, .both);

                // **Re-page only when the link was lost, never when it was
                // ended.** The rule here used to be "no re-page, ever": a
                // disconnect is nearly always the user's doing -- they tapped
                // disconnect, or switched Bluetooth off -- and a speaker that
                // pages straight back is one fighting its owner.
                //
                // That is still right for `0x13` (remote user terminated) and
                // `0x15` (remote device powered off), which are deliberate
                // acts. **It was wrong for `0x08`**, Connection Timeout: the
                // supervision timer expired because the phone went out of
                // range, which is the one case where nobody chose to leave and
                // where coming back into range should just work. Found by
                // walking out of range with music playing and returning to a
                // board that sat there -- 2026-08-27.
                //
                // The attempt counter resets, so this is the same four tries
                // over about a minute that power-on gets, not an endless page.
                if (reason == 0x08) {
                    paging.peer = link_key_peer;
                    paging.attempts = 0;
                    paging.in_flight = false;
                    paging.done = !have_link_key;
                    paging.next_us = timer.now64() +% paging.gap_us;
                    if (have_link_key) {
                        uart.print(.uart0, "   link lost, not ended -- will page that phone back\n", .{});
                    }
                } else {
                    paging.done = true;
                }

                gate.mute();

                // Stale filter history and stale audio both belong to a stream
                // that has ended. Carrying them into the next one is a glitch
                // at its start, and the ring would play the old track's tail.
                decoder.reset();
                ring = .{};
                uart.print(.uart0, "   disconnected, reason 0x{X:0>2}{s}\n", .{
                    reason,
                    if (reason == 0x13) " (remote user terminated -- the phone chose to leave)" else "",
                });
            },

            hci.event.command_status => {
                const cs = hci.parseCommandStatus(ev) catch continue;
                host.creditsFrom(cs.credits);
                if (cs.status != 0) {
                    uart.print(.uart0, "   ! command {X:0>4} refused, status 0x{X:0>2}\n", .{
                        cs.opcode, cs.status,
                    });
                }
            },

            hci.event.command_complete => {
                const cc = hci.parseCommandComplete(ev) catch continue;
                // **Before the status check, and unconditionally.** A refused
                // command still returns its allowance, and skipping the credit
                // on a failure would stall the queue behind it for good.
                host.creditsFrom(cc.credits);
                if (cc.opcode == bredr.op.write_scan_enable and cc.status == 0) {
                    scan_mode = scan_requested;
                }
                if (cc.status != 0) {
                    uart.print(.uart0, "   ! command {X:0>4} failed, status 0x{X:0>2}\n", .{
                        cc.opcode, cc.status,
                    });
                }
            },

            // Harmless and expected: slot changes, role changes, remote feature
            // reports. Counted so the tail is visible rather than assumed empty.
            // **Credits back.** This example writes ACL packets directly
            // rather than through `Host`'s command path, so it owes itself the
            // flow control `Host.absorb` would have done. Without this the
            // eight buffers drain once and `sendSig` refuses everything after,
            // which looks exactly like a peer that stopped listening.
            hci.event.number_of_completed_packets => {
                const done = hci.completedPackets(ev);
                acl_credits = @min(host.acl_max, acl_credits + done);
            },

            bredr.event.role_change => {
                // **Not ignorable now that we page.** Paging makes this device
                // the central, which is the reverse of every link the audio
                // path was tuned on, and the central is the end that schedules
                // the link. `createConnectionParams` allows the switch back;
                // whether the phone takes it is a fact about the phone.
                if (ev.params.len >= 8) {
                    // **Recorded, because the UART is a lash-up and this is
                    // now a load-bearing fact.** Bits 24..31 of `statuses`:
                    // 0 central, 1 peripheral, 0xFF never reported.
                    watch.statuses = (watch.statuses & 0x00FF_FFFF) |
                        (@as(u32, ev.params[7]) << 24);
                    uart.print(.uart0, "   role change: we are now {s}\n", .{
                        if (ev.params[7] == 0) "CENTRAL" else "peripheral",
                    });
                }
            },

            bredr.event.max_slots_change,
            bredr.event.read_remote_supported_features,
            bredr.event.page_scan_repetition_mode_change,
            bredr.event.remote_name_request_complete,
            => {},

            else => {
                uart.print(.uart0, "   [ev 0x{X:0>2}, {d} params]\n", .{ ev.code, ev.params.len });
            },
        }
    }
}

const std = @import("std");
