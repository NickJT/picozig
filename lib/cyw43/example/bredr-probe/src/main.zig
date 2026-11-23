//! bredr-probe — rung 21: does this controller do Bluetooth Classic?
//!
//! Build with `-Dfw=combined`, like `bt-probe`. The WiFi-only image has no
//! Bluetooth subsystem at all.
//!
//! ## Why this exists
//!
//! An A2DP sink — the thing that would let a phone play music through a Pico
//! Audio Pack — needs BR/EDR, and this package has none of it. Before any of
//! that tower is built, one question has to be answered on hardware: **is
//! Classic present in this chip's firmware, or is the Bluetooth side of this
//! board LE-only?** The CYW43439 is sold as Bluetooth 5.2, which includes
//! Classic, but the patchram is Infineon's build and a datasheet is not a
//! measurement.
//!
//! This rung can therefore return **no**, and that is its value. It is one
//! flash cycle against a project that would otherwise be weeks.
//!
//! ## What passing means
//!
//! **21a — the controller's own answer.** `Read_Local_Supported_Features` comes
//! back with feature No. 37 clear, and `Read_Buffer_Size` — the BR/EDR command
//! at 0x1005, not the LE one this driver already sends — reports ACL buffers
//! far larger than the 27-byte LE ones. Both are values this driver cannot
//! invent.
//!
//! **21b — an independent observer.** The board writes a loudspeaker class of
//! device, a name and scan enable, and an Android phone lists it under Pair new
//! device. Tapping it fails — nothing accepts a connection, that is rung 22 —
//! but **the `Connection_Request` reaches us first**, and printing the phone's
//! own BD_ADDR is this rung's best evidence. A cached list entry cannot page a
//! radio: an event arriving at this driver's HCI layer, carrying an address
//! nothing here could invent, has only one possible source. That is why the
//! `ntp-client` disappearance test 17b needed was not run here.
//!
//! **Both ran on 2026-08-22.** A phone answered, class `0x5A420C` — major
//! class Phone, minor Smartphone, the pager describing itself.
//!
//! **The icon was a headset, not a speaker**, and this note replaces the
//! prediction that used to stand here so the next reader is not misled by it.
//! The encoding is correct — minor 5 in bits 7..2 is `0x14` — but Android does
//! not resolve the Audio/Video minor class into distinct icons. An audio glyph
//! of any kind still proves the phone read our *major* class out of bytes this
//! board wrote; nothing observed supports a claim about the minor class
//! reaching a UI. btstack's `a2dp_sink_demo` uses `0x240400`, uncategorised
//! minor, which now reads as knowing that rather than as a shortcut.
//!
//! ## What is not proven by this
//!
//! Everything that matters afterwards. No connection is accepted, nothing
//! pairs, no L2CAP channel opens, there is no SDP record, so no phone will
//! ever offer audio to it. Above all, **this measures no throughput**: a
//! reported buffer count is not 41 KB/s sustained through the shared bus with
//! the I2S DMA running, and that measurement is a separate rung with its own
//! rig.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const bredr = cyw43.bredr;
const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// What the board calls itself over Classic. Deliberately **not**
/// `zig-pico2w`: that name belongs to the LE advertising in `bt-probe`, and two
/// rungs answering to the same name on the same phone is exactly how a stale
/// cache gets mistaken for a result.
const local_name = "zig-speaker";

/// Published over SWD. Thirty-two words, because `.faultinfo` is pinned at
/// 0x20000080 for every image in this workspace.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// The station MAC, low four then high two. BD_ADDR should be this plus
    /// one — the same cross-check `bt-probe` makes.
    mac_lo: u32,
    mac_hi: u32,
    bd_lo: u32,
    bd_hi: u32,
    /// `BT_CTRL_REG` when the firmware declared itself ready — 0x01000100.
    bt_ctrl: u32,
    /// Commands sent, events read, and the OR of every Command Complete status.
    sent: u32,
    got: u32,
    status_all: u32,
    /// **The answer.** The eight LMP feature bytes, low four then high four.
    features_lo: u32,
    features_hi: u32,
    /// Decoded, so a single word read over SWD says yes or no without needing
    /// the bit positions to hand: 1 = Classic present, 0 = LE only.
    classic: u32,
    ssp: u32,
    /// `Read_Buffer_Size`: ACL payload bytes and how many buffers.
    acl_len: u32,
    acl_count: u32,
    /// The LE figures, for contrast. These are what the driver has been using.
    le_acl_len: u32,
    le_acl_count: u32,
    /// Rung 21b. Non-zero once scan enable has been written.
    discoverable: u32,
    /// A phone that paged us: how many requests, and the last one's address.
    conn_requests: u32,
    peer_lo: u32,
    peer_hi: u32,
    /// The class of device the pager claimed — a phone reads as 0x5A020C or
    /// similar, major class Phone.
    peer_cod: u32,
    reserved: [7]u32,
};

pub const watch_magic: u32 = 0x4254_3231; // 'BT21'

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
        // **This one was missing, and it is why the first run of this probe
        // reported 99.** `buildCommand` returns it when a command's parameters
        // do not fit the host's buffer, which `Write_Local_Name`'s 248 bytes
        // did not. With no UART attached, code 99 says only "an error not on
        // this list" — the table has to name every error its own call graph
        // can produce, or the watch block is a diagnostic with a hole in it.
        error.BufferTooSmall => 51,
        // Rung 21's own: the controller answered, and the answer was no.
        error.ClassicNotSupported => 60,
        else => 99,
    };
    while (true) timer.delayMs(1000);
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch = std.mem.zeroes(Watch);
    watch.magic = watch_magic;

    uart.print(.uart0, "\n=== CYW43439 BR/EDR probe (rung 21) ===\nboard {s}\n", .{board.name});

    if (cyw43.fw_wifi.len != 231077) {
        uart.print(.uart0, "\nThis needs -Dfw=combined. The WiFi-only image has no BT subsystem.\n", .{});
        die(error.BadPatchram, 21);
    }

    const pins = cyw43.default_pins;
    const bus = cyw43.PicogSpiBus.init(pins);
    var dev = Dev.init(bus, pins);

    // -- the ladder underneath ---------------------------------------------
    //
    // **This is `bt-probe`'s bring-up sequence unchanged**, including the WiFi
    // side. A Bluetooth-only device does not need `loadClm` or `wifiOn`, and
    // trimming them is a fair later optimisation — but not in the same flash
    // cycle as the question this rung is asking. The proven sequence stays
    // proven; one variable changes at a time.
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
    watch.mac_lo = @as(u32, dev.mac[0]) | @as(u32, dev.mac[1]) << 8 |
        @as(u32, dev.mac[2]) << 16 | @as(u32, dev.mac[3]) << 24;
    watch.mac_hi = @as(u32, dev.mac[4]) | @as(u32, dev.mac[5]) << 8;
    uart.print(.uart0, "   station MAC {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
        dev.mac[0], dev.mac[1], dev.mac[2], dev.mac[3], dev.mac[4], dev.mac[5],
    });

    uart.print(.uart0, "2. Starting the Bluetooth controller...\n", .{});
    var host = cyw43.BtHost(Dev).init(&dev);
    host.start(cyw43.fw_btfw, 2000) catch |err| {
        uart.print(.uart0, "   start failed after {d} records, {d} polls\n", .{
            host.bt.patch_records, host.bt.ready_polls,
        });
        die(err, 17);
    };
    watch.bt_ctrl = dev.backplaneRead32(cyw43.btbus.reg.bt_ctrl) catch 0;
    uart.print(.uart0, "   bt_ctrl 0x{X:0>8} (expect 0x01000100)\n", .{watch.bt_ctrl});

    // `start` already asked `LE_Read_Buffer_Size`, so these are on record
    // before the BR/EDR figures arrive to be compared against them.
    watch.le_acl_len = host.acl_payload_max;
    watch.le_acl_count = host.acl_max;

    const bd = host.commandWithResult(cyw43.hci.op.read_bd_addr, &.{}) catch |err| die(err, 17);
    if (bd.len >= 6) {
        watch.bd_lo = @as(u32, bd[0]) | @as(u32, bd[1]) << 8 |
            @as(u32, bd[2]) << 16 | @as(u32, bd[3]) << 24;
        watch.bd_hi = @as(u32, bd[4]) | @as(u32, bd[5]) << 8;
        uart.print(.uart0, "   BD_ADDR {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
            bd[5], bd[4], bd[3], bd[2], bd[1], bd[0],
        });
    }
    watch.rung = 17;

    // -- rung 21a: the controller's own answer ------------------------------
    uart.print(.uart0, "\n3. Asking whether Classic is there at all...\n", .{});

    const fp = host.commandWithResult(bredr.op.read_local_supported_features, &.{}) catch |err|
        die(err, 21);
    const features = bredr.parseFeatures(fp) catch |err| die(err, 21);

    watch.features_lo = @as(u32, features.bits[0]) | @as(u32, features.bits[1]) << 8 |
        @as(u32, features.bits[2]) << 16 | @as(u32, features.bits[3]) << 24;
    watch.features_hi = @as(u32, features.bits[4]) | @as(u32, features.bits[5]) << 8 |
        @as(u32, features.bits[6]) << 16 | @as(u32, features.bits[7]) << 24;
    watch.classic = @intFromBool(features.classic());
    watch.ssp = @intFromBool(features.secureSimplePairing());

    uart.print(.uart0, "   LMP features {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{
        features.bits[0], features.bits[1], features.bits[2], features.bits[3],
        features.bits[4], features.bits[5], features.bits[6], features.bits[7],
    });
    uart.print(.uart0, "   byte 4 = 0x{X:0>2}: BR/EDR-not-supported {s}, LE-supported {s}\n", .{
        features.bits[4],
        if ((features.bits[4] & (1 << 5)) != 0) "SET" else "clear",
        if (features.le()) "set" else "CLEAR",
    });
    uart.print(.uart0, "   -> Bluetooth Classic: {s}\n", .{
        if (features.classic()) "PRESENT" else "ABSENT",
    });
    uart.print(.uart0, "   -> Secure Simple Pairing: {s}\n", .{
        if (features.secureSimplePairing()) "present" else "ABSENT",
    });

    // **The gate.** Everything downstream — pairing, L2CAP, SDP, AVDTP, an SBC
    // decoder — is unbuildable on this chip if this bit says so, and finding
    // that out here is what the rung is for.
    if (!features.classic()) {
        uart.print(.uart0,
            \\
            \\STOP. This controller reports BR/EDR Not Supported.
            \\
            \\A2DP is a Classic profile, so no amount of work above this layer
            \\reaches it on this firmware. If the goal stands, the next question
            \\is whether a different patchram build exists for this part -- not
            \\what to write next.
            \\
        , .{});
        die(error.ClassicNotSupported, 21);
    }

    // Recorded, not gated on. The bit positions in this table are numerous and
    // easy to misread, and a probe that asserts a hand-derived bit position is
    // asserting its author's memory. **The commands below are issued for real
    // instead** — a status of 0 from `Write_Scan_Enable` is better evidence
    // that `Write_Scan_Enable` works than a bit claiming it would.
    const cmds = host.commandWithResult(bredr.op.read_local_supported_commands, &.{}) catch |err|
        die(err, 21);
    uart.print(.uart0, "   supported-commands bitmap, {d} bytes, octets 0/1/8/9/14:", .{cmds.len});
    for ([_]usize{ 0, 1, 8, 9, 14 }) |i| {
        if (i < cmds.len) uart.print(.uart0, " {X:0>2}", .{cmds[i]}) else uart.print(.uart0, " --", .{});
    }
    uart.print(.uart0, "\n", .{});

    const bsp = host.commandWithResult(bredr.op.read_buffer_size, &.{}) catch |err| die(err, 21);
    const bs = bredr.parseBufferSize(bsp) catch |err| die(err, 21);
    watch.acl_len = bs.acl_len;
    watch.acl_count = bs.acl_count;

    uart.print(.uart0, "   BR/EDR buffers: ACL {d} bytes x {d}, SCO {d} x {d}\n", .{
        bs.acl_len, bs.acl_count, bs.sco_len, bs.sco_count,
    });
    uart.print(.uart0, "   LE buffers, for contrast: ACL {d} bytes x {d}\n", .{
        host.acl_payload_max, host.acl_max,
    });

    // Not a pass condition — a buffer count is not a throughput measurement,
    // and saying otherwise here would be a claim nothing measured. It is
    // printed so the number that matters later is on the record
    // next to the number it will be compared against.
    if (bs.acl_len > 0) {
        const packets_per_sec = bredr.a2dp_payload_bytes_per_sec / bs.acl_len;
        uart.print(.uart0, "   (SBC at 44.1k is ~{d} KB/s: ~{d} ACL packets/sec at this size.\n", .{
            bredr.a2dp_payload_bytes_per_sec / 1000, packets_per_sec,
        });
        uart.print(.uart0, "    Whether the shared bus sustains that is NOT measured here.)\n", .{});
    }

    watch.status_all = host.status_all;
    if (host.status_all != 0) {
        uart.print(.uart0, "\nA command was refused: status 0x{X:0>2}\n", .{host.status_all});
        die(error.NoHciAnswer, 21);
    }
    watch.rung = 0x21A;
    uart.print(.uart0, "   rung 21a: Classic present, every status 0\n", .{});

    // -- rung 21b: an observer that is not this workspace --------------------
    uart.print(.uart0, "\n4. Becoming discoverable as \"{s}\"...\n", .{local_name});

    const cod = bredr.classOfDeviceParams(bredr.cod_loudspeaker);
    host.command(bredr.op.write_class_of_device, &cod) catch |err| die(err, 21);
    uart.print(.uart0, "   class of device 0x{X:0>6} (Audio/Video, Loudspeaker)\n", .{
        @as(u32, bredr.cod_loudspeaker),
    });

    const name_params = bredr.localNameParams(local_name) catch |err| die(err, 21);
    host.command(bredr.op.write_local_name, &name_params) catch |err| die(err, 21);

    // Written even though nothing here pairs. It is one byte, the feature bit
    // above says the controller has it, and a phone decides how to offer
    // pairing partly from this — so a probe that omits it is asking the phone
    // a slightly different question than rung 22 will.
    host.command(bredr.op.write_simple_pairing_mode, &.{0x01}) catch |err| die(err, 21);

    host.command(
        bredr.op.write_scan_enable,
        &.{@backingInt(bredr.ScanEnable.both)},
    ) catch |err| die(err, 21);
    watch.discoverable = 1;
    watch.rung = 0x21B;
    watch.sent = host.commands;

    uart.print(.uart0,
        \\
        \\   Inquiry and page scan are on.
        \\
        \\   On the Android phone: Settings > Connected devices > Pair new device.
        \\   "{s}" should appear, with a SPEAKER icon -- the icon is decoded
        \\   from the class-of-device bytes this board just wrote, so it is the
        \\   phone confirming a field that came from here.
        \\
        \\   Tapping it WILL FAIL to pair. Nothing accepts a connection yet;
        \\   that is rung 22. If a Connection_Request reaches us first, it is
        \\   printed below -- and that is this rung's best evidence.
        \\
        \\   The other half: flash ntp-client over this and the entry must
        \\   disappear from the phone's list. An entry that survives the board
        \\   going away was a cache, not a sighting.
        \\
    , .{local_name});

    // -- watch for a phone --------------------------------------------------
    var rx: [260]u8 = @splat(0);
    var secs: u32 = 0;
    var last_tick: u32 = 0;

    while (true) {
        const n = host.bt.read(&rx) catch |err| {
            uart.print(.uart0, "[bt] read: {s}\n", .{@errorName(err)});
            timer.delayMs(100);
            continue;
        };

        if (n == 0) {
            timer.delayMs(10);
            secs += 1;
            watch.ticks = secs / 100;
            // A heartbeat, so a silent run is distinguishable from a hung one.
            if (watch.ticks >= last_tick + 30) {
                last_tick = watch.ticks;
                uart.print(.uart0, "   [{d}s] discoverable, {d} connection requests so far\n", .{
                    watch.ticks, watch.conn_requests,
                });
            }
            continue;
        }

        watch.got = host.events;
        const ev = cyw43.hci.parseEvent(rx[0..n]) catch continue;

        switch (ev.code) {
            bredr.event.connection_request => {
                const req = bredr.parseConnectionRequest(ev.params) catch continue;
                watch.conn_requests += 1;
                watch.peer_lo = @as(u32, req.peer[0]) | @as(u32, req.peer[1]) << 8 |
                    @as(u32, req.peer[2]) << 16 | @as(u32, req.peer[3]) << 24;
                watch.peer_hi = @as(u32, req.peer[4]) | @as(u32, req.peer[5]) << 8;
                watch.peer_cod = req.class_of_device;
                watch.rung = 21;

                uart.print(
                    .uart0,
                    "\n>> CONNECTION_REQUEST from {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n",
                    .{ req.peer[5], req.peer[4], req.peer[3], req.peer[2], req.peer[1], req.peer[0] },
                );
                uart.print(.uart0, "   class 0x{X:0>6}, link type {s}\n", .{
                    @as(u32, req.class_of_device),
                    switch (req.link_type) {
                        .acl => "ACL",
                        .sco => "SCO",
                        .esco => "eSCO",
                        _ => "unknown",
                    },
                });
                uart.print(.uart0,
                    \\   RUNG 21 PASSES. A Classic radio is scanning on this board and a
                    \\   phone paged it -- an address that exists here only because that
                    \\   happened. Not accepting it: rung 22.
                    \\
                , .{});
            },
            else => {
                uart.print(.uart0, "   [ev 0x{X:0>2}, {d} params]\n", .{ ev.code, ev.params.len });
            },
        }
    }
}

const std = @import("std");
