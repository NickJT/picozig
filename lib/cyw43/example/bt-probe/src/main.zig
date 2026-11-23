//! bt-probe — rungs 17 and 19: bring the Bluetooth side up, say who it is,
//! advertise, and let a phone connect.
//!
//! Build with `-Dfw=combined`. The WiFi-only firmware has no Bluetooth
//! subsystem in it, so the shared bus has nothing to talk to.
//!
//! ## What passing means
//!
//! Three HCI commands, three Command Complete events, each echoing back the
//! opcode it answers with status 0 — and the answers have to match what the
//! SDK's own driver got on this board, captured from a trace of it:
//!
//!     manufacturer 0x0131 (Cypress), HCI and LMP version 0x0b (Bluetooth 5.2)
//!     BD_ADDR      the station MAC plus one
//!
//! **The BD_ADDR is the weakest of those three**, and it is worth saying why:
//! it is the station MAC plus one, so it is derivable from something this
//! driver already knows. The manufacturer and LMP subversion words are not
//! derivable from anything on our side, so they carry the weight here.
//!
//! Rung 19 adds a connection, and the evidence is on the phone: nRF Connect
//! says CONNECTED, and the board prints the handle and the peer's address at
//! the same moment. Pressing DISCONNECT there has to be visible here too —
//! that is the half that makes it a demonstration rather than a screenshot.
//!
//! ## What is not proven by this
//!
//! Nothing above the connection. No L2CAP, no ATT, no GATT — a connected phone
//! can discover nothing, because there is nothing yet to discover. Rung 20.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// Published over SWD. Thirty-two words, because `.faultinfo` is pinned at
/// 0x20000080 for every image in this workspace.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    rung: u32,
    err_code: u32,
    /// The station MAC, low four then high two. BD_ADDR should be this plus one.
    mac_lo: u32,
    mac_hi: u32,
    /// Where the ring control block turned out to be, and the four indices
    /// after bring-up. All zero unless `start` completed.
    ring_base: u32,
    /// `BT_CTRL_REG` when the firmware declared itself ready — expect
    /// 0x01000100, FW_RDY | BT_AWAKE.
    bt_ctrl: u32,
    /// Patchram records written, and bytes.
    patch_records: u32,
    patch_bytes: u32,
    /// How many times the ready register was polled before it answered.
    ready_polls: u32,
    /// Commands sent, events read.
    sent: u32,
    got: u32,
    /// **The three answers.** `status_all` is the OR of every Command Complete
    /// status, so a single non-zero says one of them was refused without
    /// needing three fields to say which.
    status_all: u32,
    /// Manufacturer and the two version bytes from Read_Local_Version.
    manufacturer: u32,
    versions: u32,
    /// BD_ADDR, low four then high two. Zero unless the read succeeded.
    bd_lo: u32,
    bd_hi: u32,
    /// Rung 19. The connection handle, and the peer's address — low four
    /// bytes, then the high two with the address *type* in the third byte.
    /// A phone connects from a resolvable private address, so this changes
    /// every time and is not the phone's real one.
    conn_handle: u32,
    peer_lo: u32,
    peer_hi: u32,
    /// How many times the connection dropped and came back, which is what the
    /// DISCONNECT button in nRF Connect is for.
    disconnects: u32,
    reconnects: u32,
    /// Rung 20: ATT PDUs in and out, notifications sent, and writes the client
    /// made. A non-zero `writes` is a phone having changed something here.
    att_rx: u32,
    att_tx: u32,
    notifications: u32,
    writes: u32,
    reserved: [5]u32,
};

pub const watch_magic: u32 = 0x4254_3141; // 'BT1A'

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
        // Rung 17a.
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
        error.NoConnection => 50,
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

    uart.print(.uart0, "\n=== CYW43439 Bluetooth probe (rung 17a) ===\nboard {s}\n", .{board.name});
    uart.print(.uart0, "firmware: {d} bytes of microcode, {d} of patchram\n", .{
        cyw43.fw_wifi.len, cyw43.fw_btfw.len,
    });

    // The combined image is not optional here: the WiFi-only build has no
    // Bluetooth subsystem, so the shared bus would have nothing to answer it.
    if (cyw43.fw_wifi.len != 231077) {
        uart.print(.uart0, "\nThis needs -Dfw=combined. The WiFi-only image has no BT subsystem.\n", .{});
        die(error.BadPatchram, 17);
    }

    const pins = cyw43.default_pins;
    const bus = cyw43.PicogSpiBus.init(pins);
    var dev = Dev.init(bus, pins);

    // -- rungs 1-7, the ladder underneath ---------------------------------
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
    watch.rung = 7;
    uart.print(.uart0, "   WLAN core up on the combined image\n", .{});

    // The MAC is read so the BD_ADDR below can be compared against it. It also
    // proves the control path before anything Bluetooth is attempted.
    dev.loadClmDefault() catch |err| die(err, 11);
    dev.wifiOn(.gb) catch |err| die(err, 12);
    watch.mac_lo = @as(u32, dev.mac[0]) | @as(u32, dev.mac[1]) << 8 |
        @as(u32, dev.mac[2]) << 16 | @as(u32, dev.mac[3]) << 24;
    watch.mac_hi = @as(u32, dev.mac[4]) | @as(u32, dev.mac[5]) << 8;
    uart.print(.uart0, "   station MAC {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
        dev.mac[0], dev.mac[1], dev.mac[2], dev.mac[3], dev.mac[4], dev.mac[5],
    });
    watch.rung = 12;

    // -- rung 17a, the Bluetooth side --------------------------------------
    //
    // `Host` owns the HCI layer now — the command/answer matching and the event
    // parsing that used to sit in this file are in `bt/hci.zig`, for the same
    // reason `Session` took the WiFi recovery: rung 20 needs them too, and the
    // copy in an example is the copy that gets forked.
    uart.print(.uart0, "2. Starting the Bluetooth controller...\n", .{});
    var host = cyw43.BtHost(Dev).init(&dev);

    // **`start` resets and then unmasks the events.** The controller's default
    // Event_Mask has bit 61 clear, and every LE event — including the one that
    // says a phone connected — arrives inside an LE Meta event at that bit. A
    // host that skips this waits forever for an event that was masked off, and
    // it looks exactly like a connection that failed.
    host.start(cyw43.fw_btfw, 2000) catch |err| {
        uart.print(.uart0, "   start failed after {d} records, {d} polls\n", .{
            host.bt.patch_records, host.bt.ready_polls,
        });
        watch.patch_records = host.bt.patch_records;
        watch.ready_polls = host.bt.ready_polls;
        die(err, 17);
    };

    watch.ring_base = host.bt.base;
    watch.patch_records = host.bt.patch_records;
    watch.patch_bytes = host.bt.patch_bytes;
    watch.ready_polls = host.bt.ready_polls;
    watch.bt_ctrl = dev.backplaneRead32(cyw43.btbus.reg.bt_ctrl) catch 0;
    uart.print(.uart0, "   patchram {d} records / {d} bytes, ready after {d} polls\n", .{
        host.bt.patch_records, host.bt.patch_bytes, host.bt.ready_polls,
    });
    uart.print(.uart0, "   ring base 0x{X:0>8}, bt_ctrl 0x{X:0>8} (expect 0x01000100)\n", .{
        host.bt.base, watch.bt_ctrl,
    });

    uart.print(.uart0, "3. Asking the chip who it is...\n", .{});

    const ver = host.commandWithResult(cyw43.hci.op.read_local_version, &.{}) catch |err| die(err, 17);
    if (ver.len >= 8) {
        const manufacturer = @as(u32, ver[4]) | @as(u32, ver[5]) << 8;
        watch.manufacturer = manufacturer;
        watch.versions = @as(u32, ver[0]) | @as(u32, ver[3]) << 8 |
            @as(u32, ver[6]) << 16 | @as(u32, ver[7]) << 24;
        uart.print(.uart0, "   HCI {d}, LMP {d}, manufacturer 0x{X:0>4}, subversion 0x{X:0>2}{X:0>2}\n", .{
            ver[0], ver[3], manufacturer, ver[7], ver[6],
        });
    }

    const bd = host.commandWithResult(cyw43.hci.op.read_bd_addr, &.{}) catch |err| die(err, 17);
    if (bd.len >= 6) {
        watch.bd_lo = @as(u32, bd[0]) | @as(u32, bd[1]) << 8 |
            @as(u32, bd[2]) << 16 | @as(u32, bd[3]) << 24;
        watch.bd_hi = @as(u32, bd[4]) | @as(u32, bd[5]) << 8;
        uart.print(.uart0, "   BD_ADDR {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
            bd[5], bd[4], bd[3], bd[2], bd[1], bd[0],
        });
    }
    watch.status_all = host.status_all;
    if (host.status_all != 0) {
        uart.print(.uart0, "\nA command was refused: status 0x{X:0>2}\n", .{host.status_all});
        die(error.NoHciAnswer, 17);
    }
    watch.rung = 17;
    uart.print(.uart0, "   rung 17a: every status 0\n", .{});

    // -- rung 19, connectable advertising ----------------------------------
    //
    // **The only difference from 17b is one byte**, `ADV_IND` where that used
    // `ADV_NONCONN_IND` — and everything that follows from it. 17b advertised
    // with nothing behind it to accept a connection; this waits for one.
    uart.print(.uart0, "4. Advertising as \"{s}\", connectable...\n", .{adv_name});
    host.advertise(adv_name, .ind) catch |err| die(err, 19);
    watch.rung = 0x17B;
    watch.sent = host.commands;
    watch.got = host.events;

    uart.print(.uart0, "\nWaiting for a connection. In nRF Connect: SCAN, then CONNECT on\n", .{});
    uart.print(.uart0, "\"{s}\" at {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}.\n", .{
        adv_name,                              @as(u8, @truncate(watch.bd_hi >> 8)),
        @as(u8, @truncate(watch.bd_hi)),       @as(u8, @truncate(watch.bd_lo >> 24)),
        @as(u8, @truncate(watch.bd_lo >> 16)), @as(u8, @truncate(watch.bd_lo >> 8)),
        @as(u8, @truncate(watch.bd_lo)),
    });

    // Five minutes is generous on purpose: the observer is a person with a
    // phone, and a timeout that expires while they are finding the app proves
    // nothing about the driver.
    const conn = host.awaitConnection(300_000) catch |err| die(err, 19) orelse {
        uart.print(.uart0, "\nNobody connected in five minutes. {d} events seen.\n", .{host.events});
        die(error.NoConnection, 19);
    };

    watch.conn_handle = conn.handle;
    watch.peer_lo = @as(u32, conn.peer_addr[0]) | @as(u32, conn.peer_addr[1]) << 8 |
        @as(u32, conn.peer_addr[2]) << 16 | @as(u32, conn.peer_addr[3]) << 24;
    watch.peer_hi = @as(u32, conn.peer_addr[4]) | @as(u32, conn.peer_addr[5]) << 8 |
        @as(u32, conn.peer_addr_type) << 16;
    watch.rung = 19;

    // zig fmt: off
    uart.print(.uart0, "\nCONNECTED. handle 0x{X:0>4}, role {s}, peer {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}",
        .{
            conn.handle,        if (conn.role == 1) "peripheral" else "central",
            conn.peer_addr[5],  conn.peer_addr[4],
            conn.peer_addr[3],  conn.peer_addr[2],
            conn.peer_addr[1],  conn.peer_addr[0],
        });
    // zig fmt: on
    // A phone advertises a resolvable private address rather than its real one,
    // so this changes between connections. That is the phone behaving correctly,
    // not the board misreading it.
    uart.print(.uart0, " (type {d}{s})\n", .{
        conn.peer_addr_type,
        if (conn.peer_addr_type == 1) ", random — a phone's privacy address" else "",
    });
    uart.print(.uart0, "interval {d} x 1.25 ms, timeout {d} x 10 ms\n", .{ conn.interval, conn.timeout });

    // -- rung 20: GATT ------------------------------------------------------
    //
    // The connection is up; now there is something behind it. nRF Connect runs
    // discovery the moment it connects, which is what rung 19's spinner was —
    // the same walk, with nothing answering.
    uart.print(.uart0, "\nServing GATT. In nRF Connect the service should appear;\n", .{});
    uart.print(.uart0, "subscribe to the counter and it ticks, write the LED and it lights.\n\n", .{});

    var server = cyw43.att.Server{ .table = &attributes, .cccd_handle = h_cccd };

    var ticks: u32 = 0;
    var notified: u32 = 0;
    while (true) {
        if (!host.connected()) {
            uart.print(.uart0, "\nDISCONNECTED after {d}s. Advertising again.\n", .{ticks});
            watch.disconnects += 1;
            // A new client starts unsubscribed. **Carrying the old
            // subscription over would notify a phone that never asked**, which
            // is a protocol violation and reads as a driver that leaks state.
            server.notifications_enabled = false;
            host.advertise(adv_name, .ind) catch |err| {
                uart.print(.uart0, "re-advertise failed: {s}\n", .{@errorName(err)});
            };
            if (host.awaitConnection(300_000) catch null) |_| {
                uart.print(.uart0, "Reconnected.\n", .{});
                watch.reconnects += 1;
                ticks = 0;
            }
            continue;
        }

        // Answer whatever the client asks for a second, then update.
        const written = host.serveAtt(&server, 1000) catch |err| blk: {
            uart.print(.uart0, "[att] {s}\n", .{@errorName(err)});
            break :blk null;
        };

        if (written) |h| {
            if (h == h_led_value) {
                // **A phone's write moving a real light**, which is the part of
                // rung 20 that cannot be faked from this end.
                const on = led_storage[0] != 0;
                dev.setLed(on) catch {};
                uart.print(.uart0, "[att] LED {s} (written by the client)\n", .{
                    if (on) "ON" else "OFF",
                });
                watch.writes += 1;
            } else if (h == h_cccd) {
                uart.print(.uart0, "[att] notifications {s}\n", .{
                    if (server.notifications_enabled) "ON" else "OFF",
                });
            }
        }

        ticks += 1;
        watch.ticks = ticks;

        // The counter the client subscribes to: seconds of uptime, little
        // endian. A counter and not a sensor, so a wrong value has exactly one
        // place to have come from.
        std.mem.writeInt(u32, counter_storage[0..4], ticks, .little);
        if (host.notify(&server, h_counter_value, &counter_storage) catch false) {
            notified += 1;
            watch.notifications = notified;
        }

        watch.att_rx = host.att_received;
        watch.att_tx = host.att_sent;
        watch.got = host.events;

        // **Rung 20 is claimed only once a client has actually spoken ATT.**
        // Serving a table nobody has read is rung 19 with extra code in it, and
        // the watch block should not say otherwise.
        if (host.att_received > 0) watch.rung = 20;
    }
}

/// What the board calls itself over the air. The same string
/// `ipstack.hostname` asks DHCP for, on purpose.
const adv_name = "zig-pico2w";

// -- the attribute table ----------------------------------------------------
//
// Six attributes, and every one of them is here because a client needs it to
// find the next: the service declaration bounds the search, each characteristic
// declaration names the handle its value lives at, and the CCCD is what a
// client writes to turn notifications on.
//
// **The values are module-level storage, not literals**, because the table
// holds slices into them: `counter_storage` is rewritten every second and the
// attribute keeps pointing at the same bytes.

const h_service: u16 = 1;
const h_counter_decl: u16 = 2;
const h_counter_value: u16 = 3;
const h_cccd: u16 = 4;
const h_led_decl: u16 = 5;
const h_led_value: u16 = 6;

/// 128-bit UUIDs, **little-endian on the wire**, so these read backwards from
/// how the UUID is written down: `5A17xxxx-C0FF-EE00-1234-9A5B0C7D1E2F`.
///
/// **A custom service needs a base nobody else uses.** The first version of
/// this borrowed Nordic's UART service base, and nRF Connect duly labelled our
/// counter and LED as a Nordic UART Service with TX and RX characteristics —
/// a thing this board is not. Discovery worked and the display lied, which is
/// the wrong way round for a package whose whole method is believing the
/// observer.
const uuid_base = [16]u8{
    0x2f, 0x1e, 0x7d, 0x0c, 0x5b, 0x9a, 0x34, 0x12,
    0x00, 0xee, 0xff, 0xc0, 0x00, 0x00, 0x17, 0x5a,
};

fn customUuid(comptime id: u16) [16]u8 {
    var u = uuid_base;
    u[12] = @truncate(id);
    u[13] = @truncate(id >> 8);
    return u;
}

// `var`, because the table holds mutable slices — see `att.Attribute.value`,
// and the LED that would not light.
var service_uuid = customUuid(0x0001);
var counter_uuid = customUuid(0x0002);
var led_uuid = customUuid(0x0003);

var counter_storage = [_]u8{ 0, 0, 0, 0 };
var led_storage = [_]u8{0};
var cccd_storage = [_]u8{ 0, 0 };

/// A characteristic declaration: properties, the handle of the value, then the
/// value's UUID. **The value handle is not the declaration's own**, and a
/// client reads and subscribes at the one named here.
var counter_decl: [19]u8 = [_]u8{ cyw43.att.props.read | cyw43.att.props.notify, h_counter_value, 0x00 } ++ customUuid(0x0002);
var led_decl: [19]u8 = [_]u8{ cyw43.att.props.read | cyw43.att.props.write, h_led_value, 0x00 } ++ customUuid(0x0003);

const attributes = [_]cyw43.att.Attribute{
    .{
        .handle = h_service,
        .type = .{ .short = cyw43.att.uuid16.primary_service },
        .value = &service_uuid,
        // The last handle in this service. A client uses it to bound the
        // characteristic search, and a wrong one hides everything after it.
        .group_end = h_led_value,
    },
    .{ .handle = h_counter_decl, .type = .{ .short = cyw43.att.uuid16.characteristic }, .value = &counter_decl },
    .{ .handle = h_counter_value, .type = .{ .long = customUuid(0x0002) }, .value = &counter_storage },
    .{ .handle = h_cccd, .type = .{ .short = cyw43.att.uuid16.cccd }, .value = &cccd_storage, .writable = true },
    .{ .handle = h_led_decl, .type = .{ .short = cyw43.att.uuid16.characteristic }, .value = &led_decl },
    .{ .handle = h_led_value, .type = .{ .long = customUuid(0x0003) }, .value = &led_storage, .writable = true },
};

const std = @import("std");
