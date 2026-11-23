//! cyw43-blink — CYW43439 Onboard LED Blink tool for Pico 2 W.
//!
//! Powers up CYW43439, boots firmware microcode, and toggles the onboard green LED
//! (CYW43_GPIO_0) every 500 ms while logging state over UART0 (115200 8N1).

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const cyw43 = @import("cyw43");
const wifi_config = @import("wifi_config");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const timer = rp2350.timer;
const uart = rp2350.uart;

/// The bring-up ladder, published over SWD.
///
/// Each rung is recorded as it passes, so a board that stops partway says where
/// it stopped without needing a serial console attached. `rung` is the last one
/// reached; `err_code` is set only if that rung failed.
const Watch = extern struct {
    magic: u32,
    ticks: u32,
    led_state: u32,
    firmware_loaded: u32,
    /// Highest rung of the bring-up ladder reached.
    rung: u32,
    test_register: u32,
    err_code: u32,
    bus_control: u32,
    raw0: u32,
    resp_delay_rb: u32,
    /// Bytes wrong out of 64 in the backplane round-trip. 0 is healthy.
    bp_wrong: u32,
    /// Bytes wrong out of 64 when the downloaded microcode is read back.
    fw_wrong: u32,
    /// The word `clmload_status` returned. 0 is the chip accepting the blob;
    /// 0xFFFFFFFF means no status word came back at all.
    clm_status: u32,
    /// The station MAC read from `cur_etheraddr`, low four bytes then high two.
    /// All zero means the read returned nothing.
    mac_lo: u32,
    mac_hi: u32,
    /// Two fields in one word, because **the block is now full**: the low half
    /// is `LinkStatus` (2 is joined), the high half the `JoinState` bitfield,
    /// so a partial join says which of the three events arrived. The region is
    /// 0x80 bytes and this structure is exactly 32 words — adding a field means
    /// removing one.
    link: u32,
    /// Bisect trace: chipcommon chip-ID read after each core-prep step.
    trace: [8]u32,
    dump: [8]u32,
};

const Dev = cyw43.Device(cyw43.PicogSpiBus);

/// The 42 bytes of an ARP request, built here rather than borrowed.
///
/// **This is deliberately not a call into `lib/ipnet`.** An L1 driver's example
/// may not import an L2 library — that is the boundary the IP stack moved out
/// on 2026-08-31 to establish, and the same rule already applies to the
/// package's own round-trip test in `src/cyw43.zig`. What rung 14 asserts is
/// that a frame this station composed goes out and is answered; that the frame
/// happens to be a valid ARP request is only so a failure is readable in a
/// capture. Forty-two bytes is a small price for the layering.
///
/// RFC 826 over Ethernet II: 14 bytes of Ethernet header, then hardware type 1,
/// protocol type 0x0800, address lengths 6 and 4, opcode 1.
fn arpRequest(sender_mac: [6]u8, sender_ip: [4]u8, target_ip: [4]u8) [42]u8 {
    var f: [42]u8 = @splat(0);
    @memset(f[0..6], 0xFF); // destination: broadcast — the answer is the point
    @memcpy(f[6..12], &sender_mac);
    f[12] = 0x08;
    f[13] = 0x06; // ethertype: ARP
    f[15] = 0x01; // hardware type: Ethernet
    f[16] = 0x08; // protocol type: IPv4
    f[18] = 6; // hardware address length
    f[19] = 4; // protocol address length
    f[21] = 0x01; // opcode: request
    @memcpy(f[22..28], &sender_mac);
    @memcpy(f[28..32], &sender_ip);
    // f[32..38] — the target MAC stays zero. It is what is being asked for.
    @memcpy(f[38..42], &target_ip);
    return f;
}

const ArpReply = struct {
    sender_mac: [6]u8,
    sender_ip: [4]u8,
    target_ip: [4]u8,
};

/// Read an ARP *reply*, or return null.
///
/// **A request is not a reply**, including this station's own coming back as a
/// broadcast echo — which does happen, and reads as a successful answer to a
/// check that looks only at the ethertype. Hence the opcode test.
fn parseArpReply(frame: []const u8) ?ArpReply {
    if (frame.len < 42) return null;
    if (frame[12] != 0x08 or frame[13] != 0x06) return null; // not ARP
    if (frame[14] != 0x00 or frame[15] != 0x01) return null; // not Ethernet ARP
    if (frame[16] != 0x08 or frame[17] != 0x00) return null; // not IPv4 ARP
    if (frame[20] != 0x00 or frame[21] != 0x02) return null; // not a reply

    var r: ArpReply = undefined;
    @memcpy(&r.sender_mac, frame[22..28]);
    @memcpy(&r.sender_ip, frame[28..32]);
    @memcpy(&r.target_ip, frame[38..42]);
    return r;
}

pub const watch_magic: u32 = 0x4C45_4432; // 'LED2'

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
    // `.faultinfo` is pinned at 0x20000080, so this block cannot cross 0x80.
    // The comment above says "adding a field means removing one"; this is what
    // makes that a compile error rather than a silently clobbered fault record.
    if (@sizeOf(Watch) > 0x80) @compileError("watch block overruns .faultinfo at 0x20000080");
}

/// Report which rung failed and stop. Parking here rather than continuing is
/// the point: a driver that carries on past a failed handshake produces pages
/// of plausible-looking output about a bus that is not working.
fn die(err: anyerror, rung: u32) noreturn {
    uart.print(.uart0, "FAILED at rung {d}: {s}\n", .{ rung, @errorName(err) });
    watch.magic = 0xdead_4c45;
    watch.rung = rung;
    // A stable code rather than `@intFromError`, whose value depends on the
    // whole compilation's error set and so changes when unrelated code does.
    // Reading a failure over SWD is the point of this block; a number that
    // means something different after the next edit defeats it.
    watch.err_code = switch (err) {
        error.NotPowered => 1,
        error.ChipNotResponding => 2,
        error.BusControlMismatch => 3,
        error.AlpTimeout => 4,
        error.CoreNotInReset => 5,
        error.HtClockTimeout => 6,
        error.F2NotReady => 7,
        error.CreditTimeout => 8,
        error.BadTransferLength => 9,
        error.PayloadTooLarge => 10,
        error.FirmwareNotLoaded => 11,
        error.NotImplemented => 12,
        error.BackplaneUnreliable => 13,
        error.ClmLoadFailed => 14,
        error.IoctlRejected => 15,
        error.IoctlTimeout => 16,
        error.CountryRejected => 17,
        error.EventMaskRejected => 18,
        error.WifiUpRejected => 19,
        error.WifiNotOn => 20,
        error.JoinRejected => 21,
        error.JoinTimeout => 22,
        error.JoinFailed => 23,
        error.BadSsid => 24,
        error.BadPassphrase => 25,
        error.LinkDown => 26,
        error.BufferTooSmall => 27,
        // Rung 15. NoDhcpOffer is nobody answering; DhcpNak is a server that
        // answered and refused. Different faults, different numbers.
        error.NoDhcpOffer => 30,
        error.DhcpNak => 31,
        error.IncompleteLease => 32,
        error.GatewayUnreachable => 33,
        error.NoAddress => 34,
        // Rung 14b: the request went out and nothing came back. Distinct from
        // NoDataFrame, which is the network being quiet rather than the
        // transmit path being broken.
        error.NoArpReply => 28,
        error.NoDataFrame => 29,
        else => 99,
    };
    while (true) timer.delayMs(1000);
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== CYW43439 Onboard LED Blink Tool ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .led_state = 0,
        .firmware_loaded = 0,
        .rung = 0,
        .test_register = 0,
        .err_code = 0,
        .bus_control = 0,
        .raw0 = 0,
        .resp_delay_rb = 0,
        .bp_wrong = 0xFFFF_FFFF,
        .fw_wrong = 0xFFFF_FFFF,
        .clm_status = cyw43.no_clm_status,
        .mac_lo = 0,
        .mac_hi = 0,
        .link = 0,
        .trace = @splat(0),
        .dump = @splat(0),
    };

    const pins = cyw43.default_pins;
    uart.print(.uart0, "Pico 2 W Pins: WL_REG_ON=GP{d}, WL_DATA=GP{d}, WL_CS=GP{d}, WL_CLK=GP{d}\n", .{
        pins.wl_reg_on,
        pins.wl_data,
        pins.wl_cs,
        pins.wl_clk,
    });

    // Emit every register transaction in the canonical driver's own format, so
    // this log and the pico-sdk `blink` example's can be diffed directly.
    cyw43.gspi.trace = true;

    const bus = cyw43.PicogSpiBus.init(pins);
    var dev = Dev.init(bus, pins);

    // Each rung is proven before the next is attempted, because a failure four
    // steps up is unreadable if the three below it were never checked.

    uart.print(.uart0, "1. Powering up CYW43439 (reset cycle, 250 ms settle)...\n", .{});
    dev.powerUp() catch |err| die(err, 0);

    uart.print(.uart0, "2. Reading gSPI test register in 16-bit boot mode...\n", .{});
    // **Retry.** The first read after `WL_REG_ON` rises reliably fails on this
    // board — measured as bitmask 0b11111110 across eight reads, every time —
    // because the chip is still waking. A single read here reports a perfectly
    // healthy bus as a missing chip, which is exactly what it did before this
    // loop was added.
    var tr: u32 = 0;
    var tries: u32 = 10;
    while (tries > 0) : (tries -= 1) {
        tr = dev.readTestRegister() catch 0;
        watch.test_register = tr;
        if (tr == cyw43.gspi.test_pattern) break;
        timer.delayMs(1);
    }
    uart.print(.uart0, "   test register = 0x{X:0>8} (expect FEEDBEAD)\n", .{tr});
    if (tr != cyw43.gspi.test_pattern) die(error.ChipNotResponding, 1);
    watch.rung = 1;

    uart.print(.uart0, "3. Switching the bus to 32-bit big-endian...\n", .{});
    dev.busInit() catch |err| die(err, 2);
    watch.rung = 2;

    // Published because it is the one value that says *how* the handshake
    // landed rather than merely that it did: the chip returns 0x000200B3 for
    // the 0x000204B3 written, and knowing that is what stopped a working bus
    // being reported as a broken one.
    watch.bus_control = dev.bus.readReg32(.bus, cyw43.gspi.reg.bus_control) catch 0;
    uart.print(.uart0, "   bus control = 0x{X:0>8} (wrote 0x{X:0>8})\n", .{
        watch.bus_control,
        cyw43.gspi.bus_control_word,
    });

    watch.resp_delay_rb = dev.bus.readReg8(.bus, cyw43.gspi.reg.resp_delay_f1) catch 0xFF;

    // Rung 3 — the backplane clock.
    //
    // **Nothing behind the window answers before this.** Direct F1 registers
    // respond without a backplane clock; windowed accesses come back as
    // DATA_UNAVAILABLE, which looks like a plausible constant rather than an
    // error. A round-trip run before this step measures nothing.
    // Do ordinary (non-byte-swapped) writes reach the chip at all? Everything
    // after the handshake depends on it, and none of it has ever been checked.
    // The canonical driver reads 0x10 back from 0x1d; we read 0x04.
    const wp = dev.probeWritePath();
    uart.print(.uart0, "   write path -- 0x1d: {X:0>2} ->write10-> {X:0>2}   0x1e: {X:0>2} ->write55-> {X:0>2}\n", .{
        wp.delay_before, wp.delay_after, wp.scratch_before, wp.scratch_after,
    });

    uart.print(.uart0, "4. Requesting the ALP backplane clock...\n", .{});
    dev.requestAlp() catch |err| die(err, 3);
    // Clock CSR sampled at each stage. This is a direct F1 register, which is
    // proven reliable, so it reports the backplane clock honestly even when
    // windowed reads do not.
    watch.rung = 3;

    // Rung 4 — cores down and SOCRAM cycled, **one step at a time**.
    //
    // Windowed reads are correct before this block and degrade after it, so
    // chipcommon's chip-ID register — read-only and constant — is sampled
    // between every step. Whichever step changes it is the one to explain.
    uart.print(.uart0, "5. Preparing cores, tracing the backplane after each step...\n", .{});

    watch.trace[0] = dev.probeBackplane() catch 0xDEAD_0000;
    dev.disableCore(.wlan_arm) catch {};
    watch.trace[1] = dev.probeBackplane() catch 0xDEAD_0001;
    dev.disableCore(.socram) catch {};
    watch.trace[2] = dev.probeBackplane() catch 0xDEAD_0002;
    dev.resetCore(.socram, false) catch |err| die(err, 4);
    watch.trace[3] = dev.probeBackplane() catch 0xDEAD_0003;
    dev.disableSram3Remap() catch |err| die(err, 4);
    watch.trace[4] = dev.probeBackplane() catch 0xDEAD_0004;

    // Where does the data actually land? chipcommon's ID is 0x1545A9AF, so the
    // slot holding it is the response delay the chip is really applying — and
    // a run where no slot holds it is a run where the chip answered
    // DATA_UNAVAILABLE instead.
    // Which response delay does this chip actually apply? The clock CSR has a
    // known value here, so the slot that shows it is the answer.
    const delays = dev.measureRespDelay();
    for (delays, 0..) |d, i| watch.dump[i] = d;
    uart.print(.uart0, "   CSR by pad-words: 0=%{X:0>2} 1={X:0>2} 2={X:0>2} 3={X:0>2} 4={X:0>2}\n", .{
        delays[0], delays[1], delays[2], delays[3], delays[4],
    });

    const again = dev.remeasureAfterDelayWrite(16);
    for (again.slots, 0..) |d, i| watch.dump[i] = d;
    watch.dump[5] = again.readback;
    uart.print(.uart0, "   after rewriting 0x1d=16: reads back {X:0>2}; slots {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{
        // zig fmt: off
        again.readback,
        again.slots[0], again.slots[1], again.slots[2], again.slots[3], again.slots[4],
        // zig fmt: on
    });

    // How reliable is a windowed read here? The chip ID is a known constant, so
    // this is a straight success count.
    watch.dump[0] = dev.backplaneReadSuccessRate(32) catch 0;
    uart.print(.uart0, "   backplane reads: {d}/32 returned the true chip ID\n", .{watch.dump[0]});

    // Core wrapper status after prep. A core still in reset silently discards
    // writes to the RAM it owns, which is exactly the symptom being chased.
    watch.trace[6] = dev.coreStatus(.socram) catch 0xDEAD_0006;
    watch.trace[7] = dev.coreStatus(.wlan_arm) catch 0xDEAD_0007;
    uart.print(.uart0, "   socram ioctrl/reset=0x{X:0>4}  wlan_arm=0x{X:0>4}\n", .{
        watch.trace[6], watch.trace[7],
    });

    uart.print(.uart0, "   trace: {X:0>8} {X:0>8} {X:0>8} {X:0>8} {X:0>8}\n", .{
        watch.trace[0], watch.trace[1], watch.trace[2], watch.trace[3], watch.trace[4],
    });
    watch.rung = 4;

    // Rung 5 — now the round-trip means something.
    uart.print(.uart0, "6. Backplane round-trip through chip RAM...\n", .{});
    const bp = dev.backplaneSelfTest() catch |err| die(err, 5);
    watch.bp_wrong = bp.block_wrong;
    watch.raw0 = bp.chip_id;
    uart.print(.uart0, "   chipcommon=0x{X:0>8} word={s} block {d}/64 wrong\n", .{
        bp.chip_id,
        if (bp.word_ok) "OK" else "BAD",
        bp.block_wrong,
    });
    watch.rung = 5;

    // Rung 6 — the download itself, verified rather than assumed.
    uart.print(.uart0, "7. Downloading microcode and NVRAM ({d} + {d} bytes)...\n", .{
        cyw43.fw_wifi.len,
        cyw43.fw_nvram.len,
    });
    dev.loadFirmware(cyw43.fw_wifi, cyw43.fw_nvram) catch |err| {
        // Read the image back even on failure: a core that will not start and a
        // download that did not land are different faults with one symptom.
        var sample: [64]u8 = @splat(0);
        watch.fw_wrong = dev.verifyFirmware(cyw43.fw_wifi, &sample) catch 0xFFFF_FFFF;
        uart.print(.uart0, "   microcode readback: {d}/64 bytes wrong\n", .{watch.fw_wrong});
        // Lead 2: does windowed access recover once the ARM core is out of
        // reset? If it does, the read failures above are a transient of SOCRAM
        // being mid-cycle rather than a broken path, and the real fault is
        // much narrower than it looks.
        watch.trace[5] = dev.probeBackplane() catch 0xDEAD_0005;
        uart.print(.uart0, "   backplane after ARM release: 0x{X:0>8}\n", .{watch.trace[5]});
        die(err, 6);
    };
    watch.rung = 7;
    watch.firmware_loaded = 1;

    uart.print(.uart0, "8. Firmware up and F2 ready. Starting LED blink loop...\n", .{});
    watch.rung = 8;

    // Rung 11 — the CLM blob.
    //
    // Rungs 9 and 10 are not separate steps here: 9 is proven by the blink loop
    // sustaining, and 10 has nothing to parse until there is a join. This one
    // *is* separate, and it needs rung 9 underneath it — a CLM download whose
    // responses were not matched by id would report someone else's status.
    //
    // A first LED write goes out ahead of it, so a failure here is known to be
    // the CLM download rather than the control path in general.
    uart.print(.uart0, "9. Proving the control path with one LED write...\n", .{});
    dev.setLed(true) catch |err| die(err, 9);
    watch.rung = 9;

    uart.print(.uart0, "10. Downloading the CLM blob ({d} bytes)...\n", .{cyw43.fw_clm.len});
    dev.loadClmDefault() catch |err| {
        // The status word is the diagnosis, so publish it before parking. Its
        // absence (0xFFFFFFFF) and a non-zero value are different faults: no
        // answer at all versus the handler rejecting the blob.
        watch.clm_status = dev.clm_status;
        uart.print(.uart0, "   clmload_status = 0x{X:0>8}\n", .{dev.clm_status});
        die(err, 11);
    };
    watch.clm_status = dev.clm_status;
    uart.print(.uart0, "   CLM accepted, clmload_status = 0x{X:0>8}\n", .{dev.clm_status});
    watch.rung = 11;

    // Rung 12a — the wifi_on sequence.
    //
    // Twenty IOCTLs, every one of which must be accepted: `sendIoctl` errors on
    // a non-zero CDC status, so reaching the end of this is the chip agreeing
    // with all twenty. The MAC read out of `cur_etheraddr` is published as
    // independent evidence — it is a value the firmware cannot have invented,
    // and an all-zero one means the reads returned nothing.
    uart.print(.uart0, "11. Bringing Wi-Fi up (country GB)...\n", .{});
    const wifi_on_result = dev.wifiOn(.gb);

    // **Every step's status, pass or fail.** Some of these are non-zero in a
    // working bring-up — the reference discards them all — so the useful output
    // is the whole vector, not the first non-zero entry.
    for (dev.wifi_on_status, 0..) |st, i| {
        if (st != 0) uart.print(.uart0, "   step {d}: status 0x{X:0>8} ({d})\n", .{
            i, st, @as(i32, @bitCast(st)),
        });
    }
    watch.dump[6] = dev.ioctl_count;
    watch.dump[7] = dev.wifi_on_status[7];

    wifi_on_result catch |err| die(err, 12);
    watch.mac_lo = @as(u32, dev.mac[0]) | @as(u32, dev.mac[1]) << 8 |
        @as(u32, dev.mac[2]) << 16 | @as(u32, dev.mac[3]) << 24;
    watch.mac_hi = @as(u32, dev.mac[4]) | @as(u32, dev.mac[5]) << 8;
    uart.print(.uart0, "   Wi-Fi up. MAC {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
        dev.mac[0], dev.mac[1], dev.mac[2], dev.mac[3], dev.mac[4], dev.mac[5],
    });
    watch.rung = 12;

    // Rung 10 + 12b — the join, and the events that report it.
    //
    // `joinWifi` returning means the chip accepted `WLC_SET_SSID`, which is not
    // a join. The link only counts as up when authentication, link-up and key
    // exchange have each arrived as a separate event — so the result published
    // here comes from `awaitJoin`, never from the call above it.
    uart.print(.uart0, "12. Joining \"{s}\"...\n", .{wifi_config.ssid});
    dev.joinWifi(wifi_config.ssid, wifi_config.password, .wpa2_aes) catch |err| {
        for (dev.join_status, 0..) |st, i| {
            if (st != 0) uart.print(.uart0, "   join step {d}: status 0x{X:0>8}\n", .{ i, st });
        }
        die(err, 13);
    };

    const link = dev.awaitJoin(20000) catch |err| {
        uart.print(.uart0, "   join state: auth={} link={} keyed={} kind={d}\n", .{
            dev.join_state.auth, dev.join_state.link, dev.join_state.keyed, dev.join_state.kind,
        });
        die(err, 13);
    };

    watch.link = @backingInt(link) | @as(u32, @as(u16, @bitCast(dev.join_state))) << 16;
    uart.print(.uart0, "   link: {s} (auth={} link={} keyed={})\n", .{
        @tagName(link), dev.join_state.auth, dev.join_state.link, dev.join_state.keyed,
    });
    if (link != .joined) die(error.JoinFailed, 13);
    watch.rung = 13;

    // Rung 14 — the 802.3 data path.
    //
    // Two halves, and only the second one can fail the rung.
    //
    // 14a listens. The network's own broadcast traffic is already arriving on
    // the data channel — the census counted it long before anything could read
    // it — so a frame here proves the receive path and nothing else. **It is
    // reported rather than fatal**: whether the network broadcasts in the next
    // ten seconds is not this driver's doing, and a rung that fails on someone
    // else's silence is a rung that will be ignored.
    //
    // 14b is the one that decides. An ARP request goes out from this station,
    // and the answer comes back **addressed to this station's MAC** — a frame
    // that cannot be one of the broadcasts already in flight, and that exists
    // only because the AP received something we sent. That is the whole rung:
    // a transmit the driver cannot have faked, confirmed by a receive it cannot
    // have predicted.
    uart.print(.uart0, "13. Listening for 802.3 frames on the data channel...\n", .{});
    var frame: [cyw43.eth_max_frame]u8 = undefined;

    const first = dev.awaitEthFrame(&frame, 10_000) catch |err| die(err, 14);
    if (first == 0) {
        uart.print(.uart0, "   nothing in 10 s — the network is quiet, or nothing is being parsed\n", .{});
    } else {
        const ethertype = @as(u16, frame[12]) << 8 | frame[13];
        watch.dump[1] = dev.eth_rx_count;
        watch.dump[2] = @as(u32, ethertype) << 16 | @as(u32, @intCast(first));
        uart.print(.uart0, "   {d} bytes, type 0x{X:0>4}, to {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}" ++
            " from {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}\n", .{
            first,     ethertype,
            frame[0],  frame[1],
            frame[2],  frame[3],
            frame[4],  frame[5],
            frame[6],  frame[7],
            frame[8],  frame[9],
            frame[10], frame[11],
        });
    }

    // 14b. **The address here is static on purpose, even though rung 15 below
    // obtains a real one.** Rung 14 has to be provable without the rung above
    // it: if the ARP test used the leased address, a DHCP failure would present
    // as a data-path failure, and the ladder's whole point is that each rung
    // fails on its own account.
    const local_ip = [4]u8{ 192, 168, 86, 200 };
    const gateway_ip = [4]u8{ 192, 168, 86, 1 };
    uart.print(.uart0, "14. ARP: who has {d}.{d}.{d}.{d}? Tell {d}.{d}.{d}.{d}\n", .{
        gateway_ip[0], gateway_ip[1], gateway_ip[2], gateway_ip[3],
        local_ip[0],   local_ip[1],   local_ip[2],   local_ip[3],
    });

    const arp = arpRequest(dev.mac, local_ip, gateway_ip);

    var gateway_mac: ?[6]u8 = null;
    // Five attempts: ARP is unacknowledged, and one lost request is not a
    // broken transmit path.
    var attempt: u32 = 0;
    while (attempt < 5 and gateway_mac == null) : (attempt += 1) {
        dev.sendEthFrame(&arp) catch |err| die(err, 14);

        var waited: u32 = 0;
        while (waited < 1000) : (waited += 1) {
            const n = dev.pollEthFrame(&frame) catch |err| die(err, 14);
            if (n == 0) {
                timer.delayMs(1);
                continue;
            }

            // Addressed to us, and an actual reply. Either check alone would
            // accept something else: the broadcasts already arriving carry the
            // right ethertype, and our own request comes back as one of them.
            var mine = true;
            for (dev.mac, 0..) |b, i| {
                if (frame[i] != b) mine = false;
            }
            if (!mine) continue;

            const reply = parseArpReply(frame[0..n]) orelse continue;
            if (reply.sender_ip[3] != gateway_ip[3] or reply.target_ip[3] != local_ip[3]) continue;
            gateway_mac = reply.sender_mac;
            break;
        }
    }

    const gw = gateway_mac orelse {
        uart.print(.uart0, "   no reply after {d} requests ({d} frames sent, {d} received)\n", .{
            attempt, dev.eth_tx_count, dev.eth_rx_count,
        });
        die(error.NoArpReply, 14);
    };

    watch.dump[3] = @as(u32, gw[0]) | @as(u32, gw[1]) << 8 |
        @as(u32, gw[2]) << 16 | @as(u32, gw[3]) << 24;
    watch.dump[4] = @as(u32, gw[4]) | @as(u32, gw[5]) << 8;
    watch.dump[5] = dev.eth_tx_count;
    uart.print(.uart0, "   reply from {X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}:{X:0>2}" ++
        " — {d} frames out, {d} in\n", .{
        gw[0],            gw[1],            gw[2], gw[3], gw[4], gw[5],
        dev.eth_tx_count, dev.eth_rx_count,
    });
    watch.rung = 14;

    // **The ladder stops here, and that is the package boundary.**
    //
    // Rungs 15, 16 and 18 — a DHCP lease defended by ARP, a UDP round-trip, and
    // a network that survives the night — all passed on hardware while the IP
    // stack lived in this package. It moved to `lib/ipnet` on 2026-08-31,
    // because an IPv4 stack has no business knowing what radio is under it.
    // Those rungs are now walked by `lib/ipnet/example/net-check`, which takes
    // a lease through `ipnet.Session` and goes on to TCP, DNS and HTTP.
    //
    // What is left here is exactly what this package can prove on its own: the
    // chip is up, joined, and moving frames in both directions.

    var count: u32 = 0;
    var led_on: bool = false;
    while (true) {
        // **Draining, not sleeping.** The receive latch holds a single frame,
        // so a board that sits in `delayMs` for half a second overruns it on
        // any busy network and the census below stops meaning anything. This
        // keeps the data path moving for the same half second instead.
        //
        // Nothing is answered — that needs an address, and an address needs
        // `lib/ipnet`. `dev.census.data_dropped` is what says whether frames
        // were lost while we were not looking.
        var drained: u32 = 0;
        while (drained < 500) : (drained += 1) {
            _ = dev.pollEthFrame(&frame) catch 0;
            timer.delayMs(1);
        }
        count += 1;
        led_on = !led_on;

        dev.setLed(led_on) catch |err| {
            uart.print(.uart0, "[LED Blink] setLed error: {s}\n", .{@errorName(err)});
        };

        watch.ticks = count;
        watch.led_state = if (led_on) 1 else 0;

        uart.print(.uart0, "[LED Blink] Onboard LED: {s} [tick {d}]\n", .{
            if (led_on) "ON" else "OFF",
            count,
        });

        // Packet census every 10 ticks. The question: does the chip ever send a
        // control *response*, or is everything it returns a flow-control update
        // with no payload? Rung 9 has nothing to match against if it is the
        // latter.
        if (count % 10 == 0) {
            const c = dev.census;
            uart.print(.uart0, "[census] total={d} fc_only={d} ctrl={d} event={d} data={d} other={d} oversize={d} small={d} max={d}\n", .{
                c.total, c.flow_control_only, c.control,           c.event,        c.data,
                c.other, c.oversize,          c.control_too_small, c.largest_seen,
            });
            // The data path's own numbers. `dropped` is the one worth watching:
            // the receive latch holds a single frame, so a busy network will
            // overrun it, and this says by how much rather than leaving it to
            // be discovered later as missing traffic.
            uart.print(.uart0, "[census] eth tx={d} rx={d} dropped={d} malformed={d}\n", .{
                dev.eth_tx_count, dev.eth_rx_count, c.data_dropped, c.data_too_small,
            });
            if (c.saw_control) {
                uart.print(.uart0, "[census] CDC cmd={d} len={d} flags=0x{X:0>8} status=0x{X:0>8} id={d} (expected {d})\n", .{
                    c.ctrl_cmd, c.ctrl_len,         c.ctrl_flags, c.ctrl_status,
                    c.ctrl_id,  c.ctrl_id_expected,
                });
            }
        }
    }
}
