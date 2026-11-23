//! control-check — prove the PCM5122 control port against the real part.
//!
//! ```
//! zig build swd-control-check -Dboard=pico2
//! zig build watch-control-check -Dboard=pico2
//! ```
//!
//! **No I²S, and that is the point.** Everything here runs over I²C alone, so it
//! is the whole of the driver that can be proven before the audio path exists —
//! page select, the repeated-start register read, the volume encoding, and the
//! inverted PLL lock bit. Bring-up step 2 proper needs BCK and LRCK; this is
//! everything before it.
//!
//! ## What makes the reset-value check worth more than it looks
//!
//! Every register below has a reset value the datasheet states. Reading them and
//! finding the documented values is what proves the **register numbering** is
//! right — a `Reg` enum with a transposed address reads a plausible byte from
//! the wrong place and is invisible until something behaves oddly much later.
//! Register 63 is the sharpest of them: its reset is `0x22`, which is not a
//! round number and not zero, so matching it by accident is unlikely.
//!
//! Leaves the part exactly as it found it.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const pcm5122 = @import("pcm5122");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;

pub const watch_magic: u32 = 0x5122_0c0c;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// 1 if the part acknowledged at `default_addr`.
    probe_ok: u32,
    /// One bit per reset-value check, set when the read matched. Bit order
    /// follows `reset_checks` below.
    resets_matched: u32,
    /// How many reset checks there were, so a zero mask is not ambiguous.
    resets_total: u32,
    reg2: u32,
    reg3: u32,
    reg4: u32,
    reg60: u32,
    reg61: u32,
    reg62: u32,
    reg63: u32,
    /// `pllLocked()`. Expected **0** here: there are no clocks.
    pll_locked: u32,
    /// Volume written, read back, and decoded to the half-decibels asked for.
    volume_roundtrip_ok: u32,
    /// Register 37 written and read back.
    clock_detect_ok: u32,
    /// Everything restored to its reset value afterwards.
    restored_ok: u32,
    /// Non-zero if any transfer failed; the first error wins.
    err_code: u32,
};

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

const Bus = pcm5122.PicoI2c;
const Dev = pcm5122.Driver(Bus);
const Volume = pcm5122.Volume;

/// Register, expected reset value, and what a mismatch would mean.
const ResetCheck = struct { reg: pcm5122.Reg, expect: u8, name: []const u8 };

const reset_checks = [_]ResetCheck{
    .{ .reg = .standby_powerdown, .expect = 0x00, .name = "r2  standby/powerdown" },
    .{ .reg = .mute, .expect = 0x00, .name = "r3  mute" },
    .{ .reg = .volume_control, .expect = 0x00, .name = "r60 volume link (independent)" },
    .{ .reg = .volume_left, .expect = 0x30, .name = "r61 volume L (0.0 dB)" },
    .{ .reg = .volume_right, .expect = 0x30, .name = "r62 volume R (0.0 dB)" },
    // The sharp one: not zero, not round, and it is exactly what this driver's
    // `NormalRamp` defaults encode.
    .{ .reg = .volume_ramp_normal, .expect = 0x22, .name = "r63 normal ramp" },
};

fn note(err: *u32, e: anyerror) void {
    if (err.* == 0) err.* = 1;
    uart.print(.uart0, "  ! {s}\n", .{@errorName(e)});
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== pcm5122 control-check ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .probe_ok = 0,
        .resets_matched = 0,
        .resets_total = reset_checks.len,
        .reg2 = 0,
        .reg3 = 0,
        .reg4 = 0,
        .reg60 = 0,
        .reg61 = 0,
        .reg62 = 0,
        .reg63 = 0,
        .pll_locked = 0,
        .volume_roundtrip_ok = 0,
        .clock_detect_ok = 0,
        .restored_ok = 0,
        .err_code = 0,
    };

    // 100 kHz, not 400: the breakout's pull-ups are 10 kΩ, which is weak for a
    // fast-mode rise time. Nothing here needs the speed.
    i2c.init(.i2c0, .{ .sda = 4, .scl = 5, .clk_hz = board.sys_hz });

    var dev = Dev.init(Bus{ .inst = .i2c0 }, pcm5122.default_addr);

    // -- probe ----------------------------------------------------------
    watch.probe_ok = @intFromBool(i2c.probe(.i2c0, pcm5122.default_addr, 10_000));
    uart.print(.uart0, "probe 0x{X:0>2}: {s}\n", .{
        pcm5122.default_addr,
        if (watch.probe_ok == 1) "ack" else "NO ACK",
    });

    // -- reset values ---------------------------------------------------
    // What proves the register numbering, not merely that reads work.
    uart.print(.uart0, "reset values:\n", .{});
    for (reset_checks, 0..) |c, i| {
        const got = dev.readReg(c.reg) catch |e| {
            note(&watch.err_code, e);
            continue;
        };
        const ok = got == c.expect;
        if (ok) watch.resets_matched |= @as(u32, 1) << @intCast(i);
        uart.print(.uart0, "  {s:<30} got 0x{X:0>2} want 0x{X:0>2}  {s}\n", .{
            c.name, got, c.expect, if (ok) "ok" else "MISMATCH",
        });
        switch (c.reg) {
            .standby_powerdown => watch.reg2 = got,
            .mute => watch.reg3 = got,
            .volume_control => watch.reg60 = got,
            .volume_left => watch.reg61 = got,
            .volume_right => watch.reg62 = got,
            .volume_ramp_normal => watch.reg63 = got,
            else => {},
        }
    }

    // -- the PLL lock bit, and its inversion -----------------------------
    const r4 = dev.readReg(.pll) catch |e| blk: {
        note(&watch.err_code, e);
        break :blk 0;
    };
    watch.reg4 = r4;
    const locked = dev.pllLocked() catch |e| blk: {
        note(&watch.err_code, e);
        break :blk false;
    };
    watch.pll_locked = @intFromBool(locked);
    // PLLE (bit 0) is set out of reset; PLCK (bit 4) reads 1 for *not* locked.
    uart.print(.uart0, "r4  0x{X:0>2}: PLLE={d} PLCK={d} -> pllLocked()={s}\n", .{
        r4, r4 & 1, (r4 >> 4) & 1, if (locked) "true" else "false",
    });
    uart.print(.uart0, "  (false is correct: no BCK/LRCK yet, so nothing to lock to)\n", .{});

    // -- volume round-trip through real silicon --------------------------
    // -6.0 dB is -12 half-decibels, which the encoding puts at 0x3C.
    const want_half_db: i16 = -12;
    const encoded = Volume.fromHalfDb(want_half_db);
    if (dev.setVolumeHalfDb(want_half_db, want_half_db)) {
        const l = dev.readReg(.volume_left) catch |e| blk: {
            note(&watch.err_code, e);
            break :blk 0;
        };
        const r = dev.readReg(.volume_right) catch |e| blk: {
            note(&watch.err_code, e);
            break :blk 0;
        };
        const decoded = Volume.toHalfDb(l);
        const ok = l == encoded and r == encoded and decoded == want_half_db;
        watch.volume_roundtrip_ok = @intFromBool(ok);
        uart.print(.uart0, "volume: wrote {d} half-dB -> 0x{X:0>2}, read L 0x{X:0>2} R 0x{X:0>2}, decoded {d}  {s}\n", .{
            want_half_db, encoded, l, r, decoded, if (ok) "ok" else "MISMATCH",
        });
    } else |e| note(&watch.err_code, e);

    // -- register 37, the bits a generator without SCK sets --------------
    const want_cd: u8 = @bitCast(pcm5122.no_external_sck);
    if (dev.setClockDetect(pcm5122.no_external_sck)) {
        const got = dev.readReg(.clock_detect_ignore) catch |e| blk: {
            note(&watch.err_code, e);
            break :blk 0;
        };
        watch.clock_detect_ok = @intFromBool(got == want_cd);
        uart.print(.uart0, "r37 clock detect: wrote 0x{X:0>2} read 0x{X:0>2}  {s}\n", .{
            want_cd, got, if (got == want_cd) "ok" else "MISMATCH",
        });
    } else |e| note(&watch.err_code, e);

    // -- put it back -----------------------------------------------------
    // **Leave the part as it was found.** A device left in a state the next
    // firmware does not expect is the kind of thing that gets blamed on the
    // next firmware.
    var restored = true;
    dev.setVolumeRaw(Volume.unity, Volume.unity) catch |e| {
        note(&watch.err_code, e);
        restored = false;
    };
    dev.setClockDetect(.{}) catch |e| {
        note(&watch.err_code, e);
        restored = false;
    };
    if (restored) {
        const l = dev.readReg(.volume_left) catch 0xFF;
        const cd = dev.readReg(.clock_detect_ignore) catch 0xFF;
        restored = l == Volume.unity and cd == 0;
    }
    watch.restored_ok = @intFromBool(restored);
    uart.print(.uart0, "restored: {s}\n", .{if (restored) "ok" else "NO"});

    const all_resets: u32 = (@as(u32, 1) << reset_checks.len) - 1;
    const pass = watch.probe_ok == 1 and
        watch.resets_matched == all_resets and
        watch.volume_roundtrip_ok == 1 and
        watch.clock_detect_ok == 1 and
        watch.restored_ok == 1 and
        watch.err_code == 0;
    uart.print(.uart0, "\n{s}\n", .{if (pass) "PASS — the control port is proven" else "FAIL — see above"});

    // Idle, republishing a tick so `watch` shows the board is still alive.
    // **Nothing is driven from here**, so the example cannot wedge the board or
    // leave the DAC doing anything.
    while (true) {
        timer.delayMs(500);
        watch.ticks +%= 1;
    }
}
