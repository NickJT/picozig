//! PCM5122 control port — the page-0 registers a signal generator needs, over I²C.
//!
//! The I²S path is not here. Audio reaches this part over PIO exactly as it
//! reaches the PCM5100A, and `lib/pcm5100` already drives that; what the 5122
//! adds is a control port, and a control port is all this file is.
//!
//! **Integer only.** This runs freestanding with the FPU off, where an `f32`
//! becomes a soft-float call rather than a build error. Volume is therefore
//! expressed in **half-decibels**, which is not a compromise — it is the
//! register's own unit, so the conversion is exact in both directions.
//!
//! Two traps in the part decide the shape of the code below:
//!
//! - **`PLCK` is inverted.** Page 0 / register 4 bit 4 reads **0 when the PLL is
//!   locked** and 1 when it is not, despite being called the PLL Lock Flag.
//!   `pllLocked` is the only place that should ever know this.
//! - **`DCAS` is named for the opposite of its effect** — "Disable Clock Divider
//!   Autoset", default 0, meaning autoset is *on*.

const std = @import("std");
const rp2350 = @import("rp2350");

/// Adapter over `picozig`'s I²C for freestanding use. The `stop` parameter is
/// forwarded rather than hidden, because `readReg` needs it false.
pub const PicoI2c = struct {
    inst: rp2350.i2c.Instance,
    timeout_polls: u32 = 10_000,

    pub fn write(self: PicoI2c, addr: u7, bytes: []const u8, stop: bool) !void {
        try rp2350.i2c.write(self.inst, addr, bytes, stop, self.timeout_polls);
    }

    pub fn read(self: PicoI2c, addr: u7, out: []u8) !void {
        try rp2350.i2c.read(self.inst, addr, out, true, self.timeout_polls);
    }
};

/// The address with `A1`/`A2` open. The board's pads add 1 and 2, so the
/// reachable range is 0x4C–0x4F.
pub const default_addr: u7 = 0x4C;

/// In **any** page, register 0 selects the page for the next access.
pub const page_select_reg: u8 = 0;

/// Page 0 — the control page. The only one this driver touches; pages 1 and
/// 44-plus are analogue trim and the biquad coefficient banks.
pub const Reg = enum(u8) {
    /// `RSTM` in D4 resets the interpolation filter and DAC modules, `RSTR` in
    /// D0 resets the mode registers. Both auto-clear, and **both may only be
    /// set while the part is in standby** — the datasheet calls resetting a
    /// running DAC prohibited.
    reset = 1,
    standby_powerdown = 2,
    mute = 3,
    pll = 4,
    /// `SREF[2:0]` in D(6:4) — which clock the PLL references. **Reset is
    /// `000` = SCK**, and this board has none.
    pll_reference = 13,
    clock_detect_ignore = 37,
    volume_control = 60,
    volume_left = 61,
    volume_right = 62,
    volume_ramp_normal = 63,
    volume_ramp_emergency = 64,
    /// Detected sampling rate and SCK ratio, read-only.
    detected_fs = 91,
    /// Detected BCK-per-frame ratio, bit 8 then bits 7:0. Read-only.
    detected_bck_ratio_hi = 92,
    detected_bck_ratio_lo = 93,
    /// **The register to read when the PLL will not lock.** Per-cause flags,
    /// read-only — see `ClockStatus`.
    clock_status = 94,
    /// **The register to read when there is no sound.** Whether the analogue
    /// output is actually live, per channel, read-only — see `AnalogMute`.
    analog_mute_monitor = 108,
    /// **Line output short detect**, read-only. D4 `SDTM` is live ("a short is
    /// occurring now"); D0 `SHTM` is sticky since the last read and **clears on
    /// read**, so a second read of a one-off short returns zero.
    short_detect = 109,
    /// **What the part's own level decoder makes of the `XSMT` pin**, read-only
    /// — `MTST[1:0]` in D(1:0). Worth more than a meter on that pin, because it
    /// reports the decision rather than the voltage, and it names the
    /// indeterminate band between the thresholds. See `MuteZ`.
    mutez_status = 114,
    /// **The one register that says what the DAC is actually doing**, read-only.
    /// `PSTM[3:0]` in D(3:0) is a state, not a flag soup: playing, ramping,
    /// standby, waiting on the charge pump, or shut down into a short. D7
    /// `BOTM` is the DSP boot-done flag. See `PowerState`.
    power_state = 118,
    /// **Why the analogue output went quiet on its own.** Auto-mute is enabled
    /// out of reset (register 65 resets to `111`), so a part fed all-zero
    /// samples mutes itself with every clock still perfect. Bit 4 left, bit 0
    /// right, read-only.
    auto_mute_flags = 120,
};

/// Page 0 / register 108, read-only.
///
/// **Unlike register 94, 1 is good here.** These are "unmute" monitors, not
/// error flags, so a zero means that channel's analogue output is muted —
/// whatever the digital side thinks it is doing.
pub const AnalogMute = packed struct(u8) {
    /// True when the right channel's analogue output is live.
    right_live: bool,
    /// True when the left channel's analogue output is live.
    left_live: bool,
    _reserved: u6,

    pub fn bothLive(self: AnalogMute) bool {
        return self.left_live and self.right_live;
    }
};

/// Page 0 / register 118, `PSTM[3:0]`.
///
/// **Read this first when there is no sound.** Every other register describes a
/// condition that might cause silence; this one reports the state the part
/// actually reached, which collapses most of the search in a single read.
pub const PowerState = enum(u4) {
    powerdown = 0b0000,
    /// The charge pump has not come up, so the output stage has no negative
    /// rail to swing into. Sticking here is a supply fault, not a signal one.
    wait_for_cp_voltage = 0b0001,
    calibration_1 = 0b0010,
    calibration_2 = 0b0011,
    volume_ramp_up = 0b0100,
    /// **The only state that produces sound.**
    run = 0b0101,
    /// The part has shut its own output down into a short or a load it judges
    /// too low. Nothing on the digital side changes when this happens: clocks,
    /// PLL and volume all keep reading correct.
    line_output_short = 0b0110,
    volume_ramp_down = 0b0111,
    standby = 0b1000,
    _,

    pub fn describe(self: PowerState) []const u8 {
        return switch (self) {
            .powerdown => "powerdown",
            .wait_for_cp_voltage => "waiting for charge-pump voltage",
            .calibration_1, .calibration_2 => "calibrating",
            .volume_ramp_up => "volume ramping up",
            .run => "RUN (playing)",
            .line_output_short => "LINE OUTPUT SHORT / low impedance",
            .volume_ramp_down => "volume ramping down",
            .standby => "standby",
            _ => "unknown",
        };
    }
};

/// Page 0 / register 114, `MTST[1:0]`.
///
/// The part decodes `XSMT` into three bands rather than two, and the middle one
/// is the reason this is worth reading: a line sitting between 0.3 and 0.7 VDD
/// is neither a mute nor a release, and a meter reporting "3.27 V" cannot
/// distinguish a clean high from one the decoder never accepted.
pub const MuteZ = enum(u2) {
    /// `XSMT` below 0.3 VDD — muted.
    low = 0b00,
    /// Between the thresholds. **Not a valid level.**
    indeterminate = 0b01,
    /// `XSMT` at or above 0.7 VDD — released.
    high = 0b11,
    _,

    pub fn describe(self: MuteZ) []const u8 {
        return switch (self) {
            .low => "LOW (< 0.3 VDD) - muted",
            .indeterminate => "INDETERMINATE (0.3-0.7 VDD)",
            .high => "high (>= 0.7 VDD) - released",
            _ => "unknown",
        };
    }
};

/// Page 0 / register 94, read-only.
///
/// **Every bit is an error flag: 0 means good.** That is the opposite of how a
/// name like `bck_valid` reads, so the accessors below are worth using instead
/// of the raw bits.
pub const ClockStatus = packed struct(u8) {
    /// 1 = the sampling rate could not be detected.
    fs_invalid: bool,
    /// 1 = BCK is not a stable 32–256 f_S ratio.
    bck_invalid: bool,
    /// 1 = the SCK ratio is not detectable.
    sck_invalid: bool,
    /// 1 = **clock auto set is not possible** for this f_S / SCK combination.
    /// With no SCK at all there is no ratio to validate, so this is the flag
    /// that explains a part which is clocked correctly and still will not lock.
    autoset_impossible: bool,
    /// 1 = LRCK *and* BCK are both missing (tied low).
    lrck_bck_missing: bool,
    /// 1 = the PLL is unlocked. Same sense as `PLCK` in register 4.
    pll_unlocked: bool,
    /// 1 = SCK is missing or halted.
    sck_missing: bool,
    _reserved: u1,

    /// True when nothing but the SCK flags is complaining — which is the
    /// healthy state on a board that deliberately has no master clock.
    pub fn audioClocksGood(self: ClockStatus) bool {
        return !self.fs_invalid and !self.bck_invalid and !self.lrck_bck_missing;
    }
};

/// The PLL reference, `SREF[2:0]` in register 13 D(6:4).
/// Page 0 / register 37 — which clock detectors are allowed to raise an error.
///
/// **`ignore_sck_halt` is not optional on a board with no master clock.** A
/// halted SCK is a clock halt, and §10.5.2 says a clock error or halt puts the
/// part into standby automatically — so a board that grounds `MCK` by design
/// gets a DAC that resets straight back into standby, with `r2` reading 0x00
/// because nothing *requested* it. The part is behaving exactly as documented.
///
/// **Do not set `IDSK` (0x10) with clock auto set enabled.** Auto set overrides
/// the `SREF` reference select in register 13, whose reset is "reference is
/// SCK"; to move it to BCK it has to *detect* that SCK is absent, and `IDSK` is
/// the bit that stops it looking. Setting it is what made the PLL refuse to
/// lock earlier in this rig's history. `IDCH` suppresses the halt error without
/// blinding the detector.
pub const ignore_sck_halt: u8 = 0x08;

pub const PllReference = enum(u3) {
    /// Reset. Useless on a board with no master clock.
    sck = 0b000,
    /// What a 3-wire I²S board must select, **explicitly**, in software mode.
    bck = 0b001,
    gpio = 0b011,
};

// ---------------------------------------------------------------------------
// Volume
// ---------------------------------------------------------------------------

/// Digital volume, in the register's own unit.
///
/// The encoding runs *downward*: 0x00 is +24 dB, 0x30 is 0 dB, each step is
/// −0.5 dB, 0xFE is −103 dB and 0xFF mutes. So the register byte is
/// `0x30 − half_db`, and every value in between is exact.
pub const Volume = struct {
    /// 0.0 dB. The reset value, and what `0x30` means.
    pub const unity: u8 = 0x30;
    /// 0xFF is mute, not −103.5 dB; −103 dB is the quietest real level.
    pub const mute: u8 = 0xFF;

    /// +24.0 dB, in half-decibels. **Above unity this clips** — there is no
    /// headroom in a full-scale sample — so a caller driving an instrument
    /// should clamp at 0 rather than use it.
    pub const max_half_db: i16 = 48;
    /// −103.0 dB, in half-decibels.
    pub const min_half_db: i16 = -206;

    /// Half-decibels to a register byte, clamped to the representable range.
    /// Exact: one half-decibel is one register step, by construction.
    pub fn fromHalfDb(half_db: i16) u8 {
        const clamped = @max(min_half_db, @min(max_half_db, half_db));
        return @intCast(@as(i16, unity) - clamped);
    }

    /// The inverse. `mute` has no decibel value and reports the floor, which is
    /// why `isMute` exists separately.
    pub fn toHalfDb(reg: u8) i16 {
        if (reg == mute) return min_half_db;
        return @as(i16, unity) - @as(i16, reg);
    }

    pub fn isMute(reg: u8) bool {
        return reg == mute;
    }

    /// Tenths of a decibel to a register byte, **rounded to nearest**.
    ///
    /// Here because the host panel works in tenths and the register cannot: a
    /// tenth is a fifth of a step, so 5 of every 10 requests are not
    /// representable and the rounding has to happen deliberately somewhere.
    /// Rounds half away from zero, so it is symmetric about 0 dB.
    pub fn fromTenthDb(tenth_db: i16) u8 {
        // `tenth/5` as `(tenth*2)/10`, biased by half a divisor so the
        // truncation rounds to nearest. The fraction is always an even tenth,
        // so an exact half never arises and the sign only keeps it symmetric.
        const away: i16 = if (tenth_db < 0) -5 else 5;
        return fromHalfDb(@divTrunc(tenth_db * 2 + away, 10));
    }
};

// ---------------------------------------------------------------------------
// Register fields
// ---------------------------------------------------------------------------

/// Page 0 / register 37 — which clock faults are ignored.
///
/// **Every bit is "ignore this", and every one defaults to 0 = regard it.**
/// `DCAS` is the odd one out and is named for disabling, not enabling.
pub const ClockDetect = packed struct(u8) {
    /// Ignore PLL lock detection. The lock *flag* stays correct regardless.
    ignore_pll_lock: bool = false,
    /// **Disable** clock divider autoset. Default false = autoset enabled.
    disable_autoset: bool = false,
    ignore_lrck_bck_missing: bool = false,
    /// Ignore SCK halt. A grounded SCK is static, which is what halt
    /// detection looks for.
    ignore_sck_halt: bool = false,
    /// Ignore SCK detection. There is no external SCK on this board.
    ignore_sck: bool = false,
    /// Ignore BCK-against-LRCK. BCK must be 32–256 f_S or an error is raised.
    ignore_bck: bool = false,
    ignore_fs: bool = false,
    _reserved: u1 = 0,
};

/// For a board with no external SCK: so neither its absence nor its
/// stillness should be reported as a clock error.
pub const no_external_sck: ClockDetect = .{ .ignore_sck = true, .ignore_sck_halt = true };

/// Page 0 / register 60 — whether the two channels' volumes track.
pub const VolumeLink = enum(u2) {
    /// Default, and what an independent L/R balance needs.
    independent = 0b00,
    right_follows_left = 0b01,
    left_follows_right = 0b10,
};

/// How far the hardware volume moves per update. The DAC ramps by itself; this
/// is how fast.
pub const RampStep = enum(u2) { db_4 = 0, db_2 = 1, db_1 = 2, db_0p5 = 3 };
/// How often it updates, in frame periods. `instant` is a hard mute.
pub const RampRate = enum(u2) { every_1_fs = 0, every_2_fs = 1, every_4_fs = 2, instant = 3 };

/// Page 0 / register 63 — the normal soft mute/un-mute ramps, driven by the
/// `XSMT` pin or by register 3.
pub const NormalRamp = packed struct(u8) {
    up_step: RampStep = .db_1,
    up_rate: RampRate = .every_1_fs,
    down_step: RampStep = .db_1,
    down_rate: RampRate = .every_1_fs,
};

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// Generic over an I²C bus, which must provide:
///
/// - `write(addr: u7, bytes: []const u8, stop: bool) !void`
/// - `read(addr: u7, out: []u8) !void`
///
/// **`stop` is in the contract because a register read needs a repeated
/// start**: the register pointer is written, then the data is read without
/// releasing the bus. `picozig`'s `i2c.write` takes exactly this parameter, and
/// a combined transaction of this shape is what `200671f` fixed and proved on
/// an RV-3028.
pub fn Driver(comptime I2cBus: type) type {
    return struct {
        const Self = @This();

        bus: I2cBus,
        addr: u7,
        /// The page believed to be selected. **Believed, not known** — it is
        /// invalidated on any failed write, because a write that aborted
        /// part-way may or may not have moved the pointer.
        page: ?u8 = null,

        pub fn init(bus: I2cBus, addr: u7) Self {
            return .{ .bus = bus, .addr = addr };
        }

        /// Select a page, skipping the write when it is already selected.
        ///
        /// The cache is worth having because *every* register access would
        /// otherwise cost two transactions, and the fill loop's neighbour on
        /// this bus is a 10 Hz telemetry read.
        pub fn selectPage(self: *Self, page: u8) !void {
            if (self.page) |current| {
                if (current == page) return;
            }
            self.page = null;
            try self.bus.write(self.addr, &[_]u8{ page_select_reg, page }, true);
            self.page = page;
        }

        pub fn writeReg(self: *Self, reg: Reg, value: u8) !void {
            try self.selectPage(0);
            errdefer self.page = null;
            try self.bus.write(self.addr, &[_]u8{ @backingInt(reg), value }, true);
        }

        /// Read any register on any page. **Page 1 holds the analogue half** —
        /// under-voltage protection, analogue gain and mute — and a silent part
        /// whose page-0 registers all look correct is the case that needs it.
        pub fn readRaw(self: *Self, page: u8, reg: u8) !u8 {
            try self.selectPage(page);
            errdefer self.page = null;
            try self.bus.write(self.addr, &[_]u8{reg}, false);
            var out: [1]u8 = undefined;
            try self.bus.read(self.addr, &out);
            return out[0];
        }

        pub fn writeRaw(self: *Self, page: u8, reg: u8, value: u8) !void {
            try self.selectPage(page);
            errdefer self.page = null;
            try self.bus.write(self.addr, &[_]u8{ reg, value }, true);
        }

        pub fn readReg(self: *Self, reg: Reg) !u8 {
            try self.selectPage(0);
            errdefer self.page = null;
            // Repeated start: the pointer write must not be followed by a STOP,
            // or the part is free to consider the transaction finished.
            try self.bus.write(self.addr, &[_]u8{@backingInt(reg)}, false);
            var out: [1]u8 = undefined;
            try self.bus.read(self.addr, &out);
            return out[0];
        }

        // -- clocking -------------------------------------------------------

        /// **True when the PLL is locked.**
        ///
        /// The bit is `PLCK`, page 0 / register 4 bit 4, and it reads **0 for
        /// locked**. The datasheet calls it the PLL Lock Flag, which reads like
        /// the opposite. This function is the only place that should know.
        ///
        /// It is *"always correct"* regardless of the `ignore_pll_lock` bit, so
        /// it stays meaningful even with lock detection disabled.
        pub fn pllLocked(self: *Self) !bool {
            const v = try self.readReg(.pll);
            return v & (1 << 4) == 0;
        }

        pub fn setClockDetect(self: *Self, cfg: ClockDetect) !void {
            try self.writeReg(.clock_detect_ignore, @bitCast(cfg));
        }

        /// Read the per-cause clock flags. **What to read when the PLL will not
        /// lock** — `pllLocked` says only that it has not.
        pub fn clockStatus(self: *Self) !ClockStatus {
            return @bitCast(try self.readReg(.clock_status));
        }

        /// Whether the analogue outputs are actually live. **The first thing to
        /// read when the numbers all look right and nothing comes out** — the
        /// digital side can be clocked, locked and filling buffers while the
        /// analogue mute is still asserted.
        pub fn analogMute(self: *Self) !AnalogMute {
            return @bitCast(try self.readReg(.analog_mute_monitor));
        }

        /// Request or release standby (`RQST`, register 2 D4). Standby stops
        /// the analogue section and the charge pump; releasing it re-runs the
        /// power-up state machine, which is the only way to retry a start that
        /// failed — the part does not retry on its own.
        pub fn setStandby(self: *Self, on: bool) !void {
            try self.writeReg(.standby_powerdown, if (on) 0x10 else 0x00);
        }

        /// Reset the interpolation filter, DAC modules and mode registers
        /// (`RSTM` | `RSTR`). **Only legal while the part is in standby**, and
        /// the caller must have waited for standby to actually take effect —
        /// requesting standby only *starts* a volume ramp down, so a reset
        /// written immediately after the request lands on a running DAC and is
        /// discarded.
        pub fn resetModules(self: *Self) !void {
            try self.bus.write(self.addr, &[_]u8{ @backingInt(Reg.reset), 0x11 }, true);
            self.page = null; // a register reset takes the page select with it
        }

        /// **What the DAC is actually doing.** The first read to make when
        /// there is no sound: it distinguishes playing, ramping, standby, a
        /// charge pump that never came up, and an output shut down into a
        /// short — none of which the clock or mute registers can tell apart.
        pub fn powerState(self: *Self) !PowerState {
            return @fromBackingInt(@as(u4, @truncate(try self.readReg(.power_state))));
        }

        /// Whether the DSP has finished booting (register 118, `BOTM`).
        pub fn dspBooted(self: *Self) !bool {
            return (try self.readReg(.power_state)) & 0x80 != 0;
        }

        /// The part's own decode of the `XSMT` pin. **Reports the decision, not
        /// the voltage** — including the indeterminate band a meter cannot see.
        pub fn muteZ(self: *Self) !MuteZ {
            return @fromBackingInt(@as(u2, @truncate(try self.readReg(.mutez_status))));
        }

        /// Line-output short detect (register 109). `now` is live; `since_last`
        /// is sticky and **cleared by this read**, so a true one-off short
        /// reports once and then reads false.
        pub fn shortDetect(self: *Self) !struct { now: bool, since_last: bool } {
            const v = try self.readReg(.short_detect);
            return .{ .now = (v >> 4) & 1 == 1, .since_last = v & 1 == 1 };
        }

        /// Whether auto-mute has fired, per channel. **Read this when the
        /// analogue output is muted and nothing requested it**: auto-mute is on
        /// out of reset and fires on a run of zero samples, so it is what a DAC
        /// does when its data line has stopped carrying data while its clocks
        /// carry on perfectly.
        pub fn autoMuted(self: *Self) !struct { left: bool, right: bool } {
            const v = try self.readReg(.auto_mute_flags);
            return .{ .left = (v >> 4) & 1 == 1, .right = v & 1 == 1 };
        }

        /// Point the PLL at a reference. In software mode this is **not**
        /// optional on a board with no SCK: the reset value is SCK, and clock
        /// auto set can only override it if it can detect an SCK ratio to
        /// validate — which, with no SCK, it cannot.
        pub fn setPllReference(self: *Self, reference: PllReference) !void {
            try self.writeReg(.pll_reference, @as(u8, @backingInt(reference)) << 4);
        }

        /// The BCK-per-frame ratio the part has detected. 32 for a 16-bit
        /// stereo frame; **the part's own opinion**, not what was configured.
        pub fn detectedBckRatio(self: *Self) !u16 {
            const hi = try self.readReg(.detected_bck_ratio_hi);
            const lo = try self.readReg(.detected_bck_ratio_lo);
            return (@as(u16, hi & 1) << 8) | lo;
        }

        // -- level ----------------------------------------------------------

        /// Both channels, in half-decibels. Written as two registers because
        /// they are two registers; there is no atomic pair on this part, and
        /// the DAC's own ramp is what stops the gap being audible.
        pub fn setVolumeHalfDb(self: *Self, left: i16, right: i16) !void {
            try self.writeReg(.volume_left, Volume.fromHalfDb(left));
            try self.writeReg(.volume_right, Volume.fromHalfDb(right));
        }

        pub fn setVolumeRaw(self: *Self, left: u8, right: u8) !void {
            try self.writeReg(.volume_left, left);
            try self.writeReg(.volume_right, right);
        }

        pub fn setVolumeLink(self: *Self, link: VolumeLink) !void {
            try self.writeReg(.volume_control, @backingInt(link));
        }

        pub fn setNormalRamp(self: *Self, ramp: NormalRamp) !void {
            try self.writeReg(.volume_ramp_normal, @bitCast(ramp));
        }

        /// Soft mute, per channel. Ramped by the hardware, not a hard gate —
        /// which is the point of using it rather than writing 0xFF to the
        /// volume registers.
        pub fn setMute(self: *Self, left: bool, right: bool) !void {
            const v: u8 = (@as(u8, @intFromBool(left)) << 4) | @intFromBool(right);
            try self.writeReg(.mute, v);
        }

        // -- power ----------------------------------------------------------

        /// Standby powers down most of the part but keeps the charge pump and
        /// digital supply, so it restarts quickly. Powerdown drops the charge
        /// pump too and **takes precedence over standby**.
        pub fn setPower(self: *Self, standby: bool, powerdown: bool) !void {
            const v: u8 = (@as(u8, @intFromBool(standby)) << 4) | @intFromBool(powerdown);
            try self.writeReg(.standby_powerdown, v);
        }
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// Records every transaction so a test can assert on the bytes that would reach
/// the part, which is the only thing a driver with no hardware can be judged on.
const MockBus = struct {
    const Txn = struct { write: bool, stop: bool, bytes: [8]u8, len: usize };

    log: [64]Txn = undefined,
    count: usize = 0,
    /// Fed back by `read`, oldest first.
    canned: []const u8 = &.{},
    canned_pos: usize = 0,
    fail_after: ?usize = null,

    fn write(self: *MockBus, addr: u7, bytes: []const u8, stop: bool) !void {
        _ = addr;
        if (self.fail_after) |n| if (self.count >= n) return error.NoAcknowledge;
        var t: Txn = .{ .write = true, .stop = stop, .bytes = undefined, .len = bytes.len };
        @memcpy(t.bytes[0..bytes.len], bytes);
        self.log[self.count] = t;
        self.count += 1;
    }

    fn read(self: *MockBus, addr: u7, out: []u8) !void {
        _ = addr;
        if (self.fail_after) |n| if (self.count >= n) return error.NoAcknowledge;
        for (out) |*b| {
            b.* = if (self.canned_pos < self.canned.len) self.canned[self.canned_pos] else 0;
            self.canned_pos += 1;
        }
        self.log[self.count] = .{ .write = false, .stop = true, .bytes = undefined, .len = out.len };
        self.count += 1;
    }

    fn wrote(self: *const MockBus, i: usize) []const u8 {
        return self.log[i].bytes[0..self.log[i].len];
    }
};

const TestDriver = Driver(*MockBus);

test "the volume encoding runs downward from +24 dB, and the anchors are exact" {
    // Every value the datasheet names, in both directions.
    try testing.expectEqual(@as(u8, 0x00), Volume.fromHalfDb(48)); // +24.0 dB
    try testing.expectEqual(@as(u8, 0x2F), Volume.fromHalfDb(1)); // +0.5 dB
    try testing.expectEqual(@as(u8, 0x30), Volume.fromHalfDb(0)); // 0.0 dB
    try testing.expectEqual(@as(u8, 0x31), Volume.fromHalfDb(-1)); // -0.5 dB
    try testing.expectEqual(@as(u8, 0xFE), Volume.fromHalfDb(-206)); // -103 dB
    try testing.expectEqual(Volume.unity, Volume.fromHalfDb(0));
}

test "half-decibels round-trip exactly, because a step is the unit" {
    // The reason the interface is half-decibels and not tenths: no rounding
    // exists to get wrong.
    var hd: i16 = Volume.min_half_db;
    while (hd <= Volume.max_half_db) : (hd += 1) {
        try testing.expectEqual(hd, Volume.toHalfDb(Volume.fromHalfDb(hd)));
    }
}

test "out of range is clamped, and mute is not a decibel value" {
    try testing.expectEqual(@as(u8, 0x00), Volume.fromHalfDb(1000));
    try testing.expectEqual(@as(u8, 0xFE), Volume.fromHalfDb(-1000));
    // 0xFF is mute; it must never be produced by a level request, or asking for
    // the quietest level would silence the part instead.
    var hd: i16 = -400;
    while (hd <= 400) : (hd += 1) try testing.expect(Volume.fromHalfDb(hd) != Volume.mute);

    try testing.expect(Volume.isMute(Volume.mute));
    try testing.expect(!Volume.isMute(0xFE));
}

test "tenths round to the nearest half, symmetrically about zero" {
    try testing.expectEqual(Volume.fromHalfDb(0), Volume.fromTenthDb(0));
    try testing.expectEqual(Volume.fromHalfDb(-12), Volume.fromTenthDb(-60)); // -6.0 dB
    try testing.expectEqual(Volume.fromHalfDb(-1), Volume.fromTenthDb(-5)); // -0.5 dB
    // -0.3 dB is not representable; nearest is -0.5, not 0.
    try testing.expectEqual(Volume.fromHalfDb(-1), Volume.fromTenthDb(-3));
    try testing.expectEqual(Volume.fromHalfDb(0), Volume.fromTenthDb(-2)); // -0.2 -> 0
    // Symmetric: +0.3 and -0.3 must land the same distance from unity.
    const up = @as(i16, Volume.unity) - @as(i16, Volume.fromTenthDb(3));
    const down = @as(i16, Volume.fromTenthDb(-3)) - @as(i16, Volume.unity);
    try testing.expectEqual(up, down);
}

test "a register write selects page 0 first, then writes the pair" {
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);

    try dev.writeReg(.volume_left, 0x30);
    try testing.expectEqual(@as(usize, 2), bus.count);
    try testing.expectEqualSlices(u8, &.{ 0, 0 }, bus.wrote(0)); // page select -> 0
    try testing.expectEqualSlices(u8, &.{ 61, 0x30 }, bus.wrote(1));
}

test "the page is written once, not once per access" {
    // Every access would otherwise cost two transactions on a bus shared with
    // a 10 Hz telemetry read.
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);

    try dev.writeReg(.volume_left, 0x30);
    try dev.writeReg(.volume_right, 0x34);
    try dev.writeReg(.mute, 0);

    try testing.expectEqual(@as(usize, 4), bus.count); // 1 page + 3 registers
    try testing.expectEqualSlices(u8, &.{ 0, 0 }, bus.wrote(0));
    try testing.expectEqualSlices(u8, &.{ 62, 0x34 }, bus.wrote(2));
}

test "a failed write forgets the page rather than assuming it" {
    // A write that aborted part-way may or may not have moved the pointer, so
    // the cache has to be invalidated. Believing a stale page writes a level
    // into a coefficient bank.
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);
    try dev.writeReg(.volume_left, 0x30);
    try testing.expect(dev.page != null);

    bus.fail_after = bus.count;
    try testing.expectError(error.NoAcknowledge, dev.writeReg(.volume_right, 0x30));
    try testing.expect(dev.page == null);
}

test "a register read is a repeated start, not a stop" {
    // The pointer write must hold the bus. Releasing it lets the part treat the
    // transaction as finished, and this is the exact shape that broke a
    // touchscreen controller until picozig's write learned to wait for TX_EMPTY.
    var bus: MockBus = .{};
    bus.canned = &.{0x00};
    var dev = TestDriver.init(&bus, default_addr);

    _ = try dev.readReg(.pll);
    // page select (stop), pointer (NO stop), then the read
    try testing.expect(bus.log[0].stop);
    try testing.expectEqualSlices(u8, &.{4}, bus.wrote(1));
    try testing.expect(!bus.log[1].stop);
    try testing.expect(!bus.log[2].write);
}

test "PLCK is inverted: zero means locked" {
    // The datasheet calls it the PLL Lock Flag and reads 0 for locked. Getting
    // this backwards passes bring-up on a board that is not locked, which is
    // the failure Rung 2b exists to catch.
    var locked: MockBus = .{ .canned = &.{0x00} };
    var dev_a = TestDriver.init(&locked, default_addr);
    try testing.expect(try dev_a.pllLocked());

    var unlocked: MockBus = .{ .canned = &.{0x10} }; // bit 4 set
    var dev_b = TestDriver.init(&unlocked, default_addr);
    try testing.expect(!try dev_b.pllLocked());

    // The other bits must not be read as lock state — PLLE at bit 0 is set on
    // reset, so a naive `v != 0` would report it unlocked forever.
    var enabled: MockBus = .{ .canned = &.{0x01} };
    var dev_c = TestDriver.init(&enabled, default_addr);
    try testing.expect(try dev_c.pllLocked());
}

test "the clock-detect bits land where register 37 puts them" {
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);

    try dev.setClockDetect(no_external_sck);
    // IDSK is bit 4, IDCH is bit 3.
    try testing.expectEqualSlices(u8, &.{ 37, 0b0001_1000 }, bus.wrote(1));

    // Defaults are all-regard, which is a zero byte — and `disable_autoset`
    // false means autoset is ON, the opposite of what the name suggests.
    bus.count = 0;
    dev.page = 0;
    try dev.setClockDetect(.{});
    try testing.expectEqualSlices(u8, &.{ 37, 0 }, bus.wrote(0));
}

test "mute and power pack their two channels into bits 4 and 0" {
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);

    try dev.setMute(true, false);
    try testing.expectEqualSlices(u8, &.{ 3, 0b0001_0000 }, bus.wrote(1));

    bus.count = 0;
    try dev.setMute(true, true);
    try testing.expectEqualSlices(u8, &.{ 3, 0b0001_0001 }, bus.wrote(0));

    bus.count = 0;
    try dev.setPower(true, false); // standby, not powerdown
    try testing.expectEqualSlices(u8, &.{ 2, 0b0001_0000 }, bus.wrote(0));
}

test "the field structs encode the part's documented reset values" {
    // **This is what catches a transposed bit field.** Each of these is a reset
    // value the datasheet states, so a struct whose defaults reproduce it is a
    // struct whose bit order is right. Register 63 is the sharp one — 0x22 is
    // neither zero nor round, so matching it by accident is unlikely — and
    // `example/control-check` reads the same byte back off real silicon.
    try testing.expectEqual(@as(u8, 0x22), @as(u8, @bitCast(NormalRamp{})));
    try testing.expectEqual(@as(u8, 0x00), @as(u8, @bitCast(ClockDetect{})));
    try testing.expectEqual(@as(u8, 0x30), Volume.unity);
    try testing.expectEqual(@as(u2, 0), @backingInt(VolumeLink.independent));

    // And the no-SCK bits: IDSK is bit 4, IDCH bit 3.
    try testing.expectEqual(@as(u8, 0b0001_1000), @as(u8, @bitCast(no_external_sck)));
}

test "an independent L and R is the default, which is what a balance needs" {
    var bus: MockBus = .{};
    var dev = TestDriver.init(&bus, default_addr);
    try dev.setVolumeLink(.independent);
    try testing.expectEqualSlices(u8, &.{ 60, 0 }, bus.wrote(1));

    bus.count = 0;
    try dev.setVolumeHalfDb(0, -12); // 0 dB left, -6 dB right
    try testing.expectEqualSlices(u8, &.{ 61, 0x30 }, bus.wrote(0));
    try testing.expectEqualSlices(u8, &.{ 62, 0x3C }, bus.wrote(1));
}
