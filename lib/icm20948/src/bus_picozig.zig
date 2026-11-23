//! Transport adapter linking picozig's RP2350 I2C master driver to icm20948.

const std = @import("std");
const rp2350 = @import("rp2350");
const icm20948 = @import("icm20948");

pub const default_timeout_polls: u32 = 1_000_000;

pub const Bus = struct {
    instance: rp2350.i2c.Instance,
    timeout_polls: u32 = default_timeout_polls,

    pub const Error = rp2350.i2c.Error;

    /// `sys_hz` is the board's clk_sys, which the I2C divisor derives from.
    pub fn init(instance: rp2350.i2c.Instance, sda: rp2350.Pin, scl: rp2350.Pin, sys_hz: u32) Bus {
        rp2350.i2c.init(instance, .{
            .sda = sda,
            .scl = scl,
            .clk_hz = sys_hz,
            .speed = .standard_100k,
        });
        return .{ .instance = instance };
    }

    pub fn readReg(self: *Bus, addr: u7, reg: u8) Error!u8 {
        var val: u8 = 0;
        try self.readBytes(addr, reg, @as(*[1]u8, &val));
        return val;
    }

    pub fn writeReg(self: *Bus, addr: u7, reg: u8, val: u8) Error!void {
        const payload = [_]u8{ reg, val };
        try rp2350.i2c.write(self.instance, addr, &payload, true, self.timeout_polls);
    }

    pub fn readBytes(self: *Bus, addr: u7, reg: u8, dest: []u8) Error!void {
        const reg_buf = [_]u8{reg};
        // Write register address with stop=true
        try rp2350.i2c.write(self.instance, addr, &reg_buf, true, self.timeout_polls);
        // Read requested bytes with stop=true
        try rp2350.i2c.read(self.instance, addr, dest, true, self.timeout_polls);
    }

    pub fn writeBytes(self: *Bus, addr: u7, reg: u8, src: []const u8) Error!void {
        var tx_buf: [32]u8 = undefined;
        if (src.len + 1 > tx_buf.len) return error.Timeout;
        tx_buf[0] = reg;
        @memcpy(tx_buf[1 .. 1 + src.len], src);
        try rp2350.i2c.write(self.instance, addr, tx_buf[0 .. 1 + src.len], true, self.timeout_polls);
    }
};

pub const Device = icm20948.Device(Bus);
