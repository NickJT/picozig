//! A do-nothing firmware root, so the freestanding build of `lib/ads1115`
//! contains actual machine code for the float guard to read.
//!
//! A library with no entry point compiles to an object with no symbols, and a
//! guard over that can never fire. Exported symbols cannot be stripped, so these
//! keep the encode, decode, scaling and driver paths in the object.
//!
//! Never installed, never flashed, never called.

const ads1115 = @import("ads1115.zig");

/// A bus that goes nowhere, so the driver's register path is emitted.
const NullBus = struct {
    var sink: u16 = 0;

    pub fn writeRegs(_: NullBus, _: u7, _: u8, data: []const u8) !void {
        @as(*volatile u16, &sink).* = @as(u16, data[0]) << 8 | data[1];
    }

    pub fn readRegs(_: NullBus, _: u7, _: u8, out: []u8) !void {
        const v = @as(*volatile u16, &sink).*;
        out[0] = @truncate(v >> 8);
        out[1] = @truncate(v);
    }
};

export fn ads1115_smoke_roundtrip(raw: u16) u16 {
    return ads1115.Config.decode(raw).encode(raw & 0x8000 != 0);
}

export fn ads1115_smoke_nanovolts(code: i16, range: u8) i64 {
    return ads1115.nanovolts(code, @fromBackingInt(@as(u3, @truncate(range))));
}

export fn ads1115_smoke_driver(raw: u16) i16 {
    var dev = ads1115.Driver(NullBus).init(.{}, .gnd);
    dev.startSingle(ads1115.Config.decode(raw)) catch return 0;
    dev.setReadyThresholds() catch return 0;
    return dev.readConversion() catch 0;
}
