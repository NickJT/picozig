//! Hardware True Random Number Generator (TRNG).
//!
//! Exposes RP2350's hardware TRNG block to generate non-deterministic random
//! numbers for cryptographic keys, nonces, and packet sequence initializers.

const std = @import("std");
const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// Release the TRNG block from reset and enable the random source.
pub fn init() void {
    hw.Resets.unreset(hw.Resets.trng);
    hw.write(hw.Trng.rnd_source_enable, hw.Trng.rnd_source_enable_bit);
}

/// Returns true when a 192-bit batch of hardware entropy is ready in the
/// Entropy Holding Registers (EHR_DATA0..EHR_DATA5).
pub fn isReady() bool {
    return (hw.read(hw.Trng.valid) & hw.Trng.valid_ehr_valid) != 0;
}

/// Read a single 32-bit random value from the hardware TRNG.
///
/// Bounded: a TRNG whose source was never enabled never becomes ready, and a
/// key generator that silently stops is worse than one that says it could not
/// get entropy. **Never fall back to a fixed value on `error.Timeout`** — that
/// is how a nonce becomes a constant.
pub fn readU32() bounded.Error!u32 {
    var s = bounded.Spin.init(bounded.startup_polls);
    while (!isReady()) try s.tick();
    const val = hw.read(hw.Trng.ehr_data0);
    // Reading EHR_DATA5 signals consumption of the current 192-bit batch to the IP.
    _ = hw.read(hw.Trng.ehr_data5);
    return val;
}

/// Read a 64-bit random value from the hardware TRNG. See `readU32`.
pub fn readU64() bounded.Error!u64 {
    var s = bounded.Spin.init(bounded.startup_polls);
    while (!isReady()) try s.tick();
    const low = @as(u64, hw.read(hw.Trng.ehr_data0));
    const high = @as(u64, hw.read(hw.Trng.ehr_data1));
    _ = hw.read(hw.Trng.ehr_data5);
    return (high << 32) | low;
}

/// Fill `buf` with non-deterministic random bytes from the hardware TRNG.
pub fn fillBytes(buf: []u8) bounded.Error!void {
    var index: usize = 0;
    while (index < buf.len) {
        const val = try readU32();
        const bytes = std.mem.asBytes(&val);
        const chunk = @min(bytes.len, buf.len - index);
        @memcpy(buf[index .. index + chunk], bytes[0..chunk]);
        index += chunk;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "TRNG register address definitions match hardware spec" {
    try testing.expectEqual(@as(u32, 0x400f_8000), hw.trng_base);
    try testing.expectEqual(hw.trng_base + 0x10, hw.Trng.valid);
    try testing.expectEqual(hw.trng_base + 0x14, hw.Trng.ehr_data0);
    try testing.expectEqual(hw.trng_base + 0x28, hw.Trng.ehr_data5);
    try testing.expectEqual(hw.trng_base + 0x2c, hw.Trng.rnd_source_enable);
}
