//! A do-nothing firmware root, so the freestanding build of `lib/rotary`
//! contains actual machine code.
//!
//! Compiling `rotary.zig` for the device directly emits an object with no
//! symbols — a library with no entry point is entirely dead code, the linker
//! keeps none of it, and the float guard over such an object can never fire. It
//! type-checks, which is worth something; it proves nothing about what a
//! firmware would link.
//!
//! So this file `export`s the decoder path. Exported symbols cannot be
//! stripped, so the code is emitted and the guard reads the same instructions a
//! device build would.
//!
//! Never installed, never flashed, never called.

const decode = @import("decode.zig");
const contact = @import("contact.zig");

/// The whole hot path: one sample in, one step out.
///
/// Widened to `i8` because `i2` is not extern-compatible — only 0, 8, 16, 32,
/// 64 and 128-bit integers may cross an `arm_aapcs` boundary. The widening is
/// this wrapper's, not the decoder's.
export fn rotary_smoke_update(dec: *decode.Decoder, a: bool, b: bool) i8 {
    return dec.update(decode.code(a, b)).delta();
}

/// Construction, which is where a `Config` default could drag something in.
export fn rotary_smoke_init(dec: *decode.Decoder, at_rest: u8) void {
    dec.* = decode.Decoder.init(.{}, @truncate(at_rest));
}

/// The detentless configuration, so both branches of the emit gate are emitted.
export fn rotary_smoke_init_continuous(dec: *decode.Decoder, at_rest: u8) void {
    dec.* = decode.Decoder.init(
        .{ .quarters_per_detent = 1, .detent_code = null },
        @truncate(at_rest),
    );
}

/// The debouncer, whose wrapping tick arithmetic is the part most likely to be
/// miscompiled into something involving a float if anyone ever reaches for a
/// duration type.
export fn rotary_smoke_contact(c: *contact.Contact, raw: bool, now: u32) u8 {
    return @backingInt(c.update(raw, now));
}

/// A stand-in with `rp2350.gpio`'s shape, so the encoder is instantiated for the
/// device target. It only has to make code exist for the float guard to read.
const SmokeGpio = struct {
    pub const Pin = u6;
    pub const Trigger = enum(u2) { level_low, level_high, edge_low, edge_high };
    var bits: u32 = 0;
    pub fn get(pin: Pin) bool {
        return bits & (@as(u32, 1) << @intCast(pin & 31)) != 0;
    }
    pub fn irqPending(pin: Pin, t: Trigger) bool {
        return (bits >> @intCast(pin & 31)) & @backingInt(t) != 0;
    }
    pub fn clearIrq(pin: Pin, t: Trigger) void {
        _ = pin;
        _ = t;
    }
    pub fn setIrqEnabled(pin: Pin, t: Trigger, on: bool) void {
        _ = pin;
        _ = t;
        _ = on;
    }
};

const encoder = @import("encoder.zig");
const SmokeKnob = encoder.Encoder(SmokeGpio);

/// Both input paths and every accessor.
export fn rotary_smoke_encoder(k: *SmokeKnob, use_irq: bool) i32 {
    if (use_irq) k.onIrq() else k.poll();
    return k.steps() +% @as(i32, @bitCast(k.illegal() +% k.transitions().a));
}
