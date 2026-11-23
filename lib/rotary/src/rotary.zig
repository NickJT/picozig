//! Rotary encoder knobs, wired directly to the GPIO of the board the
//! application already runs on.
//!
//! Rung 0 is the decoder and nothing else: `decode.zig` turns a sampled contact
//! pair into detent steps and imports only `std`, so the whole of it is
//! exercised on a laptop. Where the codes come from — an interrupt, a poll — is
//! the caller's, and deliberately not decided here.

const std = @import("std");

pub const decode = @import("decode.zig");
pub const contact = @import("contact.zig");
pub const encoder = @import("encoder.zig");

pub const Code = decode.Code;
pub const Decoder = decode.Decoder;
pub const Step = decode.Step;
pub const code = decode.code;
pub const quarterStep = decode.quarterStep;

pub const Contact = contact.Contact;
pub const Edge = contact.Edge;

pub const Encoder = encoder.Encoder;
pub const Transitions = encoder.Transitions;

// `Config` is deliberately *not* re-exported flat: `decode` and `contact` each
// have one and they configure different things. `rotary.decode.Config` and
// `rotary.contact.Config` say which is meant.

test {
    // `@import` above already pulls `decode.zig`'s own tests into this build.
    // This forces every public declaration to be *analysed*, which is what
    // catches a re-export of a name that no longer exists — lazy analysis
    // otherwise leaves one standing indefinitely.
    std.testing.refAllDecls(@This());
    std.testing.refAllDecls(decode);
    std.testing.refAllDecls(contact);
    std.testing.refAllDecls(encoder);
}
