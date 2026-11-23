//! inventor-bringup.
//!
//! ```
//! zig build inventor
//! minicom -D /dev/cu.usbmodem1302 -b115200
//! ```
//!
//! ## What this is for
//! validating our inventor library by toggling pins on the header
//! and writing to the uart

const std = @import("std");

pub const name = "inventor 2350";
pub const ws2812 = @import("ws2812");
pub const servo = @import("servo");

test {
    _ = ws2812;
    _ = servo;
}

test "Test check" {
    try std.testing.expectEqualStrings(name, "inventor 2350");
}
