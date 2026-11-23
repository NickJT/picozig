//! `ipnet` — an IPv4 stack for a device that has an Ethernet frame and nothing
//! else. Freestanding-safe, allocator-free, and **it does not know what radio
//! it is sitting on.**
//!
//! ## What is here, and what it is proven against
//!
//! Every file below was written inside `lib/cyw43` and moved here unchanged on
//! 2026-08-31. That history is the point: `net.zig`, `dhcp.zig`, `ipstack.zig`,
//! `session.zig` and `ntp.zig` imported nothing but `std` and each other even
//! while they lived in a driver, so the move was a `git mv` rather than a
//! rewrite, and **everything in this package retains the hardware evidence it
//! was carrying** — rungs 15, 16 and 18 of the cyw43 bring-up: a real DHCP
//! lease from a real router, defended by ARP; NTP answers that agree with an
//! independent clock, and no answer at all when aimed at an address that cannot
//! reply; a lease renewed and a link recovered across a deliberate drop.
//!
//! ## The device seam
//!
//! There is no device type here — `IpStack` and `Session` are generic over one,
//! the same way `nrf24` is generic over its SPI bus. Two
//! contracts, deliberately separate, because the smaller one is the one a
//! non-WiFi carrier can meet:
//!
//!   * **`IpStack(Dev)`** needs `mac`, `sendEthFrame`, `pollEthFrame`,
//!     `delayMs`. Frames in, frames out, and a way to wait.
//!   * **`Session(Dev)`** needs those **plus** the join half: `wifi_on`,
//!     `link_status`, `wifiOn`, `joinWifi`, `awaitJoin`.
//!
//! Splitting them is what keeps a future wired or non-802.11 consumer from
//! having to invent a `joinWifi` that returns success.
//!
//! ## What is not here
//!
//! ARP, ICMP echo, DHCP, UDP, NTP, TCP, DNS and HTTP both ways are all here and
//! all proven on hardware — the N1-N12 ladder finished 2026-09-03. What is
//! **not** here: **no TLS**, which is a decision still to be written rather
//! than a gap to be filled; **no IP fragment reassembly**, fragments being
//! refused rather than misread; and a **deliberately narrow TCP** — one
//! connection at a time, stop-and-wait, fixed MSS, no window scaling, no SACK,
//! out-of-order segments dropped rather than queued. That contract is not to be
//! quietly widened.

const std = @import("std");

/// Submodules. Pure — no device, no state, no allocator — except `ipstack`
// and `session`, which are the two that hold any.
pub const net = @import("net.zig");
pub const dhcp = @import("dhcp.zig");
pub const tcp = @import("tcp.zig");
pub const dns = @import("dns.zig");
pub const http = @import("http.zig");
pub const ipstack = @import("ipstack.zig");
pub const session = @import("session.zig");
pub const ntp = @import("ntp.zig");

pub const IpStack = ipstack.IpStack;
pub const Session = session.Session;
pub const NtpClient = ntp.NtpClient;

// **Adding a submodule means adding a line here.** Zig analyses a file only
// when something references it, and `zig build test` collects tests only from
// files it analysed — so a module reachable through `pub const x =
// @import(...)` alone contributes *no tests* while the suite reports a total
// that looks complete. `ntp.zig`'s three tests sat unrun that way for weeks.
test {
    _ = net;
    _ = dhcp;
    _ = tcp;
    _ = dns;
    _ = http;
    _ = ipstack;
    _ = session;
    _ = ntp;
}
