//! The bond, kept across power cycles.
//!
//! A link key held only in RAM is forgotten by a power cycle while the phone
//! keeps its half, and the mismatch does not re-pair silently: **Simple Pairing
//! completes with `ERROR_CODE_AUTHENTICATION_FAILURE` (0x05)** and the device
//! has to be forgotten on the phone before it will work again. Measured on
//! 2026-08-23, and the reason this file exists.
//!
//! One slot, one device. "Last paired wins" is the whole policy, and it is
//! honest for a speaker: a table would need an eviction rule nobody has a use
//! for yet, and an unused rule is a rule nobody has tested.
//!
//! ## Where it lives, and why not at the end of flash
//!
//! `0x0020_0000` — two megabytes in, against an image that ends around
//! `0x10050000`. The obvious home is the last sector, which is what btstack
//! does, but **the end of RP2350 flash is where the bootrom looks for a
//! partition table**, and a 4 MB Pico 2 W has been seen rejecting a block
//! written there. Far from both ends costs nothing.
//!
//! ## Writing this is not free of hazards
//!
//! `flash.eraseAndProgram` masks interrupts and runs from RAM, and the erase
//! holds the CPU for tens of milliseconds with XIP inaccessible.
//!
//! **This list used to have two entries and be wrong.** It said the call was
//! safe because it only happens during pairing, and because the audio DMA reads
//! RAM rather than XIP. Both remain true. **Neither is about the Bluetooth
//! controller, and that is the one that bites** — an omission that cost most of
//! a day on 2026-08-27 and is the reason this section is now written as three
//! constraints rather than two reassurances.
//!
//! * **The CYW43's Bluetooth firmware does not survive the stall.** Tens of
//!   milliseconds with interrupts masked is the host abandoning the gSPI
//!   interface, and the controller resets itself. Measured: `fw_ready` drops in
//!   the poll immediately after the erase, with both rings drained and command
//!   credits healthy — the stall, not any packet. A disconnect that writes no
//!   bond does not do it; the same disconnect with a bond write does.
//!
//!   **And it cannot be restarted in place.** `bt.start` succeeds — patchram
//!   re-downloaded, indices zeroed, `fw_ready` set — and `HCI_Reset` then
//!   returns `NoAnswer`, six attempts over 500 ms included, because
//!   `bt2wlan_pwrup_wake` powers up a core that is already up and nothing
//!   available makes it re-execute. The vendor's `cybt_shared_bus_driver.c`
//!   has no restart path for the same reason.
//!
//!   **So the caller must reboot after calling this.** `main.zig`'s
//!   `commitPendingBond` does, and defers the write to a disconnect so the
//!   reboot costs nothing. Calling `save` anywhere the controller matters and
//!   not rebooting leaves the board deaf until power is removed.
//! * **It is only ever called with no link up.** Deferred out of pairing
//!   deliberately: doing it where the key arrives broke the session outright.
//!   The same call during playback would also stall `serviceAudio` for several
//!   buffer periods and be plainly audible.
//! * **The audio DMA reads RAM, not XIP** — `buffers` and `addr_ring` are both
//!   in `.bss`. `lib/picozig/src/rp2350/flash.zig` is explicit that no DMA may
//!   touch XIP during the erase window, and this design happens to satisfy it.
//!   It would not if the buffers ever moved to flash.

const std = @import("std");
const rp2350 = @import("rp2350");
const flash = rp2350.flash;

pub const slot_offset: u32 = 0x0020_0000;

/// 'BOND'. A slot that has never been written reads as erased flash — all
/// ones — so any constant distinguishes a real record from a blank one.
pub const magic: u32 = 0x424F_4E44;

pub const Record = extern struct {
    magic: u32,
    peer: [6]u8,
    key_type: u8,
    _pad: u8 = 0,
    key: [16]u8,
    /// **Not for tamper resistance** — for telling a half-written record from a
    /// whole one. A power cut mid-program leaves a page that is neither erased
    /// nor valid, and reading a corrupt key as a real one produces exactly the
    /// authentication failure this file exists to prevent.
    check: u32,
};

fn checksum(r: Record) u32 {
    var sum: u32 = 0x9E37_79B9;
    for (std.mem.asBytes(&r)[0 .. @sizeOf(Record) - 4]) |b| {
        sum = (sum << 1 | sum >> 31) +% b;
    }
    return sum;
}

/// The stored bond, or null if there is none this code can trust.
pub fn load() ?Record {
    var page: [@sizeOf(Record)]u8 align(4) = undefined;
    flash.readXip(slot_offset, &page);
    const r: Record = std.mem.bytesToValue(Record, &page);
    if (r.magic != magic) return null;
    if (r.check != checksum(r)) return null;
    return r;
}

/// Replace the stored bond.
///
/// **A whole 256-byte page, because the hardware has no finer granularity** and
/// `eraseAndProgram` refuses anything else rather than silently rounding. The
/// rest of the page is zero and unread.
pub fn save(peer: [6]u8, key_type: u8, key: [16]u8) !void {
    var r = Record{
        .magic = magic,
        .peer = peer,
        .key_type = key_type,
        .key = key,
        .check = 0,
    };
    r.check = checksum(r);

    var page: [flash.page_size]u8 align(4) = @splat(0);
    @memcpy(page[0..@sizeOf(Record)], std.mem.asBytes(&r));
    try flash.eraseAndProgram(slot_offset, &page);
}

/// Forget the stored bond, by writing a page that will not validate.
pub fn clear() !void {
    const page: [flash.page_size]u8 align(4) = @splat(0);
    try flash.eraseAndProgram(slot_offset, &page);
}
