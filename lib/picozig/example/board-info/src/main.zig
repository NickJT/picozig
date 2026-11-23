//! board-info — who is this chip, and how is its flash laid out?
//!
//! Everything comes from either SYSINFO (via `silicon.zig`) or the bootrom's
//! two information calls (via `bootrom.zig`), whose field layouts are taken from
//! the RP2350 datasheet §5.4.8.16, §5.4.8.17 and §5.9.4.2.
//!
//! ## What it is actually for
//!
//! Three things, in the order they come up:
//!
//!   * **Telling two boards apart.** The 64-bit device id is the only thing that
//!     does, and it is what a serial number, a MAC address or a per-device key
//!     should be derived from rather than stored.
//!   * **Confirming the image matches the silicon.** A build made for the wrong
//!     package runs perfectly until it touches a pin the package does not have.
//!     The package is read here from *both* SYSINFO and the bootrom, and they
//!     are compared — see the note on that below.
//!   * **Seeing the flash as the bootrom sees it**, which on a stock board is
//!     "no partition table", and that is not a fault.
//!
//! ## Two sources for the package, deliberately
//!
//! `silicon.package()` reads `SYSINFO.PACKAGE_SEL` directly; the bootrom
//! reports its own view through `CHIP_INFO`. They should agree, and a
//! disagreement would mean SYSINFO is being read while still in reset — which
//! returns 0 with no error at all and decodes as a perfectly plausible QFN-80.
//! **Two sources that agree is evidence; one source is an assumption**, and
//! this is the cheapest place in the workspace to make that check.
//!
//! ```bash
//! zig build swd-board-info -Dboard=pico2w
//! zig build watch-board-info      # the same facts over SWD, if the UART is out
//! ```

const std = @import("std");
const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const uart = rp2350.uart;
const timer = rp2350.timer;
const silicon = rp2350.silicon;
const bootrom = rp2350.bootrom;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    /// Device id, low and high. **The unique board id**, and the reason this
    /// block exists as well as the UART output: it is the one fact here worth
    /// reading off a board that cannot talk.
    id_lo: u32,
    id_hi: u32,
    /// `silicon.package()` | `bootrom` CHIP_INFO's PACKAGE_SEL << 16.
    /// **They must agree**; see the header.
    package_pair: u32,
    stepping: u32,
    cpu_arch: u32,
    /// partition_count | present << 8 | rom-call status << 16.
    /// Status: 0 ok, 1 unavailable, 2 rom error, 3 unsupported.
    partitions: u32,
    /// Flash size in KiB, as OpenOCD reports it independently — useful to
    /// compare against the partition table's idea of the same device.
    flash_devinfo: u32,
    /// 0 while the ROM calls are outstanding, 1 once all have been made.
    /// Distinguishes "the answers are all zero" from "we never got there".
    complete: u32,
};

pub const watch_magic: u32 = 0x62_64_69_6e; // "bdin"

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

fn statusOf(err: anyerror) u32 {
    return switch (err) {
        bootrom.Error.Unavailable => 1,
        bootrom.Error.RomError => 2,
        bootrom.Error.Unsupported => 3,
        else => 4,
    };
}

pub fn main() noreturn {
    // `.selftest` is NOLOAD and survives a reset by design, so a fresh image
    // inherits the previous one's values field for field. Initialise all of it.
    watch = .{
        .magic = 0,
        .ticks = 0,
        .id_lo = 0,
        .id_hi = 0,
        .package_pair = 0xFFFF_FFFF,
        .stepping = 0xFFFF_FFFF,
        .cpu_arch = 0xFFFF_FFFF,
        .partitions = 0,
        .flash_devinfo = 0,
        .complete = 0,
    };

    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    watch.magic = watch_magic;

    uart.print(.uart0, "\n=== board-info ===\n", .{});
    uart.print(.uart0, "board      {s}\n", .{board.name});

    // --- silicon, read directly -------------------------------------------
    const pkg = silicon.package();
    const step = silicon.stepping();
    watch.stepping = @backingInt(step);
    uart.print(.uart0, "silicon    {s} stepping, {s}\n", .{ step.name(), pkg.name() });

    if (pkg != board.expected_package) {
        uart.print(.uart0, "WARNING    image built for {s}, running on {s}\n", .{
            board.expected_package.name(),
            pkg.name(),
        });
    }

    // --- the bootrom ------------------------------------------------------
    if (!bootrom.recognised()) {
        // Everything below needs the ROM table. Say so once rather than
        // reporting four separate failures that all have one cause.
        uart.print(.uart0, "bootrom    NOT RECOGNISED — no ROM table at 0x10\n", .{});
    } else {
        if (bootrom.chipInfo()) |info| {
            watch.id_lo = @truncate(info.device_id);
            watch.id_hi = @truncate(info.device_id >> 32);
            watch.package_pair = @as(u32, @backingInt(pkg)) | (info.package_sel << 16);

            const hex = info.hex();
            uart.print(.uart0, "board id   {s}\n", .{&hex});

            // The comparison the header argues for. `PACKAGE_SEL` is the whole
            // register; `silicon.package()` decodes bit 0 of it.
            const rom_pkg_bit = info.package_sel & 1;
            const agree = rom_pkg_bit == @backingInt(pkg);
            uart.print(.uart0, "package    SYSINFO {d}, bootrom {d} — {s}\n", .{
                @backingInt(pkg),
                rom_pkg_bit,
                if (agree) "agree" else "DISAGREE: is SYSINFO out of reset?",
            });
        } else |err| {
            uart.print(.uart0, "board id   unavailable ({s})\n", .{@errorName(err)});
        }

        if (bootrom.cpuArch()) |arch| {
            watch.cpu_arch = @backingInt(arch);
            uart.print(.uart0, "cpu        {s}\n", .{switch (arch) {
                .arm => "Arm",
                .riscv => "RISC-V",
                _ => "unknown",
            }});
        } else |err| {
            uart.print(.uart0, "cpu        unavailable ({s})\n", .{@errorName(err)});
        }

        // FLASH_DEVINFO, raw. Decoding it is a job for whoever needs it; what
        // matters here is that it is reported rather than assumed.
        var buf: [4]u32 = @splat(0);
        if (bootrom.getSysInfo(&buf, bootrom.sys_info.flash_dev_info)) |got| {
            if (got.len >= 2 and got[0] & bootrom.sys_info.flash_dev_info != 0) {
                watch.flash_devinfo = got[1];
                uart.print(.uart0, "flash info 0x{x:0>8} (OTP FLASH_DEVINFO format)\n", .{got[1]});
            } else {
                uart.print(.uart0, "flash info not supported by this bootrom\n", .{});
            }
        } else |err| {
            uart.print(.uart0, "flash info unavailable ({s})\n", .{@errorName(err)});
        }

        // --- the partition table ------------------------------------------
        if (bootrom.tableInfo()) |pt| {
            watch.partitions = @as(u32, pt.partition_count) |
                (@as(u32, @intFromBool(pt.present)) << 8);
            if (pt.present) {
                uart.print(.uart0, "partitions {d}\n", .{pt.partition_count});
            } else {
                // **Normal on a stock board.** Said in full rather than left as
                // a bare zero, because "0 partitions" reads like a failure and
                // is not one.
                uart.print(.uart0, "partitions none — no partition table written (normal)\n", .{});
            }
            uart.print(.uart0, "unpart.    sectors {d}..{d}, flags 0x{x:0>8}\n", .{
                pt.unpartitioned_first_sector,
                pt.unpartitioned_last_sector,
                pt.unpartitioned_flags,
            });
            // The datasheet's own note: unpartitioned space is always reported
            // as base 0 and 0x2000 sectors, whatever the part actually holds.
            uart.print(.uart0, "           (the ROM always reports 0x2000 sectors here, not the real size)\n", .{});
        } else |err| {
            watch.partitions = statusOf(err) << 16;
            uart.print(.uart0, "partitions unavailable ({s})\n", .{@errorName(err)});
        }
    }

    watch.complete = 1;
    uart.print(.uart0, "\n", .{});

    // Reprints on a slow cycle rather than once, so a terminal opened after the
    // board booted still sees everything. `blink-hello` prints its banner once
    // and that has cost a re-flash more than once.
    while (true) {
        timer.delayMs(5000);
        watch.ticks +%= 1;
        const hex_id = blk: {
            var t: [16]u8 = @splat('?');
            if (bootrom.chipInfo()) |i| t = i.hex() else |_| {}
            break :blk t;
        };
        uart.print(.uart0, "board-info {d}: id {s}, {s}\n", .{
            watch.ticks,
            &hex_id,
            board.name,
        });
    }
}
