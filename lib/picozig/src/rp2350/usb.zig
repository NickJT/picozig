//! USB 2.0 Full Speed **device** controller. Datasheet §12.7.
//!
//! This is the chip layer and nothing more: endpoints, buffers, the setup
//! packet, and the handful of register pokes that make the controller answer a
//! host. **It knows nothing about descriptors, standard requests or device
//! classes** — those are USB protocol rather than RP2350 fact, and they live in
//! `lib/usb`. The split is the same one, for the same reason, as chip versus
//! board: this file would be identical for any RP2350 application, and the
//! protocol half would be identical on any chip.
//!
//! ## What the hardware does, and what is left to you
//!
//! The controller handles NRZI, bit stuffing, CRC, packet framing, ACK/NAK and
//! the data toggle *checking*. What remains is: point it at some buffers, tell
//! it when a buffer is ready, and answer the host's questions. That is genuinely
//! the whole job, and it is why this file is short.
//!
//! ## Three things that are not obvious
//!
//! **Endpoint control and buffer control are different arrays with different
//! starting endpoints** — see `hw.Usb.Dpram`. EP0 has buffers but no endpoint
//! control register, because its configuration is implicit.
//!
//! **`MAIN_CTRL.PHY_ISO` resets set and must be cleared.** New on RP2350, and
//! §12.7.2 calls it the one break in RP2040 software compatibility — so every
//! RP2040 example omits it, and omitting it means no enumeration and no clue.
//!
//! **Setting AVAILABLE in the same write as the rest of the buffer control word
//! is safe here, and was not on RP2040.** §12.7.2.2.3: RP2350 added a DPSRAM
//! double read, on by default, specifically so software no longer has to write
//! AVAILABLE separately after a delay. `send` and `receive` rely on it.

const std = @import("std");
const hw = @import("hw.zig");

// ---------------------------------------------------------------------------
// DPRAM access
// ---------------------------------------------------------------------------
//
// **`hw.setBits` and `hw.clrBits` must never be used on DPRAM.** They work by
// writing to the APB atomic alias regions at `+0x2000` and `+0x3000`, and DPRAM
// is dual-port SRAM with no such aliases — those offsets are 8 and 12 kB past a
// 4 kB window. The USB *registers* at `0x50110000` do support the aliases and
// the SDK's own `usb_hw_set` uses them; the DPRAM at `0x50100000` does not, and
// the SDK has no equivalent for it either. Buffer control registers live in
// DPRAM, so anything touching one reads, modifies and writes.
//
// §12.7.1.1.3: DPRAM supports 8-, 16- and 32-bit access. Writes take one cycle,
// reads two.

inline fn read8(addr: u32) u8 {
    return @as(*volatile u8, @ptrFromInt(addr)).*;
}

inline fn write8(addr: u32, value: u8) void {
    @as(*volatile u8, @ptrFromInt(addr)).* = value;
}

/// Read-modify-write a DPRAM word. The atomic aliases are not available here,
/// so this is not atomic — it is only ever used from the USB interrupt handler
/// or before the controller is running, where nothing else is writing.
inline fn dpramSetBits(addr: u32, mask: u32) void {
    hw.write(addr, hw.read(addr) | mask);
}

inline fn dpramClrBits(addr: u32, mask: u32) void {
    hw.write(addr, hw.read(addr) & ~mask);
}

pub const Dir = hw.Usb.Dir;
pub const TransferType = hw.Usb.TransferType;

/// The largest packet a Full Speed control or bulk endpoint may carry.
pub const max_packet_size: u16 = 64;

// ---------------------------------------------------------------------------
// Endpoints
// ---------------------------------------------------------------------------

/// One endpoint, its buffer, and the data toggle that belongs to it.
///
/// **`next_pid` is state, not configuration.** USB alternates DATA0 and DATA1
/// on successive packets so that a retransmission can be told from a new
/// packet, and the controller checks the toggle but does not maintain it. A
/// driver that forgets to flip it gets one packet through and then silence,
/// because every packet after the first looks like a duplicate to the host.
pub const Endpoint = struct {
    num: u4,
    dir: Dir,
    transfer: TransferType,
    /// This endpoint's buffer, as a DPRAM address.
    buf: u32,
    max_packet: u16 = max_packet_size,
    next_pid: u1 = 0,

    pub fn bufControl(self: Endpoint) u32 {
        return hw.Usb.Dpram.bufControl(self.num, self.dir);
    }

    /// The endpoint control register, or null for EP0 — which has none.
    pub fn epControl(self: Endpoint) ?u32 {
        if (self.num == 0) return null;
        return hw.Usb.Dpram.epControl(self.num, self.dir);
    }

    pub fn statusBit(self: Endpoint) u32 {
        return hw.Usb.buffStatusBit(self.num, self.dir);
    }

    /// The address a descriptor would carry for this endpoint: number in the
    /// low nibble, direction in bit 7.
    pub fn address(self: Endpoint) u8 {
        return @as(u8, self.num) | (if (self.dir == .in) @as(u8, 0x80) else 0);
    }
};

/// EP0's two halves. Their buffers are at fixed DPRAM addresses and their
/// transfer type is always control, so they are constants rather than something
/// a caller configures.
pub fn ep0In() Endpoint {
    return .{ .num = 0, .dir = .in, .transfer = .control, .buf = hw.Usb.Dpram.ep0_buf_in };
}

pub fn ep0Out() Endpoint {
    return .{ .num = 0, .dir = .out, .transfer = .control, .buf = hw.Usb.Dpram.ep0_buf_out };
}

/// A bump allocator over the free part of DPRAM.
///
/// **Buffers must be 64-byte aligned**, and hardware accepts an unaligned one
/// silently and then reads and writes somewhere else. Rounding up is therefore
/// not a tidiness measure. `alloc` returning null rather than wrapping matters
/// for the same reason: 3,712 bytes is not much, and an overflowing allocator
/// would hand out an address that overlaps the endpoint control arrays.
pub const BufferAllocator = struct {
    next: u32 = hw.Usb.Dpram.free_offset,

    pub fn alloc(self: *BufferAllocator, bytes: u16) ?u32 {
        const size = std.mem.alignForward(u32, bytes, 64);
        if (self.next + size > hw.Usb.dpram_size) return null;
        const addr = hw.Usb.dpram_base + self.next;
        self.next += size;
        return addr;
    }

    pub fn used(self: BufferAllocator) u32 {
        return self.next - hw.Usb.Dpram.free_offset;
    }

    pub fn remaining(self: BufferAllocator) u32 {
        return hw.Usb.dpram_size - self.next;
    }
};

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

/// Bring the controller up in device mode, **without presenting to the host**.
///
/// `connect` is separate on purpose: a device that asserts its pull-up before
/// its endpoints exist will be asked for descriptors it cannot yet answer, and
/// the host's first impression is the one it acts on. Configure, then connect.
///
/// `clocks.initUsb` must have run — clk_usb has to be at 48 MHz before any of
/// this means anything, and that function also enforces RP2350-E12's
/// requirement that clk_sys be above 48 MHz.
pub fn init() void {
    // **Reset the block, do not merely unreset it.** A debugger reset is a
    // processor warm reset and leaves the controller alone, so after an SWD
    // flash the previous image's device is still attached and configured. An
    // unreset is then a no-op, the host never sees a disconnect, and the new
    // image waits for an enumeration that already happened — no port, no
    // error. The reset drops the pull-up, which is the disconnect.
    hw.Resets.resetCycle(hw.Resets.usbctrl);

    // Clear DPRAM. The controller reads endpoint and buffer control out of it,
    // and whatever the bootrom's own USB left behind is not ours.
    var off: u32 = 0;
    while (off < hw.Usb.dpram_size) : (off += 4) hw.write(hw.Usb.dpram_base + off, 0);

    // **The RP2350-only step.** PHY_ISO resets set, isolating the PHY from the
    // switched core domain; nothing works until it is cleared, and no RP2040
    // example does it. Cleared before the controller is enabled, not after.
    hw.write(hw.Usb.main_ctrl, 0); // PHY_ISO is the only set bit at reset, so 0 clears it

    hw.write(hw.Usb.muxing, hw.Usb.muxing_to_phy | hw.Usb.muxing_softcon);

    // Tell the controller it is plugged in. On a board that does not wire VBUS
    // to a GPIO this is the only way it can know, and without it the device
    // never leaves its powered-off state machine.
    hw.write(hw.Usb.pwr, hw.Usb.pwr_vbus_detect | hw.Usb.pwr_vbus_detect_override_en);

    // Device mode: CONTROLLER_EN set, HOST_NDEVICE clear, PHY_ISO clear.
    hw.write(hw.Usb.main_ctrl, hw.Usb.main_controller_en);

    // One BUFF_STATUS bit per EP0 buffer completed, rather than per two.
    hw.write(hw.Usb.sie_ctrl, hw.Usb.sie_ep0_int_1buf);

    hw.write(hw.Usb.inte, hw.Usb.int_buff_status | hw.Usb.int_bus_reset | hw.Usb.int_setup_req);
}

/// Assert the DP pull-up: this is the moment the host sees a device appear.
pub fn connect() void {
    hw.setBits(hw.Usb.sie_ctrl, hw.Usb.sie_pullup_en);
}

/// Drop the pull-up. To the host this is indistinguishable from unplugging,
/// which is exactly what makes it a useful thing to be able to do in software.
pub fn disconnect() void {
    hw.clrBits(hw.Usb.sie_ctrl, hw.Usb.sie_pullup_en);
}

/// Write an endpoint's control register. EP0 has none and is skipped, which is
/// not an error — it lets a caller loop over every endpoint uniformly.
pub fn configure(ep: Endpoint) void {
    const ctrl = ep.epControl() orelse return;
    hw.write(ctrl, hw.Usb.epControlWord(ep.buf, ep.transfer, 0));
}

// ---------------------------------------------------------------------------
// Transfers
// ---------------------------------------------------------------------------

/// Hand `data` to the controller to send on the next IN token.
///
/// Copies into DPRAM, sets the length, flips the data toggle and marks the
/// buffer available in one write — see the note at the top of this file about
/// why that single write is safe on RP2350 and was not on RP2040.
pub fn send(ep: *Endpoint, data: []const u8) void {
    std.debug.assert(ep.dir == .in);
    std.debug.assert(data.len <= ep.max_packet);

    for (data, 0..) |b, i| write8(ep.buf + @as(u32, @intCast(i)), b);

    var val: u32 = @as(u32, @intCast(data.len)) | hw.Usb.buf_ctrl_avail | hw.Usb.buf_ctrl_full;
    if (ep.next_pid == 1) val |= hw.Usb.buf_ctrl_data1_pid;
    ep.next_pid ^= 1;
    hw.write(ep.bufControl(), val);
}

/// Arm an OUT endpoint to receive up to `len` bytes.
pub fn receive(ep: *Endpoint, len: u16) void {
    std.debug.assert(ep.dir == .out);
    std.debug.assert(len <= ep.max_packet);

    var val: u32 = @as(u32, len) | hw.Usb.buf_ctrl_avail;
    if (ep.next_pid == 1) val |= hw.Usb.buf_ctrl_data1_pid;
    ep.next_pid ^= 1;
    hw.write(ep.bufControl(), val);
}

/// Send a zero-length packet — the status stage of a control transfer that
/// carried no data, and the way a device says "done" to the host.
pub fn sendZlp(ep: *Endpoint) void {
    send(ep, &.{});
}

/// How many bytes the last completed transfer actually moved. For an OUT
/// endpoint this is the host's packet length, which may be shorter than what
/// was armed for.
pub fn completedLen(ep: Endpoint) u16 {
    return @intCast(hw.read(ep.bufControl()) & hw.Usb.buf_ctrl_len_mask);
}

/// Copy a completed OUT transfer out of DPRAM.
pub fn readInto(ep: Endpoint, dest: []u8) []u8 {
    const n = @min(completedLen(ep), dest.len);
    for (0..n) |i| dest[i] = read8(ep.buf + @as(u32, @intCast(i)));
    return dest[0..n];
}

/// The eight bytes of the last SETUP packet, which the controller writes to a
/// fixed place at the bottom of DPRAM.
pub fn setupPacket() [8]u8 {
    var out: [8]u8 = undefined;
    for (0..8) |i| out[i] = read8(hw.Usb.Dpram.setup_packet + @as(u32, @intCast(i)));
    return out;
}

// ---------------------------------------------------------------------------
// Status and control
// ---------------------------------------------------------------------------

/// Set the device address the controller answers to.
///
/// **Only after the status stage of `SET_ADDRESS` has completed.** The request
/// itself is answered from address 0, and a device that adopts the new address
/// when the request *arrives* stops answering before it has acknowledged. That
/// is a 50 ms-window bug: it usually works, which is worse than never working.
pub fn setAddress(addr: u7) void {
    hw.write(hw.Usb.addr_endp, @as(u32, addr) & hw.Usb.addr_mask);
}

/// Stall an endpoint — the device's way of saying "I do not support that".
///
/// **EP0 needs `EP_STALL_ARM` as well as the buffer control bit**, and the
/// controller clears that arming register itself whenever a SETUP arrives,
/// because the specification requires a STALL condition to be cleared by the
/// next setup packet.
pub fn stall(ep: Endpoint) void {
    if (ep.num == 0) {
        // EP_STALL_ARM is a register, so the atomic alias applies here...
        hw.setBits(hw.Usb.ep_stall_arm, if (ep.dir == .in)
            hw.Usb.ep_stall_arm_ep0_in
        else
            hw.Usb.ep_stall_arm_ep0_out);
    }
    // ...and the buffer control register is DPRAM, so it does not.
    dpramSetBits(ep.bufControl(), hw.Usb.buf_ctrl_stall);
}

pub fn clearStall(ep: *Endpoint) void {
    dpramClrBits(ep.bufControl(), hw.Usb.buf_ctrl_stall);
    ep.next_pid = 0;
}

/// Which endpoint buffers have completed. Bits are cleared by writing them
/// back, and it is possible for a second buffer to complete between the read
/// and the clear — which is why the clear writes the bits that were read
/// rather than all ones.
pub fn buffStatus() u32 {
    return hw.read(hw.Usb.buff_status);
}

pub fn clearBuffStatus(bits: u32) void {
    hw.write(hw.Usb.buff_status, bits);
}

pub fn intStatus() u32 {
    return hw.read(hw.Usb.ints);
}

/// Acknowledge a bus reset and return to the unaddressed state.
///
/// **SIE_STATUS bits are write-1-to-clear**, so this writes exactly the one bit
/// rather than reading, modifying and storing — which would clear every other
/// set flag as an invisible side effect.
pub fn ackBusReset() void {
    hw.write(hw.Usb.sie_status, hw.Usb.status_bus_reset);
    hw.write(hw.Usb.addr_endp, 0);
}

pub fn ackSetup() void {
    hw.write(hw.Usb.sie_status, hw.Usb.status_setup_rec);
}

/// Whether the host has the device configured and the bus is not suspended.
pub fn connected() bool {
    return hw.read(hw.Usb.sie_status) & hw.Usb.status_connected != 0;
}

// ---------------------------------------------------------------------------
// Tests — pure arithmetic over the DPRAM layout; no MMIO
// ---------------------------------------------------------------------------

const testing = std.testing;

test "EP0 has buffers but no endpoint control register" {
    const in = ep0In();
    const out = ep0Out();
    try testing.expectEqual(@as(?u32, null), in.epControl());
    try testing.expectEqual(@as(?u32, null), out.epControl());
    try testing.expectEqual(hw.Usb.Dpram.ep0_buf_in, in.buf);
    try testing.expectEqual(hw.Usb.Dpram.ep0_buf_out, out.buf);
    try testing.expectEqual(hw.Usb.Dpram.buf_ctrl_base, in.bufControl());
}

test "an endpoint's descriptor address encodes direction in bit 7" {
    const in = Endpoint{ .num = 2, .dir = .in, .transfer = .bulk, .buf = 0 };
    const out = Endpoint{ .num = 1, .dir = .out, .transfer = .bulk, .buf = 0 };
    try testing.expectEqual(@as(u8, 0x82), in.address());
    try testing.expectEqual(@as(u8, 0x01), out.address());
    // dev_lowlevel's own two endpoints, so this is checkable against the
    // reference: EP1 OUT and EP2 IN.
    try testing.expectEqual(@as(u8, 0x01), out.address());
    try testing.expectEqual(@as(u8, 0x82), in.address());
}

test "the buffer allocator rounds every allocation up to 64 bytes" {
    var a = BufferAllocator{};
    const first = a.alloc(1).?;
    try testing.expectEqual(hw.Usb.dpram_base + hw.Usb.Dpram.free_offset, first);
    const second = a.alloc(64).?;
    // A one-byte request still consumes a whole 64-byte slot, because an
    // unaligned buffer is accepted by hardware and then used incorrectly.
    try testing.expectEqual(first + 64, second);
    try testing.expectEqual(@as(u32, 128), a.used());
}

test "the allocator refuses to run past the end of DPRAM" {
    var a = BufferAllocator{};
    // 3712 free bytes is 58 buffers of 64.
    var count: u32 = 0;
    while (a.alloc(64)) |_| : (count += 1) {}
    try testing.expectEqual(@as(u32, 58), count);
    try testing.expectEqual(hw.Usb.Dpram.free_bytes, a.used());
    try testing.expectEqual(@as(u32, 0), a.remaining());
    // And every allocation stayed inside DPRAM.
    try testing.expect(hw.Usb.Dpram.free_offset + a.used() == hw.Usb.dpram_size);
}

test "a large allocation that would overflow is refused rather than wrapped" {
    var a = BufferAllocator{};
    try testing.expectEqual(@as(?u32, null), a.alloc(@intCast(hw.Usb.Dpram.free_bytes + 64)));
    // The failed attempt must not have consumed anything.
    try testing.expectEqual(@as(u32, 0), a.used());
}

test "allocated buffers never collide with the control arrays" {
    var a = BufferAllocator{};
    const first = a.alloc(64).?;
    // The last thing before free space is EP0's OUT buffer.
    try testing.expect(first >= hw.Usb.Dpram.ep0_buf_out + hw.Usb.Dpram.ep0_buf_size);
    try testing.expect(first > hw.Usb.Dpram.buf_ctrl_base);
    try testing.expect(first > hw.Usb.Dpram.ep_ctrl_base);
}

test "EP_STALL_ARM arms IN with bit 0 and OUT with bit 1" {
    // EP0 alone needs this register as well as the buffer control STALL bit.
    // Getting the two the wrong way round stalls the opposite half of a control
    // transfer, which presents as a request that hangs rather than one that
    // fails.
    try testing.expectEqual(@as(u32, 1), hw.Usb.ep_stall_arm_ep0_in);
    try testing.expectEqual(@as(u32, 2), hw.Usb.ep_stall_arm_ep0_out);
    try testing.expect(hw.Usb.ep_stall_arm_ep0_in != hw.Usb.ep_stall_arm_ep0_out);
}

test "endpoint control words are what the reference example writes" {
    // dev_lowlevel configures EP1 OUT and EP2 IN as bulk, interrupt per buffer,
    // with the buffer offset in the low bits. Reproducing its word here is the
    // closest thing to a unit test against the vendor implementation.
    var a = BufferAllocator{};
    const ep1_buf = a.alloc(64).?;
    const word = hw.Usb.epControlWord(ep1_buf, .bulk, 0);
    try testing.expectEqual(
        hw.Usb.ep_enable | hw.Usb.ep_interrupt_per_buffer |
            (@as(u32, 2) << hw.Usb.ep_buffer_type_shift) | 0x180,
        word,
    );
}
