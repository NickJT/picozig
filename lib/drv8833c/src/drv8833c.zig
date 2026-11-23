//! DRV8833 / DRV8833C Dual H-Bridge Motor & Stepper Driver.
//!
//! Controls dual DC motors or a 4-wire bipolar stepper motor using DRV8833 H-bridges
//! on RP2350 microcontrollers (Pico 2 W / Pimoroni Inventor 2350 W).

const std = @import("std");
const rp2350 = @import("rp2350");

const gpio = rp2350.gpio;
const timer = rp2350.timer;

/// Direction for motor or stepper rotation.
pub const Direction = enum {
    forward,
    reverse,
};

/// Stepper motor stepping mode.
pub const StepMode = enum {
    /// Full step (2-phase on) — maximum torque.
    full_step,
    /// Wave drive (1-phase on) — lower power consumption.
    wave_drive,
    /// Half step (8-step sequence) — higher angular resolution.
    half_step,
};

/// Bridge output pin state pair (IN1, IN2).
pub const BridgeState = struct {
    in1: bool,
    in2: bool,

    pub const coast = BridgeState{ .in1 = false, .in2 = false };
    pub const forward = BridgeState{ .in1 = true, .in2 = false };
    pub const reverse = BridgeState{ .in1 = false, .in2 = true };
    pub const brake = BridgeState{ .in1 = true, .in2 = true };
};

/// Both H-Bridge states (Bridge A and Bridge B) for a single stepper motor step.
pub const DualBridgeState = struct {
    a: BridgeState,
    b: BridgeState,
};

// ---------------------------------------------------------------------------
// Pure Stepper State Machine Logic (Host Testable)
// ---------------------------------------------------------------------------

/// 4-Step Full-Step (2-phase on) Sequence
pub const full_step_table = [4]DualBridgeState{
    .{ .a = BridgeState.forward, .b = BridgeState.forward }, // Step 0: +A, +B
    .{ .a = BridgeState.reverse, .b = BridgeState.forward }, // Step 1: -A, +B
    .{ .a = BridgeState.reverse, .b = BridgeState.reverse }, // Step 2: -A, -B
    .{ .a = BridgeState.forward, .b = BridgeState.reverse }, // Step 3: +A, -B
};

/// 4-Step Wave Drive (1-phase on) Sequence
pub const wave_drive_table = [4]DualBridgeState{
    .{ .a = BridgeState.forward, .b = BridgeState.coast }, // Step 0: +A, 0
    .{ .a = BridgeState.coast, .b = BridgeState.forward }, // Step 1: 0, +B
    .{ .a = BridgeState.reverse, .b = BridgeState.coast }, // Step 2: -A, 0
    .{ .a = BridgeState.coast, .b = BridgeState.reverse }, // Step 3: 0, -B
};

/// 8-Step Half-Step Sequence
pub const half_step_table = [8]DualBridgeState{
    .{ .a = BridgeState.forward, .b = BridgeState.coast }, // Step 0: +A, 0
    .{ .a = BridgeState.forward, .b = BridgeState.forward }, // Step 1: +A, +B
    .{ .a = BridgeState.coast, .b = BridgeState.forward }, // Step 2: 0, +B
    .{ .a = BridgeState.reverse, .b = BridgeState.forward }, // Step 3: -A, +B
    .{ .a = BridgeState.reverse, .b = BridgeState.coast }, // Step 4: -A, 0
    .{ .a = BridgeState.reverse, .b = BridgeState.reverse }, // Step 5: -A, -B
    .{ .a = BridgeState.coast, .b = BridgeState.reverse }, // Step 6: 0, -B
    .{ .a = BridgeState.forward, .b = BridgeState.reverse }, // Step 7: +A, -B
};

/// Lookup state for a given step index and stepping mode.
pub fn getStepState(index: usize, mode: StepMode) DualBridgeState {
    return switch (mode) {
        .full_step => full_step_table[index % 4],
        .wave_drive => wave_drive_table[index % 4],
        .half_step => half_step_table[index % 8],
    };
}

// ---------------------------------------------------------------------------
// Hardware Driver (freestanding)
// ---------------------------------------------------------------------------

/// Generic DRV8833 driver bound to specific GPIO control pins.
pub fn Driver(
    comptime ain1_pin: gpio.Pin,
    comptime ain2_pin: gpio.Pin,
    comptime bin1_pin: gpio.Pin,
    comptime bin2_pin: gpio.Pin,
    comptime nsleep_pin: ?gpio.Pin,
) type {
    return struct {
        const Self = @This();

        step_index: usize = 0,

        pub fn init() Self {
            // `initOutput` leaves the pad low, which is the coast state for a
            // bridge input — the four coils are quiet until `applyState`.
            gpio.initOutput(ain1_pin);
            gpio.initOutput(ain2_pin);
            gpio.initOutput(bin1_pin);
            gpio.initOutput(bin2_pin);

            if (nsleep_pin) |sleep_pin| {
                gpio.initOutput(sleep_pin);
                gpio.put(sleep_pin, true); // Wake device by default
            }

            return .{};
        }

        pub fn enable(self: *Self) void {
            _ = self;
            if (nsleep_pin) |sleep_pin| {
                gpio.put(sleep_pin, true);
            }
        }

        pub fn sleep(self: *Self) void {
            if (nsleep_pin) |sleep_pin| {
                gpio.put(sleep_pin, false);
            }
            self.coastAll();
        }

        /// Apply a dual bridge state directly to hardware pins.
        pub fn applyState(self: *Self, state: DualBridgeState) void {
            _ = self;
            gpio.put(ain1_pin, state.a.in1);
            gpio.put(ain2_pin, state.a.in2);
            gpio.put(bin1_pin, state.b.in1);
            gpio.put(bin2_pin, state.b.in2);
        }

        /// Set H-Bridge outputs to Hi-Z (Coast).
        pub fn coastAll(self: *Self) void {
            self.applyState(.{ .a = BridgeState.coast, .b = BridgeState.coast });
        }

        /// Set H-Bridge outputs to Brake (Slow Decay).
        pub fn brakeAll(self: *Self) void {
            self.applyState(.{ .a = BridgeState.brake, .b = BridgeState.brake });
        }

        /// Take a single step with a 4-wire bipolar stepper motor.
        pub fn step(self: *Self, dir: Direction, mode: StepMode) void {
            const table_len: usize = if (mode == .half_step) 8 else 4;

            if (dir == .forward) {
                self.step_index = (self.step_index + 1) % table_len;
            } else {
                self.step_index = if (self.step_index == 0) table_len - 1 else self.step_index - 1;
            }

            const state = getStepState(self.step_index, mode);
            self.applyState(state);
        }

        /// Perform `count` steps with a configurable microsecond delay between steps.
        pub fn stepN(self: *Self, count: u32, dir: Direction, mode: StepMode, step_delay_us: u32) void {
            var i: u32 = 0;
            while (i < count) : (i += 1) {
                self.step(dir, mode);
                timer.delayUs(step_delay_us);
            }
        }
    };
}

// ---------------------------------------------------------------------------
// Host Unit Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "full step sequence step table" {
    const s0 = getStepState(0, .full_step);
    try testing.expectEqual(true, s0.a.in1);
    try testing.expectEqual(false, s0.a.in2);
    try testing.expectEqual(true, s0.b.in1);
    try testing.expectEqual(false, s0.b.in2);

    const s2 = getStepState(2, .full_step);
    try testing.expectEqual(false, s2.a.in1);
    try testing.expectEqual(true, s2.a.in2);
    try testing.expectEqual(false, s2.b.in1);
    try testing.expectEqual(true, s2.b.in2);
}

test "half step sequence table wrapping" {
    const s0 = getStepState(0, .half_step);
    const s8 = getStepState(8, .half_step);

    try testing.expectEqual(s0.a.in1, s8.a.in1);
    try testing.expectEqual(s0.a.in2, s8.a.in2);
    try testing.expectEqual(s0.b.in1, s8.b.in1);
    try testing.expectEqual(s0.b.in2, s8.b.in2);
}

test "wave drive step state table" {
    const s1 = getStepState(1, .wave_drive);
    try testing.expectEqual(false, s1.a.in1);
    try testing.expectEqual(false, s1.a.in2);
    try testing.expectEqual(true, s1.b.in1);
    try testing.expectEqual(false, s1.b.in2);
}
