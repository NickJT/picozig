//! dac-output — MCP4725 12-Bit DAC example for Raspberry Pi Pico 2.
//!
//! Generates a stepped analog voltage wave on VOUT over I2C0 (GP4/GP5),
//! prints progress to UART0 (115200 8N1), and publishes a RAM block
//! at 0x20000000 for SWD monitoring.

const rp2350 = @import("rp2350");
const board = @import("board");
const boot = @import("boot");
const mcp4725 = @import("mcp4725");

const clocks = rp2350.clocks;
const gpio = rp2350.gpio;
const i2c = rp2350.i2c;
const timer = rp2350.timer;
const uart = rp2350.uart;

const Watch = extern struct {
    magic: u32,
    ticks: u32,
    err_code: u32,
    raw_val: u32,
    millivolts: u32,
    is_ready: u32,
    _pad1: u32,
    _pad2: u32,
    _pad3: u32,
};

pub const watch_magic: u32 = 0x4725_0000;

export var watch: Watch linksection(".selftest") = undefined;

comptime {
    _ = &boot;
}

pub fn main() noreturn {
    clocks.init(board.clock_plan);
    gpio.init();
    uart.init(.uart0, board.uartConfig());

    uart.print(.uart0, "\n=== MCP4725 dac-output example ===\nboard {s}\n", .{board.name});

    watch = .{
        .magic = watch_magic,
        .ticks = 0,
        .err_code = 0,
        .raw_val = 0,
        .millivolts = 0,
        .is_ready = 1,
        ._pad1 = 0,
        ._pad2 = 0,
        ._pad3 = 0,
    };

    // GP4 (SDA) / GP5 (SCL) on i2c0 (Qwiic / STEMMA QT port)
    const sda_pin: rp2350.Pin = 4;
    const scl_pin: rp2350.Pin = 5;

    i2c.init(.i2c0, .{
        .sda = sda_pin,
        .scl = scl_pin,
        .speed = .fast_400k,
        .clk_hz = board.sys_hz,
    });

    const bus = mcp4725.PicoI2c{ .inst = .i2c0 };
    var dac = mcp4725.Driver(mcp4725.PicoI2c).init(bus, mcp4725.i2c_address_default_a0_low);

    const vdd: f32 = 3.3;

    // Verify DAC responds by reading status
    if (dac.readStatus()) |st| {
        uart.print(.uart0, "MCP4725 connected! Initial DAC raw: {d}, EEPROM raw: {d}\n", .{
            st.dac_value,
            st.eeprom_dac_value,
        });
    } else |err| {
        uart.print(.uart0, "WARNING: Initial read failed ({s}); continuing...\n", .{@errorName(err)});
    }

    const steps = [_]f32{ 0.0, 0.825, 1.65, 2.475, 3.3, 2.475, 1.65, 0.825 };
    var step_idx: usize = 0;
    var count: u32 = 0;

    while (true) {
        timer.delayMs(250);
        count += 1;

        const target_v = steps[step_idx];
        step_idx = (step_idx + 1) % steps.len;

        const raw = mcp4725.voltageToRaw(target_v, vdd);

        if (dac.setRawFast(raw, .normal)) |_| {
            const mv: u32 = @intFromFloat(target_v * 1000.0);
            watch.ticks = count;
            watch.raw_val = raw;
            watch.millivolts = mv;
            watch.err_code = 0;

            if (dac.isReady()) |rdy| {
                watch.is_ready = if (rdy) 1 else 0;
            } else |_| {}

            uart.print(.uart0, "[DAC Output] Target: {d:.3}V (raw: {d}/4095) [tick {d}]\n", .{
                target_v,
                raw,
                count,
            });
        } else |err| {
            uart.print(.uart0, "[DAC Output] Write error: {s}\n", .{@errorName(err)});
            watch.magic = 0xdead_4725;
            watch.err_code = @intFromError(err);
        }
    }
}
