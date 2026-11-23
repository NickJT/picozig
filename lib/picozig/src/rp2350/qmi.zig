//! RP2350 QMI — Quad Memory Interface.
//!
//! Controls the QSPI flash and PSRAM memory bus. On RP2350, QMI replaces
//! RP2040's SSI peripheral.
//!
//! Out of reset, the Bootrom leaves QMI in 1-bit SPI mode. `initQuad()`
//! configures QMI and PADS_QSPI for 4-bit Quad-IO mode (0xEB Fast Read)
//! matching the vendor pico-sdk CRT0 startup sequence.

const hw = @import("hw.zig");
const bounded = @import("bounded.zig");

/// Times `initQuad` gave up waiting for direct mode to settle.
///
/// **Lives here rather than being returned** because `initQuad` runs from RAM
/// before there is a console, a timer, or a caller that could act on an error.
/// Non-zero means the flash interface was configured against a controller that
/// had not finished — read it over SWD when a board boots strangely.
pub var direct_mode_timeouts: u32 = 0;

pub const pads_qspi_base: u32 = 0x4004_3000;
pub const qmi_base: u32 = 0x400d_0000;
pub const xip_direct_base: u32 = 0x1400_0000;

pub const PadsQspi = struct {
    /// 0x40040004 is GPIO_QSPI_SCLK pad control
    pub const pad_sclk: u32 = 0x4004_0004;
    pub const pad_sd0: u32 = pads_qspi_base + 0x08;
    pub const pad_sd1: u32 = pads_qspi_base + 0x0c;
    pub const pad_sd2: u32 = pads_qspi_base + 0x10;
    pub const pad_sd3: u32 = pads_qspi_base + 0x14;

    /// 4 mA drive strength
    pub const drive_4ma: u32 = 2;
    /// SCLK pad setup matching pico-sdk CRT0
    pub const sclk_setup: u32 = 0x21;
};

/// **Names corrected 2026-08-14.** `m0_rcmd` and `m0_rfmt` were transposed here:
/// `m0_rcmd` named 0x10 (really `M0_RFMT`) and `m0_rfmt` named 0x14 (really
/// `M0_RCMD`). `initQuad` happened to behave correctly regardless, because the
/// values had been matched to the wrong names — 0xa0eb went to the constant
/// called `m0_rfmt`, which was really RCMD, and so on. Two errors cancelling.
///
/// Harmless where it sat, a trap for anything new: code reading `Regs.m0_rcmd`
/// and expecting RCMD got RFMT. Offsets below are the datasheet's (§12.14,
/// "List of Registers"), and `initQuad` has been updated to use them.
///
/// Note 0x00 is `DIRECT_CSR`, not an "M0_CTRL" — there is no such register.
pub const Regs = struct {
    /// Direct-mode control and status. Bit 0 `EN`, bit 1 `BUSY`.
    pub const direct_csr: u32 = qmi_base + 0x00;
    pub const direct_tx: u32 = qmi_base + 0x04;
    pub const direct_rx: u32 = qmi_base + 0x08;
    pub const m0_timing: u32 = qmi_base + 0x0c;
    /// Read *format*: phase widths and lengths. Bit 12 is `PREFIX_LEN`
    /// (0 = no prefix, 1 = 8-bit).
    pub const m0_rfmt: u32 = qmi_base + 0x10;
    /// Read *command*: `PREFIX` in bits 7:0, `SUFFIX` in 15:8.
    pub const m0_rcmd: u32 = qmi_base + 0x14;

    /// Deprecated alias for `direct_csr`, kept so the old name does not
    /// silently resolve to something else if it survives anywhere.
    pub const m0_ctrl: u32 = direct_csr;
};

/// Initialize QMI into 4-bit Quad-IO mode (0xEB command).
///
/// Runs from RAM (.ramfunc) so it does not touch XIP Flash while reconfiguring
/// the QMI controller and QSPI pad drive strengths.
pub fn initQuad() linksection(".ramfunc") callconv(.c) void {
    // 1. Configure PADS_QSPI SCLK & SD0..SD3 drive strengths
    hw.write(PadsQspi.pad_sclk, PadsQspi.sclk_setup);
    hw.write(PadsQspi.pad_sd0, PadsQspi.drive_4ma);
    hw.write(PadsQspi.pad_sd1, PadsQspi.drive_4ma);
    hw.write(PadsQspi.pad_sd2, PadsQspi.drive_4ma);
    hw.write(PadsQspi.pad_sd3, PadsQspi.drive_4ma);

    // 2. DIRECT_CSR: enter direct mode (0x07800041 — EN plus timing fields)
    const direct_csr_val: u32 = 0x0780_0041;
    hw.write(Regs.direct_csr, direct_csr_val);

    // Wait for BUSY (bit 1) to clear before relying on direct mode.
    //
    // **Bounded, and it gives up rather than propagating.** This runs from RAM
    // during early boot: there is no console, no timer and nobody to return an
    // error to. Spinning forever here is a board that never reaches the point
    // where it could report anything, so on timeout it counts and carries on —
    // read `direct_mode_timeouts` over SWD, alongside the fault block
    // `boot.zig` writes at 0x20000080.
    var s = bounded.Spin.init(bounded.startup_polls);
    while (hw.read(Regs.direct_csr) & 2 != 0) {
        s.tick() catch {
            direct_mode_timeouts +%= 1;
            break;
        };
    }

    // 3. Leave direct mode: clear EN (bit 0).
    hw.write(Regs.direct_csr, direct_csr_val & ~@as(u32, 1));

    // 4. M0_TIMING (0x40000202)
    hw.write(Regs.m0_timing, 0x4000_0202);

    // 5. M0_RCMD — prefix 0xEB (Quad-IO fast read), suffix 0xA0 (the mode bits
    //    that put the flash into continuous read).
    hw.write(Regs.m0_rcmd, 0x0000_a0eb);

    // 6. M0_RFMT — quad address and data, with the 8-bit command prefix still
    //    enabled (bit 12 `PREFIX_LEN` = 1).
    const rfmt_val: u32 = 0x0004_92a8;
    hw.write(Regs.m0_rfmt, rfmt_val);

    // 7. One dummy read through the direct XIP window, which is what actually
    //    latches the flash into continuous-read mode.
    const xip_ptr: [*]volatile u8 = @ptrFromInt(xip_direct_base);
    _ = xip_ptr[0];

    // 8. Clear `PREFIX_LEN` (bit 12) in M0_RFMT: from here reads send no
    //    command byte at all, relying on the 0xA0 mode bits latched above.
    //
    //    **This is the state that does not survive a warm reset on its own.**
    //    The flash die keeps continuous-read mode across an RP2350 reset — the
    //    datasheet is explicit that "resetting RP2350 does not reset attached
    //    QSPI devices" — and the bootrom's XIP exit sequence is what breaks it
    //    back out on the next boot. Legitimate, and what the SDK's CRT0 does,
    //    but it is why this routine must run from RAM.
    hw.write(Regs.m0_rfmt, rfmt_val & ~@as(u32, 0x1000));
}
