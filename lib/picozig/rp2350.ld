/*
 * RP2350 (Raspberry Pi Pico 2) flash image layout.
 *
 * Deliberately minimal — this is not a general-purpose linker script. It is
 * the smallest layout that produces an image the RP2350 bootrom will accept:
 *
 *   1. The Cortex-M33 vector table at the very base of the image, so the
 *      bootrom's default entry mechanism (initial SP at +0, reset handler
 *      at +4) finds it without needing an ENTRY_POINT item in the IMAGE_DEF.
 *   2. The IMAGE_DEF "picobin" block immediately after it, comfortably inside
 *      the first 4 KB of flash where the bootrom scans for it.
 *
 * No second-stage bootloader: unlike RP2040, the RP2350 bootrom sets up XIP
 * itself and will happily execute from flash using its default (slow, 03h
 * serial read) mode. That is entirely adequate here — we run the CPU off the
 * 12 MHz crystal with no PLL, so flash bandwidth is not the constraint. A
 * boot2 can be added later purely as a speed optimisation if ever needed.
 *
 * Memory map per RP2350 datasheet §2.2 (address map). The 520 KB of SRAM is
 * contiguous: 512 KB of striped banks followed by SCRATCH_X and SCRATCH_Y.
 */

ENTRY(_reset)

MEMORY
{
    /* Pimoroni Pico LiPo 2 (PIM775) carries 16 MB of QSPI flash. Its 8 MB of
     * PSRAM is deliberately absent from this map: it needs XIP chip-select
     * setup we do not do, and nothing here needs it yet. */
    FLASH (rx)  : ORIGIN = 0x10000000, LENGTH = 16M
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 520K
}

SECTIONS
{
    .text :
    {
        /* Order matters: vector table first (it IS the image base), then the
         * boot metadata block. KEEP() on both — nothing references them, so
         * --gc-sections would otherwise discard the entire bootable image. */
        KEEP(*(.vectors))
        KEEP(*(.picobin_block))

        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* Bring-up self-test results, pinned to the very base of RAM (0x20000000)
     * so a debugger can read them without needing symbols:
     *     openocd ... -c "init" -c "halt" -c "mdw 0x20000000 8"
     * NOLOAD and outside .bss, so _reset's zeroing does not touch it and the
     * block survives a warm reset. See pico/main.zig's SelfTest. */
    .selftest ORIGIN(RAM) (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.selftest))
    } > RAM

    /* Where boot.zig records a fault: which vector, the stacked PC, and the
     * SCB's own account of the cause. Pinned at a fixed offset above .selftest
     * rather than merely following it, so that
     *     mdw 0x20000080 8
     * is the same command for every image in this workspace regardless of how
     * big that image's own block is. Also NOLOAD, and for the same reason —
     * the fault worth reading is usually the one you had to reset to ask
     * about. */
    .faultinfo (ORIGIN(RAM) + 0x80) (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.faultinfo))
    } > RAM

    /* Code that must execute from RAM rather than from flash.
     *
     * Required for flash programming: the bootrom's erase/program sequence
     * begins with flash_exit_xip(), after which flash is NOT executable, so the
     * routine driving that sequence cannot itself live in flash. Loaded from
     * flash and copied into RAM by _reset alongside .data.
     *
     * ALIGN(4) on both sides: the copy loop moves words. */
    .ramfunc :
    {
        . = ALIGN(4);
        __ramfunc_start = .;
        *(.ramfunc)
        *(.ramfunc.*)
        . = ALIGN(4);
        __ramfunc_end = .;
    } > RAM AT > FLASH

    __ramfunc_load = LOADADDR(.ramfunc);

    /* Initialised globals live in RAM but are loaded from flash; _reset copies
     * them across before any Zig code runs. */
    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data*)

        /* Claimed rather than left to orphan placement. Linking a non-PIC
         * archive, or using thread-locals without -ftls-model=local-exec,
         * makes the linker emit dynamic data relocations through a GOT. With
         * no rule for it the section becomes an orphan, and a writable orphan
         * lands here in RAM with no AT > FLASH mapping behind it: allocated,
         * never copied by _reset, silently zero.
         *
         * The ASSERT below is the real point. These are expected to be empty
         * — no picozig example currently emits either — so if one ever is
         * not, the link should fail rather than the image quietly misbehave.
         *
         * pico-sdk 2.3.1 made the same change for the same reason; its
         * rationale is in pico_standard_link/script_include/
         * section_default_data.incl. */
        *(.got)
        *(.got.*)

        . = ALIGN(4);
        __data_end = .;
    } > RAM AT > FLASH

    __data_load = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    /* Full descending stack from the top of SRAM. */
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);

    ASSERT(__bss_end < __stack_top, "RAM overflow: .bss collides with the stack")
    ASSERT(SIZEOF(.selftest) <= 0x80, "self-test block has grown into .faultinfo at ORIGIN(RAM) + 0x80")
    ASSERT(SIZEOF(.got) == 0, "non-empty .got: something wants dynamic data relocations. See the note in .data")

    /* Unwind tables are meaningless with no C++ / no panic unwinding. */
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}
