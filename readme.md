# picozig

picozig is a small, easy to use Zig library for **RP2350** Raspberry Pi Pico boards. Written entirely in Zig with few external dependencies, it aims to be distinctively good at supporting one board family and disciplined in declining everything else.

## Why you may want to leave now
- picozig is not and will never be a general HAL for many different boards and processors. Other, cleverer people do that better than I ever will. 
- The code in picozig has been written with assistance from one or more AI models. If you don't approve then I respect your views and wish you well, but there are better places to discuss the costs and benefits of AI.

## Why Zig?
### Fails the build not the boot
Zig moves a class of embedded failure from run time to build time and reduces the chance of a chip booting and then silently failing, or even failing to boot at all.  
  - Comptime code gets validated against datasheet constraints at a time when the compiler can provide meaningful error messages. Fundamentals like the vector table (boot.zig) get checked early so errors like misspelling "usbctrl" fail to match against nvic.IrqNum and generate a compiler error not silent, unexpected run time behaviour. Pin allocations, pin functions and clock settings can all be comptime checked, so instead of an unresponsive chip we get:
      
      ```
      error: RP2350-E12: the USB controller needs clk_sys > 48000000 Hz, but this
      board runs at 12000000 Hz. Declare a clock_plan of at least 50 MHz before
      enabling USB.
      ```
      
  - **Struct layout** You need a 9 byte struct with a u16 at a given offset. Pad it wrong and that descriptor's fields are right but every descriptor block concatenated afterwards is wrong. Using `packed struct` with defined bit width integers and @bitSizeOf assertions at comptime avoids having to byte count a 67-byte config blob after every edit.

### Keeps the repo honest
A key ingredient for ease of use is keeping the repo structure clean. "The chip layer must not know about boards" is an easy to ignore/forget doc comment. Import rules mean layering shortcuts become compiler errors. `rp2350/` contains no board file, and board is a separate module. The compiler rejects any attempt by the chip layer to import a board file, rather than it being a rule someone has to remember.

### Turtles all the way down
- One language, one toolchain - not two (or three...). 
  - The going-in objective for picozig was that the only thing below Zig would be the datasheet. The biggest win from this was probably the PIO assembler - written in Zig with all the benefits of a high level language but no intermediate files and no external dependency on pioasm. 
  
  - Cross-compilation is a parameter, not a toolchain. Because the chip layer is pure address arithmetic, one build.zig produces both the host tests that run on a Mac, and the thumb-freestanding image from the same source. One test runner, no heavyweight mocking framework, no pre-processor macros, no second build system.  

### But not foolproof

It's still easy to get bitten by lazy analysis if a module gets imported without referencing it. Feeling very clever at writing so much flawless code - right up to the point you find there's no image.  

## Decisions taken (and not always lightly)

| Decision | Choice | Why |
|---|---|---|
| Chip scope | **RP2350 only.** RP2350A or RP2350B depending on the board| No RP2040 means no comptime chip-selection layer, no dual register maps, no boot2|
| Board scope | **Four supported boards**: Raspberry Pi **Pico 2 and Pico 2 W**, Pimoroni **Pico LiPo 2 and Pico Plus 2 W**. | Each is a small *board package* (`lib/pico2`, `lib/pico2w`, `lib/pico-lipo2`, `lib/pico-plus2w`) exporting a `board` module that the application chooses, never a fork — which is what keeps the chip/board split clean. |
| Wireless | **CYW43** support via the official Raspberry Pi blob | The CYW43 needs a firmware blob uploaded over SPI and this is **one external dependency that couldn't be avoided**. Without this there would be no wifi or bluetooth and that would have been one limitation too far.|
| Layering | **Chip** (`rp2350`, in picozig) and **board** (a board package such as `pico-lipo2`).| Board depends on chip and **never the reverse** - compiler enforced, since picozig cannot even see a board package |
| Packaging | **Simplicity** over ambition| Build in place, extract into a library when there is a second consumer. i) To avoid adopting the first use case as structure and ii) because too many projects run out of steam before the self-inflicted scaffolding is finished.|
| boot2 | **Excluded** (rather than just not implemented)| The RP2350 bootrom configures XIP itself and we are not planning to support the RP2040|
| Attacks | This is **not** a hardened library | If you need threat protection in your firmware you know far more about microcontroller coding than I do|
| Restart robustness | **Never brick a board**| Avoid presenting functions that let an unwary user modify anything that will break the BOOTSEL reset. |

---
