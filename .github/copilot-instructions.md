# STM32F103 Bare-Metal Programming Guide

## Project Overview
This is a minimal bare-metal firmware project for STM32F103 (Cortex-M3) microcontroller following the architecture from [cpq/bare-metal-programming-guide](https://github.com/cpq/bare-metal-programming-guide). No HAL/vendor frameworks—direct register manipulation via CMSIS headers.

**Developement Board:** NUCLEO-F103RB
**Target:** STM32F103xB (128KB flash, 20KB SRAM, 8MHz HSI clock)  
**Architecture:** ARM Cortex-M3, Thumb instruction set  
**Toolchain:** `arm-none-eabi-gcc` with newlib-nano

## Core Architecture

### Memory Layout (link.ld)
- **Flash:** `0x08000000` (128KB) - interrupt vectors, code (.text), constants (.rodata)
- **SRAM:** `0x20000000` (20KB) - data (.data), uninitialized (.bss), stack (grows down from end)
- Stack pointer (`_estack`) is at end of SRAM (`0x20005000`)

### Startup Sequence
1. `startup_stm32f103xb.s` (CMSIS startup) runs first: initializes vector table, copies `.data` from flash to SRAM, zeros `.bss`
2. Calls `SystemInit()` in [main.c](main.c) - configures SysTick for 1ms interrupts
3. Jumps to `main()`

### Hardware Abstraction
All peripheral manipulation is in [include/mcu.h](include/mcu.h) using CMSIS register definitions from `stm32f103xb.h`:

- **GPIO:** Custom `PIN('A', 5)` macro encodes bank+pin as `0xBBNN`. Configure via direct CRL/CRH register writes (STM32F1 uses separate config registers for pins 0-7 and 8-15)
- **UART:** Inline functions directly set BRR (baud rate register), CR1 (control), DR (data register). UART2 on PA2/PA3 is hardwired for `printf()` via syscalls
- **Timers:** Software-only `timer_expired()` compares tick counts, handles overflow

## Build System

### Standard Workflow
```bash
make build          # Compile to firmware.elf (default: debug build with -O0 -g3)
make flash          # Flash via st-flash (ST-Link required)
make clean          # Remove build artifacts
BUILD=release make  # Optimized build (-Os)
```

### Compiler Flags (Makefile)
- **Strict warnings:** `-Werror -Wundef -Wshadow -Wconversion` etc. - all warnings are errors
- **Link-time optimization:** `-ffunction-sections -fdata-sections` + `--gc-sections` removes unused code
- **Thumb mode required:** `-mthumb -mcpu=cortex-m3`
- **No stdlib:** `-nostartfiles -nostdlib` but links newlib-nano via `--specs nano.specs -lc -lgcc`

### CMSIS Dependencies
First build auto-clones:
- `cmsis_core/` - ARM's CMSIS-Core v5.9.0 (core_cm3.h, intrinsics)
- `cmsis_f1/` - ST's device headers v4.3.5 (stm32f103xb.h, startup file)

## Debugging

### VS Code Integration
- **Launch config:** "Debug (OpenOCD)" in [.vscode/launch.json](.vscode/launch.json) auto-builds and launches GDB via OpenOCD
- **Requires:** Cortex-Debug extension, `gdb-multiarch`, OpenOCD with ST-Link interface
- **Config:** [openocd.cfg](openocd.cfg) specifies `stlink.cfg` (SWD transport) + `stm32f1x.cfg` target

### Hardware Requirements
- ST-Link V2 programmer connected via SWD (SWDIO/SWCLK)
- Flash via `st-flash --reset write firmware.bin 0x08000000`

## Code Conventions

### GPIO Pin Handling
Always use `PIN()` macro - don't hardcode hex values:
```c
uint16_t led = PIN('A', 5);  // Not: 0x0005
gpio_set_mode(led, GPIO_Output_PushPull, GPIO_Speed_10MHz);
gpio_write(led, true);
```

### Peripheral Initialization Pattern
1. Enable RCC clock gate first (e.g., `RCC->APB2ENR |= RCC_APB2ENR_IOPAEN`)
2. Configure pins via `gpio_set_mode()`
3. Set peripheral registers (BRR, CR1, etc.)
4. See `uart_init()` in [mcu.h](include/mcu.h) for reference

### Standard I/O Integration
[syscalls.c](syscalls.c) redirects newlib's `_write()` to UART2, enabling `printf()` over serial:
```c
printf("Debug: %lu\r\n", value);  // Output on UART2 @ 115200 baud
```
Use `\r\n` for line endings (bare metal convention).

### Timer Usage
Use `timer_expired()` with SysTick counter for non-blocking delays:
```c
uint32_t timer = 0, period_ms = 500;
if (timer_expired(&timer, period_ms, s_ticks)) {
    // Runs every 500ms without blocking
}
```

## STM32F1-Specific Gotchas

1. **GPIO config registers:** STM32F1 uses CRL/CRH (4 bits per pin) unlike newer families' MODER/OTYPER. See `gpio_set_mode()` bit-shifting logic
2. **No GPIOF on STM32F103xB:** Only banks A-E available
3. **UART vs USART:** This chip calls them USART1-3, but code uses `UART` macros for clarity
4. **8MHz HSI default:** No PLL configuration—running at internal 8MHz oscillator speed (see `FREQ_HZ` in [mcu.h](include/mcu.h))

## Adding New Features

### New Peripheral
1. Add inline functions to [mcu.h](include/mcu.h) following existing pattern (e.g., `spi_init()`)
2. Use CMSIS register structs (`SPI_TypeDef`, etc.) - never raw pointers
3. Enable clock in RCC before accessing peripheral

### New Source File
1. Add to `SOURCES` in [Makefile](Makefile)
2. Include [mcu.h](include/mcu.h) for peripheral access
3. Ensure functions used in interrupts are marked in vector table (see startup file)

### Printf/Debugging
Serial output is via UART2 (PA2 TX, PA3 RX) at 115200 baud. Connect USB-to-serial adapter to view `printf()` output during development.
