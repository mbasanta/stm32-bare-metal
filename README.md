# STM32F103 Bare-Metal Firmware

Minimal bare-metal firmware for STM32F103 (Cortex-M3) microcontroller using direct register manipulation via CMSIS headers. Based on the architecture from [cpq/bare-metal-programming-guide](https://github.com/cpq/bare-metal-programming-guide).

## Hardware Target

- **MCU:** STM32F103xB (Blue Pill compatible)
- **Flash:** 128KB @ 0x08000000
- **SRAM:** 20KB @ 0x20000000
- **Clock:** 8MHz HSI (internal oscillator)
- **Core:** ARM Cortex-M3, Thumb instruction set

## Features

- No HAL/vendor frameworks - direct CMSIS register access
- GPIO control with custom pin encoding (`PIN('A', 5)`)
- UART communication (USART1-3) with `printf()` support via syscalls
- SysTick-based timer utilities for non-blocking delays
- Minimal startup code with proper `.data`/`.bss` initialization
- Link-time optimization to remove unused code

## Quick Start

### Prerequisites

Install required toolchain:
```bash
sudo apt install gcc-arm-none-eabi make stlink-tools
```

### Build and Flash

```bash
make build          # Compile firmware (debug build with -O0 -g3)
make flash          # Flash via st-flash (requires ST-Link)
make clean          # Remove build artifacts

BUILD=release make  # Optimized build (-Os)
```

### Example Application

The default [main.c](main.c) blinks the LED on PA5 every 500ms and outputs status via UART2:

```c
int main(void) {
    uart_init(UART2, 115200);
    uint16_t pin = PIN('A', 5);  // PA5 (Blue Pill LED)
    gpio_set_mode(pin, GPIO_Output_PushPull, GPIO_Speed_10MHz);

    uint32_t timer = 0;
    for (;;) {
        if (timer_expired(&timer, 500, s_ticks)) {
            static bool on;
            printf("LED: %d, tick: %lu\r\n", on, s_ticks);
            gpio_write(pin, on);
            on = !on;
        }
    }
}
```

Connect UART2 (PA2=TX, PA3=RX) at 115200 baud to view serial output.

## Project Structure

```
├── main.c                  # Application code
├── syscalls.c              # Newlib syscalls (redirects printf to UART2)
├── include/mcu.h           # Hardware abstraction (GPIO, UART, timers)
├── link.ld                 # Linker script (memory layout)
├── Makefile                # Build configuration
├── openocd.cfg            # OpenOCD debug configuration
├── cmsis_core/            # ARM CMSIS-Core v5.9.0 (auto-cloned)
└── cmsis_f1/              # ST device headers v4.3.5 (auto-cloned)
```

## Hardware Abstraction

All peripheral functions are in [include/mcu.h](include/mcu.h) using CMSIS register definitions:

### GPIO

```c
// Custom pin encoding: PIN('A', 5) → 0x0005
uint16_t led = PIN('C', 13);  
gpio_set_mode(led, GPIO_Output_PushPull, GPIO_Speed_10MHz);
gpio_write(led, true);  // Set high
gpio_write(led, false); // Set low
```

**GPIO Modes:** `GPIO_Input_Analog`, `GPIO_Input_Floating`, `GPIO_Input_PullUpDown`, `GPIO_Output_PushPull`, `GPIO_Output_OpenDrain`, `GPIO_Output_AltPushPull`, `GPIO_Output_AltOpenDrain`

**Speeds:** `GPIO_Speed_2MHz`, `GPIO_Speed_10MHz`, `GPIO_Speed_50MHz`

**Note:** STM32F1 uses CRL/CRH configuration registers (4 bits per pin), unlike newer families.

### UART

```c
uart_init(UART2, 115200);           // Initialize USART2
uart_write_byte(UART2, 'A');        // Send single byte
uart_write_buffer(UART2, buf, len); // Send buffer

if (uart_read_ready(UART2)) {
    uint8_t byte = uart_read_byte(UART2);
}
```

**Available UARTs:**
- UART1: PA9 (TX), PA10 (RX) - APB2
- UART2: PA2 (TX), PA3 (RX) - APB1
- UART3: PB10 (TX), PB11 (RX) - APB1

**Printf support:** `printf()` is redirected to UART2 via [syscalls.c](syscalls.c) - outputs to `stdout`/`stderr`.

### Timers

Non-blocking timer using SysTick (configured for 1ms ticks in `SystemInit()`):

```c
static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

uint32_t timer = 0;
if (timer_expired(&timer, 1000, s_ticks)) {
    // Runs every 1000ms without blocking
}
```

## Memory Layout

Defined in [link.ld](link.ld):

| Region | Start      | Size  | Contents |
|--------|------------|-------|----------|
| Flash  | 0x08000000 | 128KB | `.vectors` (ISR table), `.text` (code), `.rodata` (constants) |
| SRAM   | 0x20000000 | 20KB  | `.data` (initialized), `.bss` (zero-init), stack (grows down from 0x20005000) |

## Build System

### Makefile Targets

- `make build` - Compile to `firmware.elf` (default: debug mode)
- `make flash` - Flash `firmware.bin` via `st-flash` to 0x08000000
- `make clean` - Remove build artifacts
- `BUILD=release make` - Optimized build with `-Os`

### Compiler Flags

- **Strict warnings:** `-Werror -Wextra -Wshadow -Wconversion` (all warnings → errors)
- **Optimization:** `-O0 -g3` (debug) or `-Os` (release)
- **Dead code elimination:** `-ffunction-sections -fdata-sections` + `--gc-sections`
- **Cortex-M3:** `-mthumb -mcpu=cortex-m3`
- **Minimal libc:** `--specs nano.specs -lc -lgcc` (newlib-nano)

### Dependencies

First build auto-clones CMSIS dependencies:
- `cmsis_core/` - ARM CMSIS-Core v5.9.0 (`core_cm3.h`, intrinsics)
- `cmsis_f1/` - ST device headers v4.3.5 (`stm32f103xb.h`, `startup_stm32f103xb.s`)

## Debugging

### VS Code + OpenOCD

Debug configuration in [.vscode/launch.json](.vscode/launch.json):
1. Install Cortex-Debug extension
2. Connect ST-Link programmer via SWD
3. Press F5 to build and launch debugger

**Requirements:**
- `openocd` with ST-Link support
- `gdb-multiarch` or `arm-none-eabi-gdb`
- ST-Link V2 hardware connected (SWDIO, SWCLK, GND, 3.3V)

**Config:** [openocd.cfg](openocd.cfg) uses `stlink.cfg` interface + `stm32f1x.cfg` target.

### Serial Debugging

UART2 output at 115200 baud (PA2=TX, PA3=RX):
```c
printf("Debug: value=%lu\r\n", value);
```
Use USB-to-serial adapter or ST-Link's virtual COM port.

## Startup Sequence

1. **Reset:** `startup_stm32f103xb.s` sets stack pointer to `_estack` (0x20005000)
2. **Init:** Copies `.data` from flash to SRAM, zeros `.bss` section
3. **SystemInit():** Configures SysTick for 1ms interrupts (in [main.c](main.c))
4. **main():** Application code starts

## Adding Features

### New Peripheral

Add inline functions to [include/mcu.h](include/mcu.h):
```c
static inline void spi_init(SPI_TypeDef* spi) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable clock first!
    // Configure SPI registers...
}
```

Use CMSIS register structs (`SPI_TypeDef`, `TIM_TypeDef`, etc.) - never raw pointers.

### New Source File

1. Add to `SOURCES` in [Makefile](Makefile)
2. Include `mcu.h` for peripheral access
3. Update interrupt vector table if using ISRs (in `cmsis_f1/Source/Templates/gcc/startup_stm32f103xb.s`)

## STM32F1-Specific Notes

1. **GPIO configuration:** Uses CRL/CRH registers (4 bits per pin), not MODER/OTYPER like newer STM32 families
2. **Available GPIO banks:** A-E only (no GPIOF on STM32F103xB)
3. **USART naming:** Peripherals called USART1-3, but code uses `UART` macros for clarity
4. **No PLL configured:** Running at 8MHz HSI default clock (see `FREQ_HZ` in [mcu.h](include/mcu.h))

## License

See original guide: [cpq/bare-metal-programming-guide](https://github.com/cpq/bare-metal-programming-guide)
