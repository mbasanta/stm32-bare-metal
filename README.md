# STM32 Bare-Metal Firmware

Minimal bare-metal firmware for STM32 microcontrollers using direct register manipulation via CMSIS headers. Supports multiple targets from a single codebase. Based on architecture from [cpq/bare-metal-programming-guide](https://github.com/cpq/bare-metal-programming-guide).

## Supported Hardware

### Blue Pill - STM32F103
```bash
make build TARGET=f103
```
- **MCU:** STM32F103C8T6/RBT6 (Cortex-M3)
- **Clock:** 72MHz (HSE 8MHz)
- **Flash:** 128KB
- **SRAM:** 20KB
- **LED:** PC13 (active-low)

### Black Pill - STM32F411
```bash
make build TARGET=f411
```
- **MCU:** STM32F411CEU6 (Cortex-M4F with FPU)
- **Clock:** 96MHz (HSE 25MHz)
- **Flash:** 512KB
- **SRAM:** 128KB
- **LED:** PC13 (active-low)
- **Performance:** 33% faster clock, hardware FP, 6.4× RAM, 4× Flash vs F103

## Quick Start

### Prerequisites

```bash
sudo apt install gcc-arm-none-eabi make stlink-tools
```

### Build and Flash

```bash
# Build for Black Pill (default)
make build TARGET=f411

# Build for Blue Pill
make build TARGET=f103

# Flash to device
make flash TARGET=f103

# Clean and rebuild
make clean
make build TARGET=f411

# Release build (optimized)
BUILD=release make build TARGET=f411
```

### Example Application

The default [main.c](main.c) is target-independent - it blinks the LED and outputs status via UART2:

```c
int main(void) {
    uart_init(UART2, 115200);
    printf("Starting %s @ %lu MHz\r\n", MCU_NAME, FREQ_HZ / 1000000);
    
    gpio_set_mode(LED_PIN, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
    
    uint32_t timer = 0;
    for (;;) {
        if (timer_expired(&timer, 500, s_ticks)) {
            static bool on;
            printf("LED: %d, tick: %lu\r\n", on, s_ticks);
            gpio_write(LED_PIN, LED_ACTIVE_LOW ? !on : on);
            on = !on;
        }
    }
}
```

Connect UART2 (PA2=TX, PA3=RX) at 115200 baud to view serial output.

## Features

- **Multi-target:** Single codebase for F103 and F411
- **No HAL:** Direct CMSIS register access
- **GPIO:** Custom pin encoding (`PIN('A', 5)`)
- **UART:** Full USART support with `printf()` via syscalls
- **Timers:** SysTick-based non-blocking delays
- **Minimal:** Proper startup with `.data`/`.bss` initialization
- **Optimized:** Link-time dead code elimination
- **Debug:** VS Code integration with OpenOCD

## Project Structure

```
├── main.c                    # Target-independent application
├── syscalls.c                # Newlib syscalls (printf to UART2)
├── Makefile                  # Multi-target build system
├── link_f103.ld / link_f411.ld  # Linker scripts (memory layout)
├── openocd.cfg               # OpenOCD debug configuration
├── include/
│   ├── mcu.h                 # Main header with conditional includes
│   ├── mcu_f103.h            # F103 config (72MHz clock, pins)
│   ├── mcu_f411.h            # F411 config (96MHz clock, pins)
│   ├── gpio_f1.h             # F1 GPIO (CRL/CRH registers)
│   └── gpio_f4.h             # F4 GPIO (MODER/OTYPER/OSPEEDR)
├── .vscode/
│   ├── c_cpp_properties.json # IntelliSense for both targets
│   ├── launch.json           # Debug configs (F103/F411)
│   └── tasks.json            # Build/flash tasks
├── cmsis_core/               # ARM CMSIS-Core v5.9.0 (auto-cloned)
├── cmsis_f1/                 # ST device headers v4.3.5 (auto-cloned)
└── cmsis_f4/                 # ST device headers v2.6.10 (auto-cloned)
```

## Hardware Abstraction

All peripheral functions are in [include/mcu.h](include/mcu.h) and target-specific headers:

### GPIO

```c
// Custom pin encoding: PIN('A', 5) → 0x0005
uint16_t led = PIN('C', 13);  
gpio_set_mode(led, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
gpio_write(led, true);  // Set high
gpio_write(led, false); // Set low
```

**Common GPIO modes** (work on both targets):
- `GPIO_MODE_INPUT`, `GPIO_MODE_OUTPUT`, `GPIO_MODE_AF`, `GPIO_MODE_ANALOG`
- `GPIO_SPEED_LOW`, `GPIO_SPEED_MEDIUM`, `GPIO_SPEED_HIGH`, `GPIO_SPEED_VERY_HIGH`

**F1-specific modes** (only when `TARGET=f103`):
- `GPIO_MODE_INPUT_ANALOG`, `GPIO_MODE_INPUT_FLOATING`, `GPIO_MODE_INPUT_PULLUP/DOWN`
- `GPIO_MODE_OUTPUT_PP/OD`, `GPIO_MODE_AF_PP/OD`
- `GPIO_SPEED_2MHZ`, `GPIO_SPEED_10MHZ`, `GPIO_SPEED_50MHZ`

**F4-specific features** (only when `TARGET=f411`):
```c
gpio_set_af(PIN('A', 2), 7);  // Set alternate function (AF7 = USART2)
```

### UART

```c
uart_init(UART2, 115200);           // Initialize USART2
uart_write_byte(UART2, 'A');        // Send single byte
uart_write_buffer(UART2, buf, len); // Send buffer

if (uart_read_ready(UART2)) {
    uint8_t byte = uart_read_byte(UART2);
}

printf("Value: %lu\r\n", value);    // printf via UART2
```

**Available UARTs:**

*F103:*
- UART1: PA9 (TX), PA10 (RX) - APB2 @ 72MHz
- UART2: PA2 (TX), PA3 (RX) - APB1 @ 36MHz
- UART3: PB10 (TX), PB11 (RX) - APB1 @ 36MHz

*F411:*
- UART1: PA9 (TX), PA10 (RX) - APB2 @ 96MHz
- UART2: PA2 (TX), PA3 (RX) - APB1 @ 48MHz
- UART6: PA11 (TX), PA12 (RX) - APB2 @ 96MHz

### Timers

Non-blocking timer using SysTick (configured for 1ms ticks):

```c
static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

uint32_t timer = 0;
if (timer_expired(&timer, 1000, s_ticks)) {
    // Runs every 1000ms without blocking
}
```

## Multi-Target Architecture

### How It Works

**1. Makefile Target Selection**
```bash
make build TARGET=f103  # Cortex-M3, no FPU, cmsis_f1, link_f103.ld
make build TARGET=f411  # Cortex-M4, FPU, cmsis_f4, link_f411.ld
```

**2. Preprocessor Defines**
- Makefile passes `-DTARGET_f103` or `-DTARGET_f411`
- Headers conditionally include target-specific code

**3. Target Configs**
- `mcu_f103.h`: 72MHz clock, F103 pins, APB1/APB2 frequencies
- `mcu_f411.h`: 96MHz clock, F411 pins, APB1/APB2 frequencies

**4. GPIO Implementations**
- `gpio_f1.h`: F1 uses CRL/CRH registers (4 bits per pin)
- `gpio_f4.h`: F4 uses MODER/OTYPER/OSPEEDR (2 bits per pin)

**5. Application Code**
```c
// Target-independent - uses defines from target config
gpio_set_mode(LED_PIN, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
printf("Starting %s @ %lu MHz\r\n", MCU_NAME, FREQ_HZ / 1000000);
```

### Adding a New Target

1. Create `include/mcu_newchip.h` with clock config and pin defines
2. Create `link_newchip.ld` with memory layout
3. Add target block in `Makefile`:
```makefile
else ifeq ($(TARGET),newchip)
    MCU_TYPE = stm32xxxxxx
    CMSIS_DEVICE = cmsis_f4
    ...
```
4. Add conditional include in `include/mcu.h`

## Clock Configuration

### STM32F103 (72MHz)
```
HSE (8MHz) → PLL (×9) → 72MHz SYSCLK
├─ AHB (72MHz) → CPU, Flash, DMA
├─ APB1 (36MHz) → USART2/3, I2C, SPI2, TIM2-7
└─ APB2 (72MHz) → USART1, SPI1, TIM1, ADC
```
- Flash latency: 2 wait states
- APB1 max: 36MHz (prescaler /2)

### STM32F411 (96MHz)
```
HSE (25MHz) → PLL (÷25 ×192 ÷2) → 96MHz SYSCLK
├─ AHB (96MHz) → CPU, Flash, DMA
├─ APB1 (48MHz) → USART2, I2C, SPI2, TIM2-5
└─ APB2 (96MHz) → USART1/6, SPI1, TIM1, TIM9-11, ADC
```
- Flash latency: 3 wait states @ 3.3V
- APB1 max: 50MHz (prescaler /2)
- PLL: VCO input 1MHz, VCO output 192MHz, PLLP=2

## Memory Layout

**F103 ([link_f103.ld](link_f103.ld)):**
| Region | Start      | Size  | Contents |
|--------|------------|-------|----------|
| Flash  | 0x08000000 | 128KB | `.vectors`, `.text`, `.rodata` |
| SRAM   | 0x20000000 | 20KB  | `.data`, `.bss`, stack |

**F411 ([link_f411.ld](link_f411.ld)):**
| Region | Start      | Size  | Contents |
|--------|------------|-------|----------|
| Flash  | 0x08000000 | 512KB | `.vectors`, `.text`, `.rodata` |
| SRAM   | 0x20000000 | 128KB | `.data`, `.bss`, stack |

## Build System

### Compiler Flags
- **Strict warnings:** `-Werror -Wextra -Wshadow -Wconversion` (all warnings → errors)
- **Optimization:** `-O0 -g3` (debug) or `-Os` (release)
- **Dead code elimination:** `-ffunction-sections -fdata-sections` + `--gc-sections`
- **F103:** `-mthumb -mcpu=cortex-m3`
- **F411:** `-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
- **Minimal libc:** `--specs nano.specs -lc -lgcc` (newlib-nano)

### Dependencies

First build auto-clones CMSIS dependencies:
- `cmsis_core/` - ARM CMSIS-Core v5.9.0
- `cmsis_f1/` - ST F1 device headers v4.3.5
- `cmsis_f4/` - ST F4 device headers v2.6.10

## Debugging

### VS Code Integration

The project includes complete VS Code support:

**Debug Configurations ([.vscode/launch.json](.vscode/launch.json)):**
- "Debug F103 (Blue Pill)" - Uses `stm32f1x.cfg`, builds with `TARGET=f103`
- "Debug F411 (Black Pill)" - Uses `stm32f4x.cfg`, builds with `TARGET=f411`

**IntelliSense ([.vscode/c_cpp_properties.json](.vscode/c_cpp_properties.json)):**
- Switch between F103/F411 configurations for correct header paths and defines
- Use C/C++ configuration selector in status bar

**Tasks ([.vscode/tasks.json](.vscode/tasks.json)):**
- Build F103 / Build F411
- Flash F103 / Flash F411
- Clean

**Usage:**
1. Install Cortex-Debug extension
2. Connect ST-Link programmer via SWD
3. Select debug configuration from dropdown
4. Press F5 to build and debug

**Requirements:**
- `openocd` with ST-Link support
- `gdb-multiarch` or `arm-none-eabi-gdb`
- ST-Link V2 hardware (SWDIO, SWCLK, GND, 3.3V)

### Command Line Debugging

```bash
# Terminal 1: Start OpenOCD for F103
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg

# Terminal 2: Connect GDB
gdb-multiarch firmware.elf
(gdb) target extended-remote :3333
(gdb) load
(gdb) monitor reset halt
(gdb) continue
```

For F411, use `target/stm32f4x.cfg` instead.

### Serial Debugging

UART2 at 115200 baud (PA2=TX, PA3=RX):
```c
printf("Debug: value=%lu\r\n", value);
```
Use USB-to-serial adapter or ST-Link's virtual COM port.

## Flashing

### Via ST-Link
```bash
make flash TARGET=f103
make flash TARGET=f411
```

### Via DFU (F411 Black Pill only)
The Black Pill has a built-in USB bootloader:
1. Hold BOOT0 button, press RESET
2. `dfu-util -a 0 -s 0x08000000:leave -D firmware.bin`

## Architecture Details

### STM32F1 GPIO (CRL/CRH Registers)

F1 uses configuration registers with 4 bits per pin:
- **CRL:** Pins 0-7 (32 bits = 8 pins × 4 bits)
- **CRH:** Pins 8-15 (32 bits = 8 pins × 4 bits)
- **CNF[1:0]:** Configuration (analog, floating, pull-up/down, alt function)
- **MODE[1:0]:** Speed for outputs, must be 00 for inputs

GPIO clock on APB2 bus.

### STM32F4 GPIO (MODER/OTYPER/OSPEEDR)

F4 uses separate registers:
- **MODER:** Mode (2 bits per pin) - input/output/AF/analog
- **OTYPER:** Output type (1 bit per pin) - push-pull/open-drain
- **OSPEEDR:** Speed (2 bits per pin) - 4 speed levels
- **PUPDR:** Pull-up/pull-down (2 bits per pin)
- **AFR[0]/AFR[1]:** Alternate function (4 bits per pin)

GPIO clock on AHB1 bus.

### UART Baud Rate Calculation

Both F1 and F4:
```c
BRR = peripheral_clock / baudrate
```

Examples:
- F103 UART2 @ 36MHz: `BRR = 36000000 / 115200 = 312`
- F411 UART2 @ 48MHz: `BRR = 48000000 / 115200 = 417`

The `uart_init()` function automatically uses the correct APB clock.

## Startup Sequence

1. **Reset:** Startup assembly (`startup_stm32f10x.s` or `startup_stm32f411xe.s`)
   - Sets stack pointer to `_estack` (end of SRAM)
   - Copies `.data` from flash to SRAM
   - Zeros `.bss` section
2. **SystemInit():** Called before main
   - Runs `clock_init()` to configure PLL
3. **main():** Application code starts
   - Configures SysTick for 1ms interrupts
   - Initializes peripherals
   - Enters main loop

## Adding Features

### New Peripheral

Add inline functions to target-specific headers:
```c
// In mcu_f103.h or mcu_f411.h
static inline void spi_init(SPI_TypeDef* spi) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable clock
    // Configure SPI registers...
}
```

Always enable peripheral clock before accessing registers.

### New Source File

1. Add to `SOURCES` in [Makefile](Makefile)
2. Include `mcu.h` for peripheral access
3. Update interrupt vector table if using ISRs (in startup file)

## Target-Specific Notes

### STM32F103
- GPIO uses CRL/CRH (4 bits per pin), not MODER
- Available GPIO banks: A-E only (no GPIOF on STM32F103xB)
- USART peripherals called USART1-3
- Running at 72MHz (max for F103)
- No FPU

### STM32F411
- GPIO uses MODER/OTYPER/OSPEEDR (modern architecture)
- Available GPIO banks: A-E, H
- Must set alternate function numbers for UART/SPI/etc.
- Running at 96MHz (can go to 100MHz)
- Hardware FPU enabled (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`)
- Black Pill has USB-C with DFU bootloader
- No external LSE crystal on Black Pill

## License

See original guide: [cpq/bare-metal-programming-guide](https://github.com/cpq/bare-metal-programming-guide)
