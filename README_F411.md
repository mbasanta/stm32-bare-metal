# STM32F411 Black Pill Configuration (96MHz)

This branch adapts the bare-metal STM32 project for the **WeAct STM32F411CEU6 Black Pill** board running at **96MHz**.

## Hardware Differences from STM32F103

### Microcontroller: STM32F411CEU6
- **Architecture:** ARM Cortex-M4F (vs M3) with FPU
- **Flash:** 512KB (vs 128KB)
- **SRAM:** 128KB (vs 20KB)
- **Max Clock:** 100MHz (configured for 96MHz)
- **HSE Crystal:** 25MHz (vs 8MHz on Nucleo)

### Black Pill Board
- **LED:** PC13 (active-low)
- **USB:** USB-C connector with built-in bootloader
- **Pinout:** Different from Nucleo - check schematic

## Clock Configuration (96MHz)

### Clock Tree
```
HSE (25MHz) → PLL → 96MHz SYSCLK
├─ AHB (96MHz) → CPU, Flash, DMA
├─ APB1 (48MHz) → USART2, TIM2-5, I2C, SPI2
└─ APB2 (96MHz) → USART1, USART6, TIM1, TIM9-11, SPI1, ADC
```

### PLL Configuration
- **VCO Input:** 25MHz ÷ 25 = 1MHz (PLLM=25)
- **VCO Output:** 1MHz × 192 = 192MHz (PLLN=192)
- **SYSCLK:** 192MHz ÷ 2 = 96MHz (PLLP=2)
- **Flash Latency:** 3 wait states @ 3.3V

## GPIO Differences (F4 vs F1)

STM32F4 uses modern GPIO architecture:
- **Mode:** `MODER` register (2 bits per pin) instead of `CRL/CRH`
- **Output Type:** `OTYPER` (push-pull or open-drain)
- **Speed:** `OSPEEDR` (4 speed levels)
- **Pull-up/down:** `PUPDR` register
- **Alternate Functions:** `AFR[0]/AFR[1]` registers with AF numbers

### Example GPIO Configuration
```c
// F411 (simple API)
gpio_set_mode(PIN('C', 13), GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);

// For UART alternate function
gpio_set_mode(PIN('A', 2), GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
gpio_set_af(PIN('A', 2), 7);  // AF7 = USART2
```

## UART Configuration

### Available UARTs
- **USART1:** PA9/PA10 (AF7) - on APB2 @ 96MHz
- **USART2:** PA2/PA3 (AF7) - on APB1 @ 48MHz (default for printf)
- **USART6:** PA11/PA12 (AF8) - on APB2 @ 96MHz

### Baud Rate Calculation
```c
// USART2 on APB1
BRR = 48MHz / 115200 = 417

// USART1/6 on APB2  
BRR = 96MHz / 115200 = 833
```

## Building

```bash
make clean
make build        # Builds firmware.elf
make flash        # Flash via st-flash
```

## Flashing

### Via ST-Link
```bash
make flash
```

### Via DFU (USB Bootloader)
1. Hold BOOT0 button, press RESET
2. `dfu-util -a 0 -s 0x08000000:leave -D firmware.bin`

## Key Code Changes

### 1. Makefile
- Changed to `cortex-m4` with FPU flags (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`)
- Updated CMSIS includes to `cmsis_f4`
- Changed startup file to `startup_stm32f411xe.s`

### 2. Linker Script
```ld
MEMORY {
    flash(rx)  : ORIGIN = 0x08000000, LENGTH = 512k  # was 128k
    sram(rwx)  : ORIGIN = 0x20000000, LENGTH = 128k  # was 20k
}
```

### 3. mcu.h
- Changed include to `stm32f411xe.h`
- Rewrote `clock_init()` for F4 RCC (different PLL configuration)
- Completely new GPIO functions using MODER/OTYPER/OSPEEDR
- Added `gpio_set_af()` for alternate function configuration
- Updated UART init to set AF numbers

### 4. main.c
- Changed LED pin from PA5 to PC13
- Inverted LED logic (active-low on Black Pill)
- Updated SystemInit comment (96MHz vs 72MHz)

## Performance

Running at 96MHz vs 72MHz F103:
- **33% faster clock** (96MHz vs 72MHz)
- **FPU enabled** for floating-point operations
- **6.4× more SRAM** (128KB vs 20KB)
- **4× more Flash** (512KB vs 128KB)

## Notes

- The F411 can run up to 100MHz, but 96MHz provides cleaner PLL divisors
- Black Pill has no external LSE crystal - use HSI for RTC if needed
- USB support available but not configured in this minimal example
- SWD pins (PA13/PA14) work for debugging without external pull-ups
