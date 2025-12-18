# Multi-Target Build System

This project supports building for multiple STM32 microcontrollers from the same codebase.

## Supported Targets

### Blue Pill - STM32F103
```bash
make build TARGET=f103
```
- **MCU:** STM32F103C8T6/RBT6
- **Clock:** 72MHz (HSE 8MHz)
- **Flash:** 128KB
- **SRAM:** 20KB
- **LED:** PC13 (active-low)

### Black Pill - STM32F411
```bash
make build TARGET=f411
```
- **MCU:** STM32F411CEU6
- **Clock:** 96MHz (HSE 25MHz)
- **Flash:** 512KB
- **SRAM:** 128KB
- **LED:** PC13 (active-low)
- **FPU:** Hardware floating-point

## Build Commands

```bash
# Build for Black Pill (default)
make build TARGET=f411

# Build for Blue Pill  
make build TARGET=f103

# Clean and rebuild
make clean
make build TARGET=f103

# Flash to device
make flash TARGET=f103

# Set default target (optional)
export TARGET=f103
make build
```

## How It Works

### 1. Makefile Target Selection
The `TARGET` variable controls all build parameters:
- CPU architecture (Cortex-M3 vs M4)
- FPU flags
- CMSIS device headers
- Linker script
- Startup file

### 2. Preprocessor Defines
The Makefile passes `-DTARGET_f103` or `-DTARGET_f411` to enable conditional compilation.

### 3. Target-Specific Headers

**`include/mcu_f103.h`** - F103 configuration
- Clock settings (72MHz)
- Pin definitions
- Bus frequencies

**`include/mcu_f411.h`** - F411 configuration
- Clock settings (96MHz)
- Pin definitions
- Bus frequencies

**`include/gpio_f1.h`** - F1 GPIO implementation (CRL/CRH registers)
**`include/gpio_f4.h`** - F4 GPIO implementation (MODER/OTYPER/OSPEEDR)

### 4. Main MCU Header
`include/mcu.h` includes the right headers based on `TARGET_*` define:
```c
#if defined(TARGET_f103)
    #include "mcu_f103.h"
    #include "gpio_f1.h"
#elif defined(TARGET_f411)
    #include "mcu_f411.h"
    #include "gpio_f4.h"
#endif
```

### 5. Application Code
Your `main.c` uses target-independent APIs:
```c
gpio_set_mode(LED_PIN, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
gpio_write(LED_PIN, LED_ACTIVE_LOW ? !on : on);
printf("Starting %s @ %lu MHz\r\n", MCU_NAME, FREQ_HZ / 1000000);
```

The constants (`LED_PIN`, `MCU_NAME`, `FREQ_HZ`) are defined in the target config headers.

## File Structure

```
├── Makefile              # Multi-target build logic
├── link_f103.ld          # Linker script for F103 (128K/20K)
├── link_f411.ld          # Linker script for F411 (512K/128K)
├── main.c                # Target-independent application
├── syscalls.c            # Newlib syscalls (printf support)
└── include/
    ├── mcu.h             # Main header with conditional includes
    ├── mcu_f103.h        # F103 configuration & clock init
    ├── mcu_f411.h        # F411 configuration & clock init
    ├── gpio_f1.h         # F1 GPIO implementation
    └── gpio_f4.h         # F4 GPIO implementation
```

## Adding a New Target

1. Create `include/mcu_newchip.h` with clock config and pin defines
2. Create `link_newchip.ld` with memory layout
3. Add target configuration block in `Makefile`:
```makefile
else ifeq ($(TARGET),newchip)
    MCU_FAMILY = f4
    MCU_TYPE = stm32f4xxxx
    ...
```
4. Add conditional include in `include/mcu.h`

## GPIO API Compatibility

Both F1 and F4 GPIO implementations support a common subset:

```c
// Works on both F103 and F411
gpio_set_mode(PIN('A', 5), GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);
gpio_write(PIN('A', 5), true);
```

F1-specific modes are still available when building for F103:
```c
gpio_set_mode(PIN('A', 2), GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
```

F4-specific features (alternate functions) are available when building for F411:
```c
gpio_set_af(PIN('A', 2), 7);  // AF7 = USART2
```

## Debugging

Use OpenOCD with the correct target:
- F103: `source [find target/stm32f1x.cfg]`
- F411: `source [find target/stm32f4x.cfg]`

The `openocd.cfg` file may need updating when switching targets.
