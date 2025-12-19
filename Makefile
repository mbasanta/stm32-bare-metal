# Target selection: f103 (Blue Pill) or f411 (Black Pill)
TARGET ?= f411

# Target-specific configuration
ifeq ($(TARGET),f103)
	MCU_FAMILY = f1
	MCU_TYPE = stm32f103xb
	CMSIS_DEVICE = cmsis_f1
	CMSIS_VERSION = v4.3.5
	STARTUP_FILE = $(CMSIS_DEVICE)/Source/Templates/gcc/startup_$(MCU_TYPE).s
	LINKER_SCRIPT = link_f103.ld
	CPU_FLAGS = -mcpu=cortex-m3 -mthumb
	FPU_FLAGS =
else ifeq ($(TARGET),f411)
	MCU_FAMILY = f4
	MCU_TYPE = stm32f411xe
	CMSIS_DEVICE = cmsis_f4
	CMSIS_VERSION = v2.6.10
	STARTUP_FILE = $(CMSIS_DEVICE)/Source/Templates/gcc/startup_$(MCU_TYPE).s
	LINKER_SCRIPT = link_f411.ld
	CPU_FLAGS = -mcpu=cortex-m4 -mthumb
	FPU_FLAGS = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
else
	$(error Unknown TARGET: $(TARGET). Use TARGET=f103 or TARGET=f411)
endif

$(info Building for $(TARGET) ($(MCU_TYPE)))

OPTFLAGS  ?= -Os
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -ffunction-sections -fdata-sections \
			 -I. -Iinclude -Icmsis_core/CMSIS/Core/Include -I$(CMSIS_DEVICE)/Include \
            $(CPU_FLAGS) $(FPU_FLAGS) -DTARGET_$(TARGET) $(EXTRA_CFLAGS)
LDFLAGS ?= -T$(LINKER_SCRIPT) -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c syscalls.c
SOURCES += $(STARTUP_FILE)

BUILD ?= debug

ifeq ($(BUILD),debug)
	OPTFLAGS = -O0
	EXTRA_CFLAGS += -DDEBUG -g3
else ifeq ($(BUILD),release)
	OPTFLAGS = -Os
else
	$(error Unknown build type: $(BUILD))
endif

$(info Building with OPTFLAGS=$(OPTFLAGS))

rebuild: clean build

build: firmware.elf

firmware.elf: cmsis_core $(CMSIS_DEVICE) $(LINKER_SCRIPT) Makefile $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(OPTFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x08000000

cmsis_core:
	git clone --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_f1:
	git clone --depth 1 -b v4.3.5 https://github.com/STMicroelectronics/cmsis_device_f1 $@

cmsis_f4:
	git clone --depth 1 -b v2.6.10 https://github.com/STMicroelectronics/cmsis_device_f4 $@

clean:
	rm -rf firmware.*