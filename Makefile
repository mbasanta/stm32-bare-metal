OPTFLAGS  ?= -Os
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -ffunction-sections -fdata-sections \
			 -I. -Iinclude -Icmsis_core/CMSIS/Core/Include -Icmsis_f1/Include \
            -mcpu=cortex-m3 -mthumb $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c syscalls.c
SOURCES += cmsis_f1/Source/Templates/gcc/startup_stm32f103xb.s # ST startup file. Compiler-dependent!

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

firmware.elf: cmsis_core cmsis_f1 link.ld Makefile $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(OPTFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x08000000

cmsis_core:
	git clone --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_f1:
	git clone --depth 1 -b v4.3.5 https://github.com/STMicroelectronics/cmsis_device_f1 $@

clean:
	rm -rf firmware.*