OPTFLAGS  ?= -Os
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m3 -mthumb $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c

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

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(OPTFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x08000000

clean:
	rm -rf firmware.*