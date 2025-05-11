CFLAGS  ?= -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
           -Wformat-truncation -fno-common -Wconversion \
           -g3 -Os -ffunction-sections -fdata-sections -I. \
           -mcpu=cortex-m0plus -mthumb

LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs=nano.specs -lc -lgcc \
           -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES = main.c

build: firmware.elf firmware.bin

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

flash: firmware.bin
	st-flash --reset write firmware.bin 0x08000000

clean:
	rm -f *.o *.elf *.bin *.map