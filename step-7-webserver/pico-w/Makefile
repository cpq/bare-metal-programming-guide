CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -ffreestanding \
            -g3 -Os -ffunction-sections -fdata-sections -I. -Iinclude \
            -mcpu=cortex-m0plus -mthumb $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c

ifeq ($(OS),Windows_NT)
  RM = cmd /C del /Q /F
else
  RM = rm -f
endif

build: firmware.uf2

firmware.elf: $(SOURCES) mcu.h
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

firmware.uf2: firmware.bin bin2uf2
	./bin2uf2 $< $@
#	bin2uf2 -i $< -o $@

bin2uf2: tools/bin2uf2.c
	$(CC) -W -Wall $< -o $@

clean:
	$(RM) firmware.* bin2uf2
