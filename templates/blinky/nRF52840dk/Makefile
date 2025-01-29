CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -O0 -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections
SOURCES = main.c


firmware.elf: hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) -o $@

firwmare.hex:
	arm-none-eabi-objcopy -Oihex firmware.elf firmware.hex

flash: 
	nrfjprog -f NRF52 --program firmware.hex --recover --verify
	nrfjprog -f NRF52 --reset

clean:
	rm -rf firmware.*
