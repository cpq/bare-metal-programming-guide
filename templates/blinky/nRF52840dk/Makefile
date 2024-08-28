CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -mcpu=cortex-m4
SOURCES = main.c

build: 
	mkdir build
	arm-none-eabi-gcc $(SOURCES) -c $(CFLAGS) -o ./build/main.o
	arm-none-eabi-gcc -T link.ld -nostdlib ./build/main.o -o ./build/firmware.elf
	arm-none-eabi-objcopy -Oihex ./build/firmware.elf ./build/firmware.hex

flash: 
	nrfjprog -f NRF52 --program ./build/firmware.hex --recover --verify
	nrfjprog -f NRF52 --reset

clean:
	rm -rf ./build 
