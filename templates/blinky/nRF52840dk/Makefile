CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections

SOURCES = main.c

firmware.elf: hal.h link.ld Makefile $(SOURCES) #cmsis_core cmsis_nrf
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) -o $@
	arm-none-eabi-size $@

firmware.bin firmware.hex: firmware.elf
	arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex
	arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

flash: 
	nrfjprog -f NRF52 --program firmware.hex --recover --verify
	nrfjprog -f NRF52 --reset

cmsis_core:
	git clone -c advice.detachedHead=false -q --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_nrf:
	curl -sL https://developer.nordicsemi.com/nRF5_SDK/pieces/nRF_DeviceFamilyPack/NordicSemiconductor.nRF_DeviceFamilyPack.8.44.1.pack -o x.zip && mkdir -p $@ && (cd $@ && unzip -q ../x.zip && rm -f ../x.zip)

clean:
	rm -rf firmware.*
