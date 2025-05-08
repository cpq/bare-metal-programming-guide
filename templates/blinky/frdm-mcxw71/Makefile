CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -I. -Iinclude -Icmsis_core/CMSIS/Core/Include
CFLAGS += -Icmsis_mcxw71/devices/MCXW716C
CFLAGS += -Icmsis_mcxw71/devices/MCXW716C/periph2
CFLAGS += -DCPU_MCXW716CMFTA -D__ATOLLIC__
CFLAGS += -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard $(CFLAGS_EXTRA)

LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c syscalls.c startup.c

#LDFLAGS ?= -Tcmsis_mcxw71/devices/MCXW716C/gcc/mcxw716_flash.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
#SOURCES += cmsis_mcxw71/devices/MCXW716C/system_MCXW716C.c
#SOURCES += cmsis_mcxw71/devices/MCXW716C/gcc/startup_MCXW716C.S

build: firmware.bin

firmware.elf: cmsis_core cmsis_mcxw71 hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	(echo LoadFile $< 0; echo exit) | JLinkExe -nogui 1 -device mcxw716 -if swd -AutoConnect 1 -speed 4000

cmsis_core:
	git clone -q -c advice.detachedHead=false --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_mcxw71:
	curl -sL https://mcuxpresso.nxp.com/cmsis_pack/repo/NXP.MCXW716C_DFP.25.03.00.pack -o x.zip && mkdir -p $@ && (cd $@ && unzip -q ../x.zip && rm -f ../x.zip)

clean:
	rm -rf firmware.* cmsis_*
