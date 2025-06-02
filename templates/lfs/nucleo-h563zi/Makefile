CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion
CFLAGS += -Wformat-truncation -fno-common -Wno-sign-conversion 
#CFLAGS += -Wconversion      # For LFS
CFLAGS += -Wno-error=shadow  # For LFS
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections
CFLAGS += -I. -Imongoose -Icmsis_core/CMSIS/Core/Include -Icmsis_h5/Include
CFLAGS += -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard $(CFLAGS_EXTRA)

LDFLAGS ?= -Tlink.ld -nostdlib -nostartfiles --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES = main.c syscalls.c
SOURCES += retarget_lfs.c littlefs/lfs.c littlefs/lfs_util.c
SOURCES += cmsis_h5/Source/Templates/gcc/startup_stm32h563xx.s

all build: firmware.bin

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size --format=berkeley $<

firmware.elf: cmsis_core cmsis_h5 littlefs $(SOURCES) $(wildcard *.h) link.ld Makefile
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

flash: firmware.bin
	STM32_Programmer_CLI -c port=swd -e all -w $< 0x8000000 -hardRst

cmsis_core:
	git clone -q -c advice.detachedHead=false --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_h5:
	git clone -q -c advice.detachedHead=false --depth 1 -b v1.2.0 https://github.com/STMicroelectronics/cmsis_device_h5 $@

littlefs/lfs.c: littlefs
littlefs:
	git clone -q -c advice.detachedHead=false --depth 1 -b v2.11.0 https://github.com/littlefs-project/littlefs $@

clean:
	rm -rf firmware.* cmsis_* littlefs
