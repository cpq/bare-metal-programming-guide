CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -I. -Icmsis_core/CMSIS/Core/Include -Icmsis_f3/Include -Ilittlefs
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c syscalls.c littlefs/lfs.c littlefs/lfs_util.c
SOURCES += cmsis_f3/Source/Templates/gcc/startup_stm32f303x8.s # ST startup file. Compiler-dependent!

ifeq ($(OS),Windows_NT)
  RM = cmd /C del /Q /F
else
  RM = rm -rf
endif

build: firmware.bin

firmware.elf: cmsis_core cmsis_f3 hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	STM32_Programmer_CLI -c port=swd -e all -w $< 0x8000000 -hardRst

cmsis_core:
	git clone -q -c advice.detachedHead=false --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_f3:
	git clone -q -c advice.detachedHead=false --depth 1 -b v2.3.7 https://github.com/STMicroelectronics/cmsis_device_f3 $@

littlefs/lfs.c: littlefs
littlefs:
	git clone -q -c advice.detachedHead=false --depth 1 -b v2.6.1 https://github.com/littlefs-project/littlefs $@

clean:
	$(RM) firmware.* cmsis_* littlefs

# Automated test via https://vcon.io/automated-firmware-tests/. Set VCON_API_KEY and update DEVICE_URL
DEVICE_URL ?= https://dash.vcon.io/api/v3/devices/10
update: CFLAGS += -DUART_DEBUG=USART1
update: firmware.bin
	curl --fail-with-body -su :$(VCON_API_KEY) $(DEVICE_URL)/ota --data-binary @$<
test: update
	curl --fail-with-body -su :$(VCON_API_KEY) $(DEVICE_URL)/tx?t=5 | tee /tmp/output.txt
	egrep '^tick:.*CPU 64 MHz' /tmp/output.txt
watch: update
	curl --fail-with-body -su :$(VCON_API_KEY) $(DEVICE_URL)/tx?t=999
