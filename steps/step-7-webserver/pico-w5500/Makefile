CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion -ffreestanding \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m0plus -mthumb $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c

SOURCES += mongoose.c
CFLAGS += -DMG_ARCH=MG_ARCH_NEWLIB -DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_MIP=1

build: firmware.uf2

ifeq ($(OS),Windows_NT)
  RM = cmd /C del /Q /F
  BIN2UF2 = bin2uf2.exe
else
  BIN2UF2 = ./bin2uf2
  RM = rm -f
$(BIN2UF2): tools/bin2uf2.c
	$(CC) -W -Wall $< -o $@
endif

firmware.elf: $(SOURCES) hal.h
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

firmware.uf2: firmware.bin $(BIN2UF2)
	$(BIN2UF2) $< $@

# Requires env variable VCON_API_KEY set
DEVICE_URL ?= https://dash.vcon.io/api/v3/devices/3
test: update
	curl --fail -su :$(VCON_API_KEY) $(DEVICE_URL)/tx?t=5 | tee /tmp/output.txt
	grep 'Ethernet: up' /tmp/output.txt

update: build
	curl --fail -su :$(VCON_API_KEY) $(DEVICE_URL)/ota --data-binary @firmware.bin

clean:
	$(RM) firmware.* bin2uf2
