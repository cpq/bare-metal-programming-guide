TARGET  = firmware
CROSS   ?= arm-none-eabi
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
						-Wformat-truncation -fno-common -Wconversion \
						-g3 -Os -ffunction-sections -fdata-sections -I. \
						-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c 

all build example: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf
	$(DOCKER) $(CROSS)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES)
	$(DOCKER) $(CROSS)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

clean:
	@rm -rf $(TARGET).* *.su
