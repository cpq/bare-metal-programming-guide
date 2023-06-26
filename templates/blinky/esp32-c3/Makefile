PROG ?= firmware
ESPUTIL ?= esputil
CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow
CFLAGS += -Wdouble-promotion -fno-common -Wconversion
CFLAGS += -march=rv32imc -mabi=ilp32
CFLAGS += -Os -ffunction-sections -fdata-sections -I.
LDFLAGS ?= -Tlink.ld -nostdlib -nostartfiles -Wl,--gc-sections $(EXTRA_LINKFLAGS)
CWD ?= $(realpath $(CURDIR))
FLASH_ADDR ?= 0  # 2nd stage bootloader flash offset
DOCKER ?= docker run -it --rm -v $(CWD):$(CWD) -w $(CWD) mdashnet/riscv
C ?= $(DOCKER) riscv-none-elf-gcc
SOURCES = startup.c main.c

build: $(PROG).bin

$(PROG).elf: $(SOURCES) hal.h link.ld Makefile 
	$(C) $(CFLAGS) $(CFLAGS_EXTRA) $(SOURCES) $(LDFLAGS) -o $@

$(PROG).bin: $(PROG).elf
	$(ESPUTIL) mkbin $(PROG).elf $@

flash: $(PROG).bin
	$(ESPUTIL) flash $(FLASH_ADDR) $(PROG).bin

monitor:
	$(ESPUTIL) monitor

clean:
	@rm -rf *.{bin,elf,map,lst,tgz,zip,hex} $(PROG)*
