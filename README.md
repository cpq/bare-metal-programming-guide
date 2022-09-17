# Bare metal programming guide

This guide is written for beginners who wish to start programming
microcontrollers using GCC compiler and bare metal approach.  We are going to
use a
[Nucleo-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)
development board with STM32F429 microcontroller. But basic principles would be
applicable to any other microcontroller. To proceed, please install the
following tools:

- ARM GCC, https://launchpad.net/gcc-arm-embedded
- GNU make, http://www.gnu.org/software/make/
- ST link, https://github.com/stlink-org/stlink

Also, download a [STM32F429 datasheet](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf).

## Introduction

A microcontroller (uC, or MCU) is a small computer. Typically it has CPU, RAM,
flash to store firmware code, and a bunch of pins that stick out. Some pins are
used to power the MCU, usually marked as GND (ground) and VCC pins. Other pins
are used to communicate with the MCU, by means of high/low voltage applied to
those pins. One of the simplest ways of communication is an LED attached to a
pin: one LED contact is attached to the ground pin (GND), and another contact
is attached to a signal pin via a current-limiting resistor.  A firmware code
can set high or low voltage on a signal pin, making LED blink:

<img src="images/mcu.svg" height="200" />

### Memory and registers

The 32-bit address space of the MCU is divided by regions. For example, some
region of memory is mapped to the internal MCU flash at a specific address.
Firmware code instructions are read and executed by reading from that memory region. Another region is
RAM, which is also mapped to a specific address. We can read and write any
values to the RAM region.

From STM32F429 datasheet, we can take a look at section 2.3.1 and learn
that RAM region starts at address 0x20000000 and has size of 192KB. From section
2.4 we can learn that flash is mapped at address 0x08000000. Our MCU has
2MB flash, so flash and RAM regions are located like this:

<img src="images/mem.svg" />

From the datasheet we can also learn that there are many more memory regions.
Their address ranges are given in the section 2.3 "Memory Map". For example,
there is a "GPIOA" region that starts at 0x40020000 and has length of 1KB.

These memory regions correspond to a different "peripherals" inside the MCU -
a piece of silicon circuitry that make certain pins behave in a special way.
A peripheral memory region is a collection of 32-bit **registers**. Each
register is a 4-byte memory range at a certain address, that maps to a certain
function of the given peripheral. By writing values into a register - in other
words, by writing a 32-bit value at a certain memory address, we can control
how given peripheral should behave. By reading registers, we can read back
peripheral's data or configuration.

For example, a GPIO (General Purpose Input Output) peripheral allows to
manipulate MCU pins by writing and reading to GPIO registers.  The GPIOA
peripheral starts at 0x40020000, and we can find GPIO register description in
section 8.4. The datasheet says that `GPIOA_MODER` register has offset 0,
that means that it's address is `0x40020000 + 0`, and this is the format of
the register:

<img src="images/moder.png" style="max-width: 100%" />

The datasheet shows that the 32-bit MODER register is a collection of 2-bit
values, 16 in total. Therefore, one MODER register controls 16 physical pins,
Bits 0-1 control pin 0, bits 2-3 control pin 1, and so on. The 2-bit value
encodes pin mode: 0 means input, 1 means output, 2 means "alternate function" -
some specific behavior described elsewhere, and 3 means analog. Since the
peripheral name is "GPIOA", then pins are named "A0", "A1", etc. For peripheral
"GPIOB", pin naming would be "B0", "B1", ...

If we write 32-bit value `0` to the register MODER, we'll set all 16 pins,
from A0 to A15, to input mode:

```c
  * (volatile uint32_t *) (0x40020000 + 0) = 0;  // Set A0-A15 to input mode
```

By setting individual bits, we can selectively set specific pins to a desired
mode. For example, this snippet sets pin A3 to output mode:

```c
  * (volatile uint32_t *) (0x40020000 + 0) &= ~(3 << 6);  // CLear bits 6-7
  * (volatile uint32_t *) (0x40020000 + 0) |= 1 << 6;     // Set bits 6-7 to 1
```

Some registers are not mapped to the MCU peripherals, but they are mapped to
the ARM CPU configuration and control. For example, there is a "Reset at clock
control" unit, described in section 6 of the datasheet. It describes registers
that allow to set systems clock and other things.

## Human-readable peripherals programming

In the previous section we have learned that we can read and write peripheral
register by direct accessing certain memory addresses. Let's look at the
snippet that sets pin A3 to output mode:

```c
  * (volatile uint32_t *) (0x40020000 + 0) &= ~(3 << 6);  // CLear bits 6-7
  * (volatile uint32_t *) (0x40020000 + 0) |= 1 << 6;     // Set bits 6-7 to 1
```

That is pretty cryptic. Without extensive comments, such code would be quite
hard to understand. We can rewrite this code to a much more readable form.
The idea is to represent the whole peripheral as a structure that contains
32-bit fields. Let's see what registers exist for the GPIO
peripheral in the section 8.4 of the datasheet. They are MODER, OTYPER, OSPEEDR,
PUPDR, IDR, ODR, BSRR, LCKR, AFR. Their offsets are
with offsets 0, 4, 8, etc... . That means we can represent them as
a structure with 32-bit fields, and make a define for GPIOA:

```c
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

#define GPIOA ((struct gpio *) 0x40020000)
```

Then, for setting GPIO pin mode, we can define a function:

```c
// Enum values are per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

static inline void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode) {
  gpio->MODER &= ~(3U << (pin * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (pin * 2);   // Set new mode
}
```
Now, we can rewrite the snippet for A3 like this:

```c
gpio_set_mode(GPIOA, 3 /* pin */, GPIO_MODE_OUTPUT);  // Set A3 to output
```

Our MCU has several GPIO peripherals (also called "banks"): A, B, C, ... K.
From section 2.3 we can see that they are 1KB away from each other:
GPIOA is at address 0x40020000, GPIOB is at 0x40020400, and so on:

```c
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
```

We can create pin numbering that includes the bank and the pin number.
To do that, we use 2-byte `uint16_t` value, where upper byte indicates
GPIO bank, and lower byte indicates pin number:

```c
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
```

This way, we can specify pins for any GPIO bank:

```c
  uint16_t pin1 = PIN('A', 3);    // A3   - GPIOA pin 3
  uint16_t pin2 = PIN('G', 11);   // G11  - GPIOG pin 11
```

Let's rewrite the `gpio_set_mode()` function to take our pin specification:

```c
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin)); // GPIO bank
  uint8_t n = PINNO(pin);                 // Pin number
  gpio->MODER &= ~(3U << (n * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);   // Set new mode
}
```

Now the code for A3 is self-explanatory:

```c
  uint16_t pin = PIN('A', 3);            // Pin A3
  gpio_set_mode(pin, GPIO_MODE_OUTPUT);  // Set to output
```

Note that we have created a useful initial API for the GPIO peripheral. Other
peripherals, like UART (serial communication) and others - can be implemented
in a similar way. This is a good programming practice that makes code
self-explanatory and human readable.

## MCU boot and vector table

When STM32F429 MCU boots, it reads a so-called "vector table" from the
beginning of flash memory. A vector table is a concept common to all ARM MCUs.
That is a array of 32-bit addresses of interrupt handlers. First 16 entries
are reserved by ARM and are common to all ARM MCUs. The rest of interrupt
handlers are specific to the given MCU - these are interrupt handlers for
peripherals. Simpler MCUs with few peripherals have few interrupt handlers,
and more complex MCUs have many.

Vector table for STM32F429 is documented in Table 62. From there we can learn
that there are 91 peripheral handlers, in addition to the standard 16.

At this point, we are interested in the first two entries of the vector table,
because they play a key role in the MCU boot process. Those two first values
are: initial stack pointer, and an address of the boot function to execute
(a firmware entry point).

So now we know, that we must make sure that our firmware should be composed in
a way that the 2nd 32-bit value in the flash should contain an address of
out boot function. When MCU boots, it'll read that address from flash, and
jump to our boot function.


## Minimal firmware

Let's create a file `main.c`, and specify our boot function that initially does
nothing (falls into infinite loop), and specify a vector table that contains 16
standard entries and 91 STM32 entries. In your editor of choice, create
`main.c` file and copy/paste the following into `main.c` file:

```c
// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  for (;;) (void) 0;  // Infinite loop
}

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
  0, _reset
};
```

For function `_reset()`, we have used GCC-specific attributes `naked` and
`noreturn` - they mean, standard function's prologue and epilogue should not
be created by the compiler, and that function does not return.

The vector table `tab` we put in a separate section called `.vectors` - that
we need later to tell the linker to put that section right at the beginning
of the generated firmware - and consecutively, at the beginning of flash
memory. We leave the rest of vector table filled with zeroes.

Note that we do not set the first entry in the vector table, which is an
initial value for the stack pointer. Why? Because we don't know it. We'll
handle it later.

### Compilation

Let's compile our code:

```sh
$ arm-none-eabi-gcc -mcpu=cortex-m4 main.c -c
```

That works! The compilation produced a file `main.o` which contains
our minimal firmware that does nothing.  The `main.o` file is in ELF binary
format, which contains several sections. Let's see them:

```sh
$ arm-none-eabi-objdump -h main.o
...
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .text         00000002  00000000  00000000  00000034  2**1
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  1 .data         00000000  00000000  00000000  00000036  2**0
                  CONTENTS, ALLOC, LOAD, DATA
  2 .bss          00000000  00000000  00000000  00000036  2**0
                  ALLOC
  3 .vectors      000001ac  00000000  00000000  00000038  2**2
                  CONTENTS, ALLOC, LOAD, RELOC, DATA
  4 .comment      0000004a  00000000  00000000  000001e4  2**0
                  CONTENTS, READONLY
  5 .ARM.attributes 0000002e  00000000  00000000  0000022e  2**0
                  CONTENTS, READONLY
```

Note that VMA/LMA addresses for sections are set to 0 - meaning, `main.o`
is not yet a complete firmware, because it does not contain the information
where those section should be loaded in the address space. We need to use
a linker to produce a full firmware `firmware.elf` from `main.o`.

The section .text contains firmware code, in our case it is just a _reset()
function, 2-bytes long - a jump instruction to its own address. There is
an empty `.data` section and an empty `.bss` section
(data that is initialized to zero) . Our firmware will be copied
to the flash region at offset 0x8000000, but our data section should reside
in RAM - therefore our `_reset()` function should copy the contents of the
`.data` section to RAM. Also it has to write zeroes to the whole `.bss`
section. Our `.data` and `.bss` sections are empty, but let's modify our
`_reset()` function anyway to handle them properly.

Also, our `_reset()` function should set the initial stack pointer, cause 
our vector table has zero in the corresponding entry at index 0.

In order to do all that, we must know where stack starts, and where data
and bss section start. This we can specify in the "linker script", which is
a file with the instructions to the linker, where to put various sections
in the address space, and which symbols to create.

### Linker script

Create a minimal linker script `link.ld`, and copy-paste contents from
[minimal/link.ld](minimal/link.ld). Below is the explanation:

```
ENTRY(_reset);
```

This line tells the linker, that the program's entry point.

```
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 2048k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 192k  /* remaining 64k in a separate address space */
}
```
This tells the linker that we have two memory regions in the address space,
their addresses and sizes.

```
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */
```

This tell a linker to create a symbol `estack` with value at the very end
of the RAM memory region. That will be our initial stack value!

```
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash
```

These lines tell the linker to put vectors table on flash first,
followed by `.text` section (firmware code), followed by the read only
data `.rodata`.

The next goes `.data` section:

```
  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);
```

Note that we tell linker to create `_sdata` and `_edata` symbols. We'll
use them to copy data section to RAM in the `_reset()` function.

Same for `.bss` section:

```
  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram
```

Now we can update our `_reset()` function. We initialize stack pointer,
copy data section to RAM, and initialise bss section to zeroes. Then, we
call main() function - and fall into infinite loop in case if main() returns:

```c
int main(void) {
  return 0; // Do nothing
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  asm("ldr sp, = _estack");  // Set initial stack pointer

  // Initialise memory
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  // Call main()
  main();
  for (;;) (void) 0;  // Infinite loop
}
```

Now we are ready to produce a full firmware file `firmware.elf`:

```sh
$ arm-none-eabi-gcc -Tminimal/link.ld -nostdlib main.o -o firmware.elf
```

Let's examine sections in firmware.elf:

```sh
$ arm-none-eabi-objdump -h firmware.elf
...
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vectors      000001ac  08000000  08000000  00010000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  1 .text         00000058  080001ac  080001ac  000101ac  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
...
```

Now we can see that the .vectors section will be loaded at address 0x8000000,
and .text section right after it, at 0x80001ac. Our code does not create any
variables, so there is no data section.

## Flash firmware

We're ready to flash this firmware! First, extract sections from the
firmware.elf into a single contiguous binary blob:

```sh
$ arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
```

And use `st-link` utility to flash the firmware.bin. Plug your board to the
USB, and execute:

```sh
$ st-flash --reset write firmware.bin 0x8000000
```

Done! We've flashed a firmware that does nothing.

## Makefile: build automation

Instead of typing those compilation, linking and flashing commands, we can
use `make` command line tool to automate the whole process. `make` utility
uses a configuration file named `Makefile` where it reads instructions
how to execute actions. This automation is great because it also documents the
process of building firmware, used compilation flags, etc.

The `Makefile` format is simple:

```make
action1:
	command ...     # Comments can go after hash symbol
	command ....    # IMPORTANT: command must be preceded with the TAB character

action2:
	command ...     # Don't forget about TAB. Spaces won't work!
```

Now, we can invoke `make` with the action name (also called *target*) to execute
a corresponding action:

```sh
$ make action1
```

It is possible to define variables and use them in commands. Also, actions
can be file names that needs to be created:

```make
firmware.elf:
	COMPILATION COMMAND .....
```

And, any action can have a list of dependencies. For example, `firmware.elf`
depends on our source file `main.c`. Whenever `main.c` file changes, the
`make build` command rebuilds `firmware.elf`:

```
build: firmware.elf

firmware.elf: main.c
	COMPILATION COMMAND
```

Now we are ready to write a Makefile for our firmware. We define a `build`
action / target:

```make
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
						-Wformat-truncation -fno-common -Wconversion \
						-g3 -Os -ffunction-sections -fdata-sections -I. \
						-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c 

build: firmware.elf

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
```

There, we define compilation flags. The `?=` means that's a default value;
we could override them from the command line like this:

```sh
$ make build CFLAGS="-O2 ...."
```

We specify `CFLAGS`, `LDFLAGS` and `SOURCES` variables.
Then we tell `make`: if you're told to `build`, then create a `firmware.elf`
file. It depends on the `main.c` file, and to create it, start
`arm-none-eabi-gcc` compiler with a given flags. `$@` special variable
expands to a target name - in our case, `firmware.elf`.

Let's call `make`:

```
$ make build
arm-none-eabi-gcc main.c  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion -g3 -Os -ffunction-sections -fdata-sections -I. -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16  -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=firmware.elf.map -o firmware.elf
```

If we run it again:

```sh
$ make build
make: Nothing to be done for `build'.
```

The `make` utility examines modification times for `main.c` dependency and
`firmware.elf` - and does not do anything if `firmware.elf` is up to date.
But if we change `main.c`, then next `make build` will recompile:

```sh
$ touch main.c # Simulate changes in main.c
$ make build
```

Now, what is left - is the `flash` target:


```make
firmware.bin: firmware.elf
	$(DOCKER) $(CROSS)-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $(TARGET).bin 0x8000000
```

That's it! Now, `make flash` terminal command creates a `firmware.bin` file,
and flashes it to the board. It'll recompile the firmware if `main.c` changes,
because `firmware.bin` depends on `firmware.elf`, and it in turn depends on
`main.c`. So, now the development cycle would be these two actions in a loop:

```sh
# Develop code in main.c
$ make flash
```

It is a good idea to add a clean target to remove build artifacts:


```
clean:
	rm -rf firmware.*
```

That's it. The minimal `main.c`, `link.ld` and `Makefile` is in `minimal/` folder.

## Blinky LED

Now as we have the whole build / flash infrastructure set up, it is time to
teach our firmware to do something useful. Something useful is of course blinking
an LED. A Nucleo-F429ZI board has three built-in LEDs. We can open
a [Nucleo-F429ZI datasheet](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf)
and look into section 6.5, which tells which pins built-in LEDs are attached to:

- PB0: green LED
- PB7: blue LED
- PB14: red LED

Let's modify `main.c` file and add our definitions for PIN, `gpio_set_mode()`.
In the main() function, we set the blue LED to output mode, and start an
infinite loop:

```c
#include <inttypes.h>

#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);    // Set new mode
}

int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  for (;;) asm volatile("nop");          // Infinite loop
  return 0;
}
```

Now, what is left to do, is to find out how to set a GPIO pin on and off, and
then modify the main loop to set an LED pin on, delay, off, delay.  Looking at
the datasheet section 8.4.7, wee see that the register BSRR is responsible for
setting voltage high or low. The low 16 bit are used to set the ODR register
(i.e. set pin high), and high 16 bit are used  to reset the ODR
register (i.e. set pin low). Let's define an API function for that:

```c
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}
```

Next we need to implement a delay function. We do not require an accurate
delay at this moment, so let's define a function `spin()` that just executes
a NOP instruction a given number of times:

```c
static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}
```

Finally, we're ready to modify our main loop to implement LED blinking:

```c
  for (;;) {
    gpio_write(pin, true);
    spin(99999);
    gpio_write(pin, false);
    spin(99999);
  }
```

Run `make flash` and enjoy blue LED flashing.
A complete project source code you can find in a directory [blinky](blinky).

## Blinky with SysTick interrupt

In order to implement an accurate time keeping, we should enable ARM's SysTick
interrupt. SysTick is a configurable counter; it decrements its value 

## Add UART debug output

### GCC, newlib and syscalls: redirect printf() to UART

## Blinky using CMSIS headers
