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
32-bit fields. Let's see what are the first three registers for the GPIO
peripheral in the section 8.4 of the datasheet. They are MODER, OTYPER, OSPEEDR
with offsets 0, 4, 8 respectively. That means we can represent them as
a structure, and make a define for GPIOA:

```c
struct gpio {
  volatile uin32_t MODER, OTYPER, OSPEEDR;  // There are more registers ...
};

#define GPIOA ((struct gpio *) 0x40020000)
```

Then, for setting GPIO pin mode, we can define a function:

```c
// Enum values will be per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

static void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode) {
  gpio->MODER &= ~(3U << (pin * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (pin * 2);   // Set new mode
}
```

Now, we can rewrite the snippet for A3 like this:

```c
gpio_set_mode(GPIOA, 3, GPIO_MODE_OUTPUT);  // Set A3 to output
```

## MCU boot process

## Minimal firmware

### Compilation
### Linker script

## Makefile: build automation

## GCC, newlib and syscalls

