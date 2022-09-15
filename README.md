# Bare metal programming guide

This guide is written for beginners who wish to start programming
microcontrollers using GCC compiler and bare metal approach.  We are going to
use a
[Nucleo-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)
development board with STM32F429 microcontroller. But basic principles would be
applicable to any other microcontroller. Install the following toosl:

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

For example, a GPIO peripheral is a General Input-Output, it allows to
manipulate MCU pins by writing and reading to GPIO registers. 
The GPIOA peripheral starts at 0x40020000, and we can find GPIO register
description in section 8. Later on, we'll learn which registers and which 
values we should write to, to set pin voltage high or low - in other words,
how to make an LED blink.

Some registers are not mapped to the MCU peripherals, but they are mapped to
the ARM CPU configuration and control. For example, there is a "Reset at clock control"
unit, described in section 6 of the datasheet. It describes registers that
allow to set systems clock and other things.


