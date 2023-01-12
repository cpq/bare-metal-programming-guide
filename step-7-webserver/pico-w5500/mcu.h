// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) ((pin) &255)
#define PINBANK(pin) ((pin) >> 8)

static inline void spin(volatile unsigned long count) {
  while (count--) (void) 0;
}

// System clock
// FREQUENCY = ((FREF / REFDIV) × FBDIV / (POSTDIV1 × POSTDIV2))
//          REF    FBDIV   VCO      PDIV1   PDIV2   RESULT
// SYS PLL: 12MHz * 133 = 1596MHz /   6   /   2  =  133MHz
// USB PLL: 12MHz * 100 = 1200MHz /   5   /   5  =  48MHz
enum { F_REF = 12000000, F_REFDIV = 1 };
enum { F_SYS_FBDIV = 133, F_SYS_POSTDIV1 = 6, F_SYS_POSTDIV2 = 2 };
enum { F_USB_FBDIV = 100, F_USB_POSTDIV1 = 5, F_USB_POSTDIV2 = 5 };
#define F_SYS \
  ((F_REF / F_REFDIV) * F_SYS_FBDIV / (F_SYS_POSTDIV1 * F_SYS_POSTDIV2))
#define F_USB \
  ((F_REF / F_REFDIV) * F_USB_FBDIV / (F_USB_POSTDIV1 * F_USB_POSTDIV2))

struct nvic {
  volatile uint32_t ISER[1], RESERVED0[31], ICER[1], RSERVED1[31], ISPR[1],
      RESERVED2[21], ICPR[1], RESERVED3[31], RESERVED4[64], IP[8];
};
#define NVIC ((struct nvic *) 0xe000e100)
static inline void nvic_set_prio(uint8_t irq, uint8_t prio) {
  uint32_t shift = ((uint32_t) irq & 3U) * 8U;
  NVIC->IP[irq >> 2] = ((uint32_t) (NVIC->IP[irq >> 2] & ~(255U << shift)) |
                        (((prio << 6U) & (uint32_t) 255UL) << shift));
}
static inline void nvic_enable_irq(uint8_t irq) { NVIC->ISER[0] = BIT(irq); }

struct scb {
  volatile uint32_t CPUID, ICSR, VTOR, AIRCR, SCR, CCR;  // Incomplete
};
#define SCB ((struct scb *) 0xe000ed00)

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)

struct xip_ssi {
  volatile uint32_t CTRLR0, CTRLR1, SSIENR, MWCR, SER, BAUDR, TXFTLR, RXFTLR,
      TXFLR, RXFLR, SR, IMR, ISR, RISR, TXOICR, RXOICR, RXUICR, MSTICR, ICR,
      DMACR, DMATDLR, DMARDLR, IDR, SSI_VERSION_ID, DR0, RESERVED[35],
      RX_SAMPLE_DLY, SPI_CTRLR0, TXD_DRIVE_EDGE;
};
#define XIP_SSI ((struct xip_ssi *) 0x18000000)

struct xosc {
  volatile uint32_t CTRL, STATUS, DORMANT, STARTUP, RESERVED[3], COUNT;
};
#define XOSC ((struct xosc *) 0x40024000)

struct pll {
  volatile uint32_t CS, PWR, FBDIV_INT, PRIM;
};
#define PLL_SYS ((struct pll *) 0x40028000)
#define PLL_USB ((struct pll *) 0x4002c000)

struct clocks {
  struct {
    volatile uint32_t CTRL, DIV, SELECTED;
  } GPOUT0, GPOUT1, GPOUT2, GPOUT3, REF, SYS, PERI, USB, ADC, RTC;
  volatile uint32_t SYS_RESUS_CTRL, SYS_RESUS_STATUS, FC0_REF_KHZ, FC0_MIN_KHZ,
      FC0_MAX_KHZ, FC0_DELAY, FC0_INTERVAL, FC0_SRC, FC0_STATUS, FC0_REST,
      WAKE_EN0, WAKE_EN1, SLEEP_EN0, SLEEP_EN1, ENABLED0, ENABLED1, INTR, INTE,
      INTF, INTS;
};
#define CLOCKS ((struct clocks *) 0x40008000)

struct watchdog {
  volatile uint32_t CTRL, LOAD, REASON, SCRATCH[8], TICK;
};
#define WATCHDOG ((struct watchdog *) 0x40058000)

struct sio {
  volatile uint32_t CPUID, GPIO_IN, GPIO_HI_IN, RESERVED, GPIO_OUT,
      GPIO_OUT_SET, GPIO_OUT_CLR, GPIO_OUT_XOR, GPIO_OE, GPIO_OE_SET,
      GPIO_OE_CLR, GPIO_OE_XOR, GPIO_HI_OUT, GPIO_HI_OUT_SET, GPIO_HI_OUT_CLR,
      GPIO_HI_OUT_XOR, GPIO_HI_OE, GPIO_HI_OE_SET, GPIO_HI_OE_CLR,
      GPIO_HI_OE_XOR, FIFO_ST, FIFO_WR, FIFO_RD, SPINLOCK_ST, DIV_UDIVIDEND,
      DIV_UDIVISOR, DIV_SDIVIDEND, DIV_SDIVISOR, DIV_QUOTIENT, DIV_REMAINDER;
};
#define SIO ((struct sio *) 0xd0000000)

struct resets {
  volatile uint32_t RESET, WDSEL, DONE;
};
#define RESETS ((struct resets *) 0x4000c000)

struct io_bank {
  struct {
    volatile uint32_t STATUS, CTRL;
  } gpio[30];
  volatile uint32_t INTR[4];
  struct {
    volatile uint32_t INTE[4], INTF[4], INTS[4];
  } PROC[2];
  volatile uint32_t DORMANT_WAKE_INTE[4], DORMANT_WAKE_INTF[4],
      DORMANT_WAKE_INTS[4];
};
#define IO_BANK0 ((struct io_bank *) 0X40014000)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t fn) {
  int no = PINNO(pin);
  if (mode == GPIO_MODE_INPUT) {
    SIO->GPIO_OE &= ~BIT(no);
    IO_BANK0->gpio[no].CTRL = 5;
  } else if (mode == GPIO_MODE_OUTPUT) {
    SIO->GPIO_OE |= BIT(no);
    IO_BANK0->gpio[no].CTRL = 5;
  } else if (mode == GPIO_MODE_AF) {
    IO_BANK0->gpio[no].CTRL = fn;
  }
}

static inline void gpio_write(uint16_t pin, bool val) {
  if (val) {
    SIO->GPIO_OUT |= BIT(PINNO(pin));
  } else {
    SIO->GPIO_OUT &= ~BIT(PINNO(pin));
  }
}

static inline bool gpio_read(uint16_t pin) { return SIO->GPIO_IN & BIT(pin); }

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

static inline void enable_subsystem(uint32_t bits) {
  RESETS->RESET &= ~bits;                       // Unreset subsystem
  while ((RESETS->DONE & bits) == 0) (void) 0;  // Wait until done
}

struct usb_dpram {
  volatile uint32_t SETUP_PACKET[2];
  struct {
    volatile uint32_t IN, OUT;
  } EP_CONTROL[14];
  struct {
    volatile uint32_t IN, OUT;
  } EP_BUFFER_CONTROL[15];
};
#define USB_DPRAM ((struct usb_dpram *) 0x50100000)

struct usb_regs {
  volatile uint32_t ADDR_ENDP[16];
  volatile uint32_t MAIN_CTRL, SOF_WR, SOF_RD, SIE_CTRL, SIE_STATUS,
      INT_EP_CTRL, BUFF_STATUS, BUFF_CPU_SHOULD_HANDLE, EP_ABORT, EP_ABORT_DONE,
      EP_STALL_ARM, NAK_POLL, EP_STATUS_STALL_NAK, MUXING, PWR, PHY_DIRECT,
      PHY_DIRECT_OVERRIDE, PHY_TRIM, RESERVED[1], INTR, INTE, INTF, INTS;
};
#define USB_REGS ((struct usb_regs *) 0x50110000)
#define USB_REGS_SET ((struct usb_regs *) 0x50112000)
#define USB_REGS_CLR ((struct usb_regs *) 0x50113000)

static inline void usb_init(void) {
  RESETS->RESET |= BIT(24);                     // Reset USB
  enable_subsystem(BIT(24));                    // Enable USB
  memset(USB_DPRAM, 0, sizeof(*USB_DPRAM));     // Clear USB controller
  nvic_enable_irq(5);                           // Enable USB IRQ
  USB_REGS->MUXING = BIT(0) | BIT(3);           // TO_PHY | SOFTCON
  USB_REGS->PWR = BIT(2) | BIT(3);              // VBUS_DETECT | OVERRIDE_EN
  USB_REGS->MAIN_CTRL = BIT(0);                 // Enable
  USB_REGS->SIE_CTRL = BIT(29);                 // 1_BUF
  USB_REGS->INTE = BIT(4) | BIT(12) | BIT(16);  // STATUS | RESET | REQ
  USB_REGS->SIE_CTRL = BIT(16);                 // PULLUP_ENABLE
}

struct uart {
  volatile uint32_t DR, RSR, RESERVED[4], FR, RESERVED1[1], ILPR, IBRD, FBRD,
      LCR_H, CR, IFLS, IMSC, RIS, MIS, ICR, DMACR, RESERVED2[997], PERIPHID[4],
      PERIPHCELLID[4];
};
#define UART0 ((struct uart *) 0x40034000)
#define UART1 ((struct uart *) 0x40038000)

static inline void uart_init(struct uart *uart, unsigned baud, uint16_t rx,
                             uint16_t tx) {
  enable_subsystem(uart == UART0 ? BIT(22) : BIT(23));
  uint32_t div = (8 * F_SYS / baud);       // Baud rate divisor
  uart->IBRD = div >> 7;                   // Set integer part
  uart->FBRD = ((div & 0x7f) + 1) / 2;     // Set float part
  uart->LCR_H = BIT(4) | BIT(5) | BIT(6);  // FIFO, 8-bit, 1 stop, no parity
  uart->CR = BIT(0) | BIT(8) | BIT(9);     // Enable, TX, RX
  uart->DMACR = BIT(0) | BIT(1);           // Enable DREQ
  gpio_init(rx, GPIO_MODE_AF, 2);          // Set RX/TX pins to use the UART
  gpio_init(tx, GPIO_MODE_AF, 2);          // alternate function - 2
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  while (uart->FR & BIT(5)) (void) 0;  // Wait while TX FIFO is full
  uart->DR = byte;
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->FR & BIT(4);
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->DR & 255);
}

// Bit-bang SPI implementation
struct spi {
  int miso, mosi, clk, cs;  // SPI Pins
  int spin;                 // Number of NOP spins for bitbanging
};

static inline void spi_begin(struct spi *spi) {
  gpio_write((uint16_t) spi->cs, 0);
}

static inline void spi_end(struct spi *spi) {
  gpio_write((uint16_t) spi->cs, 1);
}

static inline bool spi_init(struct spi *spi) {
  if (spi->miso < 0 || spi->mosi < 0 || spi->clk < 0) return false;
  gpio_init((uint16_t) spi->miso, GPIO_MODE_INPUT, 0);
  gpio_init((uint16_t) spi->mosi, GPIO_MODE_OUTPUT, 0);
  gpio_init((uint16_t) spi->clk, GPIO_MODE_OUTPUT, 0);
  if (spi->cs >= 0) {
    gpio_init((uint16_t) spi->cs, GPIO_MODE_OUTPUT, 0);
    gpio_write((uint16_t) spi->cs, 1);
  }
  return true;
}

// Send a byte, and return a received byte
static inline unsigned char spi_txn(struct spi *spi, unsigned char tx) {
  unsigned count = spi->spin <= 0 ? 9 : (unsigned) spi->spin;
  unsigned char rx = 0;
  for (int i = 0; i < 8; i++) {
    gpio_write((uint16_t) spi->mosi, tx & 0x80);   // Set mosi
    spin(count);                                   // Wait half cycle
    gpio_write((uint16_t) spi->clk, 1);            // Clock high
    rx = (unsigned char) (rx << 1);                // "rx <<= 1" gives warning??
    if (gpio_read((uint16_t) spi->miso)) rx |= 1;  // Read miso
    spin(count);                                   // Wait half cycle
    gpio_write((uint16_t) spi->clk, 0);            // Clock low
    tx = (unsigned char) (tx << 1);                // Again, avoid warning
  }
  return rx;  // Return the received byte
}

static inline void clock_init(void) {
  XOSC->CTRL = 2720;         // XOSC frequency range 1-15 MHz
  XOSC->STARTUP = 47;        // About 1 ms, see 2.16.3
  XOSC->CTRL |= 4011 << 12;  // Enable XOSC
  while ((XOSC->STATUS & BIT(31)) == 0) (void) 0;  // Wait until enabled

  enable_subsystem(BIT(12));  // Reset SYS PLL
  PLL_SYS->FBDIV_INT = F_SYS_FBDIV;
  PLL_SYS->PRIM = (F_SYS_POSTDIV1 << 16) | (F_SYS_POSTDIV2 << 12);
  PLL_SYS->PWR &= ~(BIT(0) | BIT(3) | BIT(5));    // Power up
  while ((PLL_SYS->CS & BIT(31)) == 0) (void) 0;  // Wait

  enable_subsystem(BIT(13));  // Reset USB PLL
  PLL_USB->FBDIV_INT = F_USB_FBDIV;
  PLL_USB->PRIM = (F_USB_POSTDIV1 << 16) | (F_USB_POSTDIV2 << 12);
  PLL_USB->PWR &= ~(BIT(0) | BIT(3) | BIT(5));    // Power up
  while ((PLL_USB->CS & BIT(31)) == 0) (void) 0;  // Wait

  CLOCKS->REF.CTRL = (2 << 0);             // REF source is XOSC
  CLOCKS->SYS.CTRL = (0 << 5) | (1 << 0);  // SYS source is CLKSYS_AUX
  CLOCKS->PERI.CTRL = BIT(11) | (0 << 5);  // PERI clock enable, source SYS
  CLOCKS->USB.CTRL = BIT(11) | (0 << 5);   // USB clock enable, source USB PLL
  CLOCKS->ADC.CTRL = BIT(11) | (0 << 5);   // ADC clock enable, source USB PLL
  CLOCKS->RTC.DIV = (48 << 8);             // RTC divider: 12 / 48 = 0.25Mhz
  CLOCKS->RTC.CTRL = BIT(11) | (3 << 5);   // RTC clock enable, source XOSC

  enable_subsystem(BIT(5) | BIT(8));  // IO_BANK0 and PADS_BANK0

  SYSTICK->LOAD = F_SYS / 1000 - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable SysTick
}
