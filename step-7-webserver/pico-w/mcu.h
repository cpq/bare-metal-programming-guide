// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) ((pin) &255)
#define PINBANK(pin) ((pin) >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

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
#define XIP_SSI ((struct xip_ssi *) 0x18000000UL)

struct xosc {
  volatile uint32_t CTRL, STATUS, DORMANT, STARTUP, RESERVED[3], COUNT;
};
#define XOSC ((struct xosc *) 0x40024000UL)
#define XOSC_SET ((struct xosc *) 0x40026000UL)

struct pll {
  volatile uint32_t CS, PWR, FBDIV_INT, PRIM;
};
#define PLL_SYS ((struct pll *) 0x40028000UL)
#define PLL_USB ((struct pll *) 0x4002c000UL)
#define PLL_SYS_CLR ((struct pll *) 0x4002b000UL)
#define PLL_USB_CLR ((struct pll *) 0x4002f000UL)

struct clocks {
  struct {
    volatile uint32_t CTRL, DIV, SELECTED;
  } GPOUT0, GPOUT1, GPOUT2, GPOUT3, REF, SYS, PERI, USB, ADC, RTC;
  volatile uint32_t SYS_RESUS_CTRL, SYS_RESUS_STATUS, FC0_REF_KHZ, FC0_MIN_KHZ,
      FC0_MAX_KHZ, FC0_DELAY, FC0_INTERVAL, FC0_SRC, FC0_STATUS, FC0_RESULT,
      WAKE_EN0, WAKE_EN1, SLEEP_EN0, SLEEP_EN1, ENABLED0, ENABLED1, INTR, INTE,
      INTF, INTS;
};
#define CLOCKS ((struct clocks *) 0x40008000UL)
#define CLOCKS_SET ((struct clocks *) 0x4000a000UL)

struct watchdog {
  volatile uint32_t CTRL, LOAD, REASON, SCRATCH[8], TICK;
};
#define WATCHDOG ((struct watchdog *) 0x40058000UL)

struct sio {
  volatile uint32_t CPUID, GPIO_IN, GPIO_HI_IN, RESERVED, GPIO_OUT,
      GPIO_OUT_SET, GPIO_OUT_CLR, GPIO_OUT_XOR, GPIO_OE, GPIO_OE_SET,
      GPIO_OE_CLR, GPIO_OE_XOR, GPIO_HI_OUT, GPIO_HI_OUT_SET, GPIO_HI_OUT_CLR,
      GPIO_HI_OUT_XOR, GPIO_HI_OE, GPIO_HI_OE_SET, GPIO_HI_OE_CLR,
      GPIO_HI_OE_XOR, FIFO_ST, FIFO_WR, FIFO_RD, SPINLOCK_ST, DIV_UDIVIDEND,
      DIV_UDIVISOR, DIV_SDIVIDEND, DIV_SDIVISOR, DIV_QUOTIENT, DIV_REMAINDER;
};
#define SIO ((struct sio *) 0xd0000000UL)

struct resets {
  volatile uint32_t RESET, WDSEL, DONE;
};
#define RESETS ((struct resets *) 0x4000c000UL)
#define RESETS_XOR ((struct resets *) 0x4000d000UL)
#define RESETS_SET ((struct resets *) 0x4000e000UL)
#define RESETS_CLR ((struct resets *) (0x4000f000UL))

struct io_bank {
  struct {
    volatile uint32_t STATUS, CTRL;
  } gpio[30];
};
#define IO_BANK0 ((struct io_bank *) 0X40014000UL)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  int no = PINNO(pin);
  if (mode == GPIO_MODE_INPUT) {
    SIO->GPIO_OE_CLR = BIT(no);
  } else if (mode == GPIO_MODE_OUTPUT) {
    SIO->GPIO_OE_SET = BIT(no);
  }
  IO_BANK0->gpio[no].CTRL = 5;
}

static inline void gpio_write(uint16_t pin, bool val) {
  if (val) {
    SIO->GPIO_OUT_SET = BIT(PINNO(pin));
  } else {
    SIO->GPIO_OUT_CLR = BIT(PINNO(pin));
  }
}

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

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

static inline void clock_init(void) {
  XOSC->CTRL = 2720;            // XOSC frequency range 1-15 MHz
  XOSC->STARTUP = 47;           // About 1 ms, see 2.16.3
  XOSC_SET->CTRL = 4011 << 12;  // Enable XOSC
  while ((XOSC->STATUS & BIT(31)) == 0) (void) 0;  // Wait until enabled

  RESETS_CLR->RESET = BIT(12);                     // Reset SYS PLL
  while ((RESETS->DONE & BIT(12)) == 0) (void) 0;  // Wait
  PLL_SYS->FBDIV_INT = F_SYS_FBDIV;
  PLL_SYS->PRIM = (F_SYS_POSTDIV1 << 16) | (F_SYS_POSTDIV2 << 12);
  PLL_SYS_CLR->PWR = BIT(0) | BIT(3) | BIT(5);    // Power up
  while ((PLL_SYS->CS & BIT(31)) == 0) (void) 0;  // Wait

  RESETS_CLR->RESET = BIT(13);                     // Reset USB PLL
  while ((RESETS->DONE & BIT(13)) == 0) (void) 0;  // Wait
  PLL_USB->FBDIV_INT = F_USB_FBDIV;
  PLL_USB->PRIM = (F_USB_POSTDIV1 << 16) | (F_USB_POSTDIV2 << 12);
  PLL_USB_CLR->PWR = BIT(0) | BIT(3) | BIT(5);    // Power up
  while ((PLL_USB->CS & BIT(31)) == 0) (void) 0;  // Wait

  CLOCKS->REF.CTRL = (2 << 0);             // REF source is PLL
  CLOCKS->SYS.CTRL = (0 << 5) | (1 << 0);  // SYS source is PLL
  CLOCKS->PERI.CTRL = BIT(11) | (0 << 5);  // PERI clock enable, source SYS
  CLOCKS->USB.CTRL = BIT(11) | (0 << 5);   // USB clock enable, source SYS
  CLOCKS->ADC.CTRL = BIT(11) | (0 << 5);   // ADC clock enable, source SYS
  CLOCKS->RTC.DIV = (48 << 8);             // RTC divider: 12 / 48 = 0.25Mhz
  CLOCKS->RTC.CTRL = BIT(11) | (3 << 5);   // RTC clock enable, source XOSC

  RESETS_CLR->RESET = BIT(5) | BIT(8);  // Enable GPIO: BANK0, PADS_BANK0
  while ((RESETS->DONE & (BIT(5) | BIT(8))) == 0) (void) 0;  // Wait

  SYSTICK->LOAD = F_SYS / 1000 - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable SysTick
}
