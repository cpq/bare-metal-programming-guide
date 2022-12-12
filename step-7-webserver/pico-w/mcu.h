// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "rp2040.h"

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) ((pin) &255)
#define PINBANK(pin) ((pin) >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

struct iobank {
  struct {
    volatile uint32_t STATUS, CTRL;
  } gpio[30];
};
#define IOBANK0 ((struct iobank *) IO_BANK0_BASE)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  int no = PINNO(pin);
  if (mode == GPIO_MODE_INPUT) {
    SIO->GPIO_OE_CLR = BIT(no);
  } else if (mode == GPIO_MODE_OUTPUT) {
    SIO->GPIO_OE_SET = BIT(no);
  }
  IOBANK0->gpio[no].CTRL = 5;
}

static inline void gpio_write(uint16_t pin, bool val) {
  if (val) {
    SIO->GPIO_OUT_SET = BIT(PINNO(pin));
  } else {
    SIO->GPIO_OUT_CLR = BIT(PINNO(pin));
  }
}

// This snippet of code is (c) Alex Taradov,
// under the BSD-3-Clause
#define F_REF 12000000
#define F_TICK 1000
static inline void clock_init(void) {
  // Enable XOSC
  XOSC->CTRL = (XOSC_CTRL_FREQ_RANGE_1_15MHZ << XOSC_CTRL_FREQ_RANGE_Pos);
  XOSC->STARTUP = 47;  // ~1 ms @ 12 MHz
  XOSC_SET->CTRL = (XOSC_CTRL_ENABLE_ENABLE << XOSC_CTRL_ENABLE_Pos);
  while (0 == (XOSC->STATUS & XOSC_STATUS_STABLE_Msk))
    ;

  // Setup SYS PLL for 12 MHz * 40 / 4 / 1 = 120 MHz
  RESETS_CLR->RESET = RESETS_RESET_pll_sys_Msk;
  while (0 == RESETS->RESET_DONE_b.pll_sys)
    ;

  PLL_SYS->CS = (1 << PLL_SYS_CS_REFDIV_Pos);
  PLL_SYS->FBDIV_INT = 40;
  PLL_SYS->PRIM =
      (4 << PLL_SYS_PRIM_POSTDIV1_Pos) | (1 << PLL_SYS_PRIM_POSTDIV2_Pos);

  PLL_SYS_CLR->PWR = PLL_SYS_PWR_VCOPD_Msk | PLL_SYS_PWR_PD_Msk;
  while (0 == PLL_SYS->CS_b.LOCK)
    ;

  PLL_SYS_CLR->PWR = PLL_SYS_PWR_POSTDIVPD_Msk;

  // Setup USB PLL for 12 MHz * 36 / 3 / 3 = 48 MHz
  RESETS_CLR->RESET = RESETS_RESET_pll_usb_Msk;
  while (0 == RESETS->RESET_DONE_b.pll_usb)
    ;

  PLL_USB->CS = (1 << PLL_SYS_CS_REFDIV_Pos);
  PLL_USB->FBDIV_INT = 36;
  PLL_USB->PRIM =
      (3 << PLL_SYS_PRIM_POSTDIV1_Pos) | (3 << PLL_SYS_PRIM_POSTDIV2_Pos);

  PLL_USB_CLR->PWR = PLL_SYS_PWR_VCOPD_Msk | PLL_SYS_PWR_PD_Msk;
  while (0 == PLL_USB->CS_b.LOCK)
    ;

  PLL_USB_CLR->PWR = PLL_SYS_PWR_POSTDIVPD_Msk;

  // Switch clocks to their final socurces
  CLOCKS->CLK_REF_CTRL =
      (CLOCKS_CLK_REF_CTRL_SRC_xosc_clksrc << CLOCKS_CLK_REF_CTRL_SRC_Pos);

  CLOCKS->CLK_SYS_CTRL = (CLOCKS_CLK_SYS_CTRL_AUXSRC_clksrc_pll_sys
                          << CLOCKS_CLK_SYS_CTRL_AUXSRC_Pos);
  CLOCKS_SET->CLK_SYS_CTRL = (CLOCKS_CLK_SYS_CTRL_SRC_clksrc_clk_sys_aux
                              << CLOCKS_CLK_SYS_CTRL_SRC_Pos);

  CLOCKS->CLK_PERI_CTRL =
      CLOCKS_CLK_PERI_CTRL_ENABLE_Msk |
      (CLOCKS_CLK_PERI_CTRL_AUXSRC_clk_sys << CLOCKS_CLK_PERI_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_USB_CTRL = CLOCKS_CLK_USB_CTRL_ENABLE_Msk |
                         (CLOCKS_CLK_USB_CTRL_AUXSRC_clksrc_pll_usb
                          << CLOCKS_CLK_USB_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_ADC_CTRL = CLOCKS_CLK_ADC_CTRL_ENABLE_Msk |
                         (CLOCKS_CLK_ADC_CTRL_AUXSRC_clksrc_pll_usb
                          << CLOCKS_CLK_ADC_CTRL_AUXSRC_Pos);

  CLOCKS->CLK_RTC_DIV =
      (256 << CLOCKS_CLK_RTC_DIV_INT_Pos);  // 12MHz / 256 = 46875 Hz
  CLOCKS->CLK_RTC_CTRL =
      CLOCKS_CLK_RTC_CTRL_ENABLE_Msk | (CLOCKS_CLK_RTC_CTRL_AUXSRC_xosc_clksrc
                                        << CLOCKS_CLK_ADC_CTRL_AUXSRC_Pos);

  // Configure 1 ms tick for watchdog and timer
  WATCHDOG->TICK =
      ((F_REF / F_TICK) << WATCHDOG_TICK_CYCLES_Pos) | WATCHDOG_TICK_ENABLE_Msk;

  // Enable GPIOs
  RESETS_CLR->RESET = RESETS_RESET_io_bank0_Msk | RESETS_RESET_pads_bank0_Msk;
  while (0 == RESETS->RESET_DONE_b.io_bank0 ||
         0 == RESETS->RESET_DONE_b.pads_bank0)
    ;
}
