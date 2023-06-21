// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#define TM4C1294NCPDT
#include "TM4C129.h"

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((bank << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) pinbank(pin >> 8)
// This MCU doesn't have GPIOI nor GPIOO
static inline unsigned int pinbank(unsigned int bank) {
  bank = bank > 'O' ? bank - 2 : bank > 'I' ? bank - 1 : bank;
  return bank - 'A';
}
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)

// 5.5, Table 5-12: configure flash (and EEPROM) timing in accordance to clock
// freq
enum {
  PLL_CLK = 25,
  PLL_M = 96,
  PLL_N = 5,
  PLL_Q = 1,
  PSYSDIV = 4
};  // Run at 120 Mhz
#define PLL_FREQ (PLL_CLK * PLL_M / PLL_N / PLL_Q / PSYSDIV)
#define FLASH_CLKHIGH 6
#define FLASH_WAITST 5
#define FREQ (PLL_FREQ * 1000000)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SysTick->LOAD = ticks - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
}

#define GPIO(bank) ((GPIOA_AHB_Type *) (GPIOA_AHB_BASE + 0x1000U * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_HIGH };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };

static inline void gpio_write(uint16_t pin, bool val) {
  GPIOA_AHB_Type *gpio = GPIO(PINBANK(pin));
  // CMSIS forces 0xFF mask when writing to DATA (see 10.6 in datasheet) and
  // does not seem to support that feature for writing by defining RESERVED0 to read-only
  volatile uint32_t *GPIODATA = (volatile uint32_t *)gpio->RESERVED0;
  uint8_t mask = BIT(PINNO(pin));
  GPIODATA[mask] = val ? mask : 0;
}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIOA_AHB_Type *gpio = GPIO(PINBANK(pin));
  uint8_t n = (uint8_t) (PINNO(pin));
  SYSCTL->RCGCGPIO |= BIT(PINBANK(pin));  // Enable GPIO clock
  if (mode == GPIO_MODE_ANALOG) {
    gpio->AMSEL |= BIT(PINNO(pin));
    return;
  }
  if (mode == GPIO_MODE_INPUT) {
    gpio->DIR &= ~BIT(PINNO(pin));
  } else if (mode == GPIO_MODE_OUTPUT) {
    gpio->DIR |= BIT(PINNO(pin));
  } else {  // GPIO_MODE_AF
    SETBITS(gpio->PCTL, 15UL << ((n & 7) * 4),
            ((uint32_t) af) << ((n & 7) * 4));
    gpio->AFSEL |= BIT(PINNO(pin));
  }
  gpio->DEN |= BIT(PINNO(pin));  // Enable pin as digital function
  if (type == GPIO_OTYPE_OPEN_DRAIN)
    gpio->ODR |= BIT(PINNO(pin));
  else  // GPIO_OTYPE_PUSH_PULL
    gpio->ODR &= ~BIT(PINNO(pin));
  if (speed == GPIO_SPEED_LOW)
    gpio->SLR |= BIT(PINNO(pin));
  else  // GPIO_SPEED_HIGH
    gpio->SLR &= ~BIT(PINNO(pin));
  if (pull == GPIO_PULL_UP) {
    gpio->PUR |= BIT(PINNO(pin));  // setting one...
  } else if (pull == GPIO_PULL_DOWN) {
    gpio->PDR |= BIT(PINNO(pin));  // ...just clears the other
  } else {
    gpio->PUR &= ~BIT(PINNO(pin));
    gpio->PDR &= ~BIT(PINNO(pin));
  }
}

#define UART_OFFSET 0x1000
#define UART(N) ((UART0_Type *) (UART0_BASE + UART_OFFSET * (N)))
#define UARTNO(u) ((uint8_t)(((unsigned int) (u) - UART0_BASE) / UART_OFFSET))

static inline void uart_init(UART0_Type *uart, unsigned long baud) {
  struct uarthw {
    uint16_t rx, tx;  // pins
    uint8_t af;       // Alternate function
  };
  // af, rx, tx for UART0
  struct uarthw uarthw[1] = {{PIN('A', 0), PIN('A', 1), 1}};

  if (uart != UART0) return;  // uarthw is not populated for other UARTs

  uint8_t uartno = UARTNO(uart);
  SYSCTL->RCGCUART |= BIT(uartno);  // Enable peripheral clock

  gpio_init(uarthw[uartno].tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL,
            GPIO_SPEED_HIGH, 0, uarthw[uartno].af);
  gpio_init(uarthw[uartno].rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL,
            GPIO_SPEED_HIGH, 0, uarthw[uartno].af);
  // (16.3.2) ClkDiv = 16 (HSE=0)
  // BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)
  // UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
  // must write in this order
  uart->CTL = 0;                    // Disable this UART, clear HSE
  uart->IBRD = FREQ / (16 * baud);  // Baud rate, integer part
  uart->FBRD =
      ((FREQ % (16 * baud)) >> 26) & 0x3F;    // Baud rate, fractional part
  uart->LCRH = (3 << 5);                  // 8N1, no FIFOs;
  uart->CTL |= BIT(0) | BIT(9) | BIT(8);  // Set UARTEN, RXE, TXE
}
static inline void uart_write_byte(UART0_Type *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->FR & BIT(7)) == 0) spin(1);
}
static inline void uart_write_buf(UART0_Type *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(UART0_Type *uart) {
  return uart->FR & BIT(6);  // If RXFF bit is set, data is ready
}
static inline uint8_t uart_read_byte(UART0_Type *uart) {
  return (uint8_t) (uart->DR & 0xFF);
}

static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

static inline void clock_init(void) {                 // Set clock frequency
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // Enable FPU
  SETBITS(SYSCTL->MOSCCTL, BIT(3) | BIT(2),
          BIT(4));  // Enable MOSC circuit (clear NOXTAL and PWRDN, set >10MHz
                    // range)
  SETBITS(SYSCTL->MEMTIM0,
          BIT(21) | BIT(5) | 0x1F << 21 | 0xF << 16 | 0x1F << 5 | 0xF << 0,
          FLASH_CLKHIGH << 22 | FLASH_WAITST << 16 | FLASH_CLKHIGH << 5 |
              FLASH_WAITST << 0);  // Configure flash timing (not yet applied)
  SETBITS(SYSCTL->RSCLKCFG, 0xF << 24 | (BIT(9) - 1),
          3 << 24);  // Clear PLL divider, set MOSC as PLL source
  SYSCTL->PLLFREQ1 = (PLL_Q - 1) << 8 | (PLL_N - 1)
                                            << 0;  // Set PLL_Q and PLL_N
  SYSCTL->PLLFREQ0 =
      BIT(23) | PLL_M << 0;  // Set PLL_Q, power up PLL (if it were on, we'd
                             // need to set NEWFREQ in RSCLKCFG instead)
  while ((SYSCTL->PLLSTAT & BIT(0)) == 0) spin(1);  // Wait for lock
  SYSCTL->RSCLKCFG |=
      BIT(31) | BIT(28) |
      (PSYSDIV - 1) << 0;  // Update memory timing, use PLL, set clock divisor
}
