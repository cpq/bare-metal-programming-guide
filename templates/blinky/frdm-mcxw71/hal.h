// Copyright (c) 2025 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "MCXW716C.h"
#include "MCXW716C_COMMON.h"

#define BIT(x) (1UL << (x))
#define CLRSET(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

#define SYS_FREQUENCY 6000000

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };

static inline GPIO_Type *gpio_bank(uint16_t pin) {
  uint8_t bank = PINBANK(pin);
  return (GPIO_Type *) (bank == 0   ? GPIOA
                        : bank == 1 ? GPIOB
                        : bank == 2 ? GPIOC
                        : bank == 3 ? GPIOD
                                    : 0);
}

static inline void gpio_toggle(uint16_t pin) {
  GPIO_Type *gpio = gpio_bank(pin);
  gpio->PTOR |= BIT(PINNO(pin));
}

static inline bool gpio_read(uint16_t pin) {
  GPIO_Type *gpio = gpio_bank(pin);
  return gpio->PDR[PINNO(pin)];
}

static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_Type *gpio = gpio_bank(pin);
  if (val) {
    // gpio->PSOR |= BIT(PINNO(pin));
    gpio->PDR[PINNO(pin)] = 1U;
  } else {
    // gpio->PCOR |= BIT(PINNO(pin));
    gpio->PDR[PINNO(pin)] = 0;
  }
}

static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIO_Type *gpio = gpio_bank(pin);
  (void) pin, (void) mode, (void) type, (void) speed, (void) pull, (void) af;
#if 0
  if (gpio == GPIOA) {
    CLRSET(MRCC->MRCC_GPIOA, MRCC_MRCC_GPIOA_CC_MASK, 1U);
  } else if (gpio == GPIOB) {
    CLRSET(MRCC->MRCC_GPIOB, MRCC_MRCC_GPIOB_CC_MASK, 1U);
  } else if (gpio == GPIOC) {
    CLRSET(MRCC->MRCC_GPIOC, MRCC_MRCC_GPIOC_CC_MASK, 1U);
  }
  __ISB();
  __DSB();
#endif
  if (mode == GPIO_MODE_OUTPUT) {
    // gpio->PDOR &= ~BIT(PINNO(pin));
    gpio->PDOR |= BIT(PINNO(pin));
    gpio->PDDR |= BIT(PINNO(pin));
  } else {
    gpio->PDDR &= ~BIT(PINNO(pin));
  }
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

static inline void uart_init(LPUART_Type *uart, unsigned long baud) {
  uint32_t val = 0x80000000U;  // Module present
  val |= 0x40000000U;          // Release from reset
  val |= 1U;                   // Enable clock
  val |= 2U << 4;              // Clock mux: FRO-6M
  if (uart == LPUART0) {
    MRCC->MRCC_LPUART0 = val;
  } else if (uart == LPUART1) {
    MRCC->MRCC_LPUART1 = val;
  }
  (void) baud;
}

static inline void uart_write_byte(LPUART_Type *uart, uint8_t byte) {
  // while ((uart->FIFOSTAT & UART_FIFOSTAT_TXNOTFULL_MASK) == 0) spin(1);
  return;
  uart->FIFO = byte;
}
static inline void uart_write_buf(LPUART_Type *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(volatile uint64_t *t, uint64_t prd,
                                 uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
