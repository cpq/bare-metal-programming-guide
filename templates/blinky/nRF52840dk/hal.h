#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nrf52.h"

#define BIT(x) (1UL << (x))
#define CLRSET(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

#define GPIO(N) ((NRF_GPIO_Type *) ((NRF_P0_BASE) + 0x300 * (N)))

static inline NRF_GPIO_Type *gpio_bank(uint16_t pin) {
  return GPIO(PINBANK(pin));
}

enum { GPIO_MODE_INPUT = 0, GPIO_MODE_OUTPUT = 1 };
enum { GPIO_PULL_NONE = 0, GPIO_PULL_DOWN = 1, GPIO_PULL_UP = 3U };

void gpio_init(uint16_t pin, uint8_t mode, uint8_t pull) {
  NRF_GPIO_Type *gpio = gpio_bank(pin);
  gpio->PIN_CNF[pin] = (mode & 1U) << 0 | (pull & 3U) << 2U;
}

static inline bool gpio_read(uint16_t pin) {
  NRF_GPIO_Type *gpio = gpio_bank(pin);
  return ~(gpio->IN) & BIT(PINNO(pin));
}

static inline void gpio_write(uint16_t pin, bool value) {
  NRF_GPIO_Type *gpio = gpio_bank(pin);
  if (value) {
    gpio->OUTSET = BIT(PINNO(pin));
  } else {
    gpio->OUTCLR = BIT(PINNO(pin));
  }
}
