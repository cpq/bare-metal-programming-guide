// Copyright (c) 2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT
//
// https://www.st.com/resource/en/reference_manual/DM00043574-.pdf
// https://www.st.com/resource/en/datasheet/stm32f303k8.pdf

#ifndef LED_PIN
#define LED_PIN PIN('B', 3)  // Green onboard LED on Nucleo-L432KC
#endif

#pragma once
#include <stm32f303x8.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

// System clock sources
enum { CLK_SRC_HSI, CLK_SRC_HSE, CLK_SRC_PLL_HSI_DIV2, CLK_SRC_PLL_HSE_PREDIV };

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
#define RCC_PLLSOURCE_HSI RCC_CFGR_PLLSRC_HSI_PREDIV
#endif
#if defined(RCC_CFGR_PLLSRC_HSI_DIV2)
#define RCC_PLLSOURCE_HSI RCC_CFGR_PLLSRC_HSI_DIV2
#endif
static inline uint32_t clock_sys_freq(void) {
  uint32_t cfgr = RCC->CFGR, freq = 0U, hsi = 8000000, hse = 16000000;
  switch (cfgr & RCC_CFGR_SWS) {
    case RCC_CFGR_SWS_HSE: freq = hse; break;
    case RCC_CFGR_SWS_PLL: {
      uint64_t pllmul = ((cfgr & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos) + 2;
      uint64_t prediv =
          ((RCC->CFGR2 & RCC_CFGR2_PREDIV) >> RCC_CFGR2_PREDIV_Pos) + 1;
#if defined(RCC_CFGR_PLLSRC_HSI_DIV2)
      if ((cfgr & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI) {
        freq = (uint32_t) ((uint64_t) hse / prediv) * pllmul;
      } else {
        freq = (uint32_t) ((uint64_t) (hsi / 2) * pllmul);
      }
#else
      if ((cfgr & RCC_CFGR_PLLSRC_HSE_PREDIV) == RCC_CFGR_PLLSRC_HSE_PREDIV) {
        freq = (uint32_t) ((uint64_t) hse / prediv) * pllmul;
      } else {
        freq = (uint32_t) ((uint64_t) hsi / prediv) * pllmul;
      }
#endif
      break;
    }
    case RCC_CFGR_SWS_HSI:
    default: freq = hsi; break;
  }
  return freq;
}

static inline uint32_t clock_apb1_freq(void) {
  return clock_sys_freq();
}

static inline uint32_t clock_apb2_freq(void) {
  return clock_sys_freq();
}

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };
#define GPIO(N) ((GPIO_TypeDef *) (GPIOA_BASE + 0x400 * (N)))

static GPIO_TypeDef *gpio_bank(uint16_t pin) {
  return GPIO(PINBANK(pin));
}
static inline void gpio_toggle(uint16_t pin) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint32_t mask = BIT(PINNO(pin));
  gpio->BSRR = mask << (gpio->ODR & mask ? 16 : 0);
}
static inline int gpio_read(uint16_t pin) {
  return gpio_bank(pin)->IDR & BIT(PINNO(pin)) ? 1 : 0;
}
static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  gpio->BSRR = BIT(PINNO(pin)) << (val ? 0 : 16);
}
static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint8_t n = (uint8_t) (PINNO(pin));
  RCC->AHBENR |= BIT(PINBANK(pin) + RCC_AHBENR_GPIOAEN_Pos);
  SETBITS(gpio->OTYPER, 1UL << n, ((uint32_t) type) << n);
  SETBITS(gpio->OSPEEDR, 3UL << (n * 2), ((uint32_t) speed) << (n * 2));
  SETBITS(gpio->PUPDR, 3UL << (n * 2), ((uint32_t) pull) << (n * 2));
  SETBITS(gpio->AFR[n >> 3], 15UL << ((n & 7) * 4),
          ((uint32_t) af) << ((n & 7) * 4));
  SETBITS(gpio->MODER, 3UL << (n * 2), ((uint32_t) mode) << (n * 2));
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

#ifndef UART_DEBUG
#define UART_DEBUG USART2
#endif

static inline bool uart_init(USART_TypeDef *uart, unsigned long baud) {
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  uint32_t freq = 0;        // Bus frequency. UART1 is on APB2, rest on APB1

  if (uart == USART1) {
    freq = clock_apb2_freq(), RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    tx = PIN('A', 9), rx = PIN('A', 10);
  } else if (uart == USART2) {
    freq = clock_apb1_freq(), RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    tx = PIN('A', 2), rx = PIN('A', 15);
  } else {
    return false;
  }

  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, af);
  uart->CR1 = 0;                          // Disable this UART
  uart->BRR = freq / baud;                // Set baud rate
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE
  return true;
}
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1);
}
static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready
}
static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t) (uart->RDR & 255);
}

#define UUID ((uint8_t *) UID_BASE)  // Unique 96-bit chip ID. TRM 41.1

// Helper macro for MAC generation
#define GENERATE_LOCALLY_ADMINISTERED_MAC()                        \
  {                                                                \
    2, UUID[0] ^ UUID[1], UUID[2] ^ UUID[3], UUID[4] ^ UUID[5],    \
        UUID[6] ^ UUID[7] ^ UUID[8], UUID[9] ^ UUID[10] ^ UUID[11] \
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
// Fill in stack with markers, in order to calculate stack usage later
extern unsigned char _estack, _end;
static inline void stack_fill(void) {
  uint32_t dummy, *p = (uint32_t *) &_end;
  while (p < &dummy) *p++ = 0xa5a5a5a5;
}

static inline long stack_usage(void) {
  uint32_t *sp = (uint32_t *) &_estack, *end = (uint32_t *) &_end, *p = sp - 1;
  while (p > end && *p != 0xa5a5a5a5) p--;
  return (sp - p) * sizeof(*p);
}

static inline void clock_set(int clk_src, uint32_t pllmul, uint32_t prediv) {
  SCB->CPACR |= 15 << 20;             // Enable FPU
  FLASH->ACR |= FLASH_ACR_LATENCY_2;  // Set flash latency
  if (clk_src == CLK_SRC_HSE) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) spin(1);
    SETBITS(RCC->CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SW_HSE);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) spin(1);
  } else if (clk_src == CLK_SRC_PLL_HSI_DIV2) {
    if (pllmul < 2) pllmul = 2;
    if (pllmul > 16) pllmul = 16;
    SETBITS(RCC->CFGR, RCC_CFGR_PLLSRC_Msk, RCC_CFGR_PLLSRC_HSI_DIV2);
    SETBITS(RCC->CFGR, RCC_CFGR_PLLMUL_Msk,
            (pllmul - 2) << RCC_CFGR_PLLMUL_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) spin(1);
    SETBITS(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) spin(1);
  } else if (clk_src == CLK_SRC_PLL_HSE_PREDIV) {
    (void) prediv;
  }

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;     // Enable SYSCFG
  SysTick_Config(clock_sys_freq() / 1000);  // Sys tick every 1ms
}

static inline void clock_init(void) {
  clock_set(CLK_SRC_PLL_HSI_DIV2, 16, 0);  // No HSE. Max = 8 / 2 * 16 = 64 Mhz
}

// Constants defined by the TRM, section 4
#define FLASH_BLOCK_SIZE 2048  // FLash block size in bytes
#define FLASH_MAX_BLOCKS 128   // Total number of available blocks

static inline bool flash_is_err(void) {
  return FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR);
}

static inline void flash_unlock(void) {
  static bool unlocked = false;
  if (!unlocked) {
    FLASH->KEYR = 0x45670123;  // Unlock
    spin(10);                  // Wait a bit
    FLASH->KEYR = 0xCDEF89AB;  // Flash
    unlocked = true;
  }
  FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER | FLASH_CR_STRT);
  FLASH->SR = ~0UL;  // Clear any previous errors
}

static inline void flash_lock(void) {
  // FLASH->KEYR = 0;
  spin(10);
}

static inline bool flash_write(const void *from, size_t len, void *to) {
  const uint16_t *src = (uint16_t *) from;
  const uint16_t *end = (uint16_t *) ((char *) from + len);
  uint16_t *dst = (uint16_t *) to;
  bool ok = true;
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
  flash_unlock();
  FLASH->CR |= FLASH_CR_PG;
  size_t written = 0;
  while (ok && src < end) {
    *dst++ = *src++;
    while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
    *dst++ = *src++;
    while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
    ok = !flash_is_err();
    written += 4;
  }
  // printf("Wrore %u/%u %d %lx %lx\n", written, len);
  FLASH->SR &= ~FLASH_SR_EOP;
  FLASH->CR &= ~FLASH_CR_PG;
  flash_lock();
  return ok;
}

static inline bool flash_erase(uint32_t pageno) {
  bool ok = true;
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
  flash_unlock();
  FLASH->AR = pageno;          // Set PER bit and page no
  FLASH->CR |= FLASH_CR_STRT;  // Start erasing
  // printf(" -->%lu, CR %#lx SR %#lx ...\n", page, FLASH->CR, FLASH->SR);
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until erased
  flash_lock();
  return ok;
}
