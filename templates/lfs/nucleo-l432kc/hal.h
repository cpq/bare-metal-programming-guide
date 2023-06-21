// Copyright (c) 2023 Cesanta Software Limited
// https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// SPDX-License-Identifier: MIT

#ifndef LED_PIN
#define LED_PIN PIN('B', 3)  // Green onboard LED on Nucleo-L432KC
#endif

#pragma once
#include <stm32l432xx.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

// System clock
enum { AHB_DIV = 1, APB1_DIV = 1, APB2_DIV = 1 };
enum { PLL_HSI = 16, PLL_M = 1, PLL_N = 10, PLL_R = 2 };  // 80 Mhz
//#define SYS_FREQUENCY ((PLL_HSI * PLL_N / PLL_M / PLL_R) * 1000000)
#define SYS_FREQUENCY 16000000
#define APB2_FREQUENCY (SYS_FREQUENCY / APB2_DIV)
#define APB1_FREQUENCY (SYS_FREQUENCY / APB1_DIV)

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
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
  RCC->AHB2ENR |= BIT(PINBANK(pin));  // Enable GPIO clock
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
  // https://www.st.com/resource/en/datasheet/stm32l432kc.pdf
  uint8_t aftx = 7, afrx = 7;  // Alternate function
  uint16_t rx = 0, tx = 0;     // pins
  uint32_t freq = 0;           // Bus frequency. UART1 is on APB2, rest on APB1

  if (uart == USART1) {
    freq = APB2_FREQUENCY, RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    tx = PIN('A', 9), rx = PIN('A', 10);
  } else if (uart == USART2) {
    freq = APB1_FREQUENCY, RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    tx = PIN('A', 2), rx = PIN('A', 15), afrx = 3;
  } else {
    return false;
  }

  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, aftx);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH, 0, afrx);
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

static inline void rng_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
  RNG->CR |= RNG_CR_RNGEN;
}
static inline uint32_t rng_read(void) {
  while ((RNG->SR & RNG_SR_DRDY) == 0) (void) 0;
  return RNG->DR;
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

static inline void clock_init(void) {
  SCB->CPACR |= 15 << 20;  // Enable FPU
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
#if 0
#if 0
  SETBITS(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, (PLL_M - 1) << RCC_PLLCFGR_PLLM_Pos);
  SETBITS(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, PLL_N << RCC_PLLCFGR_PLLN_Pos);
  SETBITS(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLCFGR_PLLSRC_HSI);
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
#else
  RCC->PLLCFGR = BIT(24) | BIT(20) | (PLL_N << 8) | RCC_PLLCFGR_PLLSRC_HSI |
    (7 << 27) | BIT(16);
  RCC->CR |= RCC_CR_PLLON;
#endif
  while (!(RCC->CR & RCC_CR_PLLRDY)) spin(1);
#if 0
  SETBITS(RCC->CFGR, RCC_CFGR_PPRE1, (3 + APB1_DIV / 2) << RCC_CFGR_PPRE1_Pos);
  SETBITS(RCC->CFGR, RCC_CFGR_PPRE2, (3 + APB2_DIV / 2) << RCC_CFGR_PPRE1_Pos);
#endif
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) spin(1);
#else
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) spin(1);
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= (RCC_CFGR_SW_HSI);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) spin(1);
#endif

  rng_init();                              // Initialise random number generator
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
  SystemCoreClock = SYS_FREQUENCY;         // Required by CMSIS
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
}
