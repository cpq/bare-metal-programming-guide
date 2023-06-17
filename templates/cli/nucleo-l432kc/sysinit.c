// Copyright (c) 2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT
//
// This file contains essentials required by the CMSIS:
// uint32_t SystemCoreClock - holds the system core clock value
// SystemInit() - initialises the system, e.g. sets up clocks

#include "hal.h"

uint32_t SystemCoreClock = SYS_FREQUENCY;

void SystemInit(void) {    // Called automatically by startup code
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
  SystemCoreClock = PLL_HSI * 1000000;
#endif

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
  rng_init();                              // Initialise random number generator
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
  stack_fill();                            // Instrument stack for stack usage
}
