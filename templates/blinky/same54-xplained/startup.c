// SPDX-FileCopyrightText: 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

void Reset_Handler(void);    // Defined below
void Dummy_Handler(void);    // Defined below
void SysTick_Handler(void);  // Defined in main.c
void SystemInit(void);       // Defined in main.c, called by reset handler
void _estack(void);          // Defined in link.ld

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

WEAK_ALIAS void NMI_Handler(void);
WEAK_ALIAS void HardFault_Handler(void);
WEAK_ALIAS void MemoryManagement_Handler(void);
WEAK_ALIAS void BusFault_Handler(void);
WEAK_ALIAS void UsageFault_Handler(void);
WEAK_ALIAS void SVCall_Handler(void);
WEAK_ALIAS void DebugMonitor_Handler(void);
WEAK_ALIAS void PendSV_Handler(void);
WEAK_ALIAS void SysTick_Handler(void);

__attribute__((section(".vectors"))) void (*const tab[16 + 138])(void) = {
    _estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemoryManagement_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    NULL,
    NULL,
    NULL,
    NULL,
    SVCall_Handler,
    DebugMonitor_Handler,
    NULL,
    PendSV_Handler,
    SysTick_Handler,
};

__attribute__((naked, noreturn)) void Reset_Handler(void) {
  // Clear BSS section, and copy data section from flash to RAM
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  SCB->VTOR = (uint32_t) &tab;
  SystemInit();

  // Call main()
  extern void main(void);
  main();
  for (;;) (void) 0;  // Infinite loop
}

void Default_Handler(void) {
  for (;;) (void) 0;
}
