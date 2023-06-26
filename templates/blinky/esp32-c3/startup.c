// Copyright (c) 2021-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

extern int main(void);
extern void SystemInit(void);
extern char _sbss, _ebss, _end, _eram;

static char *s_heap_start, *s_heap_end, *s_brk;

void *sbrk(int diff) {
  char *old = s_brk;
  if (&s_brk[diff] > s_heap_end) return NULL;
  s_brk += diff;
  return old;
}

// Mark it weak - allow user to override it
__attribute__((weak)) void SysTick_Handler(void) {
}

// Attribute interrupt makes this function to:
// 1. Return with mret instruction
// 2. Save/restore all used registers
__attribute__((interrupt)) void irq_handler(void) {
  unsigned long mcause = CSR_READ(mcause), mepc = CSR_READ(mepc);
  // printf("IRQ cause: %lx mepc: %lx mstatus: %lx\n", mcause, mepc,
  // CSR_READ(mstatus));
  if ((mcause & BIT(31))) {
    // Interrupt
    // uint32_t no = mcause << 1 >> 1;
    // GPIO->PCPU_INT = GPIO->STATUS = 0U;  // Clear GPIO IRQ
    SYSTIMER->INT_CLR = 7U;  // Clear systimer IRQ
    SysTick_Handler();
  } else {
    // Exception
    CSR_WRITE(mepc, mepc + 4);
  }
}

// Vector table. Point all entries to the irq_handler()
__attribute__((aligned(256))) void irqtab(void) {
  asm(".rept 32");       // 32 entries
  asm("j irq_handler");  // Jump to irq_handler()
  asm(".endr");
}

// ESP32C3 lets us bind peripheral interrupts to the CPU interrupts, 1..31
static int cpu_alloc_interrupt(uint8_t prio /* 1..15 */) {
  static uint32_t allocated;
  for (uint8_t no = 1; no < 31; no++) {
    if (allocated & BIT(no)) continue;             // Used, try the next one
    allocated |= BIT(no);                          // Claim this one
    REG(C3_INTERRUPT)[0x104 / 4] = BIT(no);        // CPU_INT_ENA
    REG(C3_INTERRUPT)[0x118 / 4 + no - 1] = prio;  // CPU_INT_PRI_N
    // REG(C3_INTERRUPT)[0x108 / 4] |= BIT(no);  // Edge
    return no;
  }
  return -1;
}

// Systimer is clocked by 16Mhz. Setup alarm and bind it to the CPU IRQ
static void systimer_init(void) {
  SYSTIMER->TARGET0_CONF = BIT(30) | 16000;  // Set period
  SYSTIMER->COMP0_LOAD = BIT(0);             // Reload period
  SYSTIMER->CONF |= BIT(24);                 // Enable comparator 0
  SYSTIMER->INT_ENA |= 7U;                   // Enable interrupts on all targets

  int no = cpu_alloc_interrupt(1);
  REG(C3_INTERRUPT)[0xfc / 4] |= BIT(5) | BIT(6) | BIT(7);  // Enable CPU IRQ
  REG(C3_INTERRUPT)[0xfc / 4] |= BIT(5);                    // Enable CPU IRQ
  REG(C3_INTERRUPT)[0x94 / 4] = (uint8_t) no;  // Map systimer IRQ to CPU
}

// GPIO IRQ
// REG(C3_INTERRUPT)[0x40 / 4] = no;  // Map GPIO IRQ to CPU
//  REG(C3_INTERRUPT)[0x44 / 4] = no;  // Map GPIO IRQ to CPU
//   REG(C3_INTERRUPT)[0xf8 / 4] |= BIT(16) | BIT(17);  // Enable CPU IRQ
//  REG(C3_GPIO)
//[0x74 / 4 + BTN_PIN] |= (3U << 7) | BIT(13);  // Enable intr, any edge

void _reset(void) {
  s_heap_start = s_brk = &_end, s_heap_end = &_eram;
  for (char *p = &_sbss; p < &_ebss;) *p++ = '\0';
  soc_init();
  CSR_WRITE(mtvec, irqtab);  // Route all interrupts to the irq_handler()
  systimer_init();
  SystemInit();
  main();
  for (;;) (void) 0;
}
