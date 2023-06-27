// SPDX-FileCopyrightText: 2021-2023 Cesanta Software Limited
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

// C handlers associated with CPU interrupts, with their arguments
struct irq_data g_irq_data[32];

// Attribute interrupt makes this function to:
// 1. Return with mret instruction
// 2. Save/restore all used registers
__attribute__((interrupt)) void irq_handler(void) {
  unsigned long mcause = CSR_READ(mcause), mepc = CSR_READ(mepc);
  //printf("mcause %lx\n", mcause);
  if ((mcause & BIT(31))) {          // Interrupt
    uint32_t no = mcause << 1 >> 1;  // Interrupt number
    if (no < sizeof(g_irq_data) / sizeof(g_irq_data[0])) {
      struct irq_data *d = &g_irq_data[no];
      if (d->clr) d->clr(d->clr_arg);  // Clear interrupt
      if (d->fn) d->fn(d->arg);        // Call user handler
    }
  } else {  // Exception
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
int cpu_alloc_interrupt(uint8_t prio /* 1..15 */) {
  static uint32_t allocated;
  for (uint8_t no = 1; no < 31; no++) {
    if (allocated & BIT(no)) continue;             // Used, try the next one
    allocated |= BIT(no);                          // Claim this one
    REG(C3_INTERRUPT)[0x104 / 4] |= BIT(no);        // CPU_INT_ENA
    REG(C3_INTERRUPT)[0x118 / 4 + no - 1] = prio;  // CPU_INT_PRI_N
    // REG(C3_INTERRUPT)[0x108 / 4] |= BIT(no);  // Edge
    printf("Allocated CPU IRQ %d, prio %u\n", no, prio);
    return no;
  }
  return -1;
}

static void systimer_clear_interrupt(void *param) {
  (void) param;
  SYSTIMER->INT_CLR = 7U;  // Clear all 3 units
}

// Systimer is clocked by 16Mhz. Setup alarm and bind it to the CPU IRQ
static void systick_init(void) {
  SYSTIMER->TARGET0_CONF = BIT(30) | 16000;  // Set period
  SYSTIMER->COMP0_LOAD = BIT(0);             // Reload period
  SYSTIMER->CONF |= BIT(24);                 // Enable comparator 0
  SYSTIMER->INT_ENA |= 7U;                   // Enable interrupts on all targets

  int no = cpu_alloc_interrupt(1);
  g_irq_data[no].fn = (void (*)(void *)) SysTick_Handler;
  g_irq_data[no].clr = systimer_clear_interrupt;
  REG(C3_INTERRUPT)[0xfc / 4] |= BIT(5) | BIT(6) | BIT(7);  // Enable CPU IRQ
  //REG(C3_INTERRUPT)[0xfc / 4] |= BIT(5);                    // Enable CPU IRQ
  REG(C3_INTERRUPT)[0x94 / 4] = (uint8_t) no;  // LAST: Map systimer IRQ to CPU
}

void Reset_Handler(void) {
  s_heap_start = s_brk = &_end, s_heap_end = &_eram;
  for (char *p = &_sbss; p < &_ebss;) *p++ = '\0';
  CSR_WRITE(mtvec, irqtab);  // Route all interrupts to the irq_handler()
  soc_init();
  systick_init();
  SystemInit();
  main();
  for (;;) (void) 0;
}
