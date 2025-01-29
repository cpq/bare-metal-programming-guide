#include <stdint.h>
#include "hal.h"

#define LED1 19

int main(void) {
  set_gpio_mode(P0, LED1, GPIO_MODE_OUTPUT, 0);

  int level = 0;
  while (1) {
    gpio_write(P0, LED1, level);
    spin(9999999);
    level = !level;
  }
}

// Startup code
__attribute__((naked, noreturn)) void Reset_Handler(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 42 nRF-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 42])(void) = {
    _estack, Reset_Handler};
