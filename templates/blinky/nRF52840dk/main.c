#include "hal.h"

#define LED1 PIN('A', 17)

int main(void) {
  gpio_init(LED1, GPIO_MODE_OUTPUT, 0);

  for (;;) {
    gpio_write(LED1, true);
    spin(9999999);
    gpio_write(LED1, false);
    spin(9999999);
  }
}

// Startup code
__attribute__((naked, noreturn)) void Reset_Handler(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern uint32_t _sbss, _ebss, _sdata, _edata, _sidata;
  for (uint32_t *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (uint32_t *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 42 nRF-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 42])(void) = {
    _estack, Reset_Handler};
