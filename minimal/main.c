int main(void) {
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  asm("ldr sp, = _estack");  // Set initial stack pointer

  // Initialise memory
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  // Call main()
  main();
  for (;;) (void) 0;  // Infinite loop
}

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
  0, _reset
};
