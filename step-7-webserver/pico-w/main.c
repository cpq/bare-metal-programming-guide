// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"

#define LED  PIN('A', 25)

static volatile uint64_t s_ticks;
void SysTick_Handler(void) {  // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void blink(void) {
  static bool on = false;
  gpio_write(LED, on);
  on = !on;
}

int main(void) {
  clock_init();
  gpio_set_mode(LED, GPIO_MODE_OUTPUT);  // Set LED to output mode

  uint32_t blink_timer = 0, blink_period = 500;
  for (;;) {
    if (timer_expired(&blink_timer, blink_period, (uint32_t) s_ticks)) blink();
  }

  return 0;
}
