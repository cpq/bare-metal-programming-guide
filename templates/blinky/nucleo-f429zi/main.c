// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

#define LED PIN('B', 7)      // Use blue LED for blinking
#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz\n", (unsigned long) s_ticks,
           SystemCoreClock / 1000000);
  }
}

int main(void) {
  gpio_output(LED);               // Setup green LED
  uart_init(UART_DEBUG, 115200);  // Initialise debug printf

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
