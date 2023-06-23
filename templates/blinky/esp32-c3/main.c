// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis

void SystemInit(void) {  // Called automatically by startup code
  clock_init();
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED_PIN);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz\n", (unsigned long) s_ticks,
           clock_sys_freq() / 1000000);
  }
}

static inline uint64_t systick2(void) {
  REG(C3_SYSTIMER)[2] = BIT(30);  // TRM 10.5
  spin(1);
  // return ((uint64_t) REG(C3_SYSTIMER)[18]<< 32) | REG(C3_SYSTIMER + 0x4c);
  return ((uint64_t) REG(C3_SYSTIMER)[18] << 32) | REG(C3_SYSTIMER)[19];
}

int main(void) {
  gpio_output(LED_PIN);
  uart_init(UART_DEBUG, 115200);

  // STILL WORK IN PROGRESS!
  // TRM 10.20. SYSTIMER_TARGET1_CONF ->  ??
  R(C3_SYSTIMER + 0x38) = BIT(30) | clock_sys_freq() / 1000;
  R(C3_SYSTIMER + 0x64) = BIT(1);  // SYSTIMER_TARGET1_INT_ENA

  // TODO(cpq) - make systick interrupt work!
  for (;;) {
    printf("LED: %lu\n", (unsigned long) systick2());
    delay_ms(500);
  }

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
