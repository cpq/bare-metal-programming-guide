// Copyright (c) 2025 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

#define UART_DEBUG USART1

#define LED1_PIN PIN('C', 7)  // Green
#define LED2_PIN PIN('B', 7)  // Blue
#define LED3_PIN PIN('G', 2)  // Red

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis

static uint16_t s_pins[] = {LED1_PIN, LED2_PIN, LED3_PIN};
//static uint16_t s_pins[] = {LED1_PIN, PIN('A', 3)};
#define NUM_LEDS (sizeof(s_pins) / sizeof(s_pins[0]))

int _write(int fd, char *ptr, int len) {
  if (fd == 1) uart_write_buf(UART_DEBUG, ptr, (size_t) len);
  return len;
}

uint32_t SystemCoreClock;  // Required by CMSIS. Holds system core cock value
void SystemInit(void) {    // Called automatically by startup code
  clock_init();            // Sets SystemCoreClock
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  static size_t idx;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    for (size_t i = 0; i < NUM_LEDS; i++) gpio_write(s_pins[i], false);
    gpio_write(s_pins[idx], true);
    if (++idx >= NUM_LEDS) idx = 0;
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz\n", (unsigned long) s_ticks,
           (unsigned long) (SystemCoreClock / 1000000));
    for (size_t i = 0; i < 8; i++) {
      printf("%#lx\n", gpio_bank(i)->LCKR);
    }
  }
}

int main(void) {
  for (size_t i = 0; i < NUM_LEDS; i++) gpio_output(s_pins[i]);
  uart_init(UART_DEBUG, 115200);

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
