// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "hal.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  systick_init(FREQ / 1000);             // Tick every 1 ms
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  uart_init(UART3, 115200);              // Initialise UART
  uint32_t timer = 0, period = 500;      // Declare timer and 500ms period
  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;       // This block is executed
      gpio_write(led, on);  // Every `period` milliseconds
      on = !on;             // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks);  // Write message
    }
    // Here we could perform other activities!
  }
  return 0;
}
