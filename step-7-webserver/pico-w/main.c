// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "mcu.h"

int main(void) {
  clock_init();

  uint16_t led = PIN('A', 25);           // On-board LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set LED to output mode

  for (;;) {
    gpio_write(led, true);
    spin(9999999);
    gpio_write(led, false);
    spin(9999999);
  }

  return 0;
}
