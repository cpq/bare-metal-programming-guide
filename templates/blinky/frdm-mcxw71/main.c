// Copyright (c) 2025 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"

#define UART_DEBUG LPUART1

#define LED1_PIN PIN('A', 19)  // Green
//#define LED2_PIN PIN('A', 20)  // Blue
#define LED2_PIN PIN('C', 1)  // Blue
#define LED3_PIN PIN('A', 21)  // Red

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis

// Retarget standard IO functions ouput (printf, fwrite, etc) to use UART
int _write(int fd, char *ptr, int len) {
  if (fd == 1) uart_write_buf(UART_DEBUG, ptr, (size_t) len);
  return len;
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED2_PIN);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz\n", (unsigned long) s_ticks,
           SystemCoreClock / 1000000);
  }
}

uint32_t SystemCoreClock;
void SystemInit(void) {
  // set CP10, CP11 Full Access in Secure and Non-secure mode
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
  SystemCoreClock = SYS_FREQUENCY;         // Required by CMSIS
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
}

int main(void) {
  MRCC_Type *x = MRCC;
  gpio_output(LED1_PIN);
  gpio_write(LED1_PIN, 1);

  gpio_output(LED2_PIN);
  gpio_write(LED2_PIN, 1);

  printf("%p", x);

  uart_init(UART_DEBUG, 115200);

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
