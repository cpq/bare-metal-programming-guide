// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include <stdlib.h>
#include <string.h>
#include "hal.h"

#define LED PIN('B', 3)  // On-board LED pin (green)

static bool s_led_blink = true;               // If false, LED is on or off
static unsigned s_led_blink_period_ms = 300;  // LED blinking interval

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void led_task(void) {
  static uint64_t timer = 0;
  if (s_led_blink && timer_expired(&timer, s_led_blink_period_ms, s_ticks)) {
    gpio_toggle(LED);
  }
}

static void cli_prompt(void) {
  printf("enter command:\n");
}

static void cli_usage(void) {
  putchar('\n');
  printf("COMMANDS:\n");
  printf("  hexdump ADDR LENGTH\n");
  printf("  led <on|off|blink MILLIS>\n");
  printf("  reboot\n");
}

static void cli_led(const char *arg1, const char *arg2) {
  if (strcasecmp(arg1, "on") == 0) {
    s_led_blink = 0;
    gpio_write(LED, true);
  } else if (strcasecmp(arg1, "off") == 0) {
    s_led_blink = 0;
    gpio_write(LED, false);
  } else {
    s_led_blink = 1;
    s_led_blink_period_ms = strtoul(arg2, NULL, 0);
  }
  printf("LED status: %s, blink: %s, interval: %u ms\n",
         gpio_read(LED) ? "on" : "off", s_led_blink ? "yes" : "no",
         s_led_blink_period_ms);
}

static char nibble(char c) {
  return c < 10 ? c + '0' : c + 'W';
}

static void hexdump(const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *) buf;
  char ascii[16];
  size_t i, j, n = 0;
  for (i = 0; i < len; i++) {
    if ((i % 16) == 0) {
      // Print buffered ascii chars
      if (i > 0) {
        putchar(' '), putchar(' ');
        for (j = 0; j < sizeof(ascii); j++) putchar(ascii[j]);
        putchar('\n'), n = 0;
      }
      // Print hex address, then \t
      putchar(nibble((i >> 12) & 15)), putchar(nibble((i >> 8) & 15));
      putchar(nibble((i >> 4) & 15)), putchar('0');
      putchar(' '), putchar(' '), putchar(' ');
    }
    putchar(nibble(p[i] >> 4)), putchar(nibble(p[i] & 15));
    putchar(' ');  // Space after hex number
    if (p[i] >= ' ' && p[i] <= '~') {
      ascii[n++] = (char) p[i];  // Printable
    } else {
      ascii[n++] = '.';  // Non-printable
    }
  }
  if (n > 0) {
    while (n < 16) putchar(' '), putchar(' '), putchar(' '), ascii[n++] = ' ';
    putchar(' '), putchar(' ');
    for (j = 0; j < sizeof(ascii); j++) putchar(ascii[j]);
  }
  putchar('\n');
}

static void cli_hexdump(const char *addr, const char *len) {
  char *buf = (char *) strtoul(addr, NULL, 0);
  long n = strtol(len, NULL, 0);
  printf("Dumping %ld bytes @ %p\n", n, buf);
  hexdump(n < 0 ? buf + n : buf, (size_t) (n < 0 ? -n : n));
}

static void cli_exec(const char *command, const char *arg1, const char *arg2) {
  if (strcmp(command, "reboot") == 0) {
    NVIC_SystemReset();
  } else if (strcmp(command, "led") == 0) {
    cli_led(arg1, arg2);
  } else if (strcmp(command, "hexdump") == 0) {
    cli_hexdump(arg1, arg2);
  } else {
    printf("Unknown command '%s'\n", command);
    cli_usage();
  }
  cli_prompt();
}
static void cli_task(void) {
  static char buf[128];  // Input buffer
  static size_t len;     // Input length

  // Print CLI prompt message 1s after boot
  static bool printed = false;
  if (s_ticks > 1000 && !printed) cli_usage(), cli_prompt(), printed = true;

  if (uart_read_ready(UART_DEBUG)) {
    uint8_t input_byte = uart_read_byte(UART_DEBUG);
    if (input_byte == '\n' && len == 0) {
    } else if (input_byte == '\n') {
      char buf0[10], buf1[50], buf2[100];  // Command, arg1, arg2
      buf0[0] = buf1[0] = buf2[0] = '\0';  // NUL-terminate
      buf[len] = '\0';                     // NUL-terminate input buffer
      sscanf(buf, "%9s %49s %99[^\r\n]", buf0, buf1, buf2);  // Parse command
      cli_exec(buf0, buf1, buf2);                            // Execute command
      len = 0;  // Reset input buffer
    } else {
      if (len >= sizeof(buf)) len = 0;  // Reset input buffer on overflow
      buf[len++] = input_byte;          // Append read byte to the input buffer
    }
  }
}

int main(void) {
  gpio_output(LED);               // Setup green LED
  uart_init(UART_DEBUG, 115200);  // Initialise UART

  printf("Boot complete. CPU %lu MHz\n", SystemCoreClock / 1000000);
  for (;;) {
    led_task();
    cli_task();
  }

  return 0;
}
