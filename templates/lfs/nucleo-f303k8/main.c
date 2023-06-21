// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "hal.h"

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 1000   // Info log period in millis
#define DATA_DIR "/data"
#define DATA_FILE DATA_DIR "/boot.txt"

void SystemInit(void) {  // Called automatically by startup code
  clock_init();          // Set system clock to maximum
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static inline void list_files(const char *path) {
  if (path == NULL || path[0] == '\0') path = ".";
  DIR *dirp = opendir(path);
  if (dirp != NULL) {
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
      if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) continue;
      char buf[80];
      const char *slash = path[strlen(path) - 1] == '/' ? "" : "/";
      snprintf(buf, sizeof(buf), "%s%s%s", path, slash, dp->d_name);
      printf("  %s", buf);
      if (dp->d_type & DT_DIR) {
        putchar('/');
        list_files(buf);
      }
    }
    closedir(dirp);
  }
}

static inline long read_boot_count(const char *path) {
  long count = 0;
  FILE *fp = fopen(path, "r");
  if (fp != NULL) {
    fscanf(fp, "%ld", &count);
    fclose(fp);
  } else {
    printf("Error opening %s: %d\n", path, errno);
  }
  return count;
}

static inline void write_boot_count(const char *path, long count) {
  mkdir(DATA_DIR, 0644);
  FILE *fp = fopen(path, "w+");
  if (fp != NULL) {
    fprintf(fp, "%ld", count);
    (void) count;
    fclose(fp);
  } else {
    printf("Error opening %s: %d\n", path, errno);
  }
}

static inline void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED_PIN);
  }
}

static inline void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = ~0 - LOG_PERIOD_MS;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz %lx ", (unsigned long) s_ticks,
           clock_sys_freq() / 1000000, RCC->CFGR);
    // printf("boot count: %ld, files: ", read_boot_count(DATA_FILE));
    list_files("/");
    putchar('\n');
  }
}

int main(void) {
  gpio_output(LED_PIN);
  uart_init(UART_DEBUG, 115200);

  // Increment boot count
  write_boot_count(DATA_FILE, read_boot_count(DATA_FILE) + 1);

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
