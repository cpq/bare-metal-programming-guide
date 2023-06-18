// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include <errno.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>

#include "hal.h"

#define LED PIN('B', 3)      // Use on-board green LED for blinking
#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 5000   // Info log period in millis
#define DATA_DIR "/data"
#define DATA_FILE DATA_DIR "/boot.txt"

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static void list_files(const char *path) {
  if (path == NULL || path[0] == '\0') path = ".";
  DIR *dirp = opendir(path);
  if (dirp != NULL) {
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
      printf("  %s%s", dp->d_name, (dp->d_type & DT_DIR) ? "/" : "");
    }
    closedir(dirp);
  }
}

static long read_boot_count(const char *path) {
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

static void write_boot_count(const char *path, long count) {
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

static void led_task(void) {  // Blink LED every BLINK_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, BLINK_PERIOD_MS, s_ticks)) {
    gpio_toggle(LED);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz, boot count: %ld, files: ",
           (unsigned long) s_ticks, SystemCoreClock / 1000000,
           read_boot_count(DATA_FILE));
    list_files("/");
    list_files("/data");
    putchar('\n');
  }
}

int main(void) {
  gpio_output(LED);               // Setup green LED
  uart_init(UART_DEBUG, 115200);  // Initialise debug printf

  // Increment boot count
  write_boot_count(DATA_FILE, read_boot_count(DATA_FILE) + 1);

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
