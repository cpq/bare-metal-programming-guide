// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

#include "hal.h"

#define LED1 PIN('B', 0)  // On-board LED pin (green)
#define LED2 PIN('F', 4)  // On-board LED pin (yellow)
#define LED3 PIN('G', 4)  // On-board LED pin (red)

#define BLINK_PERIOD_MS 500  // LED blinking period in millis
#define LOG_PERIOD_MS 5000   // Info log period in millis
#define DATA_DIR "/data"
#define DATA_FILE DATA_DIR "/boot.txt"

uint32_t SystemCoreClock;  // Required by CMSIS. Holds system core cock value
void SystemInit(void) {    // Called automatically by startup code
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

bool timer_expired(volatile uint64_t *t, uint64_t prd, uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

static void list_files(const char *path) {
  if (path == NULL || path[0] == '\0') path = ".";
  DIR *dirp = opendir(path);
  if (dirp != NULL) {
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
      if (strcmp(dp->d_name, ".") == 0 || strcmp(dp->d_name, "..") == 0) continue;
      printf("  %s%s%s\n", path, dp->d_name, (dp->d_type & DT_DIR) ? "/" : "");
      if (dp->d_type & DT_DIR) {
        char dir[strlen(path) + strlen(dp->d_name) + 2];
        snprintf(dir, sizeof(dir), "%s%s/", path, dp->d_name);
        list_files(dir);
      }
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
    gpio_toggle(LED1);
  }
}

static void log_task(void) {  // Print a log every LOG_PERIOD_MS
  static uint64_t timer = 0;
  if (timer_expired(&timer, LOG_PERIOD_MS, s_ticks)) {
    printf("tick: %5lu, CPU %lu MHz, boot count: %ld",
           (unsigned long) s_ticks, SystemCoreClock / 1000000,
           read_boot_count(DATA_FILE));
    putchar('\n');
  }
}

int main(void) {
  system_init();
  clock_init();

  SystemCoreClock = SYS_FREQUENCY;
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms

  rng_init();
  ethernet_init();

  gpio_output(LED1);
  uart_init(UART_DEBUG, 115200);

  printf("Starting, CPU clock is %lu MHz\n", SystemCoreClock / 1000000);

  // Increment boot count
  write_boot_count(DATA_FILE, read_boot_count(DATA_FILE) + 1);
  list_files("/");

  for (;;) {
    led_task();
    log_task();
  }

  return 0;
}
