// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include "hal.h"
#include "mongoose.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

uint64_t mg_millis(void) {  // Declare our own uptime function
  return s_ticks;           // Return number of milliseconds since boot
}

int main(void) {
  uint16_t led = PIN('N', 1);            // LED1
  clock_init();                          // Run at 120MHz
  systick_init(FREQ / 1000);             // Tick every 1 ms
  gpio_init(led, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);          // Set LED1 to output mode
  uart_init(UART0, 115200);              // Initialise UART
  uint32_t timer = 0, period = 500;      // Declare timer and 500ms period

  // Initialise Ethernet, enable LED pins
  // See datasheet: https://www.ti.com/lit/pdf/spms433
  // Assign LED3 and LED4 to the EPHY, "activity" and "link", respectively.
  // (20.4.2.4)
  gpio_init(PIN('F', 4), GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 5);  // EN0LED1
  gpio_init(PIN('F', 0), GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 5);  // EN0LED0
  NVIC_EnableIRQ(EMAC0_IRQn);            // Setup Ethernet IRQ handler
  // Initialize Ethernet clocks, see datasheet section 5
  // Turn Flash Prefetch off (silicon errata ETH#02)
  uint32_t val = FLASH_CTRL->CONF;
  val &= ~BIT(17);
  val |= BIT(16);
  FLASH_CTRL->CONF = val;
  SYSCTL->RCGCEMAC |= BIT(0);  // Enable EMAC clock
  SYSCTL->SREMAC |= BIT(0);    // Reset EMAC
  SYSCTL->SREMAC &= ~BIT(0);
  SYSCTL->RCGCEPHY |= BIT(0);  // Enable EPHY clock
  SYSCTL->SREPHY |= BIT(0);    // Reset EPHY
  SYSCTL->SREPHY &= ~BIT(0);
  while (!(SYSCTL->PREMAC & BIT(0)) || !(SYSCTL->PREPHY & BIT(0)))
    spin(1);  // Wait for reset to complete

  struct mg_mgr mgr;        // Initialise Mongoose event manager
  mg_mgr_init(&mgr);        // and attach it to the MIP interface
  mg_log_set(MG_LL_DEBUG);  // Set log level

  // Initialise Mongoose network stack
  // Specify MAC address, either set use_dhcp or enter a static config.
  // For static configuration, specify IP/mask/GW in network byte order
  struct mip_driver_tm4c driver_data = {.mdc_cr = 1};  // See driver_tm4c.h
  struct mip_if mif = {
      .mac = {2, 0, 1, 2, 3, 5},
      .use_dhcp = true,
      .driver = &mip_driver_tm4c,
      .driver_data = &driver_data,
  };
  mip_init(&mgr, &mif);
  val = FLASH_CTRL->CONF;  // Turn Flash Prefetch on again
  val &= ~BIT(16);
  val |= BIT(17);
  FLASH_CTRL->CONF = val;
  extern void device_dashboard_fn(struct mg_connection *, int, void *, void *);
  mg_http_listen(&mgr, "http://0.0.0.0", device_dashboard_fn, &mgr);
  MG_INFO(("Init done, starting main loop"));

  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;       // This block is executed
      gpio_write(led, on);  // Every `period` milliseconds
      on = !on;             // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks);  // Write message
    }
    mg_mgr_poll(&mgr, 0);  // Handle networking
  }
  return 0;
}
