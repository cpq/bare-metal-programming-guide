// Copyright (c) 2023 Cesanta Software Limited
// All rights reserved

#include "hal.h"
#include "mongoose.h"

enum { LED = 25 };                                             // LED pins
enum { UART_TX = 0, UART_RX = 1 };                             // UART pins
enum { SPI_CS = 17, SPI_CLK = 18, SPI_TX = 19, SPI_RX = 16 };  // SPI pins
enum { STATUS_TIMER_MS = 1000, BLINK_TIMER_MS = 500 };         // Timeouts

void my_spi_begin(void *spi) { spi_begin(spi); }
void my_spi_end(void *spi) { spi_end(spi); }
uint8_t my_spi_txn(void *spi, uint8_t byte) { return spi_txn(spi, byte); }

void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  if (ev == MG_EV_HTTP_MSG) {
    mg_http_reply(c, 200, "", "ok\n");
  }
  (void) ev_data, (void) fn_data;
}

static volatile uint64_t s_ticks;
void SysTick_Handler(void) {  // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

uint64_t mg_millis(void) { return s_ticks; }

int main(void) {
  clock_init();                                // Init clocks
  uart_init(UART0, 115200, UART_RX, UART_TX);  // Init UART
  gpio_init(LED, GPIO_MODE_OUTPUT, 0);         // Init LED

  MG_INFO(("Starting ..."));

  // Init SPI
  struct spi spi0 = {
      .miso = SPI_RX, .mosi = SPI_TX, .clk = SPI_CLK, .cs = SPI_CS, .spin = 50};
  bool ok = spi_init(&spi0);
  MG_INFO(("SPI init: %d", ok));

  // Init Mongoose
  struct mip_spi spi = {&spi0, my_spi_begin, my_spi_end, my_spi_txn};
  struct mip_if mif = {.mac = {2, 0, 1, 2, 3, 5},
                       .driver = &mip_driver_w5500,
                       .driver_data = &spi};
  struct mg_mgr mgr;                                 // Declare event manager
  mg_mgr_init(&mgr);                                 // Init event manager
  mg_log_set(MG_LL_DEBUG);                           // Set DEBUG log level
  mip_init(&mgr, &mif);                              // Init TCP/IP stack
  mg_http_listen(&mgr, "http://0.0.0.0", fn, NULL);  // HTTP listener

  bool led_on = false;                         // Initial LED state
  uint64_t status_timer = 0, blink_timer = 0;  // Initial timer expirations

  // Infinite event manager loop
  for (;;) {
    if (mg_timer_expired(&blink_timer, BLINK_TIMER_MS, mg_millis())) {
      led_on = !led_on;                              // Flip LED state
      if (mip_driver_w5500.up(&mif)) led_on = true;  // Always on if Eth up
      gpio_write(LED, led_on);                       // Set LED
    }
    if (mg_timer_expired(&status_timer, STATUS_TIMER_MS, mg_millis())) {
      MG_INFO(("Ethernet: %s", mif.driver->up(&mif) ? "up" : "down"));
    }
    mg_mgr_poll(&mgr, 1);
  }

  return 0;
}
