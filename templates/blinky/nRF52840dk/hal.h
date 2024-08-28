#define P0 ((struct gpio *) 0x50000000)
#define P1 ((struct gpio *) 0x50000300)
#define BUILD_IN_LED_1 13

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT };
enum { LOW, HIGH };

struct gpio {
  volatile uint32_t RESERVED[321], OUT, OUTSET, OUTCLR, IN, DIR, DIRSET, DIRCLR, LATCH, DETECTMODE, PIN_CNF[32];
};

void set_gpio_mode(struct gpio * port, int pin, int mode) {
  port->DIR |= mode << pin;
}

void gpio_write(struct gpio * port, int pin, int value) {
  if (value == 1)
    port->OUTSET |= 1 << pin;
  else
    port->OUTCLR |= 1 << pin;
}

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}
