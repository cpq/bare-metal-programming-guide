#define P0 ((struct gpio *) 0x50000000)
#define P1 ((struct gpio *) 0x50000300)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT };
enum { LOW, HIGH };

struct gpio {
  volatile uint32_t RESERVED[321], OUT, OUTSET, OUTCLR, IN, DIR, DIRSET, DIRCLR,
      LATCH, DETECTMODE, RESERVED_SECOND[118], PIN_CNF[32];
};

void set_gpio_mode(struct gpio *port, int pin, int mode, int pull) {
  if (pull == 0) {
    port->PIN_CNF[pin] = mode << 0;
  } else {
    port->PIN_CNF[pin] = mode << 0 | pull << 2;
  }
}

int gpio_read(struct gpio *port, int pin) {
  uint32_t button = ~(port->IN);
  if (button & (1 << pin)) {
    return 1;
  } else {
    return 0;
  }
}
void gpio_write(struct gpio *port, int pin, int value) {
  if (value == 0) {
    port->OUTSET = 1 << pin;
  } else {
    port->OUTCLR = 1 << pin;
  }
}

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}
