# Blinky

In order to build this firmware,

- Install docker
- Install esputil from https://github.com/cpq/esputil

Use any ESP32-C3 board, for example ESP32-C3-DevKITM-1.
Attach LED to pin 2. Then,

```sh
$ export PORT=/dev/SERIAL_PORT
$ make flash monitor
...
tick:  1001, CPU 160 MHz
tick:  2001, CPU 160 MHz
tick:  3001, CPU 160 MHz
...
```
