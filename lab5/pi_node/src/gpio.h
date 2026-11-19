/*
 * gpio.h - direct GPIO register access on the Raspberry Pi 4 (BCM2711) through /dev/gpiomem.
 * A write is a single store (tens of ns), unlike lgpio from Python. Pins 0..31 only.
 * Not for the Pi 5 (its GPIO is on the RP1 chip).
 */
#ifndef PI_GPIO_H
#define PI_GPIO_H

int  gpio_init(void);            /* 0 = ok, -1 = /dev/gpiomem unavailable */
void gpio_output(int pin);
void gpio_write(int pin, int value);

#endif
