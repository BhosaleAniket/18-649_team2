/*
 * gpio.c - see gpio.h. BCM2711 GPIO registers: GPFSELn at word n, GPSET0 at word 7,
 * GPCLR0 at word 10.
 */
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>
#include "gpio.h"

static volatile uint32_t *gpio;

int gpio_init(void)
{
    if (gpio) return 0;
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0) return -1;
    void *m = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) return -1;
    gpio = (volatile uint32_t *)m;
    return 0;
}

void gpio_output(int pin)
{
    if (!gpio || pin < 0 || pin > 31) return;
    volatile uint32_t *fsel = gpio + pin / 10;
    int sh = (pin % 10) * 3;
    *fsel = (*fsel & ~(7u << sh)) | (1u << sh);
}

void gpio_write(int pin, int value)
{
    if (!gpio || pin < 0 || pin > 31) return;
    if (value) gpio[7] = 1u << pin;
    else       gpio[10] = 1u << pin;
}
