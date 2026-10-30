#include "rpi_gpio_qnx.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define GPIO_GPFSEL0  0x00
#define GPIO_GPSET0   0x1C
#define GPIO_GPCLR0   0x28
#define GPIO_GPLEV0   0x34

static uintptr_t gpio_regs = 0;

int gpio_sys_init(void)
{
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("ThreadCtl");
        return -1;
    }

    gpio_regs = mmap_device_io(GPIO_LEN, BCM2711_GPIO_BASE);
    if (gpio_regs == (uintptr_t)MAP_DEVICE_FAILED) {
        perror("mmap_device_io");
        gpio_regs = 0;
        return -1;
    }

    return 0;
}

void gpio_set_mode_input(int pin)
{
    if (gpio_regs == 0 || pin < 0 || pin > 27)
        return;

    unsigned offset = GPIO_GPFSEL0 + (pin / 10) * 4;
    unsigned shift = (pin % 10) * 3;
    uint32_t value = in32(gpio_regs + offset);

    value &= ~(7u << shift);
    out32(gpio_regs + offset, value);
}

void gpio_set_mode_output(int pin)
{
    if (gpio_regs == 0 || pin < 0 || pin > 27)
        return;

    unsigned offset = GPIO_GPFSEL0 + (pin / 10) * 4;
    unsigned shift = (pin % 10) * 3;
    uint32_t value = in32(gpio_regs + offset);

    value &= ~(7u << shift);
    value |= (1u << shift);
    out32(gpio_regs + offset, value);
}

void gpio_write(int pin, bool level)
{
    if (gpio_regs == 0 || pin < 0 || pin > 27)
        return;

    unsigned offset = level ? GPIO_GPSET0 : GPIO_GPCLR0;
    out32(gpio_regs + offset, 1u << pin);
}

bool gpio_read(int pin)
{
    if (gpio_regs == 0 || pin < 0 || pin > 27)
        return false;

    return (in32(gpio_regs + GPIO_GPLEV0) & (1u << pin)) != 0;
}

float measure_ultrasonic_cm(void)
{
    struct timespec start, end;

    gpio_write(GPIO_PIN_TRIG, false);
    nanosleep(&(struct timespec){0, 2000000L}, NULL);
    gpio_write(GPIO_PIN_TRIG, true);
    nanosleep(&(struct timespec){0, 10000L}, NULL);
    gpio_write(GPIO_PIN_TRIG, false);

    int timeout = 30000;
    while (!gpio_read(GPIO_PIN_ECHO) && --timeout > 0) {
        /* Wait for echo pulse. */
    }

    if (timeout <= 0)
        return -1.0f;

    clock_gettime(CLOCK_MONOTONIC, &start);

    timeout = 30000;
    while (gpio_read(GPIO_PIN_ECHO) && --timeout > 0) {
        /* Measure echo pulse. */
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    if (timeout <= 0)
        return -1.0f;

    double elapsed_us =
        (end.tv_sec - start.tv_sec) * 1000000.0 +
        (end.tv_nsec - start.tv_nsec) / 1000.0;

    return (float)(elapsed_us * 0.0343 / 2.0);
}

void set_servo_angle(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    int high_us = 500 + (angle * 2000 / 180);
    int low_us = 20000 - high_us;

    for (int i = 0; i < 10; i++) {
        gpio_write(GPIO_PIN_SERVO, true);
        nanosleep(&(struct timespec){0, high_us * 1000L}, NULL);
        gpio_write(GPIO_PIN_SERVO, false);
        nanosleep(&(struct timespec){0, low_us * 1000L}, NULL);
    }
}

void gpio_sys_cleanup(void)
{
    if (gpio_regs != 0) {
        munmap_device_io(gpio_regs, GPIO_LEN);
        gpio_regs = 0;
    }
}
