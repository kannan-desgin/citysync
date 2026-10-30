#ifndef RPI_GPIO_QNX_H_
#define RPI_GPIO_QNX_H_

#include <stdint.h>
#include <stdbool.h>

#define BCM2711_GPIO_BASE 0xFE200000
#define GPIO_LEN          0x000000B4
#define GPIO_PIN_IR       17
#define GPIO_PIN_FLAME    27
#define GPIO_PIN_TRIG     23
#define GPIO_PIN_ECHO     24
#define GPIO_PIN_SERVO    18

int gpio_sys_init(void);
void gpio_set_mode_input(int pin);
void gpio_set_mode_output(int pin);
void gpio_write(int pin, bool level);
bool gpio_read(int pin);
float measure_ultrasonic_cm(void);
void set_servo_angle(int angle);
void gpio_sys_cleanup(void);

#endif
