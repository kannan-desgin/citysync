#ifndef PCF8591_ADC_H_
#define PCF8591_ADC_H_

#include <stdint.h>

#define PCF8591_I2C_ADDR 0x48

int pcf8591_init(const char *dev_path);
int pcf8591_read_channel(int fd, uint8_t channel, uint8_t *val);
void pcf8591_close(int fd);

#endif
