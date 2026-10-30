#include "pcf8591_adc.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>

int pcf8591_init(const char *dev_path)
{
    int fd = open(dev_path, O_RDWR);

    if (fd < 0) {
        perror("Unable to open I2C device");
        return -1;
    }

    return fd;
}

int pcf8591_read_channel(int fd, uint8_t channel, uint8_t *val)
{
    if (channel > 3 || val == NULL) {
        return -1;
    }

    struct {
        i2c_sendrecv_t hdr;
        uint8_t buf[2];
    } i2c_cmd = {0};

    uint8_t ctrl_byte = 0x40 | (channel & 0x03);

    i2c_cmd.hdr.slave.addr = PCF8591_I2C_ADDR;
    i2c_cmd.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    i2c_cmd.hdr.send_len = 1;
    i2c_cmd.hdr.recv_len = 2;
    i2c_cmd.hdr.stop = 1;
    i2c_cmd.buf[0] = ctrl_byte;

    if (devctl(fd, DCMD_I2C_SENDRECV,
               &i2c_cmd, sizeof(i2c_cmd), NULL) != 0) {
        perror("I2C read failed");
        return -1;
    }

    *val = i2c_cmd.buf[1];
    return 0;
}

void pcf8591_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}
