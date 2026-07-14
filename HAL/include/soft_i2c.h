#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdint.h>

void soft_i2c_init(void);
int  soft_i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len);
int  soft_i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len);

#endif