/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __I2C_BUS_H__
#define __I2C_BUS_H__

#include <stdint.h>

#include "hardware/i2c.h"
#include "pico/mutex.h"

// Provides shared access to an i2c bus
// Note that a mutex is used, so these functions must not be called from
// an IRQ handler!

struct i2c_bus {
	i2c_inst_t *i2c;
	mutex_t lock;
};

void i2c_bus_init(struct i2c_bus *bus, i2c_inst_t *i2c, uint baudrate);

int i2c_bus_read(struct i2c_bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
int i2c_bus_write(struct i2c_bus *bus, uint8_t dev_addr, uint8_t *data, size_t len);
int i2c_bus_write_byte(struct i2c_bus *bus, uint8_t dev_addr, uint8_t data);

#endif /* __I2C_BUS_H__ */
