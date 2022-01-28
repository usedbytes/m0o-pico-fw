/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "i2c_bus.h"
#include "log.h"
#include "util.h"

void i2c_bus_init(struct i2c_bus *bus, i2c_inst_t *i2c, uint baudrate)
{
	bus->i2c = i2c;
	mutex_init(&bus->lock);
	i2c_init(bus->i2c, baudrate);
}

int i2c_bus_read_raw(struct i2c_bus *bus, uint8_t dev_addr, uint8_t *data, size_t len)
{
	mutex_enter_blocking(&bus->lock);

	int ret = i2c_read_blocking(bus->i2c, dev_addr, data, len, false);
	if (ret != len) {
		log_printf(&util_logger, "i2c_bus_read_raw R %d: %d", len, ret);
		ret = -2;
		goto done;
	}

	ret = 0;
done:
	mutex_exit(&bus->lock);
	return ret;
}

int i2c_bus_read(struct i2c_bus *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
	int ret;

	mutex_enter_blocking(&bus->lock);

	ret = i2c_write_blocking(bus->i2c, dev_addr, &reg_addr, 1, false);
	if (ret != 1) {
		log_printf(&util_logger, "i2c_bus_read W 0x%02x: %d", reg_addr, ret);
		ret = -1;
		goto done;
	}

	ret = i2c_read_blocking(bus->i2c, dev_addr, data, len, false);
	if (ret != len) {
		log_printf(&util_logger, "i2c_bus_read R 0x%02x %d: %d", reg_addr, len, ret);
		ret = -2;
		goto done;
	}

	ret = 0;
done:
	mutex_exit(&bus->lock);
	return ret;
}

int i2c_bus_write(struct i2c_bus *bus, uint8_t dev_addr, uint8_t *data, size_t len)
{
	int ret;

	mutex_enter_blocking(&bus->lock);

	ret = i2c_write_blocking(bus->i2c, dev_addr, data, len, false);
	if (ret != len) {
		log_printf(&util_logger, "i2c_bus_write W %d: %d", len, ret);
		ret = -1;
		goto done;
	}

	ret = 0;
done:
	mutex_exit(&bus->lock);
	return ret;
}

int i2c_bus_write_byte(struct i2c_bus *bus, uint8_t dev_addr, uint8_t data)
{
	return i2c_bus_write(bus, dev_addr, &data, 1);
}

int i2c_bus_read_blocking(struct i2c_bus *bus, uint8_t addr, uint8_t *dst, size_t len)
{
	int ret;

	mutex_enter_blocking(&bus->lock);

	ret = i2c_read_blocking(bus->i2c, addr, dst, len, false);
	if (ret != len) {
		log_printf(&util_logger, "i2c_bus_read 0x%02x: %d %d", addr, len, ret);
	}

	mutex_exit(&bus->lock);
	return ret;
}

int i2c_bus_write_blocking(struct i2c_bus *bus, uint8_t addr, const uint8_t *src, size_t len)
{
	int ret;

	mutex_enter_blocking(&bus->lock);

	ret = i2c_write_blocking(bus->i2c, addr, src, len, false);
	if (ret != len) {
		log_printf(&util_logger, "i2c_bus_write 0x%02x: %d %d", addr, len, ret);
	}

	mutex_exit(&bus->lock);
	return ret;
}
