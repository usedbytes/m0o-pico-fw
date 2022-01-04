// C port of https://github.com/ghirlekar/bno055-python-i2c
// SPDX-License-Identifier: MIT
//
// The MIT License (MIT)
// 
// Copyright (c) 2015 Gaurav Hirlekar
// Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "bno055.h"
#include "log.h"
#include "util.h"

static int bno055_read(struct bno055 *bno055, uint8_t addr, uint8_t *data, size_t len) {
	int ret = i2c_write_blocking(bno055->i2c, bno055->addr, &addr, 1, false);
	if (ret != 1) {
		log_printf(&util_logger, "bno055_read W 0x%02x 1: %d", addr, ret);
		return -1;
	}

	ret = i2c_read_blocking(bno055->i2c, bno055->addr, data, len, false);
	if (ret != len) {
		log_printf(&util_logger, "bno055_read R 0x%02x %d: %d", addr, len, ret);
		return -2;
	}

	return 0;
}

static int bno055_set_page(struct bno055 *bno055, uint8_t page)
{
	int ret;
	ret = i2c_write_blocking(bno055->i2c, bno055->addr, (uint8_t[]){ BNO055_REG_PAGE_ID, page }, 2, false);
	if (ret != 2) {
		return ret;
	}

	return 0;
}
 
int bno055_init(struct bno055 *bno055, i2c_inst_t *i2c, uint8_t addr)
{
	int ret;
	uint8_t rxdata;
	uint8_t txdata[2];

	bno055->i2c = i2c;
	bno055->addr = addr;

	ret = bno055_read(bno055, BNO055_REG_CHIP_ID, &rxdata, 1);
	if (ret) {
		log_printf(&util_logger, "bno055_init read chip ID %d: %x", ret, rxdata);
		return ret;
	}

	if (rxdata != BNO055_ID) {
		log_printf(&util_logger, "bno055_init chip ID: %x", rxdata);
		return -1;
	}

	ret = bno055_set_operation_mode(bno055, BNO055_OPERATION_MODE_CONFIG);
	if (ret) {
		log_printf(&util_logger, "bno055_init set_op_mode config: %d", ret);
		return ret;
	}

	ret = bno055_reset(bno055);
	if (ret) {
		log_printf(&util_logger, "bno055_init reset: %d", ret);
		return ret;
	}

	txdata[0] = BNO055_REG_PWR_MODE;
	txdata[1] = BNO055_POWER_MODE_NORMAL;
	ret = i2c_write_blocking(bno055->i2c, bno055->addr, txdata, 2, false);
	if (ret != 2) {
		log_printf(&util_logger, "bno055_init pwr_mode: %d", ret);
		return ret;
	}

	ret = bno055_set_page(bno055, 0);
	if (ret) {
		log_printf(&util_logger, "bno055_init set page: %d", ret);
		return ret;
	}

	ret = bno055_set_operation_mode(bno055, BNO055_OPERATION_MODE_NDOF);
	if (ret) {
		log_printf(&util_logger, "bno055_init set_op_mode ndof: %d", ret);
		return ret;
	}

	return 0;
}

int bno055_ping(struct bno055 *bno055)
{
	int ret;
	uint8_t rxdata;

	ret = i2c_read_blocking(bno055->i2c, bno055->addr, &rxdata, 1, false);
	if (ret != 1) {
		return ret;
	}

	return rxdata == BNO055_ID ? 0 : -1;
}

int bno055_reset(struct bno055 *bno055)
{
	int i, ret;

	ret = i2c_write_blocking(bno055->i2c, bno055->addr, (uint8_t[]){ BNO055_REG_SYS_TRIGGER, 0x20 }, 2, false);
	if (ret != 2) {
		return ret;
	}

	for (i = 0; i < 10; i++) {
		sleep_ms(100);

		ret = bno055_ping(bno055);
		if (!ret) {
			break;
		}
	}

	return ret;
}

int bno055_set_operation_mode(struct bno055 *bno055, enum bno055_operation_mode mode)
{
	int ret;
	ret = i2c_write_blocking(bno055->i2c, bno055->addr, (uint8_t[]){ BNO055_REG_OPR_MODE, mode }, 2, false);
	if (ret != 2) {
		return ret;
	}

	bno055->mode = mode;
	return 0;
}

int bno055_set_external_crystal(struct bno055 *bno055, bool external)
{
	int ret;
	uint8_t txdata[2];
	uint8_t prev = bno055->mode;

	ret = bno055_set_operation_mode(bno055, BNO055_OPERATION_MODE_CONFIG);
	if (ret) {
		return ret;
	}

	ret = bno055_set_page(bno055, 0);
	if (ret) {
		return ret;
	}

	txdata[0] = BNO055_REG_SYS_TRIGGER;
	txdata[1] = external ? 0x80 : 0;
	ret = i2c_write_blocking(bno055->i2c, bno055->addr, txdata, 2, false);
	if (ret != 2) {
		return ret;
	}

	ret = bno055_set_operation_mode(bno055, prev);
	if (ret) {
		return ret;
	}

	return 0;
}

int bno055_get_system_status(struct bno055 *bno055, uint8_t *stat, uint8_t *syserr, uint8_t *selftest)
{
	int ret;
	uint8_t data[2];

	ret = bno055_set_page(bno055, 0);
	if (ret) {
		return ret;
	}

	ret = bno055_read(bno055, BNO055_REG_SYS_STAT, data, 2);
	if (ret) {
		return ret;
	}
	*stat = data[0];
	*syserr = data[1];

	ret = bno055_read(bno055, BNO055_REG_SELFTEST_RESULT, data, 1);
	if (ret) {
		return ret;
	}
	*selftest = data[0];

	return 0;
}

int bno055_get_calibration(struct bno055 *bno055, uint8_t *calib)
{
	int ret;
	uint8_t rxdata;

	ret = bno055_read(bno055, BNO055_REG_CALIB_STAT, &rxdata, 1);
	if (ret) {
		return ret;
	}
	*calib = rxdata;

	return 0;
}

int bno055_get_temp(struct bno055 *bno055, uint8_t *temp)
{
	int ret;
	uint8_t rxdata;

	ret = bno055_read(bno055, BNO055_REG_TEMP, &rxdata, 1);
	if (ret) {
		return ret;
	}
	*temp = rxdata;

	return 0;
}

int bno055_get_vector(struct bno055 *bno055, enum bno055_vector_type type, float vec[3])
{
	int i, ret;
	int16_t raw[3];
	float scale;

	ret = bno055_read(bno055, type, (uint8_t *)raw, sizeof(raw));
	if (ret) {
		return ret;
	}

	switch (type) {
	case BNO055_VECTOR_MAGNETOMETER:
		scale = 16.0;
		break;
	case BNO055_VECTOR_GYROSCOPE:
		scale = 900.0;
		break;
	case BNO055_VECTOR_EULER:
		scale = 16.0;
		break;
	case BNO055_VECTOR_GRAVITY:
		scale = 100.0;
		break;
	default:
		scale = 1.0;
		break;
	};

	for (i = 0; i < sizeof(raw) / sizeof(raw[0]); i++) {
		vec[i] = (float)(raw[i]) / scale;
	}

	return 0;
}
