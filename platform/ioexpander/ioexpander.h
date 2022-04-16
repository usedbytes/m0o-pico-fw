/*
 * Derived from Pimoroni driver:
 * Copyright (c) 2021 Pimoroni Ltd
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 */
#ifndef __IOEXPANDER_H__
#define __IOEXPANDER_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IOE_DEFAULT_ADDR 0x18

struct ioexpander {
	int (*i2c_write_blocking)(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len);
	int (*i2c_read_blocking)(void *i2c_handle, uint8_t addr, uint8_t *src, size_t len);
	void *i2c_handle;
	uint8_t dev_addr;
};

int ioe_pwm_check_loading(struct ioexpander *ioe);

int ioe_pwm_load(struct ioexpander *ioe, bool wait);

int ioe_set_pwm_period(struct ioexpander *ioe, uint16_t period);

enum ioe_pwm_divider {
	IOE_PWM_DIVIDER_1   = 0x0,
	IOE_PWM_DIVIDER_2   = 0x1,
	IOE_PWM_DIVIDER_4   = 0x2,
	IOE_PWM_DIVIDER_8   = 0x3,
	IOE_PWM_DIVIDER_16  = 0x4,
	IOE_PWM_DIVIDER_32  = 0x5,
	IOE_PWM_DIVIDER_64  = 0x6,
	IOE_PWM_DIVIDER_128 = 0x7,
};

int ioe_set_pwm_divider(struct ioexpander *ioe, enum ioe_pwm_divider divider);

int ioe_set_pwm_duty(struct ioexpander *ioe, unsigned int pin, uint16_t duty);

int ioe_adc_sample(struct ioexpander *ioe, unsigned int pin, uint16_t *out);

enum ioe_pin_mode {
    IOE_PIN_MODE_IN         = 0x02,
    IOE_PIN_MODE_IN_PULL_UP = 0x10,
    IOE_PIN_MODE_OUT        = 0x01,
    IOE_PIN_MODE_OD         = 0x03,
    IOE_PIN_MODE_PWM        = 0x05,
    IOE_PIN_MODE_ADC        = 0x0a,
};

int ioe_set_pin_mode(struct ioexpander *ioe, unsigned int pin, enum ioe_pin_mode mode);

int ioe_init(struct ioexpander *ioe,
		int (*i2c_write)(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len),
		int (*i2c_read)(void *i2c_handle, uint8_t addr, uint8_t *src, size_t len),
		void *i2c_handle);

#endif /* __IOEXPANDER_H__ */
