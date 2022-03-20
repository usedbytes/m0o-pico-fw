/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "chassis.h"

#include "log.h"
#include "util.h"

#define PWM_MIN 80
#define PWM_MAX (PWM_MIN + 127)

void init_slice(struct slice *slice, unsigned int slice_num, unsigned int pwm_min, uint8_t pin_a)
{
	slice->slice_num = slice_num;
	slice->pwm_min = pwm_min;
	gpio_set_function(pin_a, GPIO_FUNC_PWM);
	gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);
	pwm_set_wrap(slice->slice_num, slice->pwm_min + 127 + 1);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_A, 0);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_B, 0);
	pwm_set_enabled(slice->slice_num, true);
}

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra)
{

	init_slice(&chassis->slice_l, pwm_gpio_to_slice_num(pin_la), PWM_MIN, pin_la);
	init_slice(&chassis->slice_r, pwm_gpio_to_slice_num(pin_ra), PWM_MIN, pin_ra);
}

static inline uint8_t abs8(int8_t v) {
	return v < 0 ? -v : v;
}

void slice_set_with_brake(struct slice *slice, int8_t value, bool brake)
{
	uint8_t mag = abs8(value);

	if (value == 0) {
		pwm_set_both_levels(slice->slice_num, brake ? slice->pwm_min + 127 : 0, brake ? slice->pwm_min + 127 : 0);
	} else if (value < 0) {
		pwm_set_both_levels(slice->slice_num, slice->pwm_min + mag, 0);
	} else {
		pwm_set_both_levels(slice->slice_num, 0, slice->pwm_min + mag);
	}
}

void slice_set(struct slice *slice, int8_t value)
{
	slice_set_with_brake(slice, value, false);
}

void chassis_set_raw(struct chassis *chassis, int8_t left, int8_t right)
{
	slice_set(&chassis->slice_l, left);
	slice_set(&chassis->slice_r, right);

	chassis->l = left;
	chassis->r = right;
}

void chassis_set(struct chassis *chassis, int8_t linear, int8_t rot)
{
	// Positive rotation == CCW == right goes faster

	if (linear < -127) {
		linear = -127;
	}

	if (rot < -127) {
		rot = -127;
	}

	int l = linear - rot;
	int r = linear + rot;
	int adj = 0;

	if (l > 127) {
		adj = l - 127;
	} else if (l < -127) {
		adj = l + 127;
	}else if (r > 127) {
		adj = r - 127;
	} else if (r < -127) {
		adj = r + 127;
	}

	l -= adj;
	r -= adj;

	chassis_set_raw(chassis, l, r);
}
