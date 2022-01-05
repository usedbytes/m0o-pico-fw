/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "chassis.h"

#define PWM_MIN 80
#define PWM_MAX (PWM_MIN + 127)

static inline void init_slice(uint slice, uint8_t pin_a)
{
	gpio_set_function(pin_a, GPIO_FUNC_PWM);
	gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);
	pwm_set_wrap(slice, PWM_MAX + 1);
	pwm_set_chan_level(slice, PWM_CHAN_A, 0);
	pwm_set_chan_level(slice, PWM_CHAN_B, 0);
	pwm_set_enabled(slice, true);
}

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra)
{
	chassis->slice_l = (pin_la / 2) % 8;
	chassis->slice_r = (pin_ra / 2) % 8;

	init_slice(chassis->slice_l, pin_la);
	init_slice(chassis->slice_r, pin_ra);
}

static inline uint8_t abs8(int8_t v) {
	return v < 0 ? -v : v;
}

static void slice_set(uint slice, int8_t value)
{
	uint8_t mag = abs8(value);

	if (value == 0) {
		pwm_set_both_levels(slice, 0, 0);
	} else if (value < 0) {
		pwm_set_both_levels(slice, 0, PWM_MIN + mag);
	} else {
		pwm_set_both_levels(slice, PWM_MIN + mag, 0);
	}
}

void chassis_set(struct chassis *chassis, int8_t l, int8_t r)
{
	slice_set(chassis->slice_l, l);
	slice_set(chassis->slice_r, r);
}
