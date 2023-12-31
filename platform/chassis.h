/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>

struct slice {
	unsigned int slice_num;
	unsigned int pwm_min;
};

struct chassis {
	struct slice slice_l;
	struct slice slice_r;

	int8_t l;
	int8_t r;
};

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra);

void chassis_set_raw(struct chassis *chassis, int8_t left, int8_t right);

// linear and rot can use the full range, but linear speed will be reduced
// to meet the requested rotational speed.
void chassis_set(struct chassis *chassis, int8_t linear, int8_t rot);

void slice_set(struct slice *slice, int8_t value);
void slice_set_with_brake(struct slice *slice, int8_t value, bool brake);
void init_slice(struct slice *slice, unsigned int slice_num, unsigned int pwm_min, uint8_t pin_a);

#endif /* __CHASSIS_H__ */
