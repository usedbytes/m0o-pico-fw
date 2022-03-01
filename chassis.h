/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>

struct chassis {
	uint slice_l;
	uint slice_r;

	int8_t l;
	int8_t r;
};

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra);

void chassis_set_raw(struct chassis *chassis, int8_t left, int8_t right);

// linear and rot can use the full range, but linear speed will be reduced
// to meet the requested rotational speed.
void chassis_set(struct chassis *chassis, int8_t linear, int8_t rot);

void slice_set(uint slice, int8_t value);
void slice_set_with_brake(uint slice, int8_t value, bool brake);
void init_slice(uint slice, uint8_t pin_a);

#endif /* __CHASSIS_H__ */
