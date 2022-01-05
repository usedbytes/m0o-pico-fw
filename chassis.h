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
};

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra);

void chassis_set(struct chassis *chassis, int8_t l, int8_t r);

#endif /* __CHASSIS_H__ */
