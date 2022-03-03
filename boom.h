/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOM_H__
#define __BOOM_H__
#include "i2c_bus.h"

#include <stdint.h>

int boom_extend_set(int8_t val);
bool boom_extend_at_limit();

int boom_lift_set(int8_t val);
bool boom_lift_at_limit();
int boom_lift_get_angle(int16_t *angle);
int boom_lift_reset_angle();

void boom_init(struct i2c_bus *i2c);

int16_t boom_update_count();
void boom_reset_count();

#endif /* __BOOM_H__ */
