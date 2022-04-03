/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLATFORM_VL53L0X_H__
#define __PLATFORM_VL53L0X_H__

#include "platform/platform.h"

#include "i2c_bus.h"

struct platform_vl53l0x;

struct platform_vl53l0x *platform_vl53l0x_init(struct platform *platform, struct i2c_bus *bus);

void __platform_vl53l0x_set_enabled(struct platform_vl53l0x *sens, bool enabled);

void platform_vl53l0x_get_status(struct platform_vl53l0x *sens, absolute_time_t *ts, uint16_t *range_mm, uint8_t *range_status);

#endif /* __PLATFORM_CAMERA_H__ */
