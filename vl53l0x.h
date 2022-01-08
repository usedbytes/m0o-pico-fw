/*
 * Copyright (C) 2018 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __VL53L0X_H__
#define __VL53L0X_H__
#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#include "vl53l0x/core/inc/vl53l0x_def.h"
#include "vl53l0x/platform/inc/vl53l0x_platform.h"

struct vl53l0x_dev {
	struct VL53L0X_Dev pal_dev;
	i2c_inst_t *i2c;
	uint8_t addr_7b;

	uint8_t xshut_port;
	uint16_t xshut_pin;

	bool addr_set : 1;
};

int vl53l0x_init(struct vl53l0x_dev *dev);
int vl53l0x_init_array(struct vl53l0x_dev *devs, unsigned ndevs);
int vl53l0x_set_addr(struct vl53l0x_dev *dev, uint8_t new_addr_7b);
int vl53l0x_get_platform_error(void);
int vl53l0x_reset(struct vl53l0x_dev *dev);

int vl53l0x_perform_ref_cal(struct vl53l0x_dev *dev, uint8_t *vhv, uint8_t *phase);
int vl53l0x_load_ref_cal(struct vl53l0x_dev *dev, uint8_t vhv, uint8_t phase);

int vl53l0x_perform_ref_spad(struct vl53l0x_dev *dev, uint32_t *count, uint8_t *type);
int vl53l0x_load_ref_spad(struct vl53l0x_dev *dev, uint32_t count, uint8_t type);

int vl53l0x_perform_offset_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, int32_t *offset_um);
int vl53l0x_load_offset_cal(struct vl53l0x_dev *dev, int32_t offset_um);

int vl53l0x_perform_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, FixPoint1616_t *xtalk);
int vl53l0x_load_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t xtalk);
int vl53l0x_enable_xtalk_compensation(struct vl53l0x_dev *dev);
int vl53l0x_disable_xtalk_compensation(struct vl53l0x_dev *dev);

int vl53l0x_set_measurement_time(struct vl53l0x_dev *dev, uint32_t us);
int vl53l0x_set_long_range_preset(struct vl53l0x_dev *dev);

int vl53l0x_set_measurement_mode(struct vl53l0x_dev *dev, VL53L0X_DeviceModes mode);
int vl53l0x_start_measurement(struct vl53l0x_dev *dev);
int vl53l0x_stop_measurement(struct vl53l0x_dev *dev);
int vl53l0x_check_stop_completed(struct vl53l0x_dev *dev);
int vl53l0x_check_measurement_ready(struct vl53l0x_dev *dev);
int vl53l0x_get_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data);

int vl53l0x_do_single_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data);

#endif /* __VL53L0X_H__ */
