/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdlib.h>

#include "log.h"
#include "util.h"

#include "vl53l0x.h"
#include "platform_vl53l0x.h"

enum vl53l0x_state {
	LASER_ERROR = -1,
	LASER_STOPPED = 0,
	LASER_SINGLE_START,
	LASER_SINGLE_PENDING,
	LASER_CONTINUOUS_START,
	LASER_CONTINUOUS_PENDING,
};

struct platform_vl53l0x {
	struct platform *platform;
	struct vl53l0x_dev dev;
	enum vl53l0x_state state;
	absolute_time_t timestamp;
	uint16_t range_mm;
	uint8_t range_status;
};

struct platform_vl53l0x *platform_vl53l0x_init(struct platform *platform, struct i2c_bus *bus)
{
	struct platform_vl53l0x *sens = calloc(1, sizeof(struct platform_vl53l0x));
	if (!sens) {
		return NULL;
	}

	sens->platform = platform;
	sens->dev = (struct vl53l0x_dev){
		.i2c = bus,
		.addr_7b = 0x29,
	};

	int ret = vl53l0x_init(&sens->dev);
	log_printf(&util_logger, "vl53l0x init: %d", ret);
	if (ret) {
		log_printf(&util_logger, "init fail: %x", vl53l0x_get_platform_error());
		free(sens);
		return NULL;
	}

	ret = vl53l0x_set_measurement_mode(&sens->dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		log_printf(&util_logger, "vl53l0x set mode: %d", ret);
		free(sens);
		return NULL;
	}

	sens->range_status = 0xff;

	return sens;
}

void platform_vl53l0x_get_status(struct platform_vl53l0x *sens, absolute_time_t *ts, uint16_t *range_mm, uint8_t *range_status)
{
	*ts = sens->timestamp;
	*range_mm = sens->range_mm;
	*range_status = sens->range_status;
}

static void platform_vl53l0x_run(absolute_time_t scheduled, void *data)
{
	const uint32_t poll_time_us = 4000;
	struct platform_vl53l0x *sens = data;
	int ret;
	VL53L0X_RangingMeasurementData_t meas;

	bool trigger = false;
	bool pending = false;
	enum vl53l0x_state next_state = sens->state;

	switch (sens->state) {
	case LASER_STOPPED:
	case LASER_ERROR:
		return;
	case LASER_SINGLE_START:
		trigger = true;
		next_state = LASER_SINGLE_PENDING;
		break;
	case LASER_CONTINUOUS_START:
		trigger = true;
		next_state = LASER_CONTINUOUS_PENDING;
		break;
	case LASER_SINGLE_PENDING:
		pending = true;
		// next_state depends on whether the current reading is finished.
		break;
	case LASER_CONTINUOUS_PENDING:
		pending = true;
		// trigger depends on whether the current reading is finished.
		// next_state = current state.
		break;
	}

	if (pending) {
		ret = vl53l0x_check_measurement_ready(&sens->dev);
		if (ret < 0) {
			sens->range_status = 0xff;
			sens->state = LASER_ERROR;
			return;
		} else if (ret == 1) {
			ret = vl53l0x_get_measurement(&sens->dev, &meas);
			if (ret < 0) {
				sens->range_status = 0xff;
				sens->state = LASER_ERROR;
				return;
			}

			sens->timestamp = scheduled;
			sens->range_mm = meas.RangeMilliMeter;
			sens->range_status = meas.RangeStatus;

			if (sens->state == LASER_SINGLE_PENDING) {
				next_state = LASER_STOPPED;
			}

			if (next_state != LASER_STOPPED) {
				trigger = true;
			}
		}
	}

	if (trigger) {
		ret = vl53l0x_start_measurement(&sens->dev);
		if (ret < 0) {
			sens->range_status = 0xff;
			sens->state = LASER_ERROR;
			return;
		}
	}

	sens->state = next_state;

	if (sens->state > LASER_STOPPED) {
		platform_schedule_function(sens->platform, platform_vl53l0x_run,
					   sens, get_absolute_time() + poll_time_us);
	}
}

void __platform_vl53l0x_set_continuous(struct platform_vl53l0x *sens, bool enabled)
{
	if (!sens) {
		return;
	}

	if (enabled) {
		if (sens->state > LASER_STOPPED) {
			log_printf(&util_logger, "vl53l0x: already enabled");
			return;
		}

		sens->state = LASER_CONTINUOUS_START;
		platform_schedule_function(sens->platform, platform_vl53l0x_run, sens, get_absolute_time());
	} else if ((sens->state == LASER_CONTINUOUS_START) || (sens->state == LASER_CONTINUOUS_PENDING)) {
		sens->state = LASER_STOPPED;
	}
}

void __platform_vl53l0x_trigger_single(struct platform_vl53l0x *sens)
{
	if (!sens) {
		return;
	}

	if (sens->state > LASER_STOPPED) {
		log_printf(&util_logger, "vl53l0x: already running");
		return;
	}

	sens->state = LASER_SINGLE_START;
	platform_schedule_function(sens->platform, platform_vl53l0x_run, sens, get_absolute_time());
}
