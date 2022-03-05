/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>

#include "pico/util/queue.h"

#include "bno055.h"
#include "chassis.h"
#include "i2c_bus.h"

#define PLATFORM_ALARM_POOL_SIZE    16

typedef void (*scheduled_func_t)(absolute_time_t scheduled, void *data);

struct platform_message {
#define PLATFORM_MESSAGE_RUN       1
#define PLATFORM_MESSAGE_VELOCITY  2
#define PLATFORM_MESSAGE_BOOM_HOME 3
	uint8_t type;
	uint8_t pad[3];
	union {
		struct {
			absolute_time_t scheduled;
			scheduled_func_t func;
			void *arg;
		} run;
		struct {
			int8_t linear;
			int8_t angular;
		} velocity;
	};
};

struct platform_alarm_slot {
	struct platform *volatile platform;
	struct platform_message msg;
};

enum boom_extend_state {
	BOOM_EXTEND_HOME_START = 1,
	BOOM_EXTEND_HOME_EXTENDING,
	BOOM_EXTEND_HOME_RETRACTING,
	BOOM_EXTEND_HOME_STOPPED,
	// Order is important
	BOOM_EXTEND_HOME_DONE,
	BOOM_EXTEND_HOME_ERROR,
};

enum boom_lift_state {
	BOOM_LIFT_HOME_START = 1,
	BOOM_LIFT_HOME_RAISING,
	BOOM_LIFT_HOME_LOWERING,
	BOOM_LIFT_HOME_STOPPED,
	// Order is important
	BOOM_LIFT_HOME_DONE,
	BOOM_LIFT_HOME_ERROR,
};

struct platform {
	queue_t queue;

	alarm_pool_t *alarm_pool;
	critical_section_t slot_lock;
	struct platform_alarm_slot slots[PLATFORM_ALARM_POOL_SIZE];

#define PLATFORM_STATUS_BNO055_PRESENT (1 << 0)
#define PLATFORM_STATUS_BNO055_OK      (1 << 1)
	uint32_t status;

	struct i2c_bus i2c_main;
	struct bno055 bno055;
	struct chassis chassis;

	struct i2c_bus i2c_aux;

	absolute_time_t heading_timestamp;
	uint32_t heading_update_us;
	int16_t heading;

	enum boom_extend_state boom_extend_state;
	enum boom_lift_state boom_lift_state;
};

int platform_init(struct platform *platform);

void platform_run(struct platform *platform);

/* TODO
void platform_set_boom(struct platform *platform, uint16_t height, uint16_t extension);
void platform_drive_heading(struct platform *platform, int8_t linear_speed, int16_t heading);
void platform_stop(struct platform *platform);
*/

int platform_set_velocity(struct platform *platform, int8_t linear, int8_t angular);
int platform_boom_home(struct platform *platform);

alarm_id_t platform_schedule_function(struct platform *platform, scheduled_func_t func, void *data, absolute_time_t at);
int platform_run_function(struct platform *platform, scheduled_func_t func, void *data);

#endif /* __PLATFORM_H__ */
