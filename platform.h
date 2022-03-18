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
#include "controller.h"
#include "i2c_bus.h"
#include "ioexpander.h"

#define PLATFORM_ALARM_POOL_SIZE    16

typedef void (*scheduled_func_t)(absolute_time_t scheduled, void *data);

struct platform_message {
#define PLATFORM_MESSAGE_RUN               1
#define PLATFORM_MESSAGE_VELOCITY          2
#define PLATFORM_MESSAGE_BOOM_HOME         3
#define PLATFORM_MESSAGE_BOOM_SET          4
#define PLATFORM_MESSAGE_BOOM_SET_ENABLED  5
#define PLATFORM_MESSAGE_BOOM_EXTEND_SET   6
#define PLATFORM_MESSAGE_BOOM_EXTEND_SET_ENABLED  7
#define PLATFORM_MESSAGE_BOOM_TARGET_SET   8
#define PLATFORM_MESSAGE_BOOM_TARGET_SET_ENABLED  9
#define PLATFORM_MESSAGE_IOE_SET          10
#define PLATFORM_MESSAGE_SERVO_LEVEL_SET_ENABLED  11
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
		struct {
			int16_t angle;
		} boom_set;
		struct {
			bool enabled;
		} boom_enable;
		struct {
			int16_t count;
		} boom_extend_set;
		struct {
			bool enabled;
		} boom_extend_enable;
		struct {
			int16_t x_mm;
			int16_t y_mm;
		} boom_target_set;
		struct {
			bool enabled;
		} boom_target_enable;
		struct {
			uint8_t pin;
			uint8_t pad;
			uint16_t val;
		} ioe_set;
		struct {
			bool enabled;
		} servo_level_enable;
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

struct boom_position {
	int16_t x, y;
};

struct platform {
	queue_t queue;

	alarm_pool_t *alarm_pool;
	critical_section_t slot_lock;
	struct platform_alarm_slot slots[PLATFORM_ALARM_POOL_SIZE];

#define PLATFORM_STATUS_BNO055_PRESENT (1 << 0)
#define PLATFORM_STATUS_BNO055_OK      (1 << 1)
#define PLATFORM_STATUS_IOE_PRESENT    (1 << 2)
#define PLATFORM_STATUS_IOE_OK         (1 << 3)
	uint32_t status;

	struct i2c_bus i2c_main;
	struct bno055 bno055;
	struct chassis chassis;

	struct i2c_bus i2c_aux;
	struct ioexpander ioe;

	absolute_time_t heading_timestamp;
	uint32_t heading_update_us;
	int16_t heading;

	enum boom_extend_state boom_extend_state;
	enum boom_lift_state boom_lift_state;

	struct fcontroller boom_lift_pos_controller;
	bool boom_lift_controller_enabled;

	struct fcontroller boom_extend_pos_controller;
	bool boom_extend_controller_enabled;

	struct boom_position boom_pos_target;
	bool boom_target_controller_enabled;

	bool servo_level_enabled;
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

int platform_boom_lift_controller_set(struct platform *platform, float degrees);
int platform_boom_lift_controller_set_enabled(struct platform *platform, bool enabled);

int platform_boom_extend_controller_set(struct platform *platform, float mm);
int platform_boom_extend_controller_set_enabled(struct platform *platform, bool enabled);

alarm_id_t platform_schedule_function(struct platform *platform, scheduled_func_t func, void *data, absolute_time_t at);
int platform_run_function(struct platform *platform, scheduled_func_t func, void *data);

int platform_boom_target_controller_set(struct platform *platform, int16_t x_mm, int16_t y_mm);
int platform_boom_target_controller_set_enabled(struct platform *platform, bool enabled);

int platform_ioe_set(struct platform *platform, uint8_t pin, uint16_t val);

int platform_servo_level(struct platform *platform, bool enabled);

#endif /* __PLATFORM_H__ */
