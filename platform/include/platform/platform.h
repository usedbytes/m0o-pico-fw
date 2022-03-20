/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>

#include "pico/sync.h"
#include "pico/util/queue.h"

#include "bno055.h"
#include "chassis.h"
#include "controller.h"
#include "i2c_bus.h"
#include "ioexpander.h"
#include "kinematics.h"

#define PLATFORM_ALARM_POOL_SIZE    16

typedef void (*scheduled_func_t)(absolute_time_t scheduled, void *data);

enum pid_controller_id {
	PID_CONTROLLER_ID_LIFT   = 1,
	PID_CONTROLLER_ID_EXTEND = 2,
};

enum trajectory_adjust {
	TRAJECTORY_ADJUST_RELATIVE_TO_START   = 1,
	TRAJECTORY_ADJUST_RELATIVE_TO_TARGET  = 2,
	TRAJECTORY_ADJUST_RELATIVE_TO_CURRENT = 3,
	TRAJECTORY_ADJUST_SET_ABSOLUTE        = 4,
	TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT = 5,
};

struct platform_message {
#define PLATFORM_MESSAGE_RUN               1
#define PLATFORM_MESSAGE_VELOCITY          2
#define PLATFORM_MESSAGE_BOOM_HOME         3
#define PLATFORM_MESSAGE_BOOM_SET          4
#define PLATFORM_MESSAGE_BOOM_SET_ENABLED  5
#define PLATFORM_MESSAGE_BOOM_EXTEND_SET   6
#define PLATFORM_MESSAGE_BOOM_EXTEND_SET_ENABLED  7
#define PLATFORM_MESSAGE_IOE_SET          10
#define PLATFORM_MESSAGE_SERVO_LEVEL_SET_ENABLED  11
#define PLATFORM_MESSAGE_PID_SET          12
#define PLATFORM_MESSAGE_BOOM_UPDATE      13
#define PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET         14
#define PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET_ENABLED 15
#define PLATFORM_MESSAGE_ALL_STOP         16
#define PLATFORM_MESSAGE_BOOM_TRAJECTORY_ADJUST      17
#define PLATFORM_MESSAGE_BOOM_SET_RAW     18
#define PLATFORM_MESSAGE_STATUS_REQUEST   19
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
			uint8_t pin;
			uint8_t pad;
			uint16_t val;
		} ioe_set;
		struct {
			bool enabled;
		} servo_level_enable;
		struct {
			enum pid_controller_id id;
			float kp, ki, kd;
		} pid_set;
		struct {
			struct v2 start, target;
		} trajectory;
		struct {
			bool enabled;
		} trajectory_enable;
		struct {
			struct v2 amount;
			enum trajectory_adjust relative_to;
		} trajectory_adjust;
		struct {
			int8_t lift;
			int8_t extend;
		} boom_set_raw;
		struct {
			struct platform_status_report *dst;
		} status_request;
	};
};

struct platform_alarm_slot {
	struct platform *volatile platform;
	struct platform_message msg;
};

enum boom_home_state {
	BOOM_HOME_START = 1,
	BOOM_HOME_EXTENDING,
	BOOM_HOME_RETRACTING,
	BOOM_HOME_RETRACT_STOPPED,
	BOOM_HOME_RAISING,
	BOOM_HOME_LOWERING,
	BOOM_HOME_LOWER_STOPPED,
	BOOM_HOME_DONE,
	BOOM_HOME_ERROR = BOOM_HOME_DONE + 1,
};

struct platform {
	queue_t queue;

	alarm_pool_t *alarm_pool;
	critical_section_t slot_lock;
	struct platform_alarm_slot slots[PLATFORM_ALARM_POOL_SIZE];

#define PLATFORM_STATUS_BNO055_PRESENT      (1 << 0)
#define PLATFORM_STATUS_BNO055_OK           (1 << 1)
#define PLATFORM_STATUS_IOE_PRESENT         (1 << 2)
#define PLATFORM_STATUS_IOE_OK              (1 << 3)
#define PLATFORM_STATUS_BOOM_HOMED          (1 << 4)
#define PLATFORM_STATUS_BOOM_TARGET_REACHED (1 << 5)
	uint32_t status;

	struct i2c_bus i2c_main;
	struct bno055 bno055;
	struct chassis chassis;

	struct i2c_bus i2c_aux;
	struct ioexpander ioe;

	absolute_time_t heading_timestamp;
	uint32_t heading_update_us;
	int16_t heading;

#define CONTROLLER_BOOM_LIFT       (1 << 0)
#define CONTROLLER_BOOM_EXTEND     (1 << 1)
#define CONTROLLER_BOOM_TRAJECTORY (1 << 2)
#define CONTROLLER_BOOM_FORK_LEVEL (1 << 3)
	uint32_t controllers_enabled;

	enum boom_home_state boom_home_state;
	struct fcontroller boom_lift_controller;
	struct fcontroller boom_extend_pos_controller;

	absolute_time_t boom_timestamp;
	struct v2 boom_current;

	struct {
		struct v2 start;
		struct v2 target;
		struct v2 unit;
		float mag;
	} trajectory;
};

int platform_init(struct platform *platform);

void platform_run(struct platform *platform);

int platform_set_velocity(struct platform *platform, int8_t linear, int8_t angular);
int platform_boom_home(struct platform *platform);

int platform_boom_lift_controller_set(struct platform *platform, float degrees);
int platform_boom_lift_controller_set_enabled(struct platform *platform, bool enabled);

int platform_boom_extend_controller_set(struct platform *platform, float mm);
int platform_boom_extend_controller_set_enabled(struct platform *platform, bool enabled);

alarm_id_t platform_schedule_function(struct platform *platform, scheduled_func_t func, void *data, absolute_time_t at);
int platform_run_function(struct platform *platform, scheduled_func_t func, void *data);

int platform_boom_set_raw(struct platform *platform, int8_t lift, int8_t extend);

int platform_ioe_set(struct platform *platform, uint8_t pin, uint16_t val);

int platform_servo_level(struct platform *platform, bool enabled);

int platform_set_pid_coeffs(struct platform *platform, enum pid_controller_id id, float kp, float ki, float kd);

int platform_boom_update_position(struct platform *platform);

int platform_boom_trajectory_controller_set(struct platform *platform, struct v2 start, struct v2 target);
int platform_boom_trajectory_controller_adjust_target(struct platform *platform, struct v2 vec, enum trajectory_adjust relative_to);
int platform_boom_trajectory_controller_set_enabled(struct platform *platform, bool enabled);

int platform_all_stop(struct platform *platform);

struct platform_status_report {
	absolute_time_t timestamp;
	uint32_t status;
	int16_t heading;
	struct v2 boom_pos;

	volatile bool complete;
};

int platform_get_status(struct platform *platform, struct platform_status_report *dst);

#endif /* __PLATFORM_H__ */
