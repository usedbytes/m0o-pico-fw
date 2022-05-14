/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLAN_CHASSIS_H__
#define __PLAN_CHASSIS_H__

#include <stdint.h>

#include "pico/time.h"

enum speed_control{
	SPEED_CONTROL_FIXED,
	SPEED_CONTROL_FRONT_DISTANCE,
	SPEED_CONTROL_FRONT_DISTANCE_RELATIVE,
	SPEED_CONTROL_REAR_DISTANCE,
	SPEED_CONTROL_REAR_DISTANCE_RELATIVE,
};

enum heading_control {
	HEADING_CONTROL_FORWARD,
	HEADING_CONTROL_EXPLICIT,
	HEADING_CONTROL_RED_BLOB,
};

enum sensor_state {
	SENSOR_STATE_IDLE = 0,
	SENSOR_STATE_LASER_REQUEST,
	SENSOR_STATE_LASER_PENDING,
	SENSOR_STATE_LASER_DONE,
	SENSOR_STATE_CAMERA_REQUEST,
	SENSOR_STATE_CAMERA_PENDING,
	SENSOR_STATE_CAMERA_DONE,
};

struct chassis_planner {
	enum speed_control speed_control;
	enum heading_control heading_control;
	bool controller_active;

	absolute_time_t last_change_ts;

	int8_t linear_speed;
	int8_t current_linear_speed;

	float current_heading;

#define SENSOR_FRONT_RANGE      (1 << 0)
#define SENSOR_CAMERA           (1 << 1)
#define SENSOR_REAR_RANGE       (1 << 2)
	uint32_t sensors;

	struct {
		enum sensor_state state;
		absolute_time_t trig_ts;
		uint16_t range;
		int left, right, bottom;
		bool frame_done;
		struct camera_buffer *buf;
	} front_sensors;

	struct {
		enum sensor_state state;
		absolute_time_t trig_ts;
		uint16_t range;
	} rear_sensors;

	int16_t target_distance;
	bool target_distance_valid;
	int16_t relative_distance;

	float target_heading;

#define END_COND_FRONT_DISTANCE_EQ    (1 << 0)
#define END_COND_FRONT_DISTANCE_LTE   (1 << 1)
#define END_COND_FRONT_DISTANCE_GTE   (1 << 2)
#define END_COND_REAR_DISTANCE_EQ     (1 << 3)
#define END_COND_REAR_DISTANCE_LTE    (1 << 4)
#define END_COND_REAR_DISTANCE_GTE    (1 << 5)
#define END_COND_BEAM                 (1 << 6)
#define END_COND_HEADING_EQ           (1 << 7)
	uint32_t end_conditions;

	bool active;
};

void chassis_stop(struct chassis_planner *cp);
bool chassis_done(struct chassis_planner *cp);
void chassis_tick(struct chassis_planner *cp, struct platform *platform, struct platform_status_report *status);
void chassis_set_control(
		struct chassis_planner *cp,
		enum speed_control speed_control, int16_t distance, int8_t speed,
		enum heading_control heading_control, float heading,
		uint32_t end_conditions);
#endif /* __PLAN_CHASSIS_H__ */
