/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>

#include "log.h"
#include "planner.h"
#include "util.h"

#include "platform/platform.h"

enum pick_state {
	PICK_STATE_IDLE,
	PICK_STATE_BOOM_APPROACH,
	PICK_STATE_BOOM_APPROACH_WAIT,
	PICK_STATE_CHASSIS_APPROACH,
	PICK_STATE_PICK_REACH_START,
	PICK_STATE_PICK_REACH,
	PICK_STATE_PICK_CLOSE,
	PICK_STATE_PICK_CLOSE_WAIT,
	PICK_STATE_PICK_CHASSIS_BACK_UP,
	PICK_STATE_BOOM_DROP,
	PICK_STATE_BOOM_DROP_WAIT,
	PICK_STATE_EJECT,
	PICK_STATE_EJECT_WAIT,
	PICK_STATE_NEXT,
};

struct apple_task {
	struct planner_task base;
	//enum pick_state pick_state;
	struct v2 start_pos;

	enum pick_state pick_state;
	bool state_waiting;
	int apple_idx;
	absolute_time_t timestamp;
	bool sequence;
	uint16_t target_distance;
};

#define APPLE_SERVO_APPROACH 7500
#define APPLE_SERVO_PICK     3750
#define APPLE_SERVO_EJECT    2000

const struct v2 approach_boom_targets[] = {
	{ 5, 124 },
	{ 25, 255 },
};

const unsigned int num_apples = sizeof(approach_boom_targets) / sizeof(approach_boom_targets[0]);

const struct v2 drop_boom_targets[] = {
	{ -25, 255 },
	{ -25, 255 },
};

const int reach_distances[] = {
	20,
	5,
};

static void apple_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (input->buttons.pressed & BTN_CROSS) {
		task->apple_idx = 0;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state = PICK_STATE_BOOM_APPROACH;
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		task->apple_idx = 1;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state = PICK_STATE_BOOM_APPROACH;
	}
}

static void apple_boom_approach(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	log_printf(&util_logger, "start %d", task->apple_idx);

	platform_ioe_set(platform, 2, APPLE_SERVO_APPROACH);

	platform_servo_level(platform, true);
	platform_boom_trajectory_controller_adjust_target(platform, approach_boom_targets[task->apple_idx],
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state = PICK_STATE_BOOM_APPROACH_WAIT;
}

static void apple_boom_approach_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const int8_t speed = 6;
	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	platform_set_velocity(platform, speed, 0);
	task->pick_state = PICK_STATE_CHASSIS_APPROACH;

	return;
}

static void apple_chassis_approach(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (status->adc < 3000) {
		return;
	}

	log_printf(&util_logger, "beam broken.");

	platform_set_velocity(platform, 0, 0);

	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);

	platform_set_velocity(platform, 0, 0);
	task->pick_state = PICK_STATE_PICK_REACH_START;
}

static void apple_pick_reach_start(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const uint16_t reach_distance = reach_distances[task->apple_idx];
	const int8_t speed = 3;

	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	task->target_distance = status->front_laser.range_mm - reach_distance;
	if (task->target_distance < 30) {
		task->target_distance = 30;
	}
	log_printf(&util_logger, "reach start: %d -> %d", status->front_laser.range_mm, task->target_distance);

	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	platform_set_velocity(platform, speed, 0);
	task->pick_state = PICK_STATE_PICK_REACH;
}

static void apple_pick_reach(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const int tolerance = 2;
	const int8_t speed = 2;

	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	int diff = status->front_laser.range_mm - task->target_distance;
	log_printf(&util_logger, "reach diff: %d", diff);

	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);

	if (diff > tolerance) {
		// Get closer
		platform_set_velocity(platform, speed, 0);
	} else {
		platform_set_velocity(platform, 0, 0);
		task->pick_state = PICK_STATE_PICK_CLOSE;
	}
}

static void apple_pick_close(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	platform_ioe_set(platform, 2, APPLE_SERVO_PICK);
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	task->pick_state = PICK_STATE_PICK_CLOSE_WAIT;
}

static void apple_pick_close_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const int back_up_distance = 70;
	const int8_t speed = 6;
	const uint32_t wait_time_us = 300000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	log_printf(&util_logger, "close waiting: %"PRId64, diff);
	if (diff <= wait_time_us) {
		return;
	}

	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	task->target_distance = status->front_laser.range_mm + back_up_distance;
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	platform_set_velocity(platform, -speed, 0);
	task->pick_state = PICK_STATE_PICK_CHASSIS_BACK_UP;
}

static void apple_pick_chassis_back_up(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const int tolerance = 2;
	const int8_t speed = 6;

	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	int diff = status->front_laser.range_mm - task->target_distance;

	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);

	log_printf(&util_logger, "backup diff: %d", diff);

	if (diff < -tolerance) {
		platform_set_velocity(platform, -speed, 0);
	} else {
		platform_set_velocity(platform, 0, 0);
		task->pick_state = PICK_STATE_BOOM_DROP;
	}
}

static void apple_boom_drop(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	log_printf(&util_logger, "move to drop");

	platform_boom_trajectory_controller_adjust_target(platform, drop_boom_targets[task->apple_idx],
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state = PICK_STATE_BOOM_DROP_WAIT;
}

static void apple_boom_drop_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	task->pick_state = PICK_STATE_EJECT;
}

static void apple_eject(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	platform_ioe_set(platform, 2, APPLE_SERVO_EJECT);
	task->timestamp = get_absolute_time();
	task->pick_state = PICK_STATE_EJECT_WAIT;
}

static void apple_eject_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const uint32_t wait_time_us = 300000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff <= wait_time_us) {
		return;
	}

	platform_ioe_set(platform, 2, APPLE_SERVO_APPROACH);
	task->pick_state = PICK_STATE_NEXT;
}


static void apple_next(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	task->apple_idx++;

	if ((!task->sequence) || (task->apple_idx >= num_apples)) {
		task->apple_idx = 0;
		task->pick_state = PICK_STATE_IDLE;
	} else {
		task->pick_state = PICK_STATE_BOOM_APPROACH;
	}
}

static void apple_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct apple_task *task = (struct apple_task *)ptask;

	switch (task->pick_state) {
	case PICK_STATE_BOOM_APPROACH:
		apple_boom_approach(task, platform, status);
		break;
	case PICK_STATE_BOOM_APPROACH_WAIT:
		apple_boom_approach_wait(task, platform, status);
		break;
	case PICK_STATE_CHASSIS_APPROACH:
		apple_chassis_approach(task, platform, status);
		break;
	case PICK_STATE_PICK_REACH_START:
		apple_pick_reach_start(task, platform, status);
		break;
	case PICK_STATE_PICK_REACH:
		apple_pick_reach(task, platform, status);
		break;
	case PICK_STATE_PICK_CLOSE:
		apple_pick_close(task, platform, status);
		break;
	case PICK_STATE_PICK_CLOSE_WAIT:
		apple_pick_close_wait(task, platform, status);
		break;
	case PICK_STATE_PICK_CHASSIS_BACK_UP:
		apple_pick_chassis_back_up(task, platform, status);
		break;
	case PICK_STATE_BOOM_DROP:
		apple_boom_drop(task, platform, status);
		break;
	case PICK_STATE_BOOM_DROP_WAIT:
		apple_boom_drop_wait(task, platform, status);
		break;
	case PICK_STATE_EJECT:
		apple_eject(task, platform, status);
		break;
	case PICK_STATE_EJECT_WAIT:
		apple_eject_wait(task, platform, status);
		break;
	case PICK_STATE_NEXT:
		apple_next(task, platform, status);
		break;
	case PICK_STATE_IDLE:
		break;
	}
}

static void apple_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct apple_task *task = (struct apple_task *)ptask;
	task->pick_state = PICK_STATE_IDLE;
}

struct planner_task *apples_get_task()
{
	static struct apple_task task = {
		.base = {
			.on_start = apple_task_on_start,
			.handle_input = apple_task_handle_input,
			.tick = apple_task_tick,
		},
		//.pick_state = 0,
		.pick_state = PICK_STATE_IDLE,
		.apple_idx = 0,
	};

	return &task.base;
}
