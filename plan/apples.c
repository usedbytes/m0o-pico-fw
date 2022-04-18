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
	PICK_STATE_IDLE = 0,
	PICK_STATE_START,
	PICK_STATE_PUSH,
	PICK_STATE_LIFT,
	PICK_STATE_PICK,
	PICK_STATE_RETURN,
};

enum pick_state2 {
	PICK_STATE2_IDLE,
	PICK_STATE2_YPOS,
	PICK_STATE2_YPOS_WAIT,
	PICK_STATE2_CHASSIS_XPOS,
	PICK_STATE2_BOOM_XPOS,
	PICK_STATE2_BOOM_XPOS_WAIT,
	PICK_STATE2_BOOM_XPOS_PAUSE,
	PICK_STATE2_BOOM_XPOS_BEAMBROKEN,
	PICK_STATE2_WAIT_FOR_CLOSE,
	PICK_STATE2_PLUCK,
	PICK_STATE2_EJECT,
	PICK_STATE2_WAIT_FOR_EJECT,
};

struct apple_task {
	struct planner_task base;
	//enum pick_state pick_state;
	struct v2 start_pos;

	enum pick_state2 pick_state2;
	int apple_idx;
	absolute_time_t timestamp;
	bool sequence;
};

#define APPLE_SERVO_APPROACH 7500
#define APPLE_SERVO_PICK     3750
#define APPLE_SERVO_EJECT    2000

const float approach_y_targets[] = {
	10.0,
	124.0,
	255.0,
};

const float y_targets[] = {
	0.0,
	124.0,
	255.0,
};

const uint16_t distance_targets[] = {
	268,
	232,
	134,
};

const float boom_x_targets[] = {
	92.0,
	57.0,
	20.0,
};

const float post_beam_xmove[] = {
	38,
	38,
	25,
};

/*
const struct v2 push_offset = {  60,   0 };
const struct v2 lift_offset = {   0,  35 };
const struct v2 pick_offset = { -60,   0 };

static void apple_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct apple_task *task = (struct apple_task *)ptask;

	switch (task->pick_state) {
	case PICK_STATE_START:
		log_printf(&util_logger, "pick start");
		task->start_pos = status->boom_pos;
		platform_servo_level(platform, true);
		platform_boom_trajectory_controller_adjust_target(platform, push_offset,
				TRAJECTORY_ADJUST_RELATIVE_TO_CURRENT);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->pick_state = PICK_STATE_PUSH;
		break;
	case PICK_STATE_PUSH:
		if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
			log_printf(&util_logger, "push not reached");
			break;
		}
		log_printf(&util_logger, "push reached");
		platform_boom_trajectory_controller_adjust_target(platform, lift_offset,
				TRAJECTORY_ADJUST_RELATIVE_TO_CURRENT);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->pick_state = PICK_STATE_LIFT;
		break;
	case PICK_STATE_LIFT:
		if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
			log_printf(&util_logger, "lift not reached");
			break;
		}
		log_printf(&util_logger, "lift reached");
		platform_boom_trajectory_controller_adjust_target(platform, pick_offset,
				TRAJECTORY_ADJUST_RELATIVE_TO_CURRENT);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->pick_state = PICK_STATE_PICK;
		break;
	case PICK_STATE_PICK:
		if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
			log_printf(&util_logger, "pick not reached");
			break;
		}
		log_printf(&util_logger, "pick reached");
		platform_boom_trajectory_controller_adjust_target(platform, task->start_pos,
				TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->pick_state = PICK_STATE_RETURN;
		break;
	case PICK_STATE_RETURN:
		if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
			log_printf(&util_logger, "return not reached");
			break;
		}
		log_printf(&util_logger, "return reached");
		task->pick_state = PICK_STATE_IDLE;
		break;
	case PICK_STATE_IDLE:
		break;
	}
}

static void apple_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct apple_task *task = (struct apple_task *)ptask;
	task->pick_state = PICK_STATE_START;
}

struct planner_task *apples_get_task()
{
	static struct apple_task task = {
		.base = {
			.on_start = apple_task_on_start,
			.tick = apple_task_tick,
		},
		.pick_state = 0,
	};

	return &task.base;
}
*/

static void apple_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (input->buttons.pressed & BTN_CROSS) {
		task->apple_idx = 0;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state2 = PICK_STATE2_YPOS;
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		task->apple_idx = 1;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state2 = PICK_STATE2_YPOS;
	}

	if (input->buttons.pressed & BTN_TRIANGLE) {
		task->apple_idx = 2;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state2 = PICK_STATE2_YPOS;
	}
}

static void apple_ypos(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	log_printf(&util_logger, "start ypos %d: %3.2f", task->apple_idx, y_targets[task->apple_idx]);

	platform_ioe_set(platform, 2, APPLE_SERVO_APPROACH);

	struct v2 abs_target = status->boom_pos;
	//abs_target.y = y_targets[task->apple_idx];
	abs_target.y = approach_y_targets[task->apple_idx];
	abs_target.x = boom_x_targets[task->apple_idx] - 40;

	platform_servo_level(platform, true);
	platform_boom_trajectory_controller_adjust_target(platform, abs_target,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state2 = PICK_STATE2_YPOS_WAIT;
}

static void apple_ypos_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	log_printf(&util_logger, "ypos reached");
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	task->pick_state2 = PICK_STATE2_CHASSIS_XPOS;
}

static void apple_chassis_xpos(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	// Trigger a new reading just in case
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);

	int diff = status->front_laser.range_mm - distance_targets[task->apple_idx];
	log_printf(&util_logger, "chassis: %d, %d", status->front_laser.range_mm, diff);
	if (diff > 3) {
		// Get closer, slowly
		platform_set_velocity(platform, 3, 0);
	} else if (diff < -3) {
		platform_set_velocity(platform, -3, 0);
	} else {
		platform_set_velocity(platform, 0, 0);
		task->pick_state2 = PICK_STATE2_BOOM_XPOS;
	}
}

static void apple_boom_xpos(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (status->adc > 3000) {
		log_printf(&util_logger, "beam already broken.");
		task->timestamp = get_absolute_time();
		platform_boom_trajectory_controller_set_enabled(platform, false);
		task->pick_state2 = PICK_STATE2_BOOM_XPOS_PAUSE;
		return;
	}

	struct v2 abs_target = status->boom_pos;
	abs_target.y = y_targets[task->apple_idx];
	abs_target.x = boom_x_targets[task->apple_idx];

	platform_servo_level(platform, true);
	platform_boom_trajectory_controller_adjust_target(platform, abs_target,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state2 = PICK_STATE2_BOOM_XPOS_WAIT;
}

static void apple_boom_xpos_wait(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (status->adc > 3000) {
		task->timestamp = get_absolute_time();
		platform_boom_trajectory_controller_set_enabled(platform, false);
		task->pick_state2 = PICK_STATE2_BOOM_XPOS_PAUSE;
		return;
	}

	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	log_printf(&util_logger, "target reached, beam not broken!");
	task->pick_state2 = PICK_STATE2_IDLE;
}

static void apple_boom_xpos_pause(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const uint32_t pause_time_us = 300000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff <= pause_time_us) {
		return;
	}

	struct v2 abs_target = status->boom_pos;
	abs_target.y = y_targets[task->apple_idx];
	abs_target.x += post_beam_xmove[task->apple_idx];
	log_printf(&util_logger, "beam broken! target %3.2f -> %3.2f", status->boom_pos.x, abs_target.x);
	platform_servo_level(platform, true);
	platform_boom_trajectory_controller_adjust_target(platform, abs_target,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state2 = PICK_STATE2_BOOM_XPOS_BEAMBROKEN;
}

static void apple_boom_xpos_beambroken(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	platform_ioe_set(platform, 2, APPLE_SERVO_PICK);
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	task->pick_state2 = PICK_STATE2_WAIT_FOR_CLOSE;
}

static void apple_wait_for_close(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const uint32_t wait_time_us = 300000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff <= wait_time_us) {
		return;
	}

	struct v2 abs_target = status->boom_pos;
	abs_target.y = y_targets[task->apple_idx];
	abs_target.x -= 40;
	platform_servo_level(platform, true);
	platform_boom_trajectory_controller_adjust_target(platform, abs_target,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
	task->pick_state2 = PICK_STATE2_PLUCK;
}

static void apple_pluck(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	// Trigger a new reading just in case
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);

	int diff = status->front_laser.range_mm - (distance_targets[task->apple_idx] + 80);
	log_printf(&util_logger, "chassis: %d, %d", status->front_laser.range_mm, diff);
	if (diff > 10) {
		// Get closer, slowly
		platform_set_velocity(platform, 2, 0);
		return;
	} else if (diff < -10) {
		platform_set_velocity(platform, -2, 0);
		return;
	} else {
		platform_set_velocity(platform, 0, 0);
	}

	if (!(status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED)) {
		log_printf(&util_logger, "not reached");
		return;
	}

	/*
	// HAX: Move boom out of the way of the distance sensor
	if (task->apple_idx == 0 && (diff < 0)) {
		struct v2 abs_target = status->boom_pos;
		abs_target.y = approach_y_targets[task->apple_idx] + 10;
		abs_target.x -= 40;
		platform_servo_level(platform, true);
		platform_boom_trajectory_controller_adjust_target(platform, abs_target,
				TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
	} else {
	}
	*/
	task->pick_state2 = PICK_STATE2_EJECT;
}

static void apple_eject(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	platform_ioe_set(platform, 2, APPLE_SERVO_EJECT);
	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 0);
	task->pick_state2 = PICK_STATE2_WAIT_FOR_EJECT;
}

static void apple_wait_for_eject(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const uint32_t wait_time_us = 1000000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff <= wait_time_us) {
		return;
	}

	if (task->sequence) {
		task->apple_idx++;
		if (task->apple_idx >= (sizeof(y_targets) / sizeof(y_targets[0]))) {
			task->apple_idx = 0;
			task->pick_state2 = PICK_STATE2_IDLE;
		} else {
			task->pick_state2 = PICK_STATE2_YPOS;
		}
	} else {
		task->pick_state2 = PICK_STATE2_IDLE;
	}
}

static void apple_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct apple_task *task = (struct apple_task *)ptask;

	switch (task->pick_state2) {
	case PICK_STATE2_YPOS:
		apple_ypos(task, platform, status);
		break;
	case PICK_STATE2_YPOS_WAIT:
		apple_ypos_wait(task, platform, status);
		break;
	case PICK_STATE2_CHASSIS_XPOS:
		apple_chassis_xpos(task, platform, status);
		break;
	case PICK_STATE2_BOOM_XPOS:
		apple_boom_xpos(task, platform, status);
		break;
	case PICK_STATE2_BOOM_XPOS_WAIT:
		apple_boom_xpos_wait(task, platform, status);
		break;
	case PICK_STATE2_BOOM_XPOS_PAUSE:
		apple_boom_xpos_pause(task, platform, status);
		break;
	case PICK_STATE2_BOOM_XPOS_BEAMBROKEN:
		apple_boom_xpos_beambroken(task, platform, status);
		break;
	case PICK_STATE2_WAIT_FOR_CLOSE:
		apple_wait_for_close(task, platform, status);
		break;
	case PICK_STATE2_PLUCK:
		apple_pluck(task, platform, status);
		break;
	case PICK_STATE2_EJECT:
		apple_eject(task, platform, status);
		break;
	case PICK_STATE2_WAIT_FOR_EJECT:
		apple_wait_for_eject(task, platform, status);
		break;
	}
}

static void apple_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct apple_task *task = (struct apple_task *)ptask;
	task->pick_state2 = PICK_STATE2_IDLE;
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
		.pick_state2 = PICK_STATE2_IDLE,
		.apple_idx = 0,
	};

	return &task.base;
}
