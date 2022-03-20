/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "log.h"
#include "planner.h"
#include "platform.h"
#include "util.h"

enum pick_state {
	PICK_STATE_IDLE = 0,
	PICK_STATE_START,
	PICK_STATE_PUSH,
	PICK_STATE_LIFT,
	PICK_STATE_PICK,
	PICK_STATE_RETURN,
};

struct apple_task {
	struct planner_task base;
	enum pick_state pick_state;
	struct v2 start_pos;
};

const struct v2 push_offset = {  60,   0 };
const struct v2 lift_offset = {   0,  35 };
const struct v2 pick_offset = { -60,   0 };

static void apple_task_tick(const struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
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

static void apple_task_on_start(const struct planner_task *ptask)
{
	struct apple_task *task = (struct apple_task *)ptask;
	task->pick_state = PICK_STATE_START;
}

const struct planner_task *apples_get_task()
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
