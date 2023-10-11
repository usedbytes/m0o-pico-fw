/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "planner.h"
#include "util.h"

struct spin_task {
	struct planner_task base;

	bool raising;
	bool ik;
};

const struct v2 lowered_pos = { 30.0, -45 };
const struct v2 raised_pos = { 30.0, 120 };

static void spin_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct spin_task *task = (struct spin_task *)ptask;
	platform_ioe_pwm_set_enabled(platform, 2, false);
	platform_servo_level(platform, false);

	task->ik = true;
	task->raising = false;
	platform_boom_trajectory_controller_adjust_target(platform, lowered_pos,
			TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT);
	platform_boom_trajectory_controller_adjust_target(platform, lowered_pos,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
}

static void spin_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct spin_task *task = (struct spin_task *)ptask;

	if (input->buttons.pressed & BTN_CROSS) {
		platform_set_velocity(platform, 0, -40);
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		task->ik = !task->ik;
	}
}

static void spin_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct spin_task *task = (struct spin_task *)ptask;

	if (task->ik) {
		if (status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED) {
			if (task->raising) {
				platform_boom_trajectory_controller_adjust_target(platform, lowered_pos,
						TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT);
				platform_boom_trajectory_controller_adjust_target(platform, lowered_pos,
						TRAJECTORY_ADJUST_SET_ABSOLUTE);
			} else {
				platform_boom_trajectory_controller_adjust_target(platform, raised_pos,
						TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT);
				platform_boom_trajectory_controller_adjust_target(platform, raised_pos,
						TRAJECTORY_ADJUST_SET_ABSOLUTE);
			}
			task->raising = !task->raising;
			platform_boom_trajectory_controller_set_enabled(platform, true);
		}
	} else {
		if (task->raising && status->boom_pos.y >= raised_pos.y) {
			platform_boom_set_raw(platform, -80, 0);
			task->raising = false;
		} else if (!task->raising && status->boom_pos.y <= lowered_pos.y) {
			platform_boom_set_raw(platform, 80, 0);
			task->raising = true;
		}
	}
}

struct planner_task *spin_get_task(void)
{
	static struct spin_task task = {
		.base = {
			.on_start = spin_task_on_start,
			.handle_input = spin_task_handle_input,
			.tick = spin_task_tick,
		},
	};

	return &task.base;
}
