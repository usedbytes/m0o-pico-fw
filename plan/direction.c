/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "log.h"
#include "planner.h"
#include "util.h"

#include "platform/platform.h"

struct direction_task {
	struct planner_task base;

	bool set_north;
	float north;
};

static void direction_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct direction_task *task = (struct direction_task *)ptask;

	if (input->hat.pressed & HAT_UP) {
		platform_heading_controller_set(platform, 0, task->north);
	}

	if (input->hat.pressed & HAT_LEFT) {
		platform_heading_controller_set(platform, 0, task->north - 90.0);
	}

	if (input->hat.pressed & HAT_DOWN) {
		platform_heading_controller_set(platform, 0, task->north - 180.0);
	}

	if (input->hat.pressed & HAT_RIGHT) {
		platform_heading_controller_set(platform, 0, task->north + 90.0);
	}
}

static void direction_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct direction_task *task = (struct direction_task *)ptask;

	task->set_north = true;
	platform_heading_controller_set_enabled(platform, true);
}

static float normalise_angle(float degrees)
{
	while (degrees < -180.0) {
		degrees += 360.0;
	}

	while (degrees > 180.0) {
		degrees -= 360.0;
	}

	return degrees;
}

static void direction_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct direction_task *task = (struct direction_task *)ptask;

	if (task->set_north) {
		task->north = normalise_angle(status->heading / 16.0);
		task->set_north = false;
	}
}

struct planner_task *direction_get_task(void)
{
	static struct direction_task task = {
		.base = {
			.on_start = direction_task_on_start,
			.handle_input = direction_task_handle_input,
			.tick = direction_task_tick,
		},
	};

	return &task.base;
}
