/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "log.h"
#include "planner.h"
#include "util.h"

#include "platform/platform.h"

struct servo_task {
	struct planner_task base;

	int servo;
	uint16_t val, min, max, step;
};

static void servo_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct servo_task *task = (struct servo_task *)ptask;

	if (input->buttons.pressed & BTN_TRIANGLE) {
		platform_ioe_pwm_set_enabled(platform, task->servo, false);
		task->servo++;
		if (task->servo > 6) {
			task->servo = 6;
		}
		platform_ioe_pwm_set_enabled(platform, task->servo, true);
		platform_ioe_set(platform, task->servo, task->val);
		log_printf(&util_logger, "Pick servo %d", task->servo);
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		platform_ioe_pwm_set_enabled(platform, task->servo, false);
		task->servo--;
		if (task->servo < 1) {
			task->servo = 1;
		}
		platform_ioe_pwm_set_enabled(platform, task->servo, true);
		platform_ioe_set(platform, task->servo, task->val);
		log_printf(&util_logger, "Pick servo %d", task->servo);
	}

	if (input->buttons.held & BTN_L2) {
		if ((input->hat.pressed & HAT_DOWN) && (task->max > (task->min + task->step))) {
			task->max -= task->step;
			log_printf(&util_logger, "Max %d", task->max);
		}

		if ((input->hat.pressed & HAT_UP) && (task->max < (0xffff - task->step))) {
			task->max += task->step;
			log_printf(&util_logger, "Max %d", task->max);
		}
	} else if (input->buttons.held & BTN_L1) {
		if ((input->hat.pressed & HAT_DOWN) && (task->min > task->step)) {
			task->min -= task->step;
			log_printf(&util_logger, "Min %d", task->min);
		}

		if ((input->hat.pressed & HAT_UP) && (task->min < (task->max - task->step))) {
			task->min += task->step;
			log_printf(&util_logger, "Min %d", task->min);
		}
	} else {
		if ((input->hat.pressed & HAT_DOWN) && (task->val > (task->min + task->step))) {
			task->val -= task->step;
			log_printf(&util_logger, "Set %d", task->val);
			platform_ioe_set(platform, task->servo, task->val);
		}

		if ((input->hat.pressed & HAT_UP) && (task->val < (task->max - task->step))) {
			task->val += task->step;
			log_printf(&util_logger, "Set %d", task->val);
			platform_ioe_set(platform, task->servo, task->val);
		}

		if (input->hat.pressed & HAT_LEFT) {
			task->val = task->min;
			log_printf(&util_logger, "Set %d", task->val);
			platform_ioe_set(platform, task->servo, task->val);
		}

		if (input->hat.pressed & HAT_RIGHT) {
			task->val = task->max;
			log_printf(&util_logger, "Set %d", task->val);
			platform_ioe_set(platform, task->servo, task->val);
		}
	}
}

static void servo_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct servo_task *task = (struct servo_task *)ptask;
	task->val = task->min + ((task->max - task->min) / 2);

	platform_ioe_set(platform, task->servo, task->val);
}

struct planner_task *servo_get_task()
{
	static struct servo_task task = {
		.base = {
			.on_start = servo_task_on_start,
			.handle_input = servo_task_handle_input,
		},
		.servo = 2,
		.min = 3750,
		.max = 7500,
		.step = 50,
	};

	return &task.base;
}
