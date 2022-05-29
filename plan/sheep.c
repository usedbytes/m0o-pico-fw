/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "planner.h"
#include "sheep.h"
#include "plan_boom.h"
#include "util.h"

#define SHEEP_OPEN   1350
#define SHEEP_CLOSED 7500

const float sheep_pickup_y = 33.0;
const float sheep_pickup_x = 30.0;
const float sheep_pen_safe_y = 150.0;
const float pen_safe_y = 60.0;

enum sheep_lift_state {
	SHEEP_LIFT_STATE_IDLE = 0,
	SHEEP_LIFT_STATE_PICKUP,
	SHEEP_LIFT_STATE_PEN_LIFT,
};

struct sheep_task {
	struct planner_task base;

	bool jaw_open;

	struct boom_planner bp;
	enum sheep_lift_state lift_state;
};

static void sheep_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	platform_ioe_pwm_set_enabled(platform, 2, true);
	platform_servo_level(platform, true);
}

static void sheep_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct sheep_task *task = (struct sheep_task *)ptask;

	if (input->buttons.pressed & BTN_L2) {
		task->jaw_open = !task->jaw_open;
		platform_ioe_set(platform, 2, task->jaw_open ? SHEEP_OPEN : SHEEP_CLOSED);
	}

	if ((input->hat.held == 0) && input->hat.released) {
		platform_boom_trajectory_controller_set_enabled(platform, false);
		platform_boom_trajectory_controller_adjust_target(platform,
				(struct v2){ 0, 0 },
				TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT);
		platform_boom_set_raw(platform, 0, 0);
	}

	float speed_scale = 1.0;

	if (input->hat.pressed) {
		if (input->buttons.held & (1 << BTN_BIT_L1)) {
			int8_t lift = 0;
			int8_t extend = 0;

			if (input->hat.pressed & HAT_UP) {
				lift = 127;
			}
			if (input->hat.pressed & HAT_DOWN) {
				lift = -127;
			}
			if (input->hat.pressed & HAT_RIGHT) {
				extend = 127;
			}
			if (input->hat.pressed & HAT_LEFT) {
				extend = -127;
			}

			platform_boom_set_raw(platform, lift, extend);
		} else {
#define RC_BOOM_MANUAL_MOVE 500
			struct v2 dp = { 0, 0 };
			if (input->hat.pressed & HAT_UP) {
				dp.y += RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_DOWN) {
				dp.y -= RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_RIGHT) {
				dp.x += RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_LEFT) {
				dp.x -= RC_BOOM_MANUAL_MOVE;
			}

			platform_boom_trajectory_controller_adjust_target(platform, dp, TRAJECTORY_ADJUST_RELATIVE_TO_START);
			platform_boom_trajectory_controller_set_enabled(platform, true);
		}
	}

	if (input->buttons.pressed & BTN_CROSS) {
		task->jaw_open = true;
		platform_ioe_set(platform, 2, task->jaw_open ? SHEEP_OPEN : SHEEP_CLOSED);
		task->lift_state = SHEEP_LIFT_STATE_PICKUP;
	}

	if (input->buttons.pressed & BTN_TRIANGLE) {
		task->lift_state = SHEEP_LIFT_STATE_PEN_LIFT;
	}

	if (input->buttons.held & BTN_R2) {
		speed_scale = 0.2;
	}

	int8_t linear = clamp8(-(input->axes.ly - 128) * speed_scale);
	int8_t rot = clamp8(-(input->axes.rx - 128));
	platform_set_velocity(platform, linear, rot);
}

static void sheep_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct sheep_task *task = (struct sheep_task *)ptask;

	switch (task->lift_state) {
	case SHEEP_LIFT_STATE_PICKUP:
		platform_boom_trajectory_controller_adjust_target(platform,
				(struct v2){ sheep_pickup_x, sheep_pickup_y },
				TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->lift_state = SHEEP_LIFT_STATE_IDLE;
		break;
	case SHEEP_LIFT_STATE_PEN_LIFT:
		platform_boom_trajectory_controller_adjust_target(platform,
				(struct v2){ status->boom_pos.x, sheep_pen_safe_y },
				TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
		task->lift_state = SHEEP_LIFT_STATE_IDLE;
		break;
	case SHEEP_LIFT_STATE_IDLE:
		break;
	}
}

struct planner_task *sheep_get_task(void)
{
	static struct sheep_task task = {
		.base = {
			.on_start = sheep_task_on_start,
			.handle_input = sheep_task_handle_input,
			.tick = sheep_task_tick,
		},
	};

	return &task.base;
}
