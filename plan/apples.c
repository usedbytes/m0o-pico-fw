/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <math.h>

#include "pico/stdlib.h"

#include "camera/camera.h"
#include "camera/format.h"
#include "log.h"
#include "planner.h"
#include "plan_chassis.h"
#include "plan_boom.h"
#include "util.h"

#include "platform/platform.h"

#define APPLE_SERVO_APPROACH 7500
#define APPLE_SERVO_PICK     3750
#define APPLE_SERVO_EJECT    2000

struct apples_tuning {
	int slow_speed;
	int fast_speed;
	int very_fast_speed;
#define NUM_PICKS 2
	int reach_distances[NUM_PICKS];
	int back_up_distances[NUM_PICKS];
	struct v2 approach_boom_targets[NUM_PICKS];
	struct v2 drop_boom_targets[NUM_PICKS];
#define NUM_BRANCHES 4
	int permiter_approach_distances[NUM_BRANCHES];
	enum speed_control perimeter_approach_speed_ctrl;
	uint32_t perimeter_approach_end_cond;

	int corner_approach_distances[NUM_BRANCHES];
	enum speed_control corner_approach_speed_ctrl;
	uint32_t corner_approach_end_cond;
};

#define TUNING_HOME      0
#define TUNING_MAKESPACE 1
const struct apples_tuning apples_tunings[] = {
	[TUNING_HOME] = {
		.slow_speed = 2,
		.fast_speed = 6,
		.very_fast_speed = 24,
		.approach_boom_targets = {
			{ 5, 126 },
			{ 25, 255 },
		},
		.drop_boom_targets = {
			{ -25, 255 },
			{ -25, 255 },
		},
		.reach_distances = { 15, 0 },
		.back_up_distances = { 70, 200 },

		.permiter_approach_distances = {
			650,
			650,
			650,
			650,
		},
		.perimeter_approach_speed_ctrl = SPEED_CONTROL_REAR_DISTANCE,
		.perimeter_approach_end_cond = END_COND_REAR_DISTANCE_GTE,

		.corner_approach_distances = {
			50,
			50,
			50,
			50,
		},
		.corner_approach_speed_ctrl = SPEED_CONTROL_FRONT_DISTANCE,
		.corner_approach_end_cond = END_COND_FRONT_DISTANCE_LTE,
	},
#define BUCKET_ADJ 50
	[TUNING_MAKESPACE] = {
		.slow_speed = 2,
		.fast_speed = 6,
		.very_fast_speed = 24,
		.approach_boom_targets = {
			{ 5, 126 },
			{ 25, 253 },
		},
		.drop_boom_targets = {
			{ -25, 255 },
			{ -25, 255 },
		},
		.reach_distances = { 15, 5 },
		.back_up_distances = { 70 + BUCKET_ADJ, 200 + BUCKET_ADJ },

		.permiter_approach_distances = {
			640 + BUCKET_ADJ,
			628 + BUCKET_ADJ,
			633 + BUCKET_ADJ,
			640 + BUCKET_ADJ,
		},
		.perimeter_approach_speed_ctrl = SPEED_CONTROL_REAR_DISTANCE,
		.perimeter_approach_end_cond = END_COND_REAR_DISTANCE_GTE,

		.corner_approach_distances = {
			60 + BUCKET_ADJ,
			60 + BUCKET_ADJ,
			60 + BUCKET_ADJ,
			60 + BUCKET_ADJ,
		},
		.corner_approach_speed_ctrl = SPEED_CONTROL_FRONT_DISTANCE,
		.corner_approach_end_cond = END_COND_FRONT_DISTANCE_LTE,
	},
};

/*
// Note: Wrong direction!
const int rear_distances_makespace[] = {
	600,
	575,
	575,
	570,
};

const int corner_distances_makespace[] = {
	1350,
	1350,
	1350,
	1350,
};
*/

enum pick_state {
	PICK_STATE_IDLE = 0,
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

enum coordinator_state {
	COORD_STATE_STOP = 0,

	COORD_STATE_PERIMETER_APPROACH,
	COORD_STATE_PERIMETER_APPROACH_SLOW,
	COORD_STATE_TREE_POINT,
	COORD_STATE_TREE_APPROACH,
	COORD_STATE_BRANCH_APPROACH,
	COORD_STATE_BRANCH_POSITION,
	COORD_STATE_APPLE_REACH,
	COORD_STATE_APPLE_CLOSE,
	COORD_STATE_BACK_UP,
	COORD_STATE_MOVE_TO_EJECT,
	COORD_STATE_EJECT_STATIONARY,
	COORD_STATE_TREE_TURN_AWAY,
	COORD_STATE_CORNER_APPROACH,
	COORD_STATE_CORNER_TURN,

	COORD_STATE_TEST_BRANCH_APPROACH,
	COORD_STATE_TEST_WAIT,
	COORD_STATE_TEST_MOVE_TO_EJECT,
	COORD_STATE_TEST_EJECT_STATIONARY,
};

enum picker_state {
	PICKER_STATE_IDLE     = 0,
	PICKER_STATE__ING     = (1 << 2),
	PICKER_STATE_OPEN     = 1,
	PICKER_STATE_OPENING  = PICKER_STATE_OPEN | PICKER_STATE__ING,
	PICKER_STATE_CLOSED   = 2,
	PICKER_STATE_CLOSING  = PICKER_STATE_CLOSED | PICKER_STATE__ING,
	PICKER_STATE_EJECT    = 3,
	PICKER_STATE_EJECTING = PICKER_STATE_EJECT | PICKER_STATE__ING,
};

struct picker_planner {
	enum picker_state requested;
	enum picker_state current;
	absolute_time_t end_timestamp;
};

static void picker_tick(struct picker_planner *pp, struct platform *platform, struct platform_status_report *status)
{
	const uint32_t wait_time_ms = 300;

	if ((pp->current & ~(PICKER_STATE__ING)) != pp->requested) {
		// Request has changed       
		log_printf(&util_logger, "picker changed: 0x%x", pp->requested);
		switch (pp->requested) {
		case PICKER_STATE_OPEN:
			pp->current = pp->requested | PICKER_STATE__ING;
			//platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_APPROACH);
			break;
		case PICKER_STATE_CLOSED:
			pp->current = pp->requested | PICKER_STATE__ING;
			//platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_PICK);
			break;
		case PICKER_STATE_EJECT:
			pp->current = pp->requested | PICKER_STATE__ING;
			//platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_EJECT);
			break;
		case PICKER_STATE_IDLE:
		default:
			pp->current = pp->requested;
			//platform_ioe_pwm_set_enabled(platform, 2, false);
			break;
		}
		pp->end_timestamp = make_timeout_time_ms(wait_time_ms);
	} else if (pp->current & PICKER_STATE__ING) {
		// Waiting to reach
		//log_printf(&util_logger, "picker waiting, current: %d, requested: %d", pp->current, pp->requested);
		absolute_time_t now = get_absolute_time();
		if (to_us_since_boot(now) >= pp->end_timestamp) {
			pp->current = pp->requested;
			//platform_ioe_pwm_set_enabled(platform, 2, false);
		}
	}
}

static void picker_set(struct picker_planner *pp, enum picker_state request)
{
	pp->requested = request & ~PICKER_STATE__ING;
}

static bool picker_done(struct picker_planner *pp)
{
	return pp->requested == pp->current;
}

static enum picker_state picker_state(struct picker_planner *pp)
{
	return pp->current;
}

struct apple_task {
	struct planner_task base;

	const struct apples_tuning *tuning;
	int apple_idx;
	int branch_idx;
	float west;

	bool state_entry;
	bool run_coord;
	enum coordinator_state coord_state;
	enum coordinator_state coord_end_state;
	struct chassis_planner cp;
	struct boom_planner bp;
	struct picker_planner pp;
};

static void coord_set_state(struct apple_task *task, enum coordinator_state state)
{
	task->coord_state = state;
	task->state_entry = true;

	if (task->coord_state == task->coord_end_state) {
		task->coord_state = COORD_STATE_STOP;
	}
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


static void coord_tick(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	bool entering = task->state_entry;
	const struct apples_tuning *tuning = task->tuning;
	task->state_entry = false;

	log_printf(&util_logger, ">>> coord tick: %d, %d", task->coord_state, entering);
	log_printf(&util_logger, "c: %d, p: %d, b: %d",
			chassis_done(&task->cp), picker_done(&task->pp), boom_done(&task->bp));


	switch (task->coord_state) {
	case COORD_STATE_CORNER_TURN:
		if (entering) {
			if (task->branch_idx == 0) {
				task->west = normalise_angle(status->heading / 16.0);
			}
			float heading = normalise_angle(task->west + ((task->branch_idx + 1) * 90));
			//chassis_turn_to(&task->cp, heading, 0);
			chassis_set_control(&task->cp, SPEED_CONTROL_FIXED, 0, 0,
					HEADING_CONTROL_EXPLICIT, heading,
					END_COND_HEADING_EQ);
			picker_set(&task->pp, PICKER_STATE_OPEN);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to PERIMETER_APPROACH");
			coord_set_state(task, COORD_STATE_PERIMETER_APPROACH);
		}
		break;
	case COORD_STATE_PERIMETER_APPROACH:
		if (entering) {
			boom_set(&task->bp, task->tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);

			float heading = normalise_angle(task->west + ((task->branch_idx + 1) * 90));
			chassis_set_control(&task->cp, tuning->perimeter_approach_speed_ctrl,
					tuning->permiter_approach_distances[task->branch_idx] - 100, tuning->very_fast_speed,
					HEADING_CONTROL_EXPLICIT, heading,
					tuning->perimeter_approach_end_cond);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to PERIMETER_APPROACH_SLOW");
			coord_set_state(task, COORD_STATE_PERIMETER_APPROACH_SLOW);
		}
		break;
	case COORD_STATE_PERIMETER_APPROACH_SLOW:
		if (entering) {
			boom_set(&task->bp, tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);

			float heading = normalise_angle(task->west + ((task->branch_idx + 1) * 90));
			chassis_set_control(&task->cp, tuning->perimeter_approach_speed_ctrl,
					tuning->permiter_approach_distances[task->branch_idx], tuning->slow_speed,
					HEADING_CONTROL_EXPLICIT, heading,
					tuning->perimeter_approach_end_cond);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to TREE_POINT");
			coord_set_state(task, COORD_STATE_TREE_POINT);
		}
		break;
	case COORD_STATE_TREE_POINT:
		if (entering) {
			boom_set(&task->bp, tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
			float heading = normalise_angle(task->west + ((task->branch_idx + 2) * 90));
			chassis_set_control(&task->cp, SPEED_CONTROL_FIXED, 0, 0,
					HEADING_CONTROL_EXPLICIT, heading,
					END_COND_HEADING_EQ);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to TREE_APPROACH");
			coord_set_state(task, COORD_STATE_TREE_APPROACH);
		}
		break;
	case COORD_STATE_TREE_APPROACH:
		if (entering) {
			chassis_set_control(&task->cp, SPEED_CONTROL_FRONT_DISTANCE, 210, tuning->fast_speed,
					HEADING_CONTROL_RED_BLOB, 0,
					END_COND_FRONT_DISTANCE_LTE);
			boom_set(&task->bp, tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
		}

		// FIXME: Chassis can get done "momentarily"? and we don't progress
		if (chassis_done(&task->cp) && picker_done(&task->pp) && boom_done(&task->bp)) {
			log_printf(&util_logger, "move to BRANCH_APPROACH");
			coord_set_state(task, COORD_STATE_BRANCH_APPROACH);
		}
		break;
	case COORD_STATE_BRANCH_POSITION:
		if (entering) {
			boom_set(&task->bp, tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
		}

		if (chassis_done(&task->cp) && picker_done(&task->pp) && boom_done(&task->bp)) {
			log_printf(&util_logger, "move to BRANCH_APPROACH");
			coord_set_state(task, COORD_STATE_BRANCH_APPROACH);
		}
		break;
	case COORD_STATE_BRANCH_APPROACH:
		if (entering) {
			chassis_set_control(&task->cp, SPEED_CONTROL_FIXED, 0, tuning->fast_speed,
					HEADING_CONTROL_FORWARD, 0,
					END_COND_BEAM);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to APPLE_REACH");
			coord_set_state(task, COORD_STATE_APPLE_REACH);
		}
		break;
	case COORD_STATE_APPLE_REACH:
		if (entering) {
			chassis_set_control(&task->cp, SPEED_CONTROL_FRONT_DISTANCE_RELATIVE,
					-tuning->reach_distances[task->apple_idx], tuning->slow_speed,
					HEADING_CONTROL_FORWARD, 0,
					END_COND_FRONT_DISTANCE_LTE);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to APPLE_CLOSE");
			coord_set_state(task, COORD_STATE_APPLE_CLOSE);
		}
		break;
	case COORD_STATE_APPLE_CLOSE:
		if (entering) {
			picker_set(&task->pp, PICKER_STATE_CLOSED);
		}

		if (picker_done(&task->pp)) {
			log_printf(&util_logger, "move to BACK_UP");
			coord_set_state(task, COORD_STATE_BACK_UP);
		}
		break;
	case COORD_STATE_BACK_UP:
		if (entering) {
			chassis_set_control(&task->cp, SPEED_CONTROL_FRONT_DISTANCE_RELATIVE,
					tuning->back_up_distances[task->apple_idx], tuning->fast_speed,
					HEADING_CONTROL_FORWARD, 0,
					END_COND_FRONT_DISTANCE_GTE);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to MOVE_TO_EJECT");
			coord_set_state(task, COORD_STATE_MOVE_TO_EJECT);
		}
		break;
	case COORD_STATE_MOVE_TO_EJECT:
		if (entering) {
			boom_set(&task->bp, tuning->drop_boom_targets[task->apple_idx]);
		}

		if (task->apple_idx == 0) {
			if (boom_done(&task->bp)) {
				log_printf(&util_logger, "move to EJECT_STATIONARY");
				coord_set_state(task, COORD_STATE_EJECT_STATIONARY);
			}
		} else {
			log_printf(&util_logger, "move to TREE_TURN_AWAY");
			coord_set_state(task, COORD_STATE_TREE_TURN_AWAY);
		}
		break;
	case COORD_STATE_EJECT_STATIONARY:
		if (entering) {
			picker_set(&task->pp, PICKER_STATE_EJECT);
		}

		if (picker_done(&task->pp)) {
			task->apple_idx++;
			log_printf(&util_logger, "move to BRANCH_POSITION");
			coord_set_state(task, COORD_STATE_BRANCH_POSITION);
		}
		break;
	case COORD_STATE_TREE_TURN_AWAY:
		if (entering) {
			float heading = normalise_angle(task->west + ((task->branch_idx + 1) * 90));
			chassis_set_control(&task->cp, SPEED_CONTROL_FIXED, 0, 0,
					HEADING_CONTROL_EXPLICIT, heading,
					END_COND_HEADING_EQ);
		}

		if (boom_done(&task->bp) && (picker_state(&task->pp) == PICKER_STATE_CLOSED)) {
			log_printf(&util_logger, "ejecting");
			picker_set(&task->pp, PICKER_STATE_EJECT);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to CORNER_APPROACH");
			coord_set_state(task, COORD_STATE_CORNER_APPROACH);
			task->apple_idx = 0;
		}
		break;
	case COORD_STATE_CORNER_APPROACH:
		if (entering) {
			float heading = normalise_angle(task->west + ((task->branch_idx + 1) * 90));
			chassis_set_control(&task->cp, tuning->corner_approach_speed_ctrl,
					tuning->corner_approach_distances[task->branch_idx], tuning->very_fast_speed,
					HEADING_CONTROL_EXPLICIT, heading,
					tuning->corner_approach_end_cond);
		}

		if (boom_done(&task->bp) && (picker_state(&task->pp) == PICKER_STATE_CLOSED)) {
			log_printf(&util_logger, "ejecting");
			picker_set(&task->pp, PICKER_STATE_EJECT);
		}

		if (chassis_done(&task->cp) && boom_done(&task->bp) && (picker_state(&task->pp) == PICKER_STATE_EJECT)) {
			task->apple_idx = 0;
			task->branch_idx++;

			if (task->branch_idx < NUM_BRANCHES) {
				log_printf(&util_logger, "move to CORNER_TURN");
				coord_set_state(task, COORD_STATE_CORNER_TURN);
			} else {
				log_printf(&util_logger, "move to STOP");
				coord_set_state(task, COORD_STATE_STOP);
			}
		}
		break;
	case COORD_STATE_STOP:
		if (entering) {
			picker_set(&task->pp, PICKER_STATE_OPEN);
			chassis_stop(&task->cp);
		}
		break;
	/*
	 * Testing apple dropping
	 */
	case COORD_STATE_TEST_BRANCH_APPROACH:
		if (entering) {
			//boom_set(&task->bp, task->tuning->approach_boom_targets[task->apple_idx]);
			boom_set(&task->bp, task->tuning->drop_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_CLOSED);
		}

		if (boom_done(&task->bp) && picker_done(&task->pp)) {
			log_printf(&util_logger, "move to TEST_WAIT");
			coord_set_state(task, COORD_STATE_TEST_WAIT);
		}
		break;
	case COORD_STATE_TEST_WAIT:
		break;
	case COORD_STATE_TEST_MOVE_TO_EJECT:
		if (entering) {
			boom_set(&task->bp, tuning->drop_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_CLOSED);
		}

		if (boom_done(&task->bp)) {
			log_printf(&util_logger, "move to TEST_EJECT_STATIONARY");
			coord_set_state(task, COORD_STATE_TEST_EJECT_STATIONARY);
		}
		break;
	case COORD_STATE_TEST_EJECT_STATIONARY:
		if (entering) {
			picker_set(&task->pp, PICKER_STATE_EJECT);
		}

		if (picker_done(&task->pp)) {
			coord_set_state(task, COORD_STATE_TEST_BRANCH_APPROACH);
		}
		break;
	}

	boom_tick(&task->bp, platform, status);
	picker_tick(&task->pp, platform, status);
	chassis_tick(&task->cp, platform, status);

	log_printf(&util_logger, "<<< coord tick done: %d", task->coord_state);
}

static void apple_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (input->buttons.pressed & BTN_SQUARE) {
		task->apple_idx = 0;
		task->branch_idx = 0;
		task->tuning = &apples_tunings[TUNING_HOME];
		coord_set_state(task, COORD_STATE_CORNER_TURN);
		task->run_coord = true;
		log_printf(&util_logger, "coord input: %d, %d", task->coord_state, task->state_entry);
	}

	if (input->buttons.pressed & BTN_TRIANGLE) {
		task->apple_idx = 0;
		task->branch_idx = 0;
		task->tuning = &apples_tunings[TUNING_MAKESPACE];
		coord_set_state(task, COORD_STATE_CORNER_TURN);
		task->run_coord = true;
		log_printf(&util_logger, "coord input: %d, %d", task->coord_state, task->state_entry);
	}

	if (input->buttons.pressed & BTN_HEART) {
		task->apple_idx = 0;
		task->branch_idx = 0;
		task->tuning = &apples_tunings[TUNING_MAKESPACE];
		coord_set_state(task, COORD_STATE_TEST_BRANCH_APPROACH);
		task->run_coord = true;
		log_printf(&util_logger, "coord input: %d, %d", task->coord_state, task->state_entry);
	}

	if ((input->buttons.pressed & BTN_CROSS) &&
			(task->coord_state == COORD_STATE_TEST_WAIT)) {
		task->tuning = &apples_tunings[TUNING_MAKESPACE];
		coord_set_state(task, COORD_STATE_TEST_MOVE_TO_EJECT);
	}
}

static void apple_task_on_start(struct planner_task *ptask, struct platform *platform)
{
	struct apple_task *task = (struct apple_task *)ptask;
	chassis_stop(&task->cp);
	boom_invalidate(&task->bp);
	coord_set_state(task, COORD_STATE_STOP);
	task->cp.controller_active = false;
	task->run_coord = false;
	platform_servo_level(platform, true);
	platform_ioe_pwm_set_enabled(platform, 2, true);
}

static void apple_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (task->run_coord) {
		coord_tick(task, platform, status);
	}
}

struct planner_task *apples_get_task(struct camera_buffer *buf)
{
	static struct apple_task task = {
		.base = {
			.on_start = apple_task_on_start,
			.handle_input = apple_task_handle_input,
			.tick = apple_task_tick,
		},
		.apple_idx = 0,
	};

	task.cp.front_sensors.buf = buf;

	return &task.base;
}
