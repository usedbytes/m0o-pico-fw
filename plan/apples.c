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
#include "util.h"

#include "platform/platform.h"

#define APPLE_SERVO_APPROACH 7500
#define APPLE_SERVO_PICK     3750
#define APPLE_SERVO_EJECT    2000

const struct v2 approach_boom_targets[] = {
	{ 5, 126 },
	{ 25, 257 },
};

const unsigned int num_apples = sizeof(approach_boom_targets) / sizeof(approach_boom_targets[0]);

const struct v2 drop_boom_targets[] = {
	{ -25, 255 },
	{ -25, 255 },
};

const int reach_distances[] = {
	15,
	0,
};

const int back_up_distances[] = {
	70,
	70,
};

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
			platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_APPROACH);
			break;
		case PICKER_STATE_CLOSED:
			pp->current = pp->requested | PICKER_STATE__ING;
			platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_PICK);
			break;
		case PICKER_STATE_EJECT:
			pp->current = pp->requested | PICKER_STATE__ING;
			platform_ioe_pwm_set_enabled(platform, 2, true);
			platform_ioe_set(platform, 2, APPLE_SERVO_EJECT);
			break;
		case PICKER_STATE_IDLE:
		default:
			pp->current = pp->requested;
			platform_ioe_pwm_set_enabled(platform, 2, false);
			break;
		}
		pp->end_timestamp = make_timeout_time_ms(wait_time_ms);
	} else if (pp->current & PICKER_STATE__ING) {
		// Waiting to reach
		//log_printf(&util_logger, "picker waiting, current: %d, requested: %d", pp->current, pp->requested);
		absolute_time_t now = get_absolute_time();
		if (to_us_since_boot(now) >= pp->end_timestamp) {
			pp->current = pp->requested;
			platform_ioe_pwm_set_enabled(platform, 2, false);
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

struct boom_planner {
	struct v2 requested;
	struct v2 set;
	bool done;
};

static void boom_tick(struct boom_planner *bp, struct platform *platform, struct platform_status_report *status)
{
	if ((bp->requested.x != bp->set.x) || (bp->requested.y != bp->set.y)) {
		bp->set = bp->requested;
		log_printf(&util_logger, "update boom target: (%3.2f, %3.2f)", bp->set.x, bp->set.y);
		platform_boom_trajectory_controller_adjust_target(platform,
			bp->set,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
	} else {
		if (status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED) {
			bp->done = true;
		}
	}
}

static bool boom_done(struct boom_planner *bp)
{
	return bp->done;
}

static void boom_set(struct boom_planner *bp, struct v2 abs_target)
{
	bp->requested = abs_target;
	log_printf(&util_logger, "boom_set: (%3.2f, %3.2f)", bp->requested.x, bp->requested.y);
	bp->done = false;
	/*
	if ((abs_target.x != bp->set.x) || (abs_target.y != bp->set.y)) {
		bp->done = false;
	}
	*/
}

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
	enum {
		SPEED_CONTROL_FIXED,
		SPEED_CONTROL_DISTANCE,
		SPEED_CONTROL_DISTANCE_RELATIVE,
	} speed_control;
	enum {
		HEADING_CONTROL_FORWARD,
		HEADING_CONTROL_EXPLICIT,
		HEADING_CONTROL_RED_BLOB,
	} heading_control;
	bool controller_active;

	int8_t linear_speed;
	int8_t current_linear_speed;

	float current_heading;

#define SENSOR_FRONT_RANGE      (1 << 0)
#define SENSOR_CAMERA           (1 << 1)
	uint32_t sensors;

	enum sensor_state sensor_state;

	bool camera_frame_done;
	struct camera_buffer *buf;
	int left, right, bottom;

	absolute_time_t front_range_ts;
	uint16_t front_range;
	uint16_t target_distance;
	bool target_distance_valid;
	int16_t relative_distance;

	float target_heading;

#define END_COND_DISTANCE_EQ     (1 << 0)
#define END_COND_DISTANCE_LTE    (1 << 1)
#define END_COND_DISTANCE_GTE    (1 << 2)
#define END_CONDS_DISTANCE       (END_COND_DISTANCE_EQ | END_COND_DISTANCE_LTE | END_COND_DISTANCE_GTE)
#define END_COND_BEAM            (1 << 3)
#define END_COND_HEADING_EQ      (1 << 4)
	uint32_t end_conditions;

	bool active;
};

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

struct apple_task {
	struct planner_task base;

	enum pick_state pick_state;
	bool state_waiting;
	int apple_idx;
	int branch_idx;
	absolute_time_t timestamp;
	bool sequence;
	uint16_t target_distance;

	bool run_coord;
	bool state_entry;
	enum coordinator_state coord_state;
	struct chassis_planner cp;
	struct boom_planner bp;
	struct picker_planner pp;

	float east;
};

static void chassis_camera_cb(struct camera_buffer *buf, void *p)
{
	struct chassis_planner *cp = (struct chassis_planner *)p;

	cp->camera_frame_done = true;
	log_printf(&util_logger, "camera done");
}

static const int threshold = 10;
// Search for the furthest left and furthest right pixels in 'row'
// which are brighter than 'maxval - threshold', starting at 'start_idx'
static void search_row(uint8_t *row, int len, int start_idx, uint8_t maxval,
		       int *left_out, int *right_out)
{
	int i;

	int left = start_idx;
	int right = start_idx;

	// Search to the left
	for (i = start_idx; i >= 0; i--) {
		if (maxval - row[i] <= threshold) {
			left = i;
		} else {
			break;
		}
	}

	// Search to the right
	for (i = start_idx + 1; i < len; i++) {
		if (maxval - row[i] <= threshold) {
			right = i;
		} else {
			break;
		}
	}

	*left_out = left;
	*right_out = right;
}

// Returns the middle 'X' coordinate of the brightest red blob
static int find_red_blob(struct camera_buffer *buf, int *left, int *right, int *bottom)
{
	uint32_t start = time_us_32();
	int x, y;
	uint8_t reddest = 0;
	int max_x = 0, max_y = 0;

	// First find the reddest pixel in the image.
	// It's a YCbCr image, we only care about "redness", which is Cr
	// which is stored in buf->data[2]
	for (y = 0; y < buf->height; y++) {
		for (x = 0; x < buf->width / 2; x++) {
			uint8_t pix = buf->data[2][buf->strides[2] * y + x];
			if (pix > reddest) {
				reddest = pix;
				max_x = x;
				max_y = y;
			}
		}
	}

	log_printf(&util_logger, "reddest: %d", reddest);

	// Next, we search up and down the rows, looking for ones which are also
	// part of the blob.
	// On each row, we find the leftmost and rightmost pixel which is within
	// some threshold of the reddest value.
	// Then we take the middle of that left..right range to use as the
	// starting point for searching the next row.

	// l and r track the absolute leftmost and rightmost extremes of the
	// blob, eventually giving its widest point.
	int l = max_x;
	int r = max_x;
	int tmp_l, tmp_r;

	// Search up
	int idx = max_x;
	for (y = max_y; y >= 0; y--) {
		uint8_t *row = &buf->data[2][buf->strides[2] * y];

		// Only search this row if the starting point is actually
		// red enough
		if (reddest - row[idx] < threshold) {
			// Note: buf->width / 2, because this is a YUV422 image,
			// so the Cr plane is half as wide as the image itself
			search_row(row, buf->width / 2, idx, reddest, &tmp_l, &tmp_r);

			// Update the global leftmost/rightmost values
			l = tmp_l < l ? tmp_l : l;
			r = tmp_r > r ? tmp_r : r;

			// Calculate the starting point for the next row
			idx = tmp_l + ((tmp_r - tmp_l) / 2);
		} else {
			break;
		}
	}

	// Search down, starting with the middle X coord we've found so far
	idx = l + ((r - l) / 2);
	for (y = max_y; y < buf->height; y++) {
		uint8_t *row = &buf->data[2][buf->strides[2] * y];
		if (reddest - row[idx] < threshold) {
			search_row(row, buf->width / 2, idx, reddest, &tmp_l, &tmp_r);

			l = tmp_l < l ? tmp_l : l;
			r = tmp_r > r ? tmp_r : r;
		} else {
			break;
		}
	}


	// Finally, calculate the overall middle X coord
	int mid = l + (r - l) / 2;
	*left = l;
	*right = r;
	*bottom = y;

	log_printf(&util_logger, "Process done. %d us, mid %d", time_us_32() - start, mid);

	return mid;
}

static uint32_t chassis_handle_sensors(struct chassis_planner *cp, struct platform *platform,
		struct platform_status_report *status)
{
	uint32_t readings = 0;

	enum sensor_state prev_state;
	do {
		prev_state = cp->sensor_state;

		log_printf(&util_logger, "sensor loop: %d", prev_state);

		switch (prev_state) {
		case SENSOR_STATE_IDLE:
			if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->sensor_state = SENSOR_STATE_LASER_REQUEST;
			} else if (cp->sensors & SENSOR_CAMERA) {
				cp->sensor_state = SENSOR_STATE_CAMERA_REQUEST;
			}
			break;
		case SENSOR_STATE_LASER_REQUEST:
			cp->front_range_ts = get_absolute_time();
			log_printf(&util_logger, "request ts: %"PRId64, cp->front_range_ts);
			platform_vl53l0x_trigger_single(platform, 0);
			cp->sensor_state = SENSOR_STATE_LASER_PENDING;
			break;
		case SENSOR_STATE_LASER_PENDING:
			log_printf(&util_logger, "pending ts: %"PRId64", 0x%x", status->front_laser.timestamp, status->front_laser.range_status);
			if (to_us_since_boot(status->front_laser.timestamp) > to_us_since_boot(cp->front_range_ts)) {
				cp->sensor_state = SENSOR_STATE_LASER_DONE;
			}
			break;
		case SENSOR_STATE_LASER_DONE:
			cp->front_range_ts = status->front_laser.timestamp;
			cp->front_range = status->front_laser.range_mm;
			readings |= SENSOR_FRONT_RANGE;
			if (cp->sensors & SENSOR_CAMERA) {
				cp->sensor_state = SENSOR_STATE_CAMERA_REQUEST;
			} else if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->sensor_state = SENSOR_STATE_LASER_REQUEST;
			}
			break;
		case SENSOR_STATE_CAMERA_REQUEST:
			cp->camera_frame_done = false;
			platform_camera_capture(platform, cp->buf, chassis_camera_cb, cp);
			cp->sensor_state = SENSOR_STATE_CAMERA_PENDING;
			break;
		case SENSOR_STATE_CAMERA_PENDING:
			if (cp->camera_frame_done) {
				cp->sensor_state = SENSOR_STATE_CAMERA_DONE;
			}
			break;
		case SENSOR_STATE_CAMERA_DONE:
			find_red_blob(cp->buf, &cp->left, &cp->right, &cp->bottom);
			readings |= SENSOR_CAMERA;
			if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->sensor_state = SENSOR_STATE_LASER_REQUEST;
			} else if (cp->sensors & SENSOR_CAMERA) {
				cp->sensor_state = SENSOR_STATE_CAMERA_REQUEST;
			} else {
				cp->sensor_state = SENSOR_STATE_IDLE;
			}
			break;
		}
	} while (prev_state != cp->sensor_state);

	return readings;
}

static void chassis_tick(struct chassis_planner *cp, struct platform *platform, struct platform_status_report *status)
{
	const float cone_angle = 2.0;
	const int distance_tolerance = 2;
	const int min_distance = 30;

	// Handle sensor requests

	uint32_t readings = chassis_handle_sensors(cp, platform, status);

	bool have_front_range = false;
	uint16_t front_range = 4096;
	if (readings & SENSOR_FRONT_RANGE) {
		have_front_range = true;
		front_range = cp->front_range;
	}

	bool have_frame = false;
	if (readings & SENSOR_CAMERA) {
		log_printf(&util_logger, "frame done: %d, %d, %d", cp->left, cp->right, cp->bottom);
		if ((cp->right - cp->left) >= 2) {
			have_frame = true;
		}
	}

	// Handle state changes

	log_printf(&util_logger, "have_frame: %d, have_front_range: %d, speed_control: %d, heading_control: %d",
			have_frame, have_front_range, cp->speed_control, cp->heading_control);

	if (have_front_range && (cp->speed_control == SPEED_CONTROL_DISTANCE_RELATIVE)) {
		log_printf(&util_logger, "setting relative distance");
		cp->target_distance = front_range + cp->relative_distance;
		if (cp->target_distance < min_distance) {
			cp->target_distance = min_distance;
		}
		cp->target_distance_valid = true;
		cp->speed_control = SPEED_CONTROL_DISTANCE;
	}

	// Adjust speed/heading

	int8_t linear_speed = 0;
	switch (cp->speed_control) {
	case SPEED_CONTROL_DISTANCE:
		if (have_front_range) {
			int diff = front_range - cp->target_distance;
			log_printf(&util_logger, "distance control diff: %d", diff);
			if (diff > 0) {
				// TODO: Signs.
				linear_speed = cp->linear_speed;
			} else if (diff < 0) {
				// TODO: Signs.
				linear_speed = -cp->linear_speed;
			}
		} else {
			linear_speed = cp->current_linear_speed;
		}
		break;
	case SPEED_CONTROL_FIXED:
		linear_speed = cp->linear_speed;
		break;
	default:
		linear_speed = 0;
		break;
	}

	float heading;
	switch (cp->heading_control) {
	case HEADING_CONTROL_EXPLICIT:
		heading = cp->target_heading;
		break;
	case HEADING_CONTROL_RED_BLOB:
		if (have_frame) {
			const float deg_per_pixel = 0.835;
			float diff = (cp->left - 20) * deg_per_pixel;
			log_printf(&util_logger, "blob diff: %3.2f", diff);
			heading = normalise_angle(status->heading / 16.0) + diff;
		} else {
			heading = cp->current_heading;
		}
		break;
	default:
		heading = normalise_angle(status->heading / 16.0);
		break;
	}

	// Handle termination

	uint32_t current_conditions = 0;
	if (have_front_range && cp->target_distance_valid) {
		int diff = front_range - cp->target_distance;
		if (diff < 0) {
			current_conditions |= END_COND_DISTANCE_LTE;
			if (diff > -distance_tolerance) {
				current_conditions |= END_COND_DISTANCE_EQ;
			}
		} else if (diff > 0) {
			current_conditions |= END_COND_DISTANCE_GTE;
			if (diff < distance_tolerance) {
				current_conditions |= END_COND_DISTANCE_EQ;
			}
		} else {
			current_conditions |= END_COND_DISTANCE_EQ;
		}
	}

	{
		float diff = normalise_angle(status->heading / 16.0) - heading;
		float absdiff = fabsf(diff);
		if (absdiff <= cone_angle) {
			current_conditions |= END_COND_HEADING_EQ;
		}
	}

	if (status->adc >= 3000) {
		current_conditions |= END_COND_BEAM;
	}

	log_printf(&util_logger, "current_conditions: 0x%x, end_conditions: 0x%x", current_conditions, cp->end_conditions);

	if ((current_conditions & cp->end_conditions) == cp->end_conditions) {
		// Stop!
		log_printf(&util_logger, "end conditions reached");
		linear_speed = 0;
		cp->active = false;
	}

	// Finally, apply speed/heading

	if (cp->controller_active != cp->active) {
		platform_heading_controller_set_enabled(platform, cp->active);
		cp->controller_active = cp->active;
	}

	if (cp->active) {
		log_printf(&util_logger, "chassis set %d, %3.2f", linear_speed, heading);
		platform_heading_controller_set(platform, linear_speed, heading);
	}

	cp->current_heading = heading;
	cp->current_linear_speed = linear_speed;
}

static void chassis_set_distance_lte(struct chassis_planner *cp, uint16_t distance, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_LTE;
	cp->target_distance = distance;
	cp->target_distance_valid = true;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_set_distance_gte(struct chassis_planner *cp, uint16_t distance, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_GTE;
	cp->target_distance = distance;
	cp->target_distance_valid = true;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = SENSOR_FRONT_RANGE;// | SENSOR_CAMERA;
	cp->active = true;
}

static void chassis_set_distance_lte_relative(struct chassis_planner *cp, uint16_t relative_distance, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_LTE;
	cp->relative_distance = relative_distance;
	cp->target_distance_valid = false;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE_RELATIVE;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_set_distance_gte_relative(struct chassis_planner *cp, uint16_t relative_distance, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_GTE;
	cp->relative_distance = relative_distance;
	cp->target_distance_valid = false;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE_RELATIVE;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_set_beam(struct chassis_planner *cp, int8_t speed)
{
	cp->end_conditions = END_COND_BEAM;
	cp->linear_speed = speed;
	cp->target_distance_valid = false;
	cp->speed_control = SPEED_CONTROL_FIXED;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_set_blob_to_distance(struct chassis_planner *cp, uint16_t distance, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_LTE;
	cp->target_distance = distance;
	cp->target_distance_valid = true;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_FIXED;
	cp->heading_control = HEADING_CONTROL_RED_BLOB;
	cp->sensors = SENSOR_FRONT_RANGE | SENSOR_CAMERA;
	cp->active = true;
}

static void chassis_stop(struct chassis_planner *cp)
{
	cp->end_conditions = 0;
	cp->linear_speed = 0;
	cp->target_distance_valid = false;
	cp->speed_control = SPEED_CONTROL_FIXED;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = 0;
	cp->active = true;
}

static void chassis_set_heading_to_distance(struct chassis_planner *cp, uint16_t distance, float heading, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_EQ;
	cp->target_distance = distance;
	cp->target_distance_valid = true;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE;
	cp->heading_control = HEADING_CONTROL_EXPLICIT;
	cp->target_heading = heading;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_set_heading_to_distance_lte(struct chassis_planner *cp, uint16_t distance, float heading, int8_t speed)
{
	cp->end_conditions = END_COND_DISTANCE_LTE;
	cp->target_distance = distance;
	cp->target_distance_valid = true;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_DISTANCE;
	cp->heading_control = HEADING_CONTROL_EXPLICIT;
	cp->target_heading = heading;
	cp->sensors = SENSOR_FRONT_RANGE;
	cp->active = true;
}

static void chassis_turn_to(struct chassis_planner *cp, float heading, int8_t speed)
{
	cp->end_conditions = END_COND_HEADING_EQ;
	cp->target_distance_valid = false;
	cp->linear_speed = speed;
	cp->speed_control = SPEED_CONTROL_FIXED;
	cp->heading_control = HEADING_CONTROL_EXPLICIT;
	cp->target_heading = heading;
	cp->sensors = 0;
	cp->active = true;
}

static bool chassis_done(struct chassis_planner *cp)
{
	return !cp->active;
}

static void coord_set_state(struct apple_task *task, enum coordinator_state state)
{
	task->coord_state = state;
	task->state_entry = true;
}

static void coord_tick(struct apple_task *task, struct platform *platform, struct platform_status_report *status)
{
	const int slow_speed = 2;
	const int fast_speed = 6;
	bool entering = task->state_entry;
	task->state_entry = false;

	log_printf(&util_logger, "coord tick: %d, %d", task->coord_state, entering);
	log_printf(&util_logger, "c: %d, p: %d, b: %d",
			chassis_done(&task->cp), picker_done(&task->pp), boom_done(&task->bp));


	switch (task->coord_state) {
	case COORD_STATE_PERIMETER_APPROACH:
		if (entering) {
			if (task->branch_idx == 0) {
				task->east = normalise_angle(status->heading / 16.0);
			}
			boom_set(&task->bp, approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);

			float heading = normalise_angle(task->east - (task->branch_idx * 90));
			chassis_set_heading_to_distance_lte(&task->cp, 650, heading, fast_speed * 2);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to PERIMETER_APPROACH_SLOW");
			coord_set_state(task, COORD_STATE_PERIMETER_APPROACH_SLOW);
		}
		break;
	case COORD_STATE_PERIMETER_APPROACH_SLOW:
		if (entering) {
			if (task->branch_idx == 0) {
				task->east = normalise_angle(status->heading / 16.0);
			}
			boom_set(&task->bp, approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);

			float heading = normalise_angle(task->east - (task->branch_idx * 90));
			chassis_set_heading_to_distance(&task->cp, 550, heading, slow_speed);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to TREE_POINT");
			coord_set_state(task, COORD_STATE_TREE_POINT);
		}
		break;
	case COORD_STATE_TREE_POINT:
		if (entering) {
			//set_chassis_to_heading(tree_heading[hidx]);
			boom_set(&task->bp, approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
			float heading = normalise_angle(task->east - ((task->branch_idx + 1) * 90));
			chassis_turn_to(&task->cp, heading, 0);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to TREE_APPROACH");
			coord_set_state(task, COORD_STATE_TREE_APPROACH);
		}
		break;
	case COORD_STATE_TREE_APPROACH:
		if (entering) {
			//chassis_set_distance_gte(&task->cp, 110, fast_speed);
			chassis_set_blob_to_distance(&task->cp, 210, fast_speed);
			boom_set(&task->bp, approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
		}

		if (chassis_done(&task->cp) && picker_done(&task->pp) && boom_done(&task->bp)) {
			log_printf(&util_logger, "move to BRANCH_APPROACH");
			coord_set_state(task, COORD_STATE_BRANCH_APPROACH);
		}
		break;
	case COORD_STATE_BRANCH_POSITION:
		if (entering) {
			boom_set(&task->bp, approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
		}

		if (chassis_done(&task->cp) && picker_done(&task->pp) && boom_done(&task->bp)) {
			log_printf(&util_logger, "move to BRANCH_APPROACH");
			coord_set_state(task, COORD_STATE_BRANCH_APPROACH);
		}
		break;
	case COORD_STATE_BRANCH_APPROACH:
		if (entering) {
			chassis_set_beam(&task->cp, fast_speed);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to APPLE_REACH");
			coord_set_state(task, COORD_STATE_APPLE_REACH);
		}
		break;
	case COORD_STATE_APPLE_REACH:
		if (entering) {
			chassis_set_distance_lte_relative(&task->cp, -reach_distances[task->apple_idx], slow_speed);
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
			chassis_set_distance_gte_relative(&task->cp, back_up_distances[task->apple_idx], fast_speed);
		}

		if (chassis_done(&task->cp)) {
			log_printf(&util_logger, "move to MOVE_TO_EJECT");
			coord_set_state(task, COORD_STATE_MOVE_TO_EJECT);
		}
		break;
	case COORD_STATE_MOVE_TO_EJECT:
		if (entering) {
			boom_set(&task->bp, drop_boom_targets[task->apple_idx]);
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
			float heading = normalise_angle(task->east - (task->branch_idx * 90));
			chassis_turn_to(&task->cp, heading, 0);
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
			float heading = normalise_angle(task->east - (task->branch_idx * 90));
			chassis_set_heading_to_distance_lte(&task->cp, 120, heading, fast_speed * 2);
		}

		if (boom_done(&task->bp) && (picker_state(&task->pp) == PICKER_STATE_CLOSED)) {
			log_printf(&util_logger, "ejecting");
			picker_set(&task->pp, PICKER_STATE_EJECT);
		}

		if (chassis_done(&task->cp) && boom_done(&task->bp) && (picker_state(&task->pp) == PICKER_STATE_EJECT)) {
			task->apple_idx = 0;
			task->branch_idx++;

			if (task->branch_idx <= 1) {
				log_printf(&util_logger, "move to PERIMETER_APPROACH");
				coord_set_state(task, COORD_STATE_PERIMETER_APPROACH);
			} else {
				log_printf(&util_logger, "move to STOP");
				coord_set_state(task, COORD_STATE_STOP);
			}
		}
		break;
	case COORD_STATE_STOP:
		if (entering) {
			chassis_stop(&task->cp);
		}
		break;
	}

	boom_tick(&task->bp, platform, status);
	picker_tick(&task->pp, platform, status);
	chassis_tick(&task->cp, platform, status);

	log_printf(&util_logger, "coord tick done: %d", task->coord_state);
}

static void apple_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (input->buttons.pressed & BTN_CROSS) {
		task->apple_idx = 0;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state = PICK_STATE_BOOM_APPROACH;
		task->run_coord = false;
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		task->apple_idx = 1;
		task->sequence = ((input->buttons.held & BTN_L1) != 0);
		task->pick_state = PICK_STATE_BOOM_APPROACH;
		task->run_coord = false;
	}

	if (input->buttons.pressed & BTN_TRIANGLE) {
		task->apple_idx = 0;
		task->branch_idx = 0;
		coord_set_state(task, COORD_STATE_PERIMETER_APPROACH);
		task->pick_state = PICK_STATE_IDLE;
		task->run_coord = true;
		log_printf(&util_logger, "coord input: %d, %d", task->coord_state, task->state_entry);
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

	if (task->run_coord) {
		coord_tick(task, platform, status);
		return;
	}

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
	chassis_stop(&task->cp);
	coord_set_state(task, COORD_STATE_STOP);
	task->cp.controller_active = false;
	task->run_coord = false;
	platform_servo_level(platform, true);
}

struct planner_task *apples_get_task(struct camera_buffer *buf)
{
	static struct apple_task task = {
		.base = {
			.on_start = apple_task_on_start,
			.handle_input = apple_task_handle_input,
			.tick = apple_task_tick,
		},
		.pick_state = PICK_STATE_IDLE,
		.apple_idx = 0,
	};

	task.cp.buf = buf;

	return &task.base;
}
