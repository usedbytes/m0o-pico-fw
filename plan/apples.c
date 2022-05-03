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
};

/*
const struct v2 approach_boom_targets[] = {
	{ 5, 126 },
	{ 25, 255 },
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
	200,
};

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

const int rear_distances_home[] = {
	650,
	650,
	650,
	650,
};

const int corner_distances_home[] = {
	940,
	940,
	940,
	940,
};

const int corner_front_distances_home[] = {
	80,
	80,
	80,
	80,
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
	bool invalidate;
};

static void boom_tick(struct boom_planner *bp, struct platform *platform, struct platform_status_report *status)
{
	if (bp->invalidate || (bp->requested.x != bp->set.x) || (bp->requested.y != bp->set.y)) {
		bp->set = bp->requested;
		bp->invalidate = false;
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

static void boom_invalidate(struct boom_planner *bp)
{
	bp->invalidate = true;
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

struct apple_task {
	struct planner_task base;

	const struct apples_tuning *tuning;
	int apple_idx;
	int branch_idx;
	float west;

	bool state_entry;
	bool run_coord;
	enum coordinator_state coord_state;
	struct chassis_planner cp;
	struct boom_planner bp;
	struct picker_planner pp;
};

static void chassis_camera_cb(struct camera_buffer *buf, void *p)
{
	struct chassis_planner *cp = (struct chassis_planner *)p;

	cp->front_sensors.frame_done = true;
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
		prev_state = cp->front_sensors.state;

		//log_printf(&util_logger, "sensor loop: %d", prev_state);

		switch (prev_state) {
		case SENSOR_STATE_IDLE:
			if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->front_sensors.state = SENSOR_STATE_LASER_REQUEST;
			} else if (cp->sensors & SENSOR_CAMERA) {
				cp->front_sensors.state = SENSOR_STATE_CAMERA_REQUEST;
			}
			break;
		case SENSOR_STATE_LASER_REQUEST:
			cp->front_sensors.trig_ts = get_absolute_time();
			//log_printf(&util_logger, "request ts: %"PRId64, cp->front_range_ts);
			platform_vl53l0x_trigger_single(platform, 0);
			cp->front_sensors.state = SENSOR_STATE_LASER_PENDING;
			break;
		case SENSOR_STATE_LASER_PENDING:
			//log_printf(&util_logger, "pending ts: %"PRId64", 0x%x", status->front_laser.timestamp, status->front_laser.range_status);
			if (to_us_since_boot(status->front_laser.timestamp) > to_us_since_boot(cp->front_sensors.trig_ts)) {
				cp->front_sensors.state = SENSOR_STATE_LASER_DONE;
			}
			break;
		case SENSOR_STATE_LASER_DONE:
			if (cp->sensors & SENSOR_CAMERA) {
				cp->front_sensors.state = SENSOR_STATE_CAMERA_REQUEST;
			} else if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->front_sensors.state = SENSOR_STATE_LASER_REQUEST;
			}

			// Ignore stale readings
			if (to_us_since_boot(cp->front_sensors.trig_ts) > to_us_since_boot(cp->last_change_ts)) {
				//log_printf(&util_logger, "ignoring stale reading %"PRId64" > %"PRId64,
				//	to_us_since_boot(status->front_laser.timestamp),
				//	to_us_since_boot(cp->last_change_ts));
				cp->front_sensors.range = status->front_laser.range_mm;
				readings |= SENSOR_FRONT_RANGE;
			}
			break;
		case SENSOR_STATE_CAMERA_REQUEST:
			cp->front_sensors.frame_done = false;
			platform_camera_capture(platform, cp->front_sensors.buf, chassis_camera_cb, cp);
			cp->front_sensors.state = SENSOR_STATE_CAMERA_PENDING;
			break;
		case SENSOR_STATE_CAMERA_PENDING:
			if (cp->front_sensors.frame_done) {
				cp->front_sensors.state = SENSOR_STATE_CAMERA_DONE;
			}
			break;
		case SENSOR_STATE_CAMERA_DONE:
			find_red_blob(cp->front_sensors.buf, &cp->front_sensors.left,
					&cp->front_sensors.right, &cp->front_sensors.bottom);
			readings |= SENSOR_CAMERA;
			if (cp->sensors & SENSOR_FRONT_RANGE) {
				cp->front_sensors.state = SENSOR_STATE_LASER_REQUEST;
			} else if (cp->sensors & SENSOR_CAMERA) {
				cp->front_sensors.state = SENSOR_STATE_CAMERA_REQUEST;
			} else {
				cp->front_sensors.state = SENSOR_STATE_IDLE;
			}
			break;
		}
	} while (prev_state != cp->front_sensors.state);

	do {
		prev_state = cp->rear_sensors.state;

		switch (prev_state) {
		case SENSOR_STATE_IDLE:
			if (cp->sensors & SENSOR_REAR_RANGE) {
				cp->rear_sensors.state = SENSOR_STATE_LASER_REQUEST;
			}
			break;
		case SENSOR_STATE_LASER_REQUEST:
			cp->rear_sensors.trig_ts = get_absolute_time();
			platform_vl53l0x_trigger_single(platform, 1);
			cp->rear_sensors.state = SENSOR_STATE_LASER_PENDING;
			break;
		case SENSOR_STATE_LASER_PENDING:
			if (to_us_since_boot(status->rear_laser.timestamp) > to_us_since_boot(cp->rear_sensors.trig_ts)) {
				cp->rear_sensors.state = SENSOR_STATE_LASER_DONE;
			}
			break;
		case SENSOR_STATE_LASER_DONE:
			if (cp->sensors & SENSOR_REAR_RANGE) {
				cp->rear_sensors.state = SENSOR_STATE_LASER_REQUEST;
			}

			// Ignore stale readings
			if (to_us_since_boot(cp->rear_sensors.trig_ts) > to_us_since_boot(cp->last_change_ts)) {
				cp->rear_sensors.range = status->rear_laser.range_mm;
				readings |= SENSOR_REAR_RANGE;
			}
			break;
		default:
			// No camera handling for rear sensors
			break;
		}
	} while (prev_state != cp->rear_sensors.state);

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
		front_range = cp->front_sensors.range;
	}

	bool have_rear_range = false;
	uint16_t rear_range = 4096;
	if (readings & SENSOR_REAR_RANGE) {
		have_rear_range = true;
		rear_range = cp->rear_sensors.range;
	}

	bool have_frame = false;
	if (readings & SENSOR_CAMERA) {
		log_printf(&util_logger, "frame done: %d, %d, %d",
				cp->front_sensors.left, cp->front_sensors.right, cp->front_sensors.bottom);
		if ((cp->front_sensors.right - cp->front_sensors.left) >= 2) {
			have_frame = true;
		}
	}

	// Handle state changes

	log_printf(&util_logger, "frm: %d, rng: %d (%d), rrng: %d (%d) spd: %d, hdg: %d",
			have_frame, have_front_range, front_range, have_rear_range, rear_range, cp->speed_control, cp->heading_control);

	if (have_front_range && (cp->speed_control == SPEED_CONTROL_FRONT_DISTANCE_RELATIVE)) {
		log_printf(&util_logger, "setting relative distance");
		cp->target_distance = front_range + cp->target_distance;
		if (cp->target_distance < min_distance) {
			cp->target_distance = min_distance;
		}
		cp->target_distance_valid = true;
		cp->speed_control = SPEED_CONTROL_FRONT_DISTANCE;
	}

	if (have_rear_range && (cp->speed_control == SPEED_CONTROL_REAR_DISTANCE_RELATIVE)) {
		log_printf(&util_logger, "setting relative rdistance");
		cp->target_distance = rear_range + cp->target_distance;
		if (cp->target_distance < min_distance) {
			cp->target_distance = min_distance;
		}
		cp->target_distance_valid = true;
		cp->speed_control = SPEED_CONTROL_REAR_DISTANCE;
	}

	// Adjust speed/heading

	int8_t linear_speed = 0;
	switch (cp->speed_control) {
	case SPEED_CONTROL_FRONT_DISTANCE:
		if (have_front_range) {
			int diff = front_range - cp->target_distance;
			log_printf(&util_logger, "dist ctrl diff: %d", diff);
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
	case SPEED_CONTROL_REAR_DISTANCE:
		if (have_rear_range) {
			int diff = rear_range - cp->target_distance;
			log_printf(&util_logger, "rdist ctrl diff: %d", diff);
			if (diff > 0) {
				// TODO: Signs.
				linear_speed = -cp->linear_speed;
			} else if (diff < 0) {
				// TODO: Signs.
				linear_speed = cp->linear_speed;
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
			float diff = (cp->front_sensors.left - 20) * deg_per_pixel;
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
			current_conditions |= END_COND_FRONT_DISTANCE_LTE;
			if (diff > -distance_tolerance) {
				current_conditions |= END_COND_FRONT_DISTANCE_EQ;
			}
		} else if (diff > 0) {
			current_conditions |= END_COND_FRONT_DISTANCE_GTE;
			if (diff < distance_tolerance) {
				current_conditions |= END_COND_FRONT_DISTANCE_EQ;
			}
		} else {
			current_conditions |= END_COND_FRONT_DISTANCE_EQ;
		}
	}

	if (have_rear_range && cp->target_distance_valid) {
		int diff = rear_range - cp->target_distance;
		if (diff < 0) {
			current_conditions |= END_COND_REAR_DISTANCE_LTE;
			if (diff > -distance_tolerance) {
				current_conditions |= END_COND_REAR_DISTANCE_EQ;
			}
		} else if (diff > 0) {
			current_conditions |= END_COND_REAR_DISTANCE_GTE;
			if (diff < distance_tolerance) {
				current_conditions |= END_COND_REAR_DISTANCE_EQ;
			}
		} else {
			current_conditions |= END_COND_REAR_DISTANCE_EQ;
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

	log_printf(&util_logger, "cur_cond: 0x%x, end_cond: 0x%x", current_conditions, cp->end_conditions);

	if ((current_conditions & cp->end_conditions) == cp->end_conditions) {
		// Stop!
		log_printf(&util_logger, "end cond hit");
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

static void chassis_set_control(
		struct chassis_planner *cp,
		enum speed_control speed_control, int16_t distance, int8_t speed,
		enum heading_control heading_control, float heading,
		uint32_t end_conditions)
{
	cp->end_conditions = end_conditions;
	cp->target_distance = distance;
	cp->target_distance_valid = ((speed_control == SPEED_CONTROL_FRONT_DISTANCE) ||
			(speed_control == SPEED_CONTROL_REAR_DISTANCE));
	cp->linear_speed = speed;
	cp->speed_control = speed_control;
	cp->heading_control = heading_control;
	cp->target_heading = heading;
	cp->sensors = 0;
	if ((speed_control == SPEED_CONTROL_FRONT_DISTANCE) || (speed_control == SPEED_CONTROL_FRONT_DISTANCE_RELATIVE)) {
		cp->sensors |= SENSOR_FRONT_RANGE;
	}
	if ((speed_control == SPEED_CONTROL_REAR_DISTANCE) || (speed_control == SPEED_CONTROL_REAR_DISTANCE_RELATIVE)) {
		cp->sensors |= SENSOR_REAR_RANGE;
	}
	if (heading_control == HEADING_CONTROL_RED_BLOB) {
		cp->sensors |= SENSOR_CAMERA;
	}
	cp->active = true;
	cp->last_change_ts = get_absolute_time();
}

static void chassis_stop(struct chassis_planner *cp)
{
	chassis_set_control(cp, SPEED_CONTROL_FIXED, 0, 0,
			HEADING_CONTROL_FORWARD, 0,
			0);
	/*
	cp->end_conditions = 0;
	cp->linear_speed = 0;
	cp->target_distance_valid = false;
	cp->speed_control = SPEED_CONTROL_FIXED;
	cp->heading_control = HEADING_CONTROL_FORWARD;
	cp->sensors = 0;
	cp->active = true;
	cp->last_change_ts = get_absolute_time();
	*/
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
			//chassis_set_heading_to_distance_lte(&task->cp, 650, heading, fast_speed * 2);
			//chassis_set_heading_to_distance(&task->cp, 650, heading, fast_speed * 2, false, false, DISTANCE_COND_LTE);
			//chassis_set_heading_to_distance(&task->cp, 500, heading, fast_speed * 2, true, false, DISTANCE_COND_GTE);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, 500, very_fast_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_GTE);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, rear_distances_home[task->branch_idx] - 100, very_fast_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_GTE);
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
			//chassis_set_heading_to_distance(&task->cp, 550, heading, slow_speed, false, false, DISTANCE_COND_EQ);
			//chassis_set_heading_to_distance(&task->cp, 605, heading, slow_speed, true, false, DISTANCE_COND_EQ);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, 615, slow_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_EQ);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, rear_distances_home[task->branch_idx], slow_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_EQ);
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
			//set_chassis_to_heading(tree_heading[hidx]);
			boom_set(&task->bp, tuning->approach_boom_targets[task->apple_idx]);
			picker_set(&task->pp, PICKER_STATE_OPEN);
			float heading = normalise_angle(task->west + ((task->branch_idx + 2) * 90));
			//chassis_turn_to(&task->cp, heading, 0);
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
			//chassis_set_distance_gte(&task->cp, 110, fast_speed);
			//chassis_set_blob_to_distance(&task->cp, 210, fast_speed);
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
			//chassis_set_beam(&task->cp, fast_speed);
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
			//chassis_set_distance_lte_relative(&task->cp, -reach_distances[task->apple_idx], slow_speed);
			//chassis_set_control(&task->cp, SPEED_CONTROL_FRONT_DISTANCE_RELATIVE, -reach_distances[task->apple_idx], slow_speed,
			//		HEADING_CONTROL_FORWARD, 0,
			//		END_COND_FRONT_DISTANCE_LTE);
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
			//chassis_set_distance_gte_relative(&task->cp, back_up_distances[task->apple_idx], fast_speed);
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
			//chassis_turn_to(&task->cp, heading, 0);
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
			//chassis_set_heading_to_distance_lte(&task->cp, 120, heading, fast_speed * 2);
			//chassis_set_heading_to_distance(&task->cp, 120, heading, fast_speed * 2, false, false, DISTANCE_COND_LTE);
			//chassis_set_heading_to_distance(&task->cp, 1010, heading, fast_speed * 2, true, false, DISTANCE_COND_LTE);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, 1010, very_fast_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_GTE);
			//chassis_set_control(&task->cp, SPEED_CONTROL_REAR_DISTANCE, corner_distances_home[task->branch_idx], very_fast_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_REAR_DISTANCE_GTE);
			//chassis_set_control(&task->cp, SPEED_CONTROL_FRONT_DISTANCE, corner_front_distances_home[task->branch_idx], very_fast_speed,
			//		HEADING_CONTROL_EXPLICIT, heading,
			//		END_COND_FRONT_DISTANCE_LTE);
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
	}

	boom_tick(&task->bp, platform, status);
	picker_tick(&task->pp, platform, status);
	chassis_tick(&task->cp, platform, status);

	log_printf(&util_logger, "<<< coord tick done: %d", task->coord_state);
}

static void apple_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct apple_task *task = (struct apple_task *)ptask;

	if (input->buttons.pressed & BTN_TRIANGLE) {
		task->apple_idx = 0;
		task->branch_idx = 0;
		task->tuning = &apples_tunings[TUNING_HOME];
		coord_set_state(task, COORD_STATE_CORNER_TURN);
		task->run_coord = true;
		log_printf(&util_logger, "coord input: %d, %d", task->coord_state, task->state_entry);
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
