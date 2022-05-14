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
#include "util.h"

#include "platform/platform.h"

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

void chassis_tick(struct chassis_planner *cp, struct platform *platform, struct platform_status_report *status)
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

void chassis_set_control(
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

void chassis_stop(struct chassis_planner *cp)
{
	chassis_set_control(cp, SPEED_CONTROL_FIXED, 0, 0,
			HEADING_CONTROL_FORWARD, 0,
			0);
}

bool chassis_done(struct chassis_planner *cp)
{
	return !cp->active;
}
