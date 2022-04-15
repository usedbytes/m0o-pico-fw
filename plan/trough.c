/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "pico/stdlib.h"

#include "log.h"
#include "planner.h"
#include "util.h"

#include "camera/camera.h"
#include "camera/format.h"
#include "platform/platform.h"

enum trough_state {
	TROUGH_SET_EAST,
	TROUGH_POINT,
	TROUGH_WAIT_FOR_POINT,
	TROUGH_APPROACH,
	TROUGH_PROX,
	TROUGH_BOOM_POSITION,
	TROUGH_OPEN_FLAP,
	TROUGH_WAIT_FOR_GRAIN,
	TROUGH_GO_HOME,
	TROUGH_WAIT_FOR_HOME,
	TROUGH_WAIT_AT_HOME,
	TROUGH_DONE,
};

struct trough_task {
	struct planner_task base;
	struct platform *platform;
	struct camera_buffer *buf;
	volatile bool frame_done;
	int left, right, mid, bottom;
	absolute_time_t timestamp;

	enum trough_state state;

	int trough_idx;
	float east;
	float heading;
	float return_heading;
};

const uint8_t threshold = 10;

const struct v2 barn = { 150, 100 };
const struct v2 troughs[] = {
	{ 150, 875 },
	{ 1300, 1300 },
	{ 1300, 375 },
};
const int n_troughs = sizeof(troughs) / sizeof(troughs[0]);
const float overshoot[] = {
	0.0,
	0.0,
	0.0,
};
const float home_distance[] = {
	150,
	220,
	150,
};

static void trough_camera_cb(struct camera_buffer *buf, void *p)
{
	struct trough_task *task = (struct trough_task *)p;

	//log_printf(&util_logger, "capture cb. %p", buf);

	task->frame_done = true;
}

static void request_frame(struct trough_task *task)
{
	task->frame_done = false;
	platform_camera_capture(task->platform, task->buf, trough_camera_cb, task);
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

static void trough_set_east(struct trough_task *task, struct platform_status_report *status)
{
	struct platform *platform = task->platform;
	task->east = normalise_angle(status->heading / 16.0);
	platform_heading_controller_set_enabled(platform, true);
	task->state = TROUGH_POINT;

	log_printf(&util_logger, "east: %3.2f", task->east);
}

static void trough_point(struct trough_task *task, struct platform_status_report *status)
{
	const float extra_amount = overshoot[task->trough_idx];
	struct platform *platform = task->platform;
	float dx = troughs[task->trough_idx].x - barn.x;
	float dy = troughs[task->trough_idx].y - barn.y;

	float delta = (atan2f(dy, dx) * 180 / 3.14159);
	float target_heading = normalise_angle(task->east - delta);
	float diff = normalise_angle(target_heading - (status->heading / 16.0));
	float extra = copysignf(extra_amount, diff);

	log_printf(&util_logger, "dy, dx %f, %f, delta: %3.2f, diff: %3.2f", dy, dx, delta, diff);

	task->return_heading = target_heading;
	task->heading = target_heading + extra;
	platform_heading_controller_set_enabled(platform, true);
	platform_heading_controller_set(platform, 10, task->heading);
	task->state = TROUGH_WAIT_FOR_POINT;

	log_printf(&util_logger, "heading: %3.2f", task->heading);
}

static void trough_wait_for_point(struct trough_task *task, struct platform_status_report *status)
{
	const float cone_angle = 5;
	float diff = normalise_angle((status->heading / 16.0) - task->heading);
	log_printf(&util_logger, "%3.2f -> %3.2f diff: %3.2f", status->heading / 16.0, task->heading, diff);
	if ((diff >= -cone_angle) && (diff <= cone_angle)) {
		task->state = TROUGH_APPROACH;
		request_frame(task);
	}
}

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

static void trough_approach(struct trough_task *task, struct platform_status_report *status)
{
	const float deg_per_pixel = 0.835;
	const int bottom_line = 58;
	const int min_width = 2;
	struct platform *platform = task->platform;

	if (!task->frame_done) {
		return;
	}

	task->frame_done = false;
	task->mid = find_red_blob(task->buf, &task->left, &task->right, &task->bottom);
	log_printf(&util_logger, "frame done: %d, %d, %d, %d", task->left, task->mid, task->right, task->bottom);

	float current_heading = status->heading / 16.0;

	// Try and check if we got a good read
	if ((task->right - task->left) <= min_width) {
		// TODO: What to do? Add a searching state?
		platform_heading_controller_set(platform, 10, current_heading);
		request_frame(task);
		return;
	}

	// Use vertical postiion to detect rough distance
	if (task->bottom >= bottom_line) {
		log_printf(&util_logger, "prox!");
		platform_heading_controller_set_enabled(platform, false);

		task->timestamp = get_absolute_time();
		task->state = TROUGH_PROX;
		platform_vl53l0x_trigger_single(platform, 0);
		return;
	}

	float diff = (task->left - 20) * deg_per_pixel;
	platform_heading_controller_set(platform, 30, current_heading + diff);

	request_frame(task);
}

static void trough_prox(struct trough_task *task, struct platform_status_report *status)
{
	struct platform *platform = task->platform;
	const int target_distance = 100;

	if (status->front_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	log_printf(&util_logger, "prox: %d mm", status->front_laser.range_mm);

	if (status->front_laser.range_mm > target_distance) {
		// Get a bit closer
		task->timestamp = get_absolute_time();
		platform_vl53l0x_trigger_single(platform, 0);
		platform_set_velocity(platform, 5, 0);
		return;
	}

	platform_set_velocity(platform, 0, 0);
	task->state = TROUGH_BOOM_POSITION;
	task->timestamp = get_absolute_time();
	platform_boom_trajectory_controller_adjust_target(platform, (struct v2){ status->front_laser.range_mm - 55, 40 },
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);
}

static void trough_boom_position(struct trough_task *task, struct platform_status_report *status)
{
	struct platform *platform = task->platform;
	const int64_t timeout_ns = 2000000;
	const int range_slop = 30;
	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);

	float x = status->boom_pos.x;
	bool timeout = diff > timeout_ns;
	bool in_range = (x > ((status->front_laser.range_mm - range_slop)));

	if ((status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED) || (timeout && in_range)) {
		log_printf(&util_logger, "Boom positioned!");
		platform_boom_trajectory_controller_set_enabled(platform, false);
		task->state = TROUGH_OPEN_FLAP;
		return;
	}
}

static void trough_open_flap(struct trough_task *task, struct platform_status_report *status)
{
	struct platform *platform = task->platform;

#define GRAIN_FLAP_OPEN   4850
	log_printf(&util_logger, "open flap!");
	platform_ioe_set(platform, 2, GRAIN_FLAP_OPEN);
	task->state = TROUGH_WAIT_FOR_GRAIN;
	task->timestamp = get_absolute_time();
}

static void trough_wait_for_grain(struct trough_task *task, struct platform_status_report *status)
{
	const int grain_drop_ms = 3300;
	struct platform *platform = task->platform;

#define GRAIN_FLAP_CLOSED 3200
	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	//log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff >= grain_drop_ms * 1000) {
		log_printf(&util_logger, "close flap!");
		platform_ioe_set(platform, 2, GRAIN_FLAP_CLOSED);
		task->state = TROUGH_GO_HOME;
	}
}

static void trough_go_home(struct trough_task *task, struct platform_status_report *status)
{
	struct platform *platform = task->platform;

	//platform_heading_controller_set(platform, -30, task->return_heading);
	platform_heading_controller_set_enabled(platform, true);
	platform_heading_controller_set(platform, -30, task->return_heading);
	platform_vl53l0x_trigger_single(platform, 1);

	platform_boom_trajectory_controller_adjust_target(platform, (struct v2){ 30, 90 },
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
	platform_boom_trajectory_controller_set_enabled(platform, true);

	task->state = TROUGH_WAIT_FOR_HOME;

	log_printf(&util_logger, "return heading: %3.2f", task->return_heading);
}

static void trough_wait_for_home(struct trough_task *task, struct platform_status_report *status)
{
	const int target_distance = home_distance[task->trough_idx];
	struct platform *platform = task->platform;

	if (status->rear_laser.timestamp <= task->timestamp) {
		// Range not measured yet
		return;
	}

	log_printf(&util_logger, "rear: %d mm", status->rear_laser.range_mm);

	if (status->rear_laser.range_mm <= target_distance) {
		log_printf(&util_logger, "reached home");

		task->trough_idx++;
		if (task->trough_idx >= n_troughs) {
			task->trough_idx = 0;
			task->state = TROUGH_DONE;
		} else {
			task->timestamp = get_absolute_time();
			task->state = TROUGH_WAIT_AT_HOME;
		}

		platform_heading_controller_set_enabled(platform, false);
		platform_set_velocity(platform, 0, 0);
		return;
	}

	task->timestamp = get_absolute_time();
	platform_vl53l0x_trigger_single(platform, 1);
}

static void trough_wait_at_home(struct trough_task *task, struct platform_status_report *status)
{
	const int wait_time_ms = 2000;

	absolute_time_t now = get_absolute_time();
	int64_t diff = absolute_time_diff_us(task->timestamp, now);
	//log_printf(&util_logger, "waiting: %"PRId64, diff);
	if (diff >= wait_time_ms * 1000) {
		log_printf(&util_logger, "let's go!");
		task->state = TROUGH_POINT;
	}
}


static void trough_task_tick(struct planner_task *ptask, struct platform *platform, struct platform_status_report *status)
{
	struct trough_task *task = (struct trough_task *)ptask;

	switch (task->state) {
	case TROUGH_SET_EAST:
		trough_set_east(task, status);
		break;
	case TROUGH_POINT:
		trough_point(task, status);
		break;
	case TROUGH_WAIT_FOR_POINT:
		trough_wait_for_point(task, status);
		break;
	case TROUGH_APPROACH:
		trough_approach(task, status);
		break;
	case TROUGH_PROX:
		trough_prox(task, status);
		break;
	case TROUGH_BOOM_POSITION:
		trough_boom_position(task, status);
		break;
	case TROUGH_OPEN_FLAP:
		trough_open_flap(task, status);
		break;
	case TROUGH_WAIT_FOR_GRAIN:
		trough_wait_for_grain(task, status);
		break;
	case TROUGH_GO_HOME:
		trough_go_home(task, status);
		break;
	case TROUGH_WAIT_FOR_HOME:
		trough_wait_for_home(task, status);
		break;
	case TROUGH_WAIT_AT_HOME:
		trough_wait_at_home(task, status);
		break;
	case TROUGH_DONE:
		break;
	}
}

static void trough_task_handle_input(struct planner_task *ptask, struct platform *platform, struct input_state *input)
{
	struct trough_task *task = (struct trough_task *)ptask;

	if ((task->state == TROUGH_WAIT_FOR_HOME) && (input->buttons.pressed & BTN_TRIANGLE)) {
		log_printf(&util_logger, "reached home");
		task->trough_idx++;
		if (task->trough_idx >= n_troughs) {
			task->trough_idx = 0;
			task->state = TROUGH_DONE;
		} else {
			task->timestamp = get_absolute_time();
			task->state = TROUGH_WAIT_AT_HOME;
		}

		platform_heading_controller_set_enabled(platform, false);
		platform_set_velocity(platform, 0, 0);
	}
}

static void trough_task_on_start(struct planner_task *ptask)
{
	struct trough_task *task = (struct trough_task *)ptask;

	task->state = TROUGH_SET_EAST;
	task->trough_idx = 0;
	platform_heading_controller_set_enabled(task->platform, true);
	request_frame(task);
}

struct planner_task *trough_get_task(struct platform *platform)
{
	static struct trough_task task = {
		.base = {
			.handle_input = trough_task_handle_input,
			.on_start = trough_task_on_start,
			.tick = trough_task_tick,
		},
	};

	task.platform = platform;
	task.buf = camera_buffer_alloc(FORMAT_YUV422, 80, 60);
	if (!task.buf) {
		log_printf(&util_logger, "buffer alloc failed");
		return NULL;
	}

	return &task.base;
}
