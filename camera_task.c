/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#include "camera/camera.h"
#include "camera/format.h"

#include "camera_task.h"
#include "log.h"
#include "util.h"

#define PIN_D0        16
#define GPIO_XCLK     21

#define CAMERA_PIO           pio0
#define CAMERA_DMA_CHAN_BASE 0

#define CMD_SNAP    (('S' << 0) | ('N' << 8) | ('A' << 16) | ('P' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))
#define CMD_CAMERA  (('C' << 0) | ('A' << 8) | ('M' << 16) | ('C' << 24))

static queue_t camera_queue;

#define CAMERA_HOST_CMD_TRIGGER     1
#define CAMERA_HOST_CMD_GET_REG_BUF 2
#define CAMERA_HOST_CMD_INIT        3
#define CAMERA_HOST_CMD_FORMAT      4

struct camera_host_cmd {
	uint8_t cmd;
	union {
		struct {
			uint8_t n_regs;
		} set_reg_buffer;
		struct {
			uint8_t format_idx;
		} format;
		struct {
			uint8_t pad[3];
		} pad;
	};
	uint32_t pdata;
};

enum host_state {
	HOST_STATE_NO_BUFFER = 0,
	HOST_STATE_IDLE,
	HOST_STATE_PENDING,
	HOST_STATE_COMPLETE,
};

volatile enum host_state host_state;
struct camera_buffer *host_buf;

void host_capture_cb(struct camera_buffer *buf, void *data)
{
	log_printf(&util_logger, "Host frame done");
	host_state = HOST_STATE_COMPLETE;

	struct camera_queue_item qitem;
	qitem.type = CAMERA_QUEUE_ITEM_HOST_COMPLETE;
	queue_try_add(&camera_queue, &qitem);
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

const struct comm_command camera_host_comm_cmd = {
	.opcode = CMD_CAMERA,
	.nargs = 2,
	.resp_nargs = 2,
	.size = NULL,
	.handle = &handle_camera_cmd,
};

static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct camera_host_cmd *cmd_in = (struct camera_host_cmd *)args_in;
	struct camera_queue_item qitem;

	switch (cmd_in->cmd) {
	case CAMERA_HOST_CMD_FORMAT:
		if (host_state == HOST_STATE_PENDING) {
			log_printf(&util_logger, "capture pending, can't free");
		}

		host_state = HOST_STATE_NO_BUFFER;
		camera_buffer_free((struct camera_buffer *)host_buf);
		host_buf = NULL;

		qitem.type = CAMERA_QUEUE_ITEM_HOST_ALLOC;
		switch (cmd_in->format.format_idx) {
		case 0:
			qitem.host_alloc.format = FORMAT_YUYV;
			break;
		case 1:
			qitem.host_alloc.format = FORMAT_RGB565;
			break;
		case 2:
			qitem.host_alloc.format = FORMAT_YUV422;
			break;
		default:
			qitem.host_alloc.format = FORMAT_RGB565;
			break;
		}
		queue_try_add(&camera_queue, &qitem);
		break;
	case CAMERA_HOST_CMD_TRIGGER:
		if (host_state == HOST_STATE_NO_BUFFER) {
			log_printf(&util_logger, "no host buffer");
		}

		host_state = HOST_STATE_PENDING;

		qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
		qitem.capture.buf = host_buf;
		qitem.capture.frame_cb = host_capture_cb;
		queue_try_add(&camera_queue, &qitem);
		break;
	default:
		return COMM_RSP_ERR;
	}

	return COMM_RSP_OK;
}

const struct comm_command snapget_cmd = {
	// SGET w h addr size
	.opcode = CMD_SNAPGET,
	.nargs = 0,
	.resp_nargs = 0,
	.size = &size_snapget,
	.handle = &handle_snapget,
};

static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	*resp_data_len_out = sizeof(struct camera_buffer);

	return COMM_RSP_OK;
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	if (host_state == HOST_STATE_COMPLETE) {
		memcpy(resp_data_out, host_buf, sizeof(*host_buf));
	} else {
		memset(resp_data_out, 0, sizeof(*host_buf));
	}

	return COMM_RSP_OK;
}

void local_capture_cb(struct camera_buffer *buf, void *data)
{
	struct camera_queue_item qitem;

	//log_printf(&util_logger, "Local frame done");

	qitem.type = CAMERA_QUEUE_ITEM_LOCAL_PROCESS;
	qitem.local_process.buf = buf;
	queue_add_blocking(&camera_queue, &qitem);
}

const uint8_t threshold = 10;

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
static int find_red_blob(struct camera_buffer *buf)
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

	log_printf(&util_logger, "Process done. %d us, mid %d", time_us_32() - start, mid);

	return mid;
}

void camera_queue_add_blocking(struct camera_queue_item *qitem)
{
	queue_add_blocking(&camera_queue, qitem);
}

static inline int __i2c_write_blocking(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len)
{
	return i2c_bus_write_blocking((struct i2c_bus *)i2c_handle, addr, src, len);
}

static inline int __i2c_read_blocking(void *i2c_handle, uint8_t addr, uint8_t *dst, size_t len)
{
	return i2c_bus_read_blocking((struct i2c_bus *)i2c_handle, addr, dst, len);
}

#define to_camera_task(_t) ((struct camera_task *)(_t))

uint8_t camera_on_command(struct task *t, absolute_time_t tick, absolute_time_t *schedule, uint16_t prop, uint32_t *value)
{
	struct camera_task *task = to_camera_task(t);

	switch ((enum camera_task_prop)prop) {
	case CAMERA_TASK_CAPTURE:
		if (task->queued) {
			// FIXME: What to do here? Want to sleep for frame completion
			*schedule = tick + 1000;

			// Busy
			return 0;
		} else {
			task->queued = (struct camera_buffer *)(*value);

			// Immediately re-schedule to trigger frame.
			// TODO: Just trigger from here instead?
			*schedule = tick;
			return task->frame_no++;
		}
		break;
	case CAMERA_TASK_GET_FRAME_COMPLETED:
		*value = task->completed_frame_no;
		return 0;
	default:
		*schedule = at_the_end_of_time;
		return 0;
	}
}

void camera_task_frame_cb(struct camera_buffer *buf, void *p)
{
	volatile uint8_t *completed = p;

	// Signal completion
	*completed = *completed + 1;
}

absolute_time_t camera_on_tick(struct task *t, absolute_time_t tick)
{
	struct camera_task *task = to_camera_task(t);

	if (task->queued) {
		int ret = camera_capture_with_cb(&task->camera, task->queued, true,
				camera_task_frame_cb, (void *)&task->completed_frame_no);
		if (ret) {
			// Defer and try again later
			// FIXME: Can only handle transient errors here.
			// -2 indicates a frame is pending.
			assert(ret == -2);

			// TODO: Would ideally sleep for frame completion
			return tick + 1000;
		}

		task->queued = NULL;
	}

	return at_the_end_of_time;
}

int camera_task_init(struct camera_task *task, struct camera_platform_config *platform)
{
	int ret = camera_init(&task->camera, platform);
	if (ret) {
		log_printf(&util_logger, "camera_init failed: %d", ret);
		return ret;
	}

	task->frame_no = 1;
	task->completed_frame_no = 0;
	task->queued = NULL;

	task->task = (struct task){
		.on_command = camera_on_command,
		.on_tick = camera_on_tick,
	};

	return 0;
}

void run_camera(queue_t *pos_queue, struct i2c_bus *i2c)
{
	log_printf(&util_logger, "run_camera()");
	queue_init(&camera_queue, sizeof(struct camera_queue_item), 8);

	struct camera camera;
	struct camera_platform_config platform = {
		.i2c_write_blocking = __i2c_write_blocking,
		.i2c_read_blocking = __i2c_read_blocking,
		.i2c_handle = i2c,

		.pio = CAMERA_PIO,
		.xclk_pin = GPIO_XCLK,
		.xclk_divider = 9,
		.base_pin = PIN_D0,
		.base_dma_channel = -1,
	};

	int ret = camera_init(&camera, &platform);
	if (ret) {
		log_printf(&util_logger, "camera_init failed: %d", ret);
		return;
	}

	log_printf(&util_logger, "camera_init done");

	const uint16_t width = CAMERA_WIDTH_DIV8;
	const uint16_t height = CAMERA_HEIGHT_DIV8;
	const uint32_t format = FORMAT_YUV422;

	camera_configure(&camera, format, width, height);

	host_buf = camera_buffer_alloc(FORMAT_YUV422, width, height);
	assert(host_buf);
	host_state = HOST_STATE_IDLE;

	log_printf(&util_logger, "host_buf: %p", host_buf);

	int pos;

	//struct camera_queue_item local_capture_qitem;

	//struct camera_buffer *local_buf = camera_buffer_alloc(FORMAT_YUV422, width, height);
	//local_capture_qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
	//local_capture_qitem.capture.buf = local_buf;
	//local_capture_qitem.capture.frame_cb = local_capture_cb;
	//queue_add_blocking(&camera_queue, &qitem);

	struct camera_queue_item qitem;
	while (1) {
		queue_remove_blocking(&camera_queue, &qitem);
		switch (qitem.type) {
		case CAMERA_QUEUE_ITEM_CAPTURE:
			ret = camera_capture_with_cb(&camera, qitem.capture.buf, true, qitem.capture.frame_cb, qitem.capture.cb_data);
			if (ret < 0) {
				log_printf(&util_logger, "Failed %d", ret);
				// FIXME: How to better handle re-submit?
				// Also, this could deadlock...
				sleep_ms(15);
				queue_add_blocking(&camera_queue, &qitem);
			}
			break;
		case CAMERA_QUEUE_ITEM_HOST_ALLOC: {
			char format[4];
			memcpy(format, &qitem.host_alloc.format, 4);

			host_buf = camera_buffer_alloc(qitem.host_alloc.format, width, height);
			host_state = HOST_STATE_IDLE;
			log_printf(&util_logger, "Alloc host buf %c%c%c%c, %p",
					format[0], format[1], format[2], format[3], host_buf);
			log_printf(&util_logger, "w, h: %d, %d",
					host_buf->width, host_buf->height);
			log_printf(&util_logger, "stride: %d, %d, %d",
					host_buf->strides[0], host_buf->strides[1], host_buf->strides[2]);
			log_printf(&util_logger, "size: %d, %d, %d",
					host_buf->sizes[0], host_buf->sizes[1], host_buf->sizes[2]);
			log_printf(&util_logger, "data: %d, %d, %d",
					host_buf->data[0], host_buf->data[1], host_buf->data[2]);
			break;
			}
		case CAMERA_QUEUE_ITEM_LOCAL_PROCESS: {
			struct camera_buffer *buf = qitem.local_process.buf;
			pos = find_red_blob(buf);
			queue_add_blocking(pos_queue, &pos);
			break;
			}
		case CAMERA_QUEUE_ITEM_HOST_COMPLETE:
			// TODO: Could deadlock
			//queue_add_blocking(&camera_queue, &local_capture_qitem);
			break;
		}
	}
}
