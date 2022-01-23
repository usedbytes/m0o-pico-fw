/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <stdint.h>

#include "pico/util/queue.h"

#define FORMAT_YUYV (('Y' << 0) | ('U' << 8) | ('Y' << 16) | ('V' << 24))
#define FORMAT_RGB565 (('R' << 0) | ('G' << 8) | ('1' << 16) | ('6' << 24))
#define FORMAT_YUV422 (('Y' << 0) | ('U' << 8) | ('1' << 16) | ('6' << 24))

struct camera_buffer {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint32_t strides[3];
	uint32_t sizes[3];
	uint8_t *data[3];
};

typedef void (*camera_frame_cb)(struct camera_buffer *buf, void *p);

enum camera_queue_item_type {
	CAMERA_QUEUE_ITEM_CAPTURE = 0,
	CAMERA_QUEUE_ITEM_HOST_ALLOC,
	CAMERA_QUEUE_ITEM_LOCAL_PROCESS,
	CAMERA_QUEUE_ITEM_HOST_COMPLETE,
};

struct camera_queue_item {
	uint8_t type;
	uint8_t pad[3];
	union {
		struct {
			struct camera_buffer *buf;
			camera_frame_cb frame_cb;
			void *cb_data;
		} capture;
		struct {
			struct camera_buffer *buf;
		} local_process;
		struct {
			uint32_t format;
		} host_alloc;
		struct {
			uint32_t pad[3];
		} body_pad;
	};
};

struct camera_buffer *camera_buffer_alloc(uint32_t format, uint16_t width, uint16_t height);
void camera_buffer_free(struct camera_buffer *buf);

void camera_queue_add_blocking(struct camera_queue_item *qitem);
void run_camera(queue_t *pos_queue);

#endif /* __CAMERA_H__ */
