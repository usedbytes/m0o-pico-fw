/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __CAMERA_TASK_H__
#define __CAMERA_TASK_H__

#include <stdint.h>

#include "pico/util/queue.h"

#include "camera/camera.h"

#include "i2c_bus.h"
#include "scheduler.h"

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
void run_camera(queue_t *pos_queue, struct i2c_bus *i2c);

struct camera_task {
	struct task task;
	struct camera camera;

	uint8_t frame_no;
	volatile uint8_t completed_frame_no;

	struct camera_buffer *queued;
};

enum camera_task_prop {
	CAMERA_TASK_CAPTURE = 1,
	CAMERA_TASK_GET_FRAME_COMPLETED = 2,
};

int camera_task_init(struct camera_task *task, struct camera_platform_config *platform);

#endif /* __CAMERA_TASK_H__ */
