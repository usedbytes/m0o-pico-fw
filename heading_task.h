/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __HEADING_TASK_H__
#define __HEADING_TASK_H__

#include <stdint.h>

#include "pico/util/queue.h"

#include "bno055.h"

#include "i2c_bus.h"
#include "scheduler.h"

struct heading_reading {
	absolute_time_t timestamp;
	int16_t heading;
};

struct heading_task {
	struct task task;
	struct bno055 bno055;

	uint32_t poll_time_us;
	struct heading_reading last;
};

enum heading_task_prop {
	// value is pointer to struct heading_reading
	HEADING_TASK_GET_HEADING = 1,
};

int heading_task_init(struct heading_task *task, struct i2c_bus *bus, uint8_t addr_7b, uint32_t poll_time_us);

#endif /* __HEADING_TASK_H__ */
