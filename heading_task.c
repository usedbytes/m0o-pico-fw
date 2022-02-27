/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "heading_task.h"

#include <string.h>

#define to_heading_task(_t) ((struct heading_task *)(_t))

absolute_time_t heading_task_on_register(struct task *t, absolute_time_t tick)
{
	struct heading_task *task = to_heading_task(t);

	return tick + task->poll_time_us;
}

absolute_time_t heading_task_on_tick(struct task *t, absolute_time_t tick)
{
	struct heading_task *task = to_heading_task(t);

	int16_t heading;
	int ret = bno055_get_heading(&task->bno055, &heading);
	if (!ret) {
		task->last.timestamp = tick;
		task->last.heading = heading;
	}

	return tick + task->poll_time_us;
}

uint8_t heading_task_on_command(struct task *t, absolute_time_t tick, absolute_time_t *schedule, uint16_t prop, uint32_t *value)
{
	struct heading_task *task = to_heading_task(t);

	switch ((enum heading_task_prop)prop) {
	case HEADING_TASK_GET_HEADING: {
		struct heading_reading *out = (struct heading_reading *)(*value);
		memcpy(out, &task->last, sizeof(*out));
		return 0;
	}
	default:
		return 0xff;
	};
}

int heading_task_init(struct heading_task *task, struct i2c_bus *bus, uint8_t addr_7b, uint32_t poll_time_us)
{
	int ret = bno055_init(&task->bno055, bus, addr_7b);
	if (ret) {
		return ret;
	}

	task->poll_time_us = poll_time_us;
	task->last = (struct heading_reading){ 0 };
	task->task = (struct task){
		.on_register = heading_task_on_register,
		.on_tick = heading_task_on_tick,
		.on_command = heading_task_on_command,
	};

	return 0;
}
