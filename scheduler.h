/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <stdint.h>

#include "pico/time.h"

struct scheduler;

struct task {
	absolute_time_t (*on_register)(struct task *task, absolute_time_t tick);
	void (*on_remove)(struct task *task);
	absolute_time_t (*on_tick)(struct task *task, absolute_time_t tick);
	// FIXME: Ugly interface.
	absolute_time_t (*on_command)(struct task *task, absolute_time_t tick, uint16_t prop, uint32_t *value, uint8_t *result);
};

typedef uint8_t task_id_t;

struct scheduler *scheduler_create(uint8_t n_slots);

absolute_time_t scheduler_next_tick(struct scheduler *sched);
absolute_time_t scheduler_tick(struct scheduler *sched);

task_id_t scheduler_task_register(struct scheduler *sched, struct task *task);
void scheduler_task_command(struct scheduler *sched, task_id_t tid, uint16_t prop, uint32_t *value, uint8_t *result);

#endif /* __SCHEDULER_H__ */
