/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdlib.h>

#include "scheduler.h"

struct scheduler_slot {
	absolute_time_t scheduled;
	struct task *task;
};

struct scheduler {
	uint n_slots;
	// TODO: Implement a smarter structure for sorting tasks
	struct scheduler_slot *slots;
};

static task_id_t alloc_task_id(struct scheduler *sched)
{
	// TODO: Implement a smarter ID allocator
	uint8_t i;
	for (i = 0; i < sched->n_slots; i++) {
		if (!sched->slots[i].task) {
			return i + 1;
		}
	}

	return 0;
}

#define MAX_SLOTS 32 // Arbitrary, but should be sufficient and also the current
                     // data structures won't scale well

struct scheduler *scheduler_create(uint8_t n_slots)
{
	if (n_slots >= MAX_SLOTS) {
		return NULL;
	}

	struct scheduler *sched = calloc(1, sizeof(struct scheduler));
	if (!sched) {
		return NULL;
	}

	sched->n_slots = n_slots;
	sched->slots = calloc(n_slots, sizeof(struct scheduler_slot));
	if (!sched->slots) {
		free(sched);
		return NULL;
	}

	return sched;
}

absolute_time_t scheduler_next_tick(struct scheduler *sched)
{
	absolute_time_t tick = at_the_end_of_time;
	uint8_t i;
	for (i = 0; i < sched->n_slots; i++) {
		struct scheduler_slot *slot = &sched->slots[i];
		if (!slot->task) {
			continue;
		}

		if (slot->scheduled < tick) {
			tick = slot->scheduled;
		}
	}

	return tick;
}

absolute_time_t scheduler_tick(struct scheduler *sched)
{
	absolute_time_t now = get_absolute_time();
	absolute_time_t next = at_the_end_of_time;

	uint8_t i;
	for (i = 0; i < sched->n_slots; i++) {
		struct scheduler_slot *slot = &sched->slots[i];
		if (!slot->task) {
			continue;
		}

		// TODO: Doesn't run tasks in order
		if (slot->scheduled < now) {
			slot->scheduled = slot->task->on_tick(slot->task, now);
		}

		if (slot->scheduled < next) {
			next = slot->scheduled;
		}
	}

	return next;
}

task_id_t scheduler_task_register(struct scheduler *sched, struct task *task)
{
	assert(task && task->on_tick);

	task_id_t id = alloc_task_id(sched);
	if (id == 0) {
		return 0;
	}

	struct scheduler_slot *slot = &sched->slots[id - 1];

	absolute_time_t now = get_absolute_time();
	slot->task = task;
	if (task->on_register) {
		slot->scheduled = task->on_register(task, now);
	} else {
		slot->scheduled = at_the_end_of_time;
	}

	return id;
}

void scheduler_task_command(struct scheduler *sched, task_id_t tid, uint16_t prop, uint32_t *value, uint8_t *result)
{
	hard_assert(tid <= sched->n_slots);

	struct scheduler_slot *slot = &sched->slots[tid - 1];

	if (!slot->task->on_command) {
		*result = 0xff;
		return;
	}

	absolute_time_t now = get_absolute_time();
	slot->scheduled = slot->task->on_command(slot->task, now, prop, value, result);
}
