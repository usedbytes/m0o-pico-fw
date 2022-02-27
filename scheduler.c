/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <stdlib.h>

#include "scheduler.h"

struct list_node {
	struct list_node *prev;
	struct list_node *next;
};

struct scheduler_slot {
	struct list_node node;
	absolute_time_t scheduled;
	struct task *task;
};

struct scheduler {
	uint n_slots;
	struct list_node head;
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
	if (sched->head.next) {
		tick = ((struct scheduler_slot *)(sched->head.next))->scheduled;
	}

	return tick;
}

static void list_remove(struct list_node *node)
{
	struct list_node *prev_node = node->prev;
	struct list_node *next_node = node->next;

	prev_node->next = node->next;
	if (next_node) {
		next_node->prev = node->prev;
	}

	node->next = NULL;
	node->prev = NULL;
}

static void list_insert_after(struct list_node *insert, struct list_node *after)
{
	struct list_node *next_node = after->next;

	after->next = insert;
	insert->prev = after;
	insert->next = next_node;
	if (next_node) {
		next_node->prev = insert;
	}
}

static void insert_sorted(struct scheduler *sched, struct scheduler_slot *insert)
{
	struct list_node *node = &sched->head;
	while (node->next) {
		struct scheduler_slot *next_slot = (struct scheduler_slot *)node->next;
		if (next_slot->scheduled > insert->scheduled) {
			break;
		}

		node = node->next;
	}

	list_insert_after(&insert->node, node);
}

void scheduler_tick(struct scheduler *sched)
{
	absolute_time_t now = get_absolute_time();

	struct scheduler_slot *slot = (struct scheduler_slot *)sched->head.next;
	while (slot && slot->scheduled <= now) {
		struct scheduler_slot *next_slot = (struct scheduler_slot *)slot->node.next;

		list_remove(&slot->node);
		slot->scheduled = slot->task->on_tick(slot->task, now);
		insert_sorted(sched, slot);

		slot = next_slot;
	}
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

	insert_sorted(sched, slot);

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
	absolute_time_t schedule;
	*result = slot->task->on_command(slot->task, now, &schedule, prop, value);
	if (schedule != slot->scheduled) {
		list_remove(&slot->node);
		insert_sorted(sched, slot);
	}
}
