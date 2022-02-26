/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "comm.h"
#include "log.h"
#include "scheduler.h"
#include "util.h"

// This list is ordered to try and put the most frequent messages near the start
const struct comm_command *const cmds[] = {
	&util_sync_cmd,
	&util_logs_cmd,
	&util_reboot_cmd,
	&util_read_cmd,
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

struct command {
	task_id_t target;
	uint8_t result;
	uint16_t prop;
	uint32_t value;
};

struct command_list {
	uint16_t n_cmds;
	uint16_t capacity;
	bool pending;
	volatile bool complete;
	struct command commands[];
};

struct command_list *command_list_alloc(uint16_t n_cmds)
{
	struct command_list *cmd_list = calloc(1, sizeof(struct command_list) + n_cmds * sizeof(struct command));
	if (!cmd_list) {
		return NULL;
	}

	cmd_list->capacity = n_cmds;

	return cmd_list;
}

void command_list_free(struct command_list *cmd_list)
{
	free(cmd_list);
}

void command_list_reset(struct command_list *cmd_list)
{
	assert(cmd_list->pending == cmd_list->complete);

	cmd_list->n_cmds = 0;
	cmd_list->pending = false;
	cmd_list->complete = false;
	memset(cmd_list->commands, 0, sizeof(struct command) * cmd_list->capacity);
}

struct command *command_list_alloc_commands(struct command_list *cmd_list, uint16_t n_cmds)
{
	if (cmd_list->pending != cmd_list->complete) {
		// Busy
		return NULL;
	}

	if (cmd_list->n_cmds + n_cmds > cmd_list->capacity) {
		// No space
		return NULL;
	}

	struct command *ret = &cmd_list->commands[cmd_list->n_cmds];
	cmd_list->n_cmds += n_cmds;

	return ret;
}

void command_list_submit_blocking(struct command_list *cmd_list, queue_t *to)
{
	assert(cmd_list->pending == cmd_list->complete);

	cmd_list->complete = false;
	cmd_list->pending = true;
	queue_add_blocking(to, &cmd_list);
}

bool command_list_try_submit(struct command_list *cmd_list, queue_t *to)
{
	assert(cmd_list->pending == cmd_list->complete);

	cmd_list->complete = false;
	cmd_list->pending = true;
	return queue_try_add(to, &cmd_list);
}

void command_list_signal_completion(struct command_list *cmd_list)
{
	assert(cmd_list->pending);
	cmd_list->complete = true;
}

void command_list_wait_for_completion(struct command_list *cmd_list)
{
	// TODO: could use WFE?
	while (cmd_list->pending != cmd_list->complete) {
		tight_loop_contents();
	}
}

#define MAX_N_TASKS 16

int64_t scheduler_kick_cb(alarm_id_t id, void *user_data)
{
	static struct command_list cmd_list = { 0 };
	queue_t *cmdq = (queue_t*)user_data;

	// If this fails then there's already commands in the queue to wake
	// up the thread, so it really doesn't matter
	command_list_try_submit(&cmd_list, cmdq);

	// Never reschedule.
	return 0;
}

struct {
	queue_t command_queue;
} core1_thread_state;

enum scheduler_command {
	// "value" must be a pointer to struct task
	SCHEDULER_TASK_REGISTER = 1,
};

static void core1_main(void)
{
	queue_t *cmdq = &core1_thread_state.command_queue;
	struct scheduler *sched = scheduler_create(MAX_N_TASKS);
	hard_assert(sched);

	// Dedicated alarm pool so we only use IRQs on this core
	alarm_pool_t *alarm_pool = alarm_pool_create(PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM - 1, 2);
	hard_assert(alarm_pool);
	alarm_id_t kick_alarm = 0;

	// Signal core 0 that init is done
	multicore_fifo_push_blocking(0);

	log_printf(&util_logger, "core1 booted");

	for ( ;; ) {
		struct command_list *cmd_list;

		// Peek for next deadline
		absolute_time_t next_tick = scheduler_next_tick(sched);

		// There's no timeout for blocking dequeue, so just insert
		// a dummy command list at the scheduled time
		// Presumably setting an alarm for at_the_end_of_time is OK
		kick_alarm = alarm_pool_add_alarm_at(alarm_pool, next_tick, scheduler_kick_cb, cmdq, true);

		// Wait for scheduled time or command list to arrive
		queue_remove_blocking(cmdq, &cmd_list);

		// Unconditionally cancel, we don't much care if it passed yet
		// or not
		alarm_pool_cancel_alarm(alarm_pool, kick_alarm);
		kick_alarm = 0;

		// Run any due tasks
		scheduler_tick(sched);

		// Then apply any updates. We already dequeued at least one command list
		do {
			uint i;
			for (i = 0; i < cmd_list->n_cmds; i++) {
				struct command *cmd = &cmd_list->commands[i];
				if (cmd->target == 0) {
					switch ((enum scheduler_command)cmd->prop) {
					case SCHEDULER_TASK_REGISTER:
						cmd->result = scheduler_task_register(sched, (struct task *)cmd->value);
						break;
					}
				} else {
					scheduler_task_command(sched, cmd->target, cmd->prop, &cmd->value, &cmd->result);
				}
			}
			command_list_signal_completion(cmd_list);
		} while (queue_try_remove(cmdq, &cmd_list));
	}
}

struct blink_task {
	struct task task;
	uint pin;
	uint period_ms;
	bool on;
};
#define to_blink_task(_t) ((struct blink_task *)(_t))

enum blink_task_prop {
	BLINK_TASK_PERIOD = 1,
};

absolute_time_t blink_on_register(struct task *t, absolute_time_t tick)
{
	struct blink_task *task = to_blink_task(t);

	return tick + (task->period_ms * 1000 / 2);
}

absolute_time_t blink_on_tick(struct task *t, absolute_time_t tick)
{
	struct blink_task *task = to_blink_task(t);

	task->on = !task->on;
	gpio_put(task->pin, task->on);

	return tick + (task->period_ms * 1000 / 2);
}

absolute_time_t blink_on_command(struct task *t, absolute_time_t tick, uint16_t prop, uint32_t *value, uint8_t *result)
{
	struct blink_task *task = to_blink_task(t);

	switch ((enum blink_task_prop)prop) {
	case BLINK_TASK_PERIOD:
		task->period_ms = *value;
		break;
	default:
		*result = 1;
		return at_the_end_of_time;
	}

	return tick + (task->period_ms * 1000 / 2);
}

void blink_task_init(struct blink_task *task, uint pin, uint period_ms)
{
	gpio_init(pin);
	gpio_set_dir(pin, GPIO_OUT);

	task->on = false;
	task->pin = pin;
	task->period_ms = period_ms;
	task->task = (struct task){
		.on_register = blink_on_register,
		.on_tick = blink_on_tick,
		.on_command = blink_on_command,
	};
}

struct count_task {
	struct task task;
	uint offset;
	uint count;
	uint increment;
};
#define to_count_task(_t) ((struct count_task *)(_t))

absolute_time_t count_on_register(struct task *t, absolute_time_t tick)
{
	struct count_task *task = to_count_task(t);

	return tick + task->offset + (1000 * 1000);
}

absolute_time_t count_on_tick(struct task *t, absolute_time_t tick)
{
	struct count_task *task = to_count_task(t);

	log_printf(&util_logger, "count (%5d) %d", task->offset, task->count);
	task->count += task->increment;

	return tick + (1000 * 1000);
}

void count_task_init(struct count_task *task, uint count, uint increment, uint offset)
{
	task->count = count;
	task->increment = increment;
	task->offset = offset;
	task->task = (struct task){
		.on_register = count_on_register,
		.on_tick = count_on_tick,
	};
}

int main()
{
	queue_t *cmdq = &core1_thread_state.command_queue;

	util_init();
	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	queue_init(cmdq, sizeof(struct command_list *), 5);
	multicore_launch_core1(core1_main);
	// Wait for "core 1 ready"
	multicore_fifo_pop_blocking();

	struct blink_task blink_task;
	blink_task_init(&blink_task, PICO_DEFAULT_LED_PIN, 300);

	struct count_task counters[3];
	const uint ncounters = sizeof(counters) / sizeof(counters[0]);

	struct command_list *cmd_list = command_list_alloc(1 + ncounters);
	struct command *cmds = command_list_alloc_commands(cmd_list, 1 + ncounters);

	cmds[0].target = 0;
	cmds[0].prop = SCHEDULER_TASK_REGISTER;
	cmds[0].value = (uint32_t)&blink_task;

	uint i;
	for (i = 0; i < ncounters; i++) {
		count_task_init(&counters[i], i, ncounters, i * 1000);

		cmds[1 + i].target = 0;
		cmds[1 + i].prop = SCHEDULER_TASK_REGISTER;
		cmds[1 + i].value = (uint32_t)&counters[i];
	}

	command_list_submit_blocking(cmd_list, cmdq);
	command_list_wait_for_completion(cmd_list);

	task_id_t blink_task_id = cmds[0].result;
	command_list_reset(cmd_list);

	struct command *cmd = command_list_alloc_commands(cmd_list, 1);

	while (1) {
		cmd->target = blink_task_id;
		cmd->prop = BLINK_TASK_PERIOD;
		cmd->value = 200;
		command_list_submit_blocking(cmd_list, cmdq);

		sleep_ms(2000);
		command_list_wait_for_completion(cmd_list);

		cmd->target = blink_task_id;
		cmd->prop = BLINK_TASK_PERIOD;
		cmd->value = 400;
		command_list_submit_blocking(cmd_list, cmdq);

		sleep_ms(2000);
		command_list_wait_for_completion(cmd_list);
	}
}
