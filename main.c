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

#include "boom.h"
#include "comm.h"
#include "input.h"
#include "kinematics.h"
#include "log.h"
#include "platform.h"
#include "util.h"

#define CONTROL_QUEUE_LENGTH 32

const struct comm_command pid_set_cmd;

// This list is ordered to try and put the most frequent messages near the start
const struct comm_command *const cmds[] = {
	&input_cmd,
	&util_sync_cmd,
	&util_logs_cmd,
	&util_reboot_cmd,
	&util_read_cmd,
	&pid_set_cmd,
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

struct pid_coeffs_event {
	enum pid_controller_id id;
	float kp, ki, kd;
};

static uint32_t handle_pid(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	int32_t *coeffs = (int32_t *)&args_in[1];
	struct pid_coeffs_event ev;

	ev.id = args_in[0];
	ev.kp = coeffs[0] / 65536.0;
	ev.ki = coeffs[1] / 65536.0;
	ev.kd = coeffs[2] / 65536.0;

	control_event_send(CONTROL_EVENT_TYPE_PID, &ev, sizeof(ev));

	return COMM_RSP_OK;
}

#define CMD_PID_SET (('P' << 0) | ('I' << 8) | ('D' << 16) | ('T' << 24))
const struct comm_command pid_set_cmd = {
	.opcode = CMD_PID_SET,
	.nargs = 4,
	.resp_nargs = 0,
	.size = NULL,
	.handle = &handle_pid,
};

static void core1_main(void)
{
	int ret;
	struct platform platform;

	ret = platform_init(&platform);
	log_printf(&util_logger, "platform_init: %d", ret);

	multicore_fifo_push_blocking(platform.status);
	multicore_fifo_push_blocking((uint32_t)&platform);

	platform_run(&platform);
}

struct heading_result {
	absolute_time_t timestamp;
	int16_t heading;
};

struct heading_closure {
	struct platform *platform;
	struct heading_result *result;

	volatile bool done;
};

static void __get_heading_cb(absolute_time_t t, void *data)
{
	struct heading_closure *closure = (struct heading_closure *)data;

	closure->result->timestamp = closure->platform->heading_timestamp;
	closure->result->heading = closure->platform->heading;

	closure->done = true;
}

static void get_heading(struct platform *platform, struct heading_result *out)
{
	struct heading_closure closure = {
		.platform = platform,
		.result = out,
		.done = false,
	};

	platform_run_function(platform, __get_heading_cb, &closure);

	while (!closure.done);
}

struct boom_result {
	absolute_time_t timestamp;
	struct v2 pos;
};

struct boom_closure {
	struct platform *platform;
	struct boom_result *result;

	volatile bool done;
};

static void __get_boom_cb(absolute_time_t t, void *data)
{
	struct boom_closure *closure = (struct boom_closure *)data;

	closure->result->timestamp = closure->platform->boom_timestamp;
	closure->result->pos = closure->platform->boom_current;

	closure->done = true;
}

static void get_boom_position(struct platform *platform, struct boom_result *out)
{
	struct boom_closure closure = {
		.platform = platform,
		.result = out,
		.done = false,
	};

	platform_boom_update_position(platform);
	platform_run_function(platform, __get_boom_cb, &closure);

	while (!closure.done);
}

static int64_t __timer_dummy_event_cb(alarm_id_t id, void *user_data) {
	control_event_send_dummy();
	return 0;
}

const uint16_t y_offs = 55;
const uint16_t middle_apple_y = 180 - y_offs;
const uint16_t middle_apple_x = 100;

static void handle_pid_event(struct platform *platform, struct control_event *cev)
{
	struct pid_coeffs_event *ev = (struct pid_coeffs_event *)&cev->body_pad;

	log_printf(&util_logger, "PID set: %d, %.5f, %.5f, %.5f", ev->id, ev->kp, ev->ki, ev->kd);

	platform_set_pid_coeffs(platform, ev->id, ev->kp, ev->ki, ev->kd);
}

struct planner_task {
	void (*handle_input)(struct planner_task *task, struct platform *platform, struct input_state *input);
};

static void rc_task_handle_input(struct planner_task *task, struct platform *platform, struct input_state *input)
{
	if ((input->hat.held == 0) && input->hat.released) {
		platform_boom_trajectory_controller_set_enabled(platform, false);
		platform_boom_trajectory_controller_adjust_target(platform,
				(struct v2){ 0, 0 },
				TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT);
		platform_boom_set_raw(platform, 0, 0);
	}

	if (input->hat.pressed) {
		if (input->buttons.held & (1 << BTN_BIT_L1)) {
			int8_t lift = 0;
			int8_t extend = 0;

			if (input->hat.pressed & HAT_UP) {
				lift = 127;
			}
			if (input->hat.pressed & HAT_DOWN) {
				lift = -127;
			}
			if (input->hat.pressed & HAT_RIGHT) {
				extend = 127;
			}
			if (input->hat.pressed & HAT_LEFT) {
				extend = -127;
			}

			platform_boom_set_raw(platform, lift, extend);
		} else {
#define RC_BOOM_MANUAL_MOVE 500
			struct v2 dp = { 0, 0 };
			if (input->hat.pressed & HAT_UP) {
				dp.y += RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_DOWN) {
				dp.y -= RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_RIGHT) {
				dp.x += RC_BOOM_MANUAL_MOVE;
			}
			if (input->hat.pressed & HAT_LEFT) {
				dp.x -= RC_BOOM_MANUAL_MOVE;
			}

			platform_boom_trajectory_controller_adjust_target(platform, dp, TRAJECTORY_ADJUST_RELATIVE_TO_START);
			platform_boom_trajectory_controller_set_enabled(platform, true);
		}
	}

	if (input->buttons.pressed & BTN_SQUARE) {
		platform_servo_level(platform, true);
	}

	if ((input->buttons.pressed & BTN_TRIANGLE) && input->buttons.held & (1 << BTN_BIT_START)) {
		platform_boom_home(platform);
	}
}

struct planner_task rc_task = {
	.handle_input = rc_task_handle_input,
};

int main()
{
	struct platform *platform;
	uint32_t platform_status;
	queue_t control_queue;

	queue_init(&control_queue, sizeof(struct control_event), CONTROL_QUEUE_LENGTH);
	util_init(&control_queue);

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	// Start platform thread
	multicore_launch_core1(core1_main);
	platform_status = multicore_fifo_pop_blocking();
	platform = (struct platform *)multicore_fifo_pop_blocking();

	log_printf(&util_logger, "platform_status: 0x%08x", platform_status);

	// Only register comms after everything is initialised
	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	struct control_event ev;
	struct input_state *input;
	struct planner_task *current = &rc_task;

	add_alarm_in_us(100000, __timer_dummy_event_cb, NULL, false);

	while (1) {
		control_event_get_blocking(&ev);
		do {
			switch (ev.type) {
			case CONTROL_EVENT_TYPE_DUMMY:
				break;
			case CONTROL_EVENT_TYPE_INPUT:
				input = (struct input_state *)&ev.body_pad;
				input_state_print(input);

				if (input->buttons.pressed & BTN_CIRCLE) {
					int ret = platform_all_stop(platform);
					if (ret) {
						log_printf(&util_logger, "failed to send all stop!");
					}
					current = &rc_task;
				}

				if (input->buttons.pressed & BTN_SELECT) {
					util_reboot(input->buttons.held & BTN_START);
				}

				if (current && current->handle_input) {
					current->handle_input(current, platform, input);
				}

				break;
			case CONTROL_EVENT_TYPE_PID:
				handle_pid_event(platform, &ev);
				break;
			}
		} while (control_event_try_get(&ev));

		/*
		struct heading_result heading;
		get_heading(platform, &heading);
		log_printf(&util_logger, "heading @%"PRIu64": %3.2f", heading.timestamp, heading.heading / 16.0);
		*/
		//log_printf(&util_logger, "count: %d", boom_update_count());

		add_alarm_in_us(100000, __timer_dummy_event_cb, NULL, false);
	}
}
