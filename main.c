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

static int64_t __timer_dummy_event_cb(alarm_id_t id, void *user_data) {
	control_event_send_dummy();
	return 0;
}

void reset_count_func(absolute_time_t scheduled, void *data)
{
	log_printf(&util_logger, "Reset count");
	boom_reset_count();
}

void print_count_func(absolute_time_t scheduled, void *data)
{
	int16_t count = boom_update_count();
	float mm = boom_extend_count_to_mm(count);
	log_printf(&util_logger, "count: %d, %3.2f mm", count, mm);
}

void print_degrees_func(absolute_time_t scheduled, void *data)
{
	int16_t angle;
	int ret = boom_lift_get_angle(&angle);
	float degrees = boom_lift_angle_to_degrees(angle);
	log_printf(&util_logger, "degrees:%3.2f", degrees);
}

void boom_extend_set_func(absolute_time_t scheduled, void *data)
{
	int8_t *val = data;
	boom_extend_set(*val);
}

void boom_lift_set_func(absolute_time_t scheduled, void *data)
{
	int8_t *val = data;
	boom_lift_set(*val);
}

const uint16_t y_offs = 55;
const uint16_t middle_apple_y = 180 - y_offs;
const uint16_t middle_apple_x = 100;

static void handle_input_event(struct platform *platform, struct control_event *cev)
{
	static uint8_t prev_hat = 0;
	static uint16_t btn_held = 0;
	static uint16_t pwm_val = 7000;
	const uint16_t PWM_STEP = 50;

	static int8_t extend_val;
	static int8_t lift_val;

	struct input_event *iev = (struct input_event *)&cev->body_pad;

	log_printf(&util_logger, "L: %d,%d R: %d,%d, Hat: %1x, Buttons: %04x/%04x",
			iev->lx, iev->ly, iev->rx, iev->ry, iev->hat, iev->btn_down, iev->btn_up);

	btn_held |= iev->btn_down;
	btn_held &= ~iev->btn_up;

	if (iev->btn_down) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
	} else {
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
	}

	if (iev->btn_up & (1 << BTN_BIT_SELECT)) {
		util_reboot(btn_held & (1 << BTN_BIT_START));
	}

	/*
	   int8_t linear = clamp8(-iev->ly);
	   int8_t rot = clamp8(-iev->rx);
	   platform_set_velocity(platform, linear, rot);
	   */

	if (iev->hat == 0 && prev_hat != 0) {
		extend_val = 0;
		lift_val = 0;
		platform_run_function(platform, boom_extend_set_func, &extend_val);
		platform_run_function(platform, boom_lift_set_func, &lift_val);
	}

	if (btn_held & (1 << BTN_BIT_R1)) {
		if (iev->hat & HAT_UP && !(prev_hat & HAT_UP)) {
			pwm_val += PWM_STEP;
			platform_ioe_set(platform, 1, pwm_val);
			log_printf(&util_logger, "pwm: %d", pwm_val);
			platform_run_function(platform, print_degrees_func, NULL);
		}

		if (iev->hat & HAT_DOWN && !(prev_hat & HAT_DOWN)) {
			pwm_val -= PWM_STEP;
			platform_ioe_set(platform, 1, pwm_val);
			log_printf(&util_logger, "pwm: %d", pwm_val);
			platform_run_function(platform, print_degrees_func, NULL);
		}
	} else {
		if (iev->hat & HAT_RIGHT && !(prev_hat & HAT_RIGHT)) {
			extend_val = 127;
			platform_run_function(platform, boom_extend_set_func, &extend_val);
		}

		if (iev->hat & HAT_LEFT && !(prev_hat & HAT_LEFT)) {
			extend_val = -127;
			platform_run_function(platform, boom_extend_set_func, &extend_val);
		}

		if (iev->hat & HAT_UP && !(prev_hat & HAT_UP)) {
			lift_val = 127;
			platform_run_function(platform, boom_lift_set_func, &lift_val);
		}

		if (iev->hat & HAT_DOWN && !(prev_hat & HAT_DOWN)) {
			lift_val = -127;
			platform_run_function(platform, boom_lift_set_func, &lift_val);
		}
	}

	if (iev->btn_down & (1 << BTN_BIT_X)) {
		platform_boom_home(platform);
	}

	if (iev->btn_down & (1 << BTN_BIT_Y)) {
		//platform_boom_lift_controller_set_enabled(platform, true);
		//platform_boom_extend_controller_set_enabled(platform, true);
		//platform_boom_target_controller_set_enabled(platform, true);
		platform_servo_level(platform, true);
	}

	if (iev->btn_down & (1 << BTN_BIT_A)) {
		//platform_boom_lift_controller_set_enabled(platform, false);
		//platform_boom_extend_controller_set_enabled(platform, false);
		platform_boom_target_controller_set_enabled(platform, false);
		platform_servo_level(platform, false);
	}

	/*
	   if (iev->btn_down & (1 << BTN_BIT_R1)) {
//platform_boom_lift_controller_set(platform, 60);
platform_boom_target_controller_set(platform, 50, 80);
platform_boom_target_controller_set_enabled(platform, true);
}
*/

if (iev->btn_down & (1 << BTN_BIT_L1)) {
	//platform_boom_lift_controller_set(platform, 0);
	platform_boom_target_controller_set(platform, middle_apple_x, middle_apple_y);
	platform_boom_target_controller_set_enabled(platform, true);
	platform_servo_level(platform, true);
}

if (iev->btn_down & (1 << BTN_BIT_L2)) {
	//platform_boom_extend_controller_set(platform, 60);
	platform_boom_target_controller_set(platform, 20, middle_apple_y);
	platform_boom_target_controller_set_enabled(platform, true);
	platform_servo_level(platform, true);
}

prev_hat = iev->hat;
}

static void handle_pid_event(struct platform *platform, struct control_event *cev)
{
	struct pid_coeffs_event *ev = (struct pid_coeffs_event *)&cev->body_pad;

	log_printf(&util_logger, "PID set: %d, %.5f, %.5f, %.5f", ev->id, ev->kp, ev->ki, ev->kd);

	platform_set_pid_coeffs(platform, ev->id, ev->kp, ev->ki, ev->kd);
}

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

	add_alarm_in_us(100000, __timer_dummy_event_cb, NULL, false);

	while (1) {
		control_event_get_blocking(&ev);
		do {
			switch (ev.type) {
			case CONTROL_EVENT_TYPE_DUMMY:
				break;
			case CONTROL_EVENT_TYPE_INPUT:
				handle_input_event(platform, &ev);
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
