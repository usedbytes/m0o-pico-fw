/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/util/queue.h"

#include "input.h"
#include "log.h"
#include "util.h"

#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))
#define INPUT_QUEUE_LENGTH 32

const uint8_t hat_pos_to_dirs[] = {
	[0] = HAT_UP,
	[1] = HAT_UP | HAT_RIGHT,
	[2] = HAT_RIGHT,
	[3] = HAT_RIGHT | HAT_DOWN,
	[4] = HAT_DOWN,
	[5] = HAT_DOWN | HAT_LEFT,
	[6] = HAT_LEFT,
	[7] = HAT_LEFT | HAT_UP,
};

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t hat;
	uint8_t pad;
};

queue_t input_queue;

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	static uint16_t prev_buttons = 0;
	struct bt_hid_state state;

	memcpy(&state, data_in, sizeof(state));

	//log_printf(&util_logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x",
	//		state.lx, state.ly, state.rx, state.ry, state.hat, state.buttons);

	struct input_event ev = {
		.btn_down = state.buttons & ~prev_buttons,
		.btn_up = prev_buttons & ~state.buttons,
		.lx = state.lx - 128,
		.ly = state.ly - 128,
		.rx = state.rx - 128,
		.ry = state.ry - 128,
	};

	prev_buttons = state.buttons;

	if (state.hat <= 7) {
		ev.hat = hat_pos_to_dirs[state.hat];
	}

	if (!queue_try_add(&input_queue, &ev)) {
		log_printf(&util_logger, "input event dropped");
	}

	return COMM_RSP_OK;
}

const struct comm_command input_cmd = {
	.opcode = CMD_INPUT,
	.nargs = 0,
	.resp_nargs = 0,
	.size = &size_input,
	.handle = &handle_input,
};

void input_init()
{
	queue_init(&input_queue, sizeof(struct input_event), INPUT_QUEUE_LENGTH);
}

void input_get_event_blocking(struct input_event *event)
{
	queue_remove_blocking(&input_queue, event);
}

bool input_try_get_event(struct input_event *event)
{
	return queue_try_remove(&input_queue, event);
}

void input_send_dummy_event()
{
	struct input_event ev = {
		.flags = INPUT_FLAG_DUMMY,
	};

	if (!queue_try_add(&input_queue, &ev)) {
		log_printf(&util_logger, "dummy input event dropped");
	}
}
