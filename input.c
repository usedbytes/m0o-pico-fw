/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "input.h"
#include "log.h"
#include "util.h"

#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))

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

queue_t *event_queue;

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	static struct input_state state = { 0 };
	struct bt_hid_state hid_state;

	memcpy(&hid_state, data_in, sizeof(hid_state));

	//log_printf(&util_logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x",
	//		state.lx, state.ly, state.rx, state.ry, state.hat, state.buttons);

	state.axes.lx = hid_state.lx - 128;
	state.axes.ly = hid_state.ly - 128;
	state.axes.rx = hid_state.rx - 128;
	state.axes.ry = hid_state.ry - 128;

	uint8_t hat = 0;
	if (hid_state.hat <= 7) {
		hat = hat_pos_to_dirs[hid_state.hat];
	}

	state.hat.pressed = hat & ~state.hat.held;
	state.hat.released = state.hat.held & ~hat;
	state.hat.held |= state.hat.pressed;
	state.hat.held &= ~state.hat.released;

	state.buttons.pressed = hid_state.buttons & ~state.hat.held;
	state.buttons.released = state.hat.held & ~hid_state.buttons;
	state.buttons.held |= state.buttons.pressed;
	state.buttons.held &= ~state.buttons.released;

	control_event_send(CONTROL_EVENT_TYPE_INPUT, &state, sizeof(state));

	return COMM_RSP_OK;
}

const struct comm_command input_cmd = {
	.opcode = CMD_INPUT,
	.nargs = 0,
	.resp_nargs = 0,
	.size = &size_input,
	.handle = &handle_input,
};

void input_state_print(struct input_state *input)
{
	log_printf(&util_logger, "L: %d,%d R: %d,%d, Hat: p%01x/h%01x/r%01x, Buttons: p%04x/h%04x/r%04x",
			input->axes.lx, input->axes.ly, input->axes.rx, input->axes.ry,
			input->hat.pressed, input->hat.held, input->hat.released,
			input->buttons.pressed, input->buttons.held, input->buttons.released);
}
