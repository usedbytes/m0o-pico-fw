/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"

#include "bno055.h"
#include "chassis.h"
#include "comm.h"
#include "log.h"
#include "util.h"

#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))

#define MOTOR_PIN_L_A 14
#define MOTOR_PIN_L_B 15
#define MOTOR_PIN_R_A 12
#define MOTOR_PIN_R_B 13

#define BTN_BIT_A       0
#define BTN_BIT_B       1
#define BTN_BIT_X       2
#define BTN_BIT_Y       3
#define BTN_BIT_L1      4
#define BTN_BIT_L2      5
#define BTN_BIT_L3      6
#define BTN_BIT_R1      7
#define BTN_BIT_R2      8
#define BTN_BIT_R3      9
#define BTN_BIT_START   10
#define BTN_BIT_SELECT  11
#define BTN_BIT_HEART   12
#define BTN_BIT_STAR    13

struct chassis chassis;
volatile bool heading_mode;

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t hat;
	uint8_t pad;
};

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

extern const struct comm_command snap_cmd;
extern const struct comm_command snapget_cmd;

// This list is ordered to try and put the most frequent messages near the start
const struct comm_command *const cmds[] = {
	&(const struct comm_command){
		.opcode = CMD_INPUT,
		.nargs = 0,
		.resp_nargs = 0,
		.size = &size_input,
		.handle = &handle_input,
	},
	&util_sync_cmd,
	&util_logs_cmd,
	&util_reboot_cmd,
	&util_read_cmd,
	&snap_cmd,
	&snapget_cmd,
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct bt_hid_state state;

	memcpy(&state, data_in, sizeof(state));

	log_printf(&util_logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x",
			state.lx, state.ly, state.rx, state.ry, state.hat, state.buttons);

	if (state.buttons) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
	} else {
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
	}

	if (state.buttons & (1 << BTN_BIT_Y)) {
		heading_mode = true;
	}

	if (state.buttons & (1 << BTN_BIT_A)) {
		heading_mode = false;
	}

	chassis_set(&chassis, state.ly - 128, state.ry - 128);

	return COMM_RSP_OK;
}

extern void run_camera(void);

static void core1_main(void)
{
	run_camera();

	// Should hopefully never reach here.
	while (1) {
		tight_loop_contents();
	}
}

static int8_t clamp8(int16_t value) {
	if (value > 127) {
		return 127;
	} else if (value < -128) {
		return -128;
	}

	return value;
}

int main()
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	util_init();

	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	chassis_init(&chassis, MOTOR_PIN_L_A, MOTOR_PIN_R_A);

	//run_camera();

	//multicore_launch_core1(core1_main);

	i2c_init(i2c0, 100 * 1000);
	gpio_set_function(0, GPIO_FUNC_I2C);
	gpio_set_function(1, GPIO_FUNC_I2C);
	gpio_pull_up(0);
	gpio_pull_up(1);

	sleep_ms(1000);

	struct bno055 bno055;
	int ret = bno055_init(&bno055, i2c0, 0x29);
	log_printf(&util_logger, "init: %d", ret);

	int16_t target_heading = 0;
	int16_t kp = -(1 << 6);
	int i = 0;
	while (1) {
		if (!heading_mode) {
			sleep_ms(300);
			continue;
		}

		sleep_ms(10);

		int16_t current_heading;
		int ret = bno055_get_heading(&bno055, &current_heading);
		if (ret != 0) {
			heading_mode = false;
			log_printf(&util_logger, "error: %d", ret);
			ret = bno055_init(&bno055, i2c0, 0x29);
			log_printf(&util_logger, "re-init: %d", ret);
			continue;
		}

		int16_t heading_diff = current_heading - target_heading;
		while (heading_diff > (180 * 16)) {
			heading_diff -= (360 * 16);
		}

		int16_t omega = (heading_diff * kp) >> 8;

		//log_printf(&util_logger, "heading_diff: %d, omega: %f\n", heading_diff, omega / 256.0);

		chassis_set(&chassis, clamp8(-omega), clamp8(omega));
	}
}
