/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "log.h"
#include "comm.h"
#include "util.h"

#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))

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

#define PWM_SLICE_A   7
#define MOTOR_PIN_A_A 14
#define MOTOR_PIN_A_B 15
#define PWM_SLICE_B 6
#define MOTOR_PIN_B_A 12
#define MOTOR_PIN_B_B 13
#define PWM_MAX (127 + 120)
#define PWM_MIN (PWM_MAX - 127)

enum motor_id {
	MOTOR_A,
	MOTOR_B,
};

enum motor_dir {
	MOTOR_DIR_FWD,
	MOTOR_DIR_REV,
};

static void set_motor(enum motor_id motor, uint8_t value)
{
	uint slice = motor == MOTOR_A ? PWM_SLICE_A : PWM_SLICE_B;

	if (value == 0x80) {
		pwm_set_chan_level(slice, PWM_CHAN_A, 0);
		pwm_set_chan_level(slice, PWM_CHAN_B, 0);
	} else {
		if (value < 0x80) {
			pwm_set_chan_level(slice, PWM_CHAN_A, 0);
			pwm_set_chan_level(slice, PWM_CHAN_B, PWM_MIN + ((0x80 - value) / 2));
		} else {
			pwm_set_chan_level(slice, PWM_CHAN_A, PWM_MIN + ((value - 0x80) / 2));
			pwm_set_chan_level(slice, PWM_CHAN_B, 0);
		}
	}
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

	set_motor(MOTOR_A, state.ly);
	set_motor(MOTOR_B, state.ry);

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

int main()
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	util_init();

	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	gpio_set_function(MOTOR_PIN_A_A, GPIO_FUNC_PWM);
	gpio_set_function(MOTOR_PIN_A_B, GPIO_FUNC_PWM);
	pwm_set_wrap(PWM_SLICE_A, PWM_MAX + 1);
	pwm_set_chan_level(PWM_SLICE_A, PWM_CHAN_A, 0);
	pwm_set_chan_level(PWM_SLICE_A, PWM_CHAN_B, 0);
	pwm_set_enabled(PWM_SLICE_A, true);
	gpio_set_function(MOTOR_PIN_B_A, GPIO_FUNC_PWM);
	gpio_set_function(MOTOR_PIN_B_B, GPIO_FUNC_PWM);
	pwm_set_wrap(PWM_SLICE_B, PWM_MAX + 1);
	pwm_set_chan_level(PWM_SLICE_B, PWM_CHAN_A, 0);
	pwm_set_chan_level(PWM_SLICE_B, PWM_CHAN_B, 0);
	pwm_set_enabled(PWM_SLICE_B, true);

	run_camera();

	//multicore_launch_core1(core1_main);

	int i = 0;
	while (1) {
		//log_printf(&util_logger, "From core 0: %d", i++);
		sleep_ms(100);
	}
}
