/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/watchdog.h"

#include "log.h"
#include "comm.h"

#define ENTRY_MAGIC 0xb105f00d

#define CMD_SYNC     (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_LOGS     (('L' << 0) | ('O' << 8) | ('G' << 16) | ('S' << 24))
#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))
#define CMD_REBOOT   (('B' << 0) | ('O' << 8) | ('O' << 16) | ('T' << 24))
#define RSP_SYNC_APP (('R' << 0) | ('B' << 8) | ('N' << 16) | ('D' << 24))

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

struct log_buffer logger;

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_logs(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_logs(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_reboot(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_reboot(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

const struct comm_command *const cmds[] = {
	&(const struct comm_command){
		.opcode = CMD_SYNC,
		.nargs = 0,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_sync,
	},
	&(const struct comm_command){
		.opcode = CMD_LOGS,
		.nargs = 0,
		.resp_nargs = 1,
		.size = &size_logs,
		.handle = &handle_logs,
	},
	&(const struct comm_command){
		.opcode = CMD_INPUT,
		.nargs = 0,
		.resp_nargs = 0,
		.size = &size_input,
		.handle = &handle_input,
	},
	&(const struct comm_command){
		// BOOT to_bootloader
		// NO RESPONSE
		.opcode = CMD_REBOOT,
		.nargs = 1,
		.resp_nargs = 0,
		.size = &size_reboot,
		.handle = &handle_reboot,
	},
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	return RSP_SYNC_APP;
}

static uint32_t size_logs(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	// TODO: This is an unfortunate limitation of the interface, we don't
	// know the real response size until we drain the buffer in handle_logs
	// so we just say we'll use the whole buffer.
	// This means the transfer will be larger than it needs to be.
	*resp_data_len_out = COMM_MAX_DATA_LEN;

	return COMM_RSP_OK;
}

static uint32_t handle_logs(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint16_t drained = log_drain(&logger, resp_data_out, COMM_MAX_DATA_LEN);
	resp_args_out[0] = drained;

	return COMM_RSP_OK;
}

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

#define PWM_SLICE_A 1
#define PWM_SLICE_B 5
#define PWM_MAX (127)
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
			pwm_set_chan_level(slice, PWM_CHAN_B, PWM_MIN + (0x80 - value));
		} else {
			pwm_set_chan_level(slice, PWM_CHAN_A, PWM_MIN + (value - 0x80));
			pwm_set_chan_level(slice, PWM_CHAN_B, 0);
		}
	}
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct bt_hid_state state;

	memcpy(&state, data_in, sizeof(state));

	log_printf(&logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x",
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

static void do_reboot(bool to_bootloader)
{
	hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
	if (to_bootloader) {
		watchdog_hw->scratch[5] = ENTRY_MAGIC;
		watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
	} else {
		watchdog_hw->scratch[5] = 0;
		watchdog_hw->scratch[6] = 0;
	}

	// This probably isn't strictly necessary, because the watchdog should
	// take care of it.
	multicore_reset_core1();

	watchdog_reboot(0, 0, 0);
	while (1) {
		tight_loop_contents();
		asm("");
	}
}

static uint32_t size_reboot(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

static uint32_t handle_reboot(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	// Will never return
	do_reboot(args_in[0]);

	return COMM_RSP_ERR;
}

static void core1_main(void)
{
	int i = 0;
	while (1) {
		sleep_ms(1000);
		log_printf(&logger, "From core 1: %d", i++);
	}
}

int main()
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	comm_init(cmds, N_CMDS, CMD_SYNC);

	log_init(&logger);

	gpio_set_function(18, GPIO_FUNC_PWM);
	gpio_set_function(19, GPIO_FUNC_PWM);
	pwm_set_wrap(PWM_SLICE_A, PWM_MAX + 1);
	pwm_set_chan_level(PWM_SLICE_A, PWM_CHAN_A, 0);
	pwm_set_chan_level(PWM_SLICE_A, PWM_CHAN_B, 0);
	pwm_set_enabled(PWM_SLICE_A, true);
	gpio_set_function(26, GPIO_FUNC_PWM);
	gpio_set_function(27, GPIO_FUNC_PWM);
	pwm_set_wrap(PWM_SLICE_B, PWM_MAX + 1);
	pwm_set_chan_level(PWM_SLICE_B, PWM_CHAN_A, 0);
	pwm_set_chan_level(PWM_SLICE_B, PWM_CHAN_B, 0);
	pwm_set_enabled(PWM_SLICE_B, true);

	multicore_launch_core1(core1_main);

	int i = 0;
	while (1) {
		log_printf(&logger, "From core 0: %d", i++);
		sleep_ms(100);
	}
}
