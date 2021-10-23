/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/watchdog.h"

#include "log.h"

#define ENTRY_MAGIC 0xb105f00d

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD   921600

#define CMD_SYNC  (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_LOGS  (('L' << 0) | ('O' << 8) | ('G' << 16) | ('S' << 24))
#define CMD_INPUT (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))
#define RSP_SYNC_APP (('R' << 0) | ('B' << 8) | ('N' << 16) | ('D' << 24))
#define RSP_SYNC_BL (('P' << 0) | ('I' << 8) | ('C' << 16) | ('O' << 24))
#define RSP_OK   (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define RSP_ERR  (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

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

struct command_desc {
	uint32_t opcode;
	uint32_t nargs;
	uint32_t resp_nargs;
	uint32_t (*size)(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
	uint32_t (*handle)(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
};

const struct command_desc cmds[] = {
	{
		.opcode = CMD_SYNC,
		.nargs = 0,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_sync,
	},
	{
		.opcode = CMD_LOGS,
		.nargs = 0,
		.resp_nargs = 1,
		.size = &size_logs,
		.handle = &handle_logs,
	},
	{
		.opcode = CMD_INPUT,
		.nargs = 0,
		.resp_nargs = 0,
		.size = &size_input,
		.handle = &handle_input,
	},
};

const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));
const uint32_t MAX_NARG = 5;
const uint32_t MAX_DATA_LEN = 1024;

static bool is_error(uint32_t status)
{
	return status == RSP_ERR;
}

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	return RSP_SYNC_APP;
}

void reboot_to_bootloader()
{
	hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
	watchdog_hw->scratch[5] = ENTRY_MAGIC;
	watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
	watchdog_reboot(0, 0, 0);
	tight_loop_contents();
}

static const struct command_desc *find_command_desc(uint32_t opcode)
{
	unsigned int i;

	for (i = 0; i < N_CMDS; i++) {
		if (cmds[i].opcode == opcode) {
			return &cmds[i];
		}
	}

	return NULL;
}

struct cmd_context {
	uint8_t *uart_buf;
	const struct command_desc *desc;
	uint32_t opcode;
	uint32_t status;
	uint32_t *args;
	uint8_t *data;
	uint32_t *resp_args;
	uint8_t *resp_data;
	uint32_t data_len;
	uint32_t resp_data_len;
};

enum state {
	STATE_WAIT_FOR_SYNC,
	STATE_READ_OPCODE,
	STATE_READ_ARGS,
	STATE_READ_DATA,
	STATE_HANDLE_DATA,
	STATE_ERROR,
};

static enum state state_wait_for_sync(struct cmd_context *ctx)
{
	int idx = 0;
	uint8_t *recv = (uint8_t *)&ctx->opcode;
	uint8_t *match = (uint8_t *)&ctx->status;

	ctx->status = CMD_SYNC;

	gpio_put(PICO_DEFAULT_LED_PIN, 1);

	while (idx < sizeof(ctx->opcode)) {
		uart_read_blocking(uart0, &recv[idx], 1);
		gpio_xor_mask((1 << PICO_DEFAULT_LED_PIN));

		if (recv[idx] != match[idx]) {
			// Start again
			idx = 0;
		} else {
			// Move on
			idx++;
		}
	}

	assert(ctx->opcode == CMD_SYNC);

	return STATE_READ_ARGS;
}

static enum state state_read_opcode(struct cmd_context *ctx)
{
	uart_read_blocking(uart0, (uint8_t *)&ctx->opcode, sizeof(ctx->opcode));

	return STATE_READ_ARGS;
}

static enum state state_read_args(struct cmd_context *ctx)
{
	const struct command_desc *desc = find_command_desc(ctx->opcode);
	if (!desc) {
		// TODO: Error handler that can do args?
		ctx->status = RSP_ERR;
		//ctx->status = ctx->opcode;
		return STATE_ERROR;
	}

	ctx->desc = desc;
	ctx->args = (uint32_t *)(ctx->uart_buf + sizeof(ctx->opcode));
	ctx->data = (uint8_t *)(ctx->args + desc->nargs);
	ctx->resp_args = ctx->args;
	ctx->resp_data = (uint8_t *)(ctx->resp_args + desc->resp_nargs);

	uart_read_blocking(uart0, (uint8_t *)ctx->args, sizeof(*ctx->args) * desc->nargs);

	return STATE_READ_DATA;
}

static enum state state_read_data(struct cmd_context *ctx)
{
	const struct command_desc *desc = ctx->desc;

	if (desc->size) {
		ctx->status = desc->size(ctx->args, &ctx->data_len, &ctx->resp_data_len);
		if (is_error(ctx->status)) {
			return STATE_ERROR;
		}
	} else {
		ctx->data_len = 0;
		ctx->resp_data_len = 0;
	}

	// TODO: Check sizes

	uart_read_blocking(uart0, (uint8_t *)ctx->data, ctx->data_len);

	return STATE_HANDLE_DATA;
}

static enum state state_handle_data(struct cmd_context *ctx)
{
	const struct command_desc *desc = ctx->desc;

	if (desc->handle) {
		ctx->status = desc->handle(ctx->args, ctx->data, ctx->resp_args, ctx->resp_data);
		if (is_error(ctx->status)) {
			return STATE_ERROR;
		}
	} else {
		// TODO: Should we just assert(desc->handle)?
		ctx->status = RSP_OK;
	}

	size_t resp_len = sizeof(ctx->status) + (sizeof(*ctx->resp_args) * desc->resp_nargs) + ctx->resp_data_len;
	memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
	uart_write_blocking(uart0, ctx->uart_buf, resp_len);

	return STATE_READ_OPCODE;
}

static enum state state_error(struct cmd_context *ctx)
{
	size_t resp_len = sizeof(ctx->status);
	memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
	uart_write_blocking(uart0, ctx->uart_buf, resp_len);

	return STATE_WAIT_FOR_SYNC;
	//return STATE_READ_OPCODE;
}

static uint32_t size_logs(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	// TODO: This is an unfortunate limitation of the interface, we don't
	// know the real response size until we drain the buffer in handle_logs
	// so we just say we'll use the whole buffer.
	// This means the transfer will be larger than it needs to be.
	*resp_data_len_out = MAX_DATA_LEN;

	return RSP_OK;
}

static uint32_t handle_logs(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	//log_printf(&logger, "logs requested");
	uint16_t drained = log_drain(&logger, resp_data_out, MAX_DATA_LEN);
	resp_args_out[0] = drained;

	return RSP_OK;
}

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return RSP_OK;
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct bt_hid_state state;

	memcpy(&state, data_in, sizeof(state));

	log_printf(&logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x\n",
			state.lx, state.ly, state.rx, state.ry, state.hat, state.buttons);

	return RSP_OK;
}

int main() {
	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_hw_flow(uart0, false, false);

	log_init(&logger);

	struct cmd_context ctx;
	uint8_t uart_buf[(sizeof(uint32_t) * (1 + MAX_NARG)) + MAX_DATA_LEN];
	ctx.uart_buf = uart_buf;
	enum state state = STATE_WAIT_FOR_SYNC;

	int i = 0;
	while (1) {
		switch (state) {
		case STATE_WAIT_FOR_SYNC:
			log_write(&logger, "waiting for sync", strlen("waiting for sync"));
			state = state_wait_for_sync(&ctx);
			//log_write(&logger, "done waiting", strlen("done waiting"));
			break;
		case STATE_READ_OPCODE:
			//log_printf(&logger, "read opcode %d", i++);
			state = state_read_opcode(&ctx);
			//log_write(&logger, "done opcode", strlen("done opcode"));
			break;
		case STATE_READ_ARGS:
			//log_write(&logger, "read args", strlen("read args"));
			state = state_read_args(&ctx);
			//log_write(&logger, "done args", strlen("done args"));
			break;
		case STATE_READ_DATA:
			//log_write(&logger, "read data", strlen("read data"));
			state = state_read_data(&ctx);
			//log_write(&logger, "done data", strlen("done data"));
			break;
		case STATE_HANDLE_DATA:
			//log_write(&logger, "handle data", strlen("handle data"));
			state = state_handle_data(&ctx);
			//log_write(&logger, "done handle", strlen("done handle"));
			break;
		case STATE_ERROR:
			//log_write(&logger, "do error", strlen("do error"));
			state = state_error(&ctx);
			//log_write(&logger, "done error", strlen("done error"));
			break;
		}
	}
}
