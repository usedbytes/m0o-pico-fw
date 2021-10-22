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

#define ENTRY_MAGIC 0xb105f00d

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD   921600

#define CMD_SYNC  (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_LOGS  (('L' << 0) | ('O' << 8) | ('G' << 16) | ('S' << 24))
#define RSP_SYNC (('P' << 0) | ('I' << 8) | ('C' << 16) | ('O' << 24))
#define RSP_OK   (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define RSP_ERR  (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

struct log_buffer logger;

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_logs(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_logs(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

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
	return RSP_SYNC;
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
}

#define LOG_BUFFER_SIZE 1024

struct log_message {
	uint32_t timestamp;
	uint16_t len;
	// char string[];
} __attribute__ ((packed));

struct log_buffer {
	uint8_t buf[LOG_BUFFER_SIZE];
	mutex_t lock;
	volatile uint16_t insert_idx;
	volatile uint16_t extract_idx;
	volatile uint16_t space;
};

void log_init(struct log_buffer *log)
{
	mutex_init(&log->lock);
	memset(log->buf, 0, sizeof(log->buf));
	log->space = sizeof(log->buf);
	log->insert_idx = 0;
	log->extract_idx = 0;
}

static inline void __log_write_bytes(struct log_buffer *log, const uint8_t *data, uint16_t len)
{
	uint16_t insert_idx = log->insert_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - insert_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(log->buf + insert_idx, data, first_len);
	memcpy(log->buf, data + first_len, len - first_len);

	log->insert_idx = (insert_idx + len) % sizeof(log->buf);
	log->space -= len;
}

static inline void __log_peek_bytes(struct log_buffer *log, uint8_t *data, uint16_t len)
{
	uint16_t extract_idx = log->extract_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - extract_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(data, log->buf + extract_idx, first_len);
	memcpy(data + first_len, log->buf, len - first_len);
}

static inline void __log_read_bytes(struct log_buffer *log, uint8_t *data, uint16_t len)
{
	uint16_t extract_idx = log->extract_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - extract_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(data, log->buf + extract_idx, first_len);
	memcpy(data + first_len, log->buf, len - first_len);

	log->extract_idx = (extract_idx + len) % sizeof(log->buf);
	log->space += len;
}

static inline void log_make_space(struct log_buffer *log, uint32_t at_least)
{
	int space = log->space;
	int extract_idx = log->extract_idx;

	while (space < at_least) {
		struct log_message msg;
		__log_peek_bytes(log, (uint8_t *)&msg, sizeof(msg));

		uint16_t msg_size = sizeof(msg) + msg.len;

		extract_idx = (extract_idx + msg_size) % sizeof(log->buf);
		space += msg_size;
	}
	log->extract_idx = extract_idx;
	log->space = space;
}

void log_write(struct log_buffer *log, const char *message, uint16_t len)
{
	uint32_t space_reqd = sizeof(struct log_message) + len;

	// Will never fit, so just drop it.
	if ((space_reqd > UINT16_MAX) || space_reqd >= sizeof(log->buf)) {
		return;
	}

	mutex_enter_blocking(&log->lock);

	log_make_space(log, space_reqd);

	struct log_message msg = {
		.timestamp = time_us_32(),
		.len = len,
	};

	__log_write_bytes(log, (const uint8_t *)&msg, sizeof(msg));
	__log_write_bytes(log, (const uint8_t *)message, len);

	mutex_exit(&log->lock);
}

uint16_t log_drain(struct log_buffer *log, uint8_t *buf, uint16_t size)
{
	mutex_enter_blocking(&log->lock);

	uint16_t extract_idx = log->extract_idx;
	uint16_t insert_idx = log->insert_idx;
	uint16_t used = 0;

	while (extract_idx != insert_idx) {
		struct log_message msg;
		__log_peek_bytes(log, (uint8_t *)&msg, sizeof(msg));

		uint16_t msg_size = sizeof(msg) + msg.len;
		if (used + msg_size >= size) {
			break;
		}

		__log_read_bytes(log, buf + used, msg_size);
		used += msg_size;
		extract_idx = (extract_idx + msg_size) % sizeof(log->buf);
	}

	mutex_exit(&log->lock);

	return used;
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
	uint16_t drained = log_drain(&logger, resp_data_out, MAX_DATA_LEN);
	resp_args_out[0] = drained;

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

	while (1) {
		switch (state) {
		case STATE_WAIT_FOR_SYNC:
			log_write(&logger, "waiting for sync", strlen("waiting for sync"));
			state = state_wait_for_sync(&ctx);
			log_write(&logger, "done waiting", strlen("done waiting"));
			break;
		case STATE_READ_OPCODE:
			log_write(&logger, "read opcode", strlen("read opcode"));
			state = state_read_opcode(&ctx);
			log_write(&logger, "done opcode", strlen("done opcode"));
			break;
		case STATE_READ_ARGS:
			log_write(&logger, "read args", strlen("read args"));
			state = state_read_args(&ctx);
			log_write(&logger, "done args", strlen("done args"));
			break;
		case STATE_READ_DATA:
			log_write(&logger, "read data", strlen("read data"));
			state = state_read_data(&ctx);
			log_write(&logger, "done data", strlen("done data"));
			break;
		case STATE_HANDLE_DATA:
			log_write(&logger, "handle data", strlen("handle data"));
			state = state_handle_data(&ctx);
			log_write(&logger, "done handle", strlen("done handle"));
			break;
		case STATE_ERROR:
			log_write(&logger, "do error", strlen("do error"));
			state = state_error(&ctx);
			log_write(&logger, "done error", strlen("done error"));
			break;
		}
	}
}
