/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/structs/uart.h"

#include "comm.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD   921600

enum comm_state {
	COMM_STATE_WAIT_FOR_SYNC = 0,
	COMM_STATE_START_OPCODE,
	COMM_STATE_END_OPCODE,
	COMM_STATE_START_ARGS,
	COMM_STATE_END_ARGS,
	COMM_STATE_START_DATA,
	COMM_STATE_END_DATA,
	COMM_STATE_START_RESPONSE,
	COMM_STATE_END_RESPONSE,
};

struct comm_context {
	const struct comm_command *cmd;
	const struct comm_command *cmds;
	int n_cmds;
	enum comm_state state;
	uint32_t sync_opcode;
	dma_channel_config dma_config;
	bool error;
	uint32_t data_len;
	uint32_t resp_data_len;
};

const uint comm_dma_channel = 11;
volatile uint8_t comm_buf[(sizeof(uint32_t) * (1 + COMM_MAX_NARG)) + COMM_MAX_DATA_LEN];
volatile struct comm_context ctx;

static void comm_state_machine(void);

static bool is_error(uint32_t status)
{
	return status == COMM_RSP_ERR;
}

static const struct comm_command *find_command_desc(uint32_t opcode)
{
	unsigned int i;

	for (i = 0; i < ctx.n_cmds; i++) {
		if (ctx.cmds[i].opcode == opcode) {
			return &ctx.cmds[i];
		}
	}

	return NULL;
}

static void comm_begin_sync(void);
static void comm_end_sync(void);
static void comm_begin_args(void);
static void comm_end_args(void);
static void comm_begin_data(void);
static void comm_end_data(void);
static void comm_begin_response(void);
static void comm_end_response(void);

static void comm_begin_sync(void)
{
	irq_set_enabled(UART0_IRQ, true);
}

static void comm_end_sync(void)
{
	ctx.state = COMM_STATE_END_OPCODE;
	irq_set_enabled(UART0_IRQ, false);
	irq_clear(UART0_IRQ);
	comm_state_machine();
}

static void comm_begin_opcode(void)
{
	channel_config_set_read_increment(&ctx.dma_config, false);
	channel_config_set_write_increment(&ctx.dma_config, true);
	channel_config_set_dreq(&ctx.dma_config, DREQ_UART0_RX);

	dma_channel_configure(comm_dma_channel, &ctx.dma_config,
				comm_buf, &uart0_hw->dr, sizeof(uint32_t),
				true);

	ctx.state = COMM_STATE_END_OPCODE;
}

static void comm_do_error()
{
	((volatile uint32_t *)comm_buf)[0] = COMM_RSP_ERR;
	// TODO: Just taking the first command is not correct.
	ctx.cmd = &ctx.cmds[0];
	ctx.resp_data_len = 0;
	ctx.error = true;
}

static void comm_end_opcode(void)
{
	ctx.cmd = find_command_desc(((volatile uint32_t *)comm_buf)[0]);
	if (ctx.cmd) {
		ctx.state = COMM_STATE_START_ARGS;
	} else {
		comm_do_error();
		ctx.state = COMM_STATE_START_RESPONSE;
	}

	comm_state_machine();
}

static void comm_begin_args(void)
{
	if (ctx.cmd->nargs == 0) {
		comm_end_args();
	} else {
		channel_config_set_read_increment(&ctx.dma_config, false);
		channel_config_set_write_increment(&ctx.dma_config, true);
		channel_config_set_dreq(&ctx.dma_config, DREQ_UART0_RX);

		dma_channel_configure(comm_dma_channel, &ctx.dma_config,
					comm_buf + 4, &uart0_hw->dr, ctx.cmd->nargs * 4,
					true);

		ctx.state = COMM_STATE_END_ARGS;
	}
}

static void comm_end_args(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (cmd->size) {
		uint32_t status = cmd->size((volatile uint32_t *)(comm_buf + 4), &ctx.data_len, &ctx.resp_data_len);
		if (is_error(status)) {
			comm_do_error();
			ctx.state = COMM_STATE_START_RESPONSE;
		}
	} else {
		ctx.data_len = 0;
		ctx.resp_data_len = 0;
	}

	ctx.state = COMM_STATE_START_DATA;
	comm_state_machine();
}

static void comm_begin_data(void)
{
	if (ctx.data_len == 0) {
		comm_end_data();
	} else {
		channel_config_set_read_increment(&ctx.dma_config, false);
		channel_config_set_write_increment(&ctx.dma_config, true);
		channel_config_set_dreq(&ctx.dma_config, DREQ_UART0_RX);

		dma_channel_configure(comm_dma_channel, &ctx.dma_config,
					comm_buf + ((ctx.cmd->nargs + 1) * 4), &uart0_hw->dr, ctx.data_len,
					true);

		ctx.state = COMM_STATE_END_DATA;
	}
}

static void comm_end_data(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (cmd->handle) {
		uint32_t status = cmd->handle(comm_buf + 4, comm_buf + ((ctx.cmd->nargs + 1) * 4),
						comm_buf + 4, comm_buf + ((ctx.cmd->resp_nargs + 1) * 4));
		if (is_error(status)) {
			comm_do_error();
		} else {
			((volatile uint32_t *)comm_buf)[0] = status;
		}
	} else {
		// TODO: Should we just assert(desc->handle)?
		((volatile uint32_t *)comm_buf)[0] = COMM_RSP_OK;
	}

	ctx.state = COMM_STATE_START_RESPONSE;
	comm_state_machine();
}

static void comm_begin_response(void)
{
	channel_config_set_read_increment(&ctx.dma_config, true);
	channel_config_set_write_increment(&ctx.dma_config, false);
	channel_config_set_dreq(&ctx.dma_config, DREQ_UART0_TX);

	dma_channel_configure(comm_dma_channel, &ctx.dma_config,
				&uart0_hw->dr, comm_buf,
				sizeof(uint32_t) + (sizeof(uint32_t) * ctx.cmd->resp_nargs) + ctx.resp_data_len,
				true);

	ctx.state = COMM_STATE_END_RESPONSE;
}

static void comm_end_response(void)
{
	if (ctx.error) {
		ctx.state = COMM_STATE_WAIT_FOR_SYNC;
	} else {
		ctx.state = COMM_STATE_START_OPCODE;
	}
	ctx.error = false;
	comm_state_machine();
}

static void comm_state_machine(void)
{
	switch (ctx.state) {
	case COMM_STATE_WAIT_FOR_SYNC:
		comm_begin_sync();
		break;
	case COMM_STATE_START_OPCODE:
		comm_begin_opcode();
		break;
	case COMM_STATE_END_OPCODE:
		comm_end_opcode();
		break;
	case COMM_STATE_START_ARGS:
		comm_begin_args();
		break;
	case COMM_STATE_END_ARGS:
		comm_end_args();
		break;
	case COMM_STATE_START_DATA:
		comm_begin_data();
		break;
	case COMM_STATE_END_DATA:
		comm_end_data();
		break;
	case COMM_STATE_START_RESPONSE:
		comm_begin_response();
		break;
	case COMM_STATE_END_RESPONSE:
		comm_end_response();
		break;
	}
}

static void uart_irq_handler(void)
{
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	static int idx = 0;
	volatile uint8_t *recv = comm_buf;
	const uint8_t *match = (const uint8_t *)&ctx.sync_opcode;

	while (uart_is_readable(uart0)) {
		uart_read_blocking(uart0, recv + idx, 1);

		if (recv[idx] != match[idx]) {
			// Start again
			idx = 0;
		} else {
			// Move on
			idx++;
		}
	}

	if (idx == sizeof(ctx.sync_opcode)) {
		comm_end_sync();
	}
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

static void dma_irq_handler(void)
{
	dma_hw->ints0 = 1u << comm_dma_channel;

	comm_state_machine();
}

void comm_init(const struct comm_command *cmds, int n_cmds, uint32_t sync_opcode)
{
	struct comm_command *cmd = cmds;
	int i;

	dma_channel_claim(comm_dma_channel);

	for (i = 0; i < n_cmds; i++, cmd++) {
		assert(cmd->nargs <= MAX_NARG);
		assert(cmd->resp_nargs <= MAX_NARG);
	}

	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_hw_flow(uart0, false, false);
	uart_set_irq_enables(uart0, true, false);

	memset(comm_buf, 0, sizeof(comm_buf));
	ctx.cmd = NULL;
	ctx.cmds = cmds;
	ctx.n_cmds = n_cmds;
	ctx.sync_opcode = sync_opcode;
	ctx.state = COMM_STATE_WAIT_FOR_SYNC;

	ctx.dma_config = dma_channel_get_default_config(comm_dma_channel);
	channel_config_set_read_increment(&ctx.dma_config, false);
	channel_config_set_write_increment(&ctx.dma_config, true);
	channel_config_set_dreq(&ctx.dma_config, DREQ_UART0_RX);
	channel_config_set_transfer_data_size(&ctx.dma_config, DMA_SIZE_8);
	channel_config_set_enable(&ctx.dma_config, true);

	dma_channel_set_irq0_enabled(comm_dma_channel, true);

	irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);

	comm_begin_sync();
}
