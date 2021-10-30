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

#define COMM_BUF_OPCODE(_buf)       ((uint32_t *)((uint8_t *)(_buf)))
#define COMM_BUF_ARGS(_buf)         ((uint32_t *)((uint8_t *)(_buf) + sizeof(uint32_t)))
#define COMM_BUF_BODY(_buf, _nargs) ((uint8_t *)(_buf) + (sizeof(uint32_t) * ((_nargs) + 1)))

struct comm_context {
	volatile uint8_t comm_buf[(sizeof(uint32_t) * (1 + COMM_MAX_NARG)) + COMM_MAX_DATA_LEN];
	const struct comm_command *cmds;
	int n_cmds;
	uint32_t sync_opcode;

	dma_channel_config rx_config;
	dma_channel_config tx_config;

	const struct comm_command *cmd;
	void (*volatile dma_cb)(void);
	uint32_t data_len;
	uint32_t resp_data_len;
	bool error;
};

const uint comm_dma_channel = 11;
struct comm_context ctx;

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

static void comm_handle_error(uint32_t status);
static void comm_begin_sync(void);
static void comm_end_sync(void);
static void comm_begin_opcode(void);
static void comm_end_opcode(void);
static void comm_begin_args(void);
static void comm_end_args(void);
static void comm_begin_data(void);
static void comm_end_data(void);
static void comm_begin_response(void);
static void comm_end_response(void);

static void comm_handle_error(uint32_t status)
{
	*COMM_BUF_OPCODE(ctx.comm_buf) = status;
	ctx.error = true;

	ctx.dma_cb = comm_end_response;
	dma_channel_configure(comm_dma_channel,
			      &ctx.tx_config,
			      &uart0_hw->dr,
			      COMM_BUF_OPCODE(ctx.comm_buf),
			      sizeof(uint32_t),
			      true);

}

static void comm_begin_sync(void)
{
	irq_set_enabled(UART0_IRQ, true);
}

static void comm_end_sync(void)
{
	irq_set_enabled(UART0_IRQ, false);
	irq_clear(UART0_IRQ);
	comm_end_opcode();
}

static void comm_begin_opcode(void)
{
	ctx.dma_cb = comm_end_opcode;
	dma_channel_configure(comm_dma_channel,
			      &ctx.rx_config,
			      COMM_BUF_OPCODE(ctx.comm_buf),
			      &uart0_hw->dr,
			      sizeof(uint32_t),
			      true);
}

static void comm_end_opcode(void)
{
	ctx.cmd = find_command_desc(*COMM_BUF_OPCODE(ctx.comm_buf));
	if (!ctx.cmd) {
		comm_handle_error(COMM_RSP_ERR);
		return;
	}

	comm_begin_args();
}

static void comm_begin_args(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (cmd->nargs == 0) {
		comm_end_args();
	} else {
		ctx.dma_cb = comm_end_args;
		dma_channel_configure(comm_dma_channel,
				      &ctx.rx_config,
				      COMM_BUF_ARGS(ctx.comm_buf),
				      &uart0_hw->dr,
				      ctx.cmd->nargs * 4,
				      true);
	}
}

static void comm_end_args(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (cmd->size) {
		uint32_t status = cmd->size(COMM_BUF_ARGS(ctx.comm_buf),
					    &ctx.data_len,
					    &ctx.resp_data_len);
		if (is_error(status)) {
			comm_handle_error(status);
			return;
		}
	} else {
		ctx.data_len = 0;
		ctx.resp_data_len = 0;
	}

	comm_begin_data();
}

static void comm_begin_data(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (ctx.data_len == 0) {
		comm_end_data();
	} else {
		ctx.dma_cb = comm_end_data;
		dma_channel_configure(comm_dma_channel,
				      &ctx.rx_config,
				      COMM_BUF_BODY(ctx.comm_buf, cmd->nargs),
				      &uart0_hw->dr,
				      ctx.data_len,
				      true);
	}
}

static void comm_end_data(void)
{
	const struct comm_command *cmd = ctx.cmd;

	if (cmd->handle) {
		uint32_t status = cmd->handle(COMM_BUF_ARGS(ctx.comm_buf),
					      COMM_BUF_BODY(ctx.comm_buf, cmd->nargs),
					      COMM_BUF_ARGS(ctx.comm_buf),
					      COMM_BUF_BODY(ctx.comm_buf, cmd->resp_nargs));
		if (is_error(status)) {
			comm_handle_error(status);
			return;
		}

		*COMM_BUF_OPCODE(ctx.comm_buf) = status;
	} else {
		// TODO: Should we just assert(desc->handle)?
		*COMM_BUF_OPCODE(ctx.comm_buf) = COMM_RSP_OK;
	}

	comm_begin_response();
}

static void comm_begin_response(void)
{
	const struct comm_command *cmd = ctx.cmd;

	ctx.dma_cb = comm_end_response;
	dma_channel_configure(comm_dma_channel,
			      &ctx.tx_config,
			      &uart0_hw->dr,
			      COMM_BUF_OPCODE(ctx.comm_buf),
			      sizeof(uint32_t) + (sizeof(uint32_t) * cmd->resp_nargs) + ctx.resp_data_len,
			      true);
}

static void comm_end_response(void)
{
	if (ctx.error) {
		ctx.error = false;
		comm_begin_sync();
	} else {
		comm_begin_opcode();
	}
}

static void uart_irq_handler(void)
{
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	static int idx = 0;
	uint8_t *recv = (uint8_t *)COMM_BUF_OPCODE(ctx.comm_buf);
	const uint8_t *match = (const uint8_t *)&ctx.sync_opcode;

	while (uart_is_readable(uart0) && (idx < sizeof(ctx.sync_opcode))) {
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
		idx = 0;
		comm_end_sync();
	}
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

static void dma_irq_handler(void)
{
	dma_hw->ints0 = 1u << comm_dma_channel;

	assert(ctx.dma_cb);

	ctx.dma_cb();
}

void comm_init(const struct comm_command *cmds, int n_cmds, uint32_t sync_opcode)
{
	const struct comm_command *cmd = cmds;
	int i;

	for (i = 0; i < n_cmds; i++, cmd++) {
		assert(cmd->nargs <= MAX_NARG);
		assert(cmd->resp_nargs <= MAX_NARG);
	}

	memset((void *)ctx.comm_buf, 0, sizeof(ctx.comm_buf));
	ctx.cmd = NULL;
	ctx.cmds = cmds;
	ctx.n_cmds = n_cmds;
	ctx.sync_opcode = sync_opcode;

	dma_channel_claim(comm_dma_channel);
	dma_channel_set_irq0_enabled(comm_dma_channel, true);

	ctx.rx_config = dma_channel_get_default_config(comm_dma_channel);
	channel_config_set_read_increment(&ctx.rx_config, false);
	channel_config_set_write_increment(&ctx.rx_config, true);
	channel_config_set_dreq(&ctx.rx_config, DREQ_UART0_RX);
	channel_config_set_transfer_data_size(&ctx.rx_config, DMA_SIZE_8);
	channel_config_set_enable(&ctx.rx_config, true);

	ctx.tx_config = dma_channel_get_default_config(comm_dma_channel);
	channel_config_set_read_increment(&ctx.tx_config, true);
	channel_config_set_write_increment(&ctx.tx_config, false);
	channel_config_set_dreq(&ctx.tx_config, DREQ_UART0_TX);
	channel_config_set_transfer_data_size(&ctx.tx_config, DMA_SIZE_8);
	channel_config_set_enable(&ctx.tx_config, true);

	irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);

	uart_init(uart0, UART_BAUD);
	uart_set_hw_flow(uart0, false, false);
	uart_set_irq_enables(uart0, true, false);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

	comm_begin_sync();
}
