/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

#include "log.h"
#include "comm.h"
#include "util.h"

#define ENTRY_MAGIC 0xb105f00d

#define RSP_SYNC_APP (('R' << 0) | ('B' << 8) | ('N' << 16) | ('D' << 24))

struct log_buffer util_logger;

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_logs(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_logs(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_reboot(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_reboot(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

const struct comm_command util_sync_cmd = {
	.opcode = UTIL_CMD_SYNC,
	.nargs = 0,
	.resp_nargs = 0,
	.size = NULL,
	.handle = &handle_sync,
};

const struct comm_command util_logs_cmd = {
	.opcode = UTIL_CMD_LOGS,
	.nargs = 0,
	.resp_nargs = 1,
	.size = &size_logs,
	.handle = &handle_logs,
};

const struct comm_command util_reboot_cmd = {
	// BOOT to_bootloader
	// NO RESPONSE
	.opcode = UTIL_CMD_REBOOT,
	.nargs = 1,
	.resp_nargs = 0,
	.size = &size_reboot,
	.handle = &handle_reboot,
};
const struct comm_command util_read_cmd = {
	// READ addr len
	// OKOK [data]
	.opcode = UTIL_CMD_READ,
	.nargs = 2,
	.resp_nargs = 0,
	.size = &size_read,
	.handle = &handle_read,
};

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
	uint16_t drained = log_drain(&util_logger, resp_data_out, COMM_MAX_DATA_LEN);
	resp_args_out[0] = drained;

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

static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	uint32_t size = args_in[1];
	if (size > COMM_MAX_DATA_LEN) {
		return COMM_RSP_ERR;
	}

	// TODO: Validate address

	*data_len_out = 0;
	*resp_data_len_out = size;

	return COMM_RSP_OK;
}

static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	memcpy(resp_data_out, (void *)addr, size);

	return COMM_RSP_OK;
}


void util_init(void)
{
	log_init(&util_logger);
}
