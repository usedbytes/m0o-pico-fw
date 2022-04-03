/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "camera/camera.h"
#include "camera/format.h"
#include "log.h"
#include "platform/platform.h"
#include "util.h"

#define CMD_CAMERA  (('C' << 0) | ('A' << 8) | ('M' << 16) | ('C' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))

#define CAMERA_HOST_CMD_TRIGGER     1
#define CAMERA_HOST_CMD_GET_REG_BUF 2
#define CAMERA_HOST_CMD_INIT        3
#define CAMERA_HOST_CMD_FORMAT      4

struct camera_host_cmd {
	uint8_t cmd;
	union {
		struct {
			uint8_t n_regs;
		} set_reg_buffer;
		struct {
			uint8_t format_idx;
		} format;
		struct {
			uint8_t pad[3];
		} pad;
	};
	uint32_t pdata;
};

enum host_state {
	HOST_STATE_NO_BUFFER = 0,
	HOST_STATE_IDLE,
	HOST_STATE_PENDING,
	HOST_STATE_COMPLETE,
};

volatile enum host_state host_state;
struct camera_buffer *host_buf;
struct platform *host_platform;

const uint16_t WIDTH = 80;
const uint16_t HEIGHT = 60;

void host_capture_cb(struct camera_buffer *buf, void *data)
{
	log_printf(&util_logger, "Host frame done");
	host_state = HOST_STATE_COMPLETE;
}

static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

const struct comm_command camera_host_comm_cmd = {
	.opcode = CMD_CAMERA,
	.nargs = 2,
	.resp_nargs = 2,
	.size = NULL,
	.handle = &handle_camera_cmd,
};

static inline void handle_camera_cmd_alloc(struct camera_host_cmd *cmd)
{
	uint32_t format;
	if (host_state == HOST_STATE_PENDING) {
		log_printf(&util_logger, "capture pending, can't free");
	}

	host_state = HOST_STATE_NO_BUFFER;
	camera_buffer_free((struct camera_buffer *)host_buf);
	host_buf = NULL;

	switch (cmd->format.format_idx) {
	case 0:
		format = FORMAT_YUYV;
		break;
	case 1:
		format = FORMAT_RGB565;
		break;
	case 2:
		format = FORMAT_YUV422;
		break;
	default:
		format = FORMAT_RGB565;
		break;
	}

	host_buf = camera_buffer_alloc(format, WIDTH, HEIGHT);
	host_state = HOST_STATE_IDLE;
	log_printf(&util_logger, "Alloc host buf %c%c%c%c, %p",
			((char *)&format)[0], ((char *)&format)[1], ((char *)&format)[2], ((char *)&format)[3], host_buf);
}

static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct camera_host_cmd *cmd_in = (struct camera_host_cmd *)args_in;

	switch (cmd_in->cmd) {
	case CAMERA_HOST_CMD_FORMAT:
		log_printf(&util_logger, "Allocate...");
		handle_camera_cmd_alloc(cmd_in);
		break;
	case CAMERA_HOST_CMD_TRIGGER:
		if (host_state == HOST_STATE_NO_BUFFER) {
			log_printf(&util_logger, "no host buffer");
		}

		host_state = HOST_STATE_PENDING;
		platform_camera_capture(host_platform, host_buf, host_capture_cb, NULL);
		break;
	default:
		return COMM_RSP_ERR;
	}

	return COMM_RSP_OK;
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);

const struct comm_command snapget_cmd = {
	// SGET w h addr size
	.opcode = CMD_SNAPGET,
	.nargs = 0,
	.resp_nargs = 0,
	.size = &size_snapget,
	.handle = &handle_snapget,
};

static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	*resp_data_len_out = sizeof(struct camera_buffer);

	return COMM_RSP_OK;
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	if (host_state == HOST_STATE_COMPLETE) {
		memcpy(resp_data_out, host_buf, sizeof(*host_buf));
	} else {
		memset(resp_data_out, 0, sizeof(*host_buf));
	}

	return COMM_RSP_OK;
}

void host_init(struct platform *platform)
{
	host_platform = platform;
}
