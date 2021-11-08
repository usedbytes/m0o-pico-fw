/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "log.h"
#include "util.h"
#include "ov7670_reg.h"

#include "camera.pio.h"

#define I2C_BUS      i2c1
#define I2C_PIN_SDA  14
#define I2C_PIN_SCL  15
#define PIN_D0       16

#define IMG_W 160
#define IMG_H 72

#define CAMERA_PIO           pio0
#define CAMERA_PIO_FRAME_SM  0
#define CAMERA_DMA_CHAN_BASE 0
#define CAMERA_MAX_N_PLANES  1

#define CMD_SNAP    (('S' << 0) | ('N' << 8) | ('A' << 16) | ('P' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))

#define FORMAT_YUYV (('Y' << 0) | ('U' << 8) | ('Y' << 16) | ('V' << 24))

struct camera_buffer {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint32_t sizes[3];
	uint8_t *data[3];
};

struct camera_config {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint dma_transfers[3];
	dma_channel_config dma_cfgs[3];
	pio_sm_config sm_cfgs[4];
};

struct camera {
	PIO pio;
	uint frame_offset;
	uint shift_byte_offset;
	int dma_channels[3];

	struct camera_config config;
};

struct camera_buffer *cam_buf;

static uint8_t format_num_planes(uint32_t format)
{
	switch (format) {
	case FORMAT_YUYV:
		return 1;
	default:
		return 0;
	}
}

static uint8_t format_bytes_per_pixel(uint32_t format, uint8_t plane)
{
	switch (format) {
	case FORMAT_YUYV:
		return 2;
	default:
		return 0;
	}
}

static enum dma_channel_transfer_size format_transfer_size(uint32_t format, uint8_t plane)
{
	switch (format) {
	case FORMAT_YUYV:
		return DMA_SIZE_32;
	default:
		return 0;
	}
}

static uint8_t __dma_transfer_size_to_bytes(enum dma_channel_transfer_size dma)
{
	switch (dma) {
	case DMA_SIZE_8:
		return 1;
	case DMA_SIZE_16:
		return 2;
	case DMA_SIZE_32:
		return 4;
	}

	return 0;
}

static uint8_t format_hsub(uint32_t format, uint8_t plane)
{
	switch (format) {
	case FORMAT_YUYV:
		return 1;
	default:
		return 0;
	}
}

static uint32_t format_plane_size(uint32_t format, uint8_t plane, uint16_t width, uint16_t height)
{
	switch (format) {
	case FORMAT_YUYV:
		return format_bytes_per_pixel(format, plane) * width * height / format_hsub(format, plane);
	default:
		return 0;
	}
}

static void camera_buffer_free(struct camera_buffer *buf)
{
	uint8_t num_planes = format_num_planes(buf->format);
	for (int i = 0; i < num_planes; i++) {
		if (buf->data[i]) {
			free(buf->data[i]);
		}
	}
	free(buf);
}

static struct camera_buffer *camera_buffer_alloc(uint32_t format, uint16_t width, uint16_t height)
{
	struct camera_buffer *buf = malloc(sizeof(*buf));
	if (!buf) {
		return NULL;
	}

	*buf = (struct camera_buffer){
		.format = format,
		.width = width,
		.height = height,
	};

	uint8_t num_planes = format_num_planes(format);
	for (int i = 0; i < num_planes; i++) {
		buf->sizes[i] = format_plane_size(format, i, width, height);
		buf->data[i] = malloc(buf->sizes[i]);
		if (!buf->data[i]) {
			goto error;
		}
	}

	return buf;

error:
	for (int i = 0; i < num_planes; i++) {
		if (buf->data[i]) {
			free(buf->data[i]);
		}
	}
	free(buf);
	return NULL;
}

static void camera_pio_init(struct camera *camera)
{
	camera->shift_byte_offset = pio_add_program(camera->pio, &camera_shift_byte_program);
	camera->frame_offset = pio_add_program(camera->pio, &camera_frame_oneplane_program);
	for (int i = 0; i < 4; i++) {
		camera_pio_init_gpios(camera->pio, i, PIN_D0);
	}
}

static void camera_pio_configure(struct camera *camera)
{
	// First disable everything
	pio_set_sm_mask_enabled(camera->pio, 0xf, false);

	// Then reset everything
	pio_restart_sm_mask(camera->pio, 0xf);

	// Drain the FIFOs
	for (int i = 0; i < 4; i++) {
		pio_sm_clear_fifos(camera->pio, i);
	}

	// TODO: Patch the pixel loop

	// Finally we can configure and enable the state machines
	uint8_t num_planes = format_num_planes(camera->config.format);
	for (int i = 0; i < num_planes; i++) {
		pio_sm_init(camera->pio, i + 1, camera->shift_byte_offset, &camera->config.sm_cfgs[i + 1]);
		pio_sm_set_enabled(camera->pio, i + 1, true);
	}

	pio_sm_init(camera->pio, CAMERA_PIO_FRAME_SM, camera->frame_offset, &camera->config.sm_cfgs[CAMERA_PIO_FRAME_SM]);
	pio_sm_set_enabled(camera->pio, CAMERA_PIO_FRAME_SM, true);
}

static void camera_configure(struct camera *camera, uint32_t format, uint16_t width, uint16_t height)
{
	camera->config.format = format;
	camera->config.width = width;
	camera->config.height = height;

	// TODO: Reconfigure ov7670

	camera->config.sm_cfgs[CAMERA_PIO_FRAME_SM] =
		camera_pio_get_frame_sm_config(camera->pio, CAMERA_PIO_FRAME_SM, camera->frame_offset, PIN_D0);

	uint8_t num_planes = format_num_planes(format);
	for (int i = 0; i < num_planes; i++) {
		enum dma_channel_transfer_size xfer_size = format_transfer_size(format, i);
		dma_channel_config c = dma_channel_get_default_config(camera->dma_channels[i]);
		channel_config_set_transfer_data_size(&c, xfer_size);
		channel_config_set_read_increment(&c, false);
		channel_config_set_write_increment(&c, true);
		channel_config_set_dreq(&c, pio_get_dreq(camera->pio, i + 1, false));

		camera->config.dma_cfgs[i] = c;

		uint8_t xfer_bytes = __dma_transfer_size_to_bytes(xfer_size);
		camera->config.dma_transfers[i] = format_plane_size(format, i, width, height) / xfer_bytes,

		camera->config.sm_cfgs[i + 1] = camera_pio_get_pixel_sm_config(camera->pio, i + 1,
							camera->shift_byte_offset, PIN_D0,
							xfer_bytes * 8);
	}

	camera_pio_configure(camera);
}

static void camera_init(struct camera *camera, PIO pio, uint base_dma_chan)
{
	// XXX: i2c, ov7670 init etc.

	*camera = (struct camera){ 0 };

	dma_claim_mask(((1 << CAMERA_MAX_N_PLANES) - 1) << base_dma_chan);
	for (int i = 0; i < CAMERA_MAX_N_PLANES; i++) {
		camera->dma_channels[i] = base_dma_chan + i;
	}
	camera->pio = pio;
	camera_pio_init(camera);
}

static int camera_do_frame(struct camera *camera, struct camera_buffer *buf)
{
	if ((camera->config.format != buf->format) ||
	    (camera->config.width != buf->width) ||
	    (camera->config.height != buf->height)) {
		return -1;
	}

	uint8_t num_planes = format_num_planes(camera->config.format);
	for (int i = 0; i < num_planes; i++) {
		dma_channel_configure(camera->dma_channels[i],
				&camera->config.dma_cfgs[i],
				buf->data[i],
				&camera->pio->rxf[i + 1],
				camera->config.dma_transfers[i],
				true);
	}

	pio_sm_put_blocking(camera->pio, CAMERA_PIO_FRAME_SM, buf->height - 1);
	pio_sm_put_blocking(camera->pio, CAMERA_PIO_FRAME_SM, buf->width - 1);

	return 0;
}

static void camera_wait_for_completion(struct camera *camera)
{
	uint8_t num_planes = format_num_planes(camera->config.format);
	for (int i = 0; i < num_planes; i++) {
		dma_channel_wait_for_finish_blocking(camera->dma_channels[i]);
	}
}

volatile bool go = false;
volatile bool done = false;

static uint32_t handle_snap(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);

const struct comm_command snap_cmd = {
	.opcode = CMD_SNAP,
	.nargs = 0,
	.resp_nargs = 0,
	.size = NULL,
	.handle = &handle_snap,
};

const struct comm_command snapget_cmd = {
	// SGET w h addr size
	.opcode = CMD_SNAPGET,
	.nargs = 0,
	.resp_nargs = 0,
	.size = &size_snapget,
	.handle = &handle_snapget,
};

static uint32_t handle_snap(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	go = true;
	done = false;

	return COMM_RSP_OK;
}

static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = 0;
	*resp_data_len_out = sizeof(struct camera_buffer);

	return COMM_RSP_OK;
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	if (done) {
		memcpy(resp_data_out, cam_buf, sizeof(*cam_buf));
	} else {
		memset(resp_data_out, 0, sizeof(*cam_buf));
	}

	return COMM_RSP_OK;
}

static int ov7670_write(uint8_t addr, uint8_t value) {
	int ret;
	static uint8_t data[2];

	data[0] = addr;
	data[1] = value;
	ret = i2c_write_blocking(I2C_BUS, OV7670_ADDR, data, 2, false);
	if (ret != 2) {
		log_printf(&util_logger, "ov7670_write 0x%02x %d: %d", addr, value, ret);
		return -1;
	}

	return 0;
}

static int ov7670_read(uint8_t addr, uint8_t *value) {
	int ret = i2c_write_blocking(I2C_BUS, OV7670_ADDR, &addr, 1, false);
	if (ret != 1) {
		log_printf(&util_logger, "ov7670_read W 0x%02x 1: %d", addr, ret);
		return -1;
	}

	ret = i2c_read_blocking(I2C_BUS, OV7670_ADDR, value, 1, false);
	if (ret != 1) {
		log_printf(&util_logger, "ov7670_read R 0x%02x 1: %d", addr, ret);
		return -2;
	}

	return 0;
}

static int ov7670_init(void) {
#if 0
	int ret;

	// Bring clock down to ~1MHz
	ret = ov7670_write(OV7670_REG_CLKRC, CLKRC_SET_SCALE(1));
	if (ret) {
		return ret;
	}

	// Output QCIF YUV422
	ret = ov7670_write(OV7670_REG_COM7, COM7_SIZE_QCIF /*| (1 << 1)*/);
	if (ret) {
		return ret;
	}

	/* Enable scaling and downsample/crop/window */
	ov7670_write(OV7670_REG_COM3, 0x0C);
	/* Stop PCLK during Hblank */
	ov7670_write(OV7670_REG_COM10, (1 << 5));
#endif

	/*
	// Preset QCIF from OV7670 IM
	ov7670_write(OV7670_REG_CLKRC,  0x02); // Note: 0x01 in the IM
	ov7670_write(OV7670_REG_COM7,   0x00);
	ov7670_write(OV7670_REG_COM3,   0x0C);
	ov7670_write(OV7670_REG_COM14,  0x11);
	ov7670_write(OV7670_REG_XSCALE, 0x3A);
	ov7670_write(OV7670_REG_YSCALE, 0x35);
	ov7670_write(OV7670_REG_SCDCW,  0x11);
	ov7670_write(OV7670_REG_SCPCLK, 0xF1);
	ov7670_write(OV7670_REG_SCPDLY, 0x52);
	*/
	// Preset QQCIF
	ov7670_write(OV7670_REG_CLKRC,  0x01);
	ov7670_write(OV7670_REG_COM7,   0x00);
	ov7670_write(OV7670_REG_COM3,   0x0C);
	ov7670_write(OV7670_REG_COM14,  0x12);
	ov7670_write(OV7670_REG_XSCALE, 0x3A);
	ov7670_write(OV7670_REG_YSCALE, 0x35);
	ov7670_write(OV7670_REG_SCDCW,  0x22);
	ov7670_write(OV7670_REG_SCPCLK, 0xF2);
	ov7670_write(OV7670_REG_SCPDLY, 0x2A);

	return 0;
}

void run_camera(void)
{
	log_printf(&util_logger, "run_camera()");

	// 125 MHz / 10 = 12.5 MHz
	// Any higher doesn't work - either the shift register isn't fast enough
	// or the PIO program isn't
	clock_gpio_init(GPIO_XCLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 10);

	i2c_init(I2C_BUS, 100000);
	gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_PIN_SDA);
	gpio_pull_up(I2C_PIN_SCL);

	int ret;
	uint8_t val = 0;

	sleep_ms(1000);

	int tries = 5;
	while (tries--) {
		ret = ov7670_read(OV7670_REG_PID, &val);
		if (ret) {
			log_printf(&util_logger, "%d: Error reading PID: %d", tries, ret);
			continue;
		}

		if (val == 0x76) {
			break;
		}

		log_printf(&util_logger, "%d: Unexpected PID: 0x%02x", tries, val);
	}

	if (tries <= 0) {
		log_printf(&util_logger, "Camera ID failed.");
		return;
	} else {
		log_printf(&util_logger, "Camera ID'd: 0x%02x", val);
	}

	ov7670_init();

	struct camera camera;
	camera_init(&camera, CAMERA_PIO, CAMERA_DMA_CHAN_BASE);

	camera_configure(&camera, FORMAT_YUYV, 80, 72);

	cam_buf = camera_buffer_alloc(FORMAT_YUYV, 80, 72);
	assert(cam_buf);

	while (1) {
		while (!go);
		go = false;

		log_printf(&util_logger, "Start frame");
		camera_do_frame(&camera, cam_buf);
		log_printf(&util_logger, "Waiting");
		camera_wait_for_completion(&camera);
		done = true;
		log_printf(&util_logger, "Frame done");
	}
}
