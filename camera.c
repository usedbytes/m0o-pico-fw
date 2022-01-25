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
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#include "camera.h"
#include "log.h"
#include "util.h"
//#include "ov7670_reg.h"
#include "ov7670.h"

#include "camera.pio.h"

#define I2C_BUS      i2c0
#define I2C_PIN_SDA  0
#define I2C_PIN_SCL  1
#define PIN_D0       16

#define CAMERA_PIO           pio0
#define CAMERA_PIO_FRAME_SM  0
#define CAMERA_DMA_CHAN_BASE 0
#define CAMERA_MAX_N_PLANES  3

#define CMD_SNAP    (('S' << 0) | ('N' << 8) | ('A' << 16) | ('P' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))
#define CMD_CAMERA  (('C' << 0) | ('A' << 8) | ('M' << 16) | ('C' << 24))

static int ov7670_write(uint8_t addr, uint8_t value);
static int ov7670_read(uint8_t addr, uint8_t *value);
static int ov7670_init(void);

static struct camera camera;

static queue_t camera_queue;

struct camera_config {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint dma_transfers[3];
	uint dma_offset[3];
	dma_channel_config dma_cfgs[3];
	pio_sm_config sm_cfgs[4];
};

struct camera {
	PIO pio;
	OV7670_host host;
	uint frame_offset;
	uint shift_byte_offset;
	int dma_channels[3];

	struct camera_config config;

	volatile struct camera_buffer *pending;
	volatile camera_frame_cb pending_cb;
	volatile void *cb_data;
};

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

static uint8_t format_num_planes(uint32_t format)
{
	switch (format) {
	case FORMAT_YUYV:
		/* Fallthrough */
	case FORMAT_RGB565:
		return 1;
	case FORMAT_YUV422:
		return 3;
	default:
		return 0;
	}
}

static uint8_t format_bytes_per_pixel(uint32_t format, uint8_t plane)
{
	switch (format) {
	case FORMAT_YUYV:
		/* Fallthrough */
	case FORMAT_RGB565:
		return 2;
	case FORMAT_YUV422:
		return 1;
	default:
		return 0;
	}
}

static uint8_t format_pixels_per_chunk(uint32_t format)
{
	switch (format) {
	case FORMAT_YUYV:
		/* Fallthrough */
	case FORMAT_RGB565:
		return 2;
	case FORMAT_YUV422:
		return 2;
	default:
		return 1;
	}
}

static enum dma_channel_transfer_size format_transfer_size(uint32_t format, uint8_t plane)
{
	switch (format) {
	case FORMAT_YUYV:
		/* Fallthrough */
	case FORMAT_RGB565:
		return DMA_SIZE_32;
	case FORMAT_YUV422:
		return plane ? DMA_SIZE_8 : DMA_SIZE_16;
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
		/* Fallthrough */
	case FORMAT_RGB565:
		return 1;
	case FORMAT_YUV422:
		return plane ? 2 : 1;
	default:
		return 0;
	}
}

static uint32_t format_stride(uint32_t format, uint8_t plane, uint16_t width)
{
	return format_bytes_per_pixel(format, plane) * width / format_hsub(format, plane);
}

static uint32_t format_plane_size(uint32_t format, uint8_t plane, uint16_t width, uint16_t height)
{
	return format_stride(format, plane, width) * height;
}

static const pio_program_t *format_get_pixel_loop(uint32_t format)
{
	switch (format) {
	case FORMAT_YUYV:
		return &pixel_loop_yuyv_program;
	case FORMAT_RGB565:
		// XXX: Should really use a 16-bit loop and swap to 2 bytes
		// per chunk instead of 4.
		return &pixel_loop_yuyv_program;
	case FORMAT_YUV422:
		return &pixel_loop_yu16_program;
	default:
		return NULL;
	}
}

void camera_buffer_free(struct camera_buffer *buf)
{
	if (buf == NULL) {
		return;
	}

	uint8_t num_planes = format_num_planes(buf->format);
	for (int i = 0; i < num_planes; i++) {
		if (buf->data[i]) {
			free(buf->data[i]);
		}
	}
	free(buf);
}

struct camera_buffer *camera_buffer_alloc(uint32_t format, uint16_t width, uint16_t height)
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
		buf->strides[i] = format_stride(format, i, width);
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
	camera->shift_byte_offset = pio_add_program(camera->pio, &camera_pio_shift_byte_program);
	camera->frame_offset = pio_add_program(camera->pio, &camera_pio_frame_program);
	for (int i = 0; i < 4; i++) {
		camera_pio_init_gpios(camera->pio, i, PIN_D0);
	}
	camera->pio->inte0 = (1 << (8 + CAMERA_PIO_FRAME_SM));
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

	// Patch the pixel loop
	const pio_program_t *pixel_loop = format_get_pixel_loop(camera->config.format);
	camera_pio_patch_pixel_loop(camera->pio, camera->frame_offset, pixel_loop);

	// Finally we can configure and enable the state machines
	uint8_t num_planes = format_num_planes(camera->config.format);
	for (int i = 0; i < num_planes; i++) {
		pio_sm_init(camera->pio, i + 1, camera->shift_byte_offset, &camera->config.sm_cfgs[i + 1]);
		pio_sm_set_enabled(camera->pio, i + 1, true);
	}

	pio_sm_init(camera->pio, CAMERA_PIO_FRAME_SM, camera->frame_offset, &camera->config.sm_cfgs[CAMERA_PIO_FRAME_SM]);
	pio_sm_set_enabled(camera->pio, CAMERA_PIO_FRAME_SM, true);
}

static OV7670_colorspace format_to_colorspace(uint32_t format)
{
	switch (format) {
	case FORMAT_RGB565:
		return OV7670_COLOR_RGB;
	case FORMAT_YUYV:
		/* Fallthrough */
	case FORMAT_YUV422:
		return OV7670_COLOR_YUV;
	}

	return 0;
}

static void camera_configure(struct camera *camera, uint32_t format, uint16_t width, uint16_t height)
{
	camera->config.format = format;
	camera->config.width = width;
	camera->config.height = height;

	OV7670_set_format(camera->host.platform, format_to_colorspace(format));
	// TODO: Handle other sizes
	OV7670_set_size(camera->host.platform, OV7670_SIZE_DIV8);

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
		camera->config.dma_offset[i] = 4 - xfer_bytes,
		camera->config.dma_transfers[i] = format_plane_size(format, i, width, height) / xfer_bytes,

		camera->config.sm_cfgs[i + 1] = camera_pio_get_shift_byte_sm_config(camera->pio, i + 1,
							camera->shift_byte_offset, PIN_D0,
							xfer_bytes * 8);
		log_printf(&util_logger, "dma_cfg %d, xfer_size %d %d bytes, transfers: %d, offset: %d",
				i, xfer_size, xfer_bytes, camera->config.dma_transfers[i], camera->config.dma_offset[i]);
	}

	camera_pio_configure(camera);
}

static void camera_isr_pio0(void)
{
	if (!camera.pending) {
		return;
	}

	if (camera.pending_cb) {
		camera.pending_cb((struct camera_buffer *)camera.pending, (void *)camera.cb_data);
	}

	pio_interrupt_clear(CAMERA_PIO, 0);
	camera.pending = NULL;
}

static void camera_init(struct camera *camera, PIO pio, uint base_dma_chan)
{
	*camera = (struct camera){ 0 };

	camera->host = (OV7670_host){
		.pins = &(OV7670_pins){
			.enable = -1,
			.reset = -1,
		},
		.platform = camera,
	};

	OV7670_begin(&camera->host, OV7670_COLOR_YUV, OV7670_SIZE_DIV8, 20.0);

	dma_claim_mask(((1 << CAMERA_MAX_N_PLANES) - 1) << base_dma_chan);
	for (int i = 0; i < CAMERA_MAX_N_PLANES; i++) {
		camera->dma_channels[i] = base_dma_chan + i;
	}
	camera->pio = pio;
	irq_set_exclusive_handler(PIO0_IRQ_0, camera_isr_pio0);
	camera_pio_init(camera);
	irq_set_enabled(PIO0_IRQ_0, true);
}

static int camera_do_frame(struct camera *camera, struct camera_buffer *buf, camera_frame_cb complete_cb, void *cb_data)
{
	if (camera->pending) {
		return -2;
	}

	if ((camera->config.format != buf->format) ||
	    (camera->config.width != buf->width) ||
	    (camera->config.height != buf->height)) {
		camera_configure(camera, buf->format, buf->width, buf->height);
	}

	uint8_t num_planes = format_num_planes(camera->config.format);
	for (int i = 0; i < num_planes; i++) {
		dma_channel_configure(camera->dma_channels[i],
				&camera->config.dma_cfgs[i],
				buf->data[i],
				((char *)&camera->pio->rxf[i + 1]) + camera->config.dma_offset[i],
				camera->config.dma_transfers[i],
				true);
	}

	uint32_t num_loops = buf->width / format_pixels_per_chunk(buf->format);

	camera->pending = buf;
	camera->pending_cb = complete_cb;
	camera->cb_data = cb_data;

	camera_pio_trigger_frame(camera->pio, num_loops, buf->height);

	return 0;
}

static void camera_wait_for_completion(struct camera *camera)
{
	camera_pio_wait_for_frame_done(camera->pio);
}

enum host_state {
	HOST_STATE_NO_BUFFER = 0,
	HOST_STATE_IDLE,
	HOST_STATE_PENDING,
	HOST_STATE_COMPLETE,
};

volatile enum host_state host_state;
struct camera_buffer *host_buf;

void host_capture_cb(struct camera_buffer *buf, void *data)
{
	log_printf(&util_logger, "Host frame done");
	host_state = HOST_STATE_COMPLETE;

	struct camera_queue_item qitem;
	qitem.type = CAMERA_QUEUE_ITEM_HOST_COMPLETE;
	queue_try_add(&camera_queue, &qitem);
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_snapget(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

const struct comm_command camera_host_comm_cmd = {
	.opcode = CMD_CAMERA,
	.nargs = 2,
	.resp_nargs = 2,
	.size = NULL,
	.handle = &handle_camera_cmd,
};

static uint32_t handle_camera_cmd(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct camera_host_cmd *cmd_in = (struct camera_host_cmd *)args_in;
	struct camera_queue_item qitem;

	switch (cmd_in->cmd) {
	case CAMERA_HOST_CMD_FORMAT:
		if (host_state == HOST_STATE_PENDING) {
			log_printf(&util_logger, "capture pending, can't free");
		}

		host_state = HOST_STATE_NO_BUFFER;
		camera_buffer_free((struct camera_buffer *)host_buf);
		host_buf = NULL;

		qitem.type = CAMERA_QUEUE_ITEM_HOST_ALLOC;
		switch (cmd_in->format.format_idx) {
		case 0:
			qitem.host_alloc.format = FORMAT_YUYV;
			break;
		case 1:
			qitem.host_alloc.format = FORMAT_RGB565;
			break;
		case 2:
			qitem.host_alloc.format = FORMAT_YUV422;
			break;
		default:
			qitem.host_alloc.format = FORMAT_RGB565;
			break;
		}
		queue_try_add(&camera_queue, &qitem);
		break;
	case CAMERA_HOST_CMD_TRIGGER:
		if (host_state == HOST_STATE_NO_BUFFER) {
			log_printf(&util_logger, "no host buffer");
		}

		host_state = HOST_STATE_PENDING;

		qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
		qitem.capture.buf = host_buf;
		qitem.capture.frame_cb = host_capture_cb;
		queue_try_add(&camera_queue, &qitem);
		break;
	default:
		return COMM_RSP_ERR;
	}

	return COMM_RSP_OK;
}

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

#endif
	return 0;
}

static bool ov7670_detect(void)
{
	uint8_t val = 0;

	int tries = 5;
	while (tries--) {
		int ret = ov7670_read(OV7670_REG_PID, &val);
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
		return false;
	} else {
		log_printf(&util_logger, "Camera ID'd: 0x%02x", val);
	}

	return true;
}

void local_capture_cb(struct camera_buffer *buf, void *data)
{
	struct camera_queue_item qitem;

	//log_printf(&util_logger, "Local frame done");

	qitem.type = CAMERA_QUEUE_ITEM_LOCAL_PROCESS;
	qitem.local_process.buf = buf;
	queue_add_blocking(&camera_queue, &qitem);
}

const uint8_t threshold = 10;

// Search for the furthest left and furthest right pixels in 'row'
// which are brighter than 'maxval - threshold', starting at 'start_idx'
static void search_row(uint8_t *row, int len, int start_idx, uint8_t maxval,
		       int *left_out, int *right_out)
{
	int i;

	int left = start_idx;
	int right = start_idx;

	// Search to the left
	for (i = start_idx; i >= 0; i--) {
		if (maxval - row[i] <= threshold) {
			left = i;
		} else {
			break;
		}
	}

	// Search to the right
	for (i = start_idx + 1; i < len; i++) {
		if (maxval - row[i] <= threshold) {
			right = i;
		} else {
			break;
		}
	}

	*left_out = left;
	*right_out = right;
}

// Returns the middle 'X' coordinate of the brightest red blob
static int find_red_blob(struct camera_buffer *buf)
{
	uint32_t start = time_us_32();
	int x, y;
	uint8_t reddest = 0;
	int max_x = 0, max_y = 0;

	// First find the reddest pixel in the image.
	// It's a YCbCr image, we only care about "redness", which is Cr
	// which is stored in buf->data[2]
	for (y = 0; y < buf->height; y++) {
		for (x = 0; x < buf->width / 2; x++) {
			uint8_t pix = buf->data[2][buf->strides[2] * y + x];
			if (pix > reddest) {
				reddest = pix;
				max_x = x;
				max_y = y;
			}
		}
	}

	log_printf(&util_logger, "reddest: %d", reddest);

	// Next, we search up and down the rows, looking for ones which are also
	// part of the blob.
	// On each row, we find the leftmost and rightmost pixel which is within
	// some threshold of the reddest value.
	// Then we take the middle of that left..right range to use as the
	// starting point for searching the next row.

	// l and r track the absolute leftmost and rightmost extremes of the
	// blob, eventually giving its widest point.
	int l = max_x;
	int r = max_x;
	int tmp_l, tmp_r;

	// Search up
	int idx = max_x;
	for (y = max_y; y >= 0; y--) {
		uint8_t *row = &buf->data[2][buf->strides[2] * y];

		// Only search this row if the starting point is actually
		// red enough
		if (reddest - row[idx] < threshold) {
			// Note: buf->width / 2, because this is a YUV422 image,
			// so the Cr plane is half as wide as the image itself
			search_row(row, buf->width / 2, idx, reddest, &tmp_l, &tmp_r);

			// Update the global leftmost/rightmost values
			l = tmp_l < l ? tmp_l : l;
			r = tmp_r > r ? tmp_r : r;

			// Calculate the starting point for the next row
			idx = tmp_l + ((tmp_r - tmp_l) / 2);
		} else {
			break;
		}
	}

	// Search down, starting with the middle X coord we've found so far
	idx = l + ((r - l) / 2);
	for (y = max_y; y < buf->height; y++) {
		uint8_t *row = &buf->data[2][buf->strides[2] * y];
		if (reddest - row[idx] < threshold) {
			search_row(row, buf->width / 2, idx, reddest, &tmp_l, &tmp_r);

			l = tmp_l < l ? tmp_l : l;
			r = tmp_r > r ? tmp_r : r;
		} else {
			break;
		}
	}

	// Finally, calculate the overall middle X coord
	int mid = l + (r - l) / 2;

	log_printf(&util_logger, "Process done. %d us, mid %d", time_us_32() - start, mid);

	return mid;
}

void camera_queue_add_blocking(struct camera_queue_item *qitem)
{
	queue_add_blocking(&camera_queue, qitem);
}

void run_camera(queue_t *pos_queue)
{
	log_printf(&util_logger, "run_camera()");
	queue_init(&camera_queue, sizeof(struct camera_queue_item), 8);

	// 125 MHz / 8 = 13.8 MHz
	clock_gpio_init(GPIO_XCLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 9);

	i2c_init(I2C_BUS, 100000);
	gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_PIN_SDA);
	gpio_pull_up(I2C_PIN_SCL);

	sleep_ms(1000);

	if (!ov7670_detect()) {
		return;
	}

	const uint16_t width = 80;
	const uint16_t height = 60;
	const uint32_t format = FORMAT_YUV422;

	camera_init(&camera, CAMERA_PIO, CAMERA_DMA_CHAN_BASE);
	camera_configure(&camera, format, width, height);

	host_buf = camera_buffer_alloc(FORMAT_YUV422, width, height);
	assert(host_buf);
	host_state = HOST_STATE_IDLE;

	int ret;
	int pos;

	//struct camera_queue_item local_capture_qitem;

	//struct camera_buffer *local_buf = camera_buffer_alloc(FORMAT_YUV422, width, height);
	//local_capture_qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
	//local_capture_qitem.capture.buf = local_buf;
	//local_capture_qitem.capture.frame_cb = local_capture_cb;
	//queue_add_blocking(&camera_queue, &qitem);

	struct camera_queue_item qitem;
	while (1) {
		queue_remove_blocking(&camera_queue, &qitem);
		switch (qitem.type) {
		case CAMERA_QUEUE_ITEM_CAPTURE:
			//log_printf(&util_logger, "Start frame %p", qitem.capture.buf);
			ret = camera_do_frame(&camera, qitem.capture.buf, qitem.capture.frame_cb, qitem.capture.cb_data);
			if (ret < 0) {
				log_printf(&util_logger, "Failed %d", ret);
				// FIXME: How to better handle re-submit?
				// Also, this could deadlock...
				sleep_ms(15);
				queue_add_blocking(&camera_queue, &qitem);
			}
			break;
		case CAMERA_QUEUE_ITEM_HOST_ALLOC: {
			char format[4];
			memcpy(format, &qitem.host_alloc.format, 4);

			host_buf = camera_buffer_alloc(qitem.host_alloc.format, width, height);
			host_state = HOST_STATE_IDLE;
			log_printf(&util_logger, "Alloc host buf %c%c%c%c, %p",
					format[0], format[1], format[2], format[3], host_buf);
			log_printf(&util_logger, "w, h: %d, %d",
					host_buf->width, host_buf->height);
			log_printf(&util_logger, "stride: %d, %d, %d",
					host_buf->strides[0], host_buf->strides[1], host_buf->strides[2]);
			log_printf(&util_logger, "size: %d, %d, %d",
					host_buf->sizes[0], host_buf->sizes[1], host_buf->sizes[2]);
			log_printf(&util_logger, "data: %d, %d, %d",
					host_buf->data[0], host_buf->data[1], host_buf->data[2]);
			break;
			}
		case CAMERA_QUEUE_ITEM_LOCAL_PROCESS: {
			struct camera_buffer *buf = qitem.local_process.buf;
			pos = find_red_blob(buf);
			queue_add_blocking(pos_queue, &pos);
			break;
			}
		case CAMERA_QUEUE_ITEM_HOST_COMPLETE:
			// TODO: Could deadlock
			//queue_add_blocking(&camera_queue, &local_capture_qitem);
			break;
		}
	}
}

void OV7670_print(char *str)
{
	log_printf(&util_logger, str);
}

int OV7670_read_register(void *platform, uint8_t reg)
{
	uint8_t value;

	ov7670_read(reg, &value);

	return value;
}

void OV7670_write_register(void *platform, uint8_t reg, uint8_t value)
{
	ov7670_write(reg, value);
}
