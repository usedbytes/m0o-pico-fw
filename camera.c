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
#define CAMERA_MAX_N_PLANES  1

#define CMD_SNAP    (('S' << 0) | ('N' << 8) | ('A' << 16) | ('P' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))
#define CMD_CAMERA  (('C' << 0) | ('A' << 8) | ('M' << 16) | ('C' << 24))

#define FORMAT_YUYV (('Y' << 0) | ('U' << 8) | ('Y' << 16) | ('V' << 24))
#define FORMAT_RGB565 (('R' << 0) | ('G' << 8) | ('1' << 16) | ('6' << 24))

static int ov7670_write(uint8_t addr, uint8_t value);
static int ov7670_read(uint8_t addr, uint8_t *value);
static int ov7670_init(void);

static struct camera camera;

static queue_t camera_queue;

struct camera_buffer {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint32_t strides[3];
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

typedef void (*camera_frame_cb)(struct camera_buffer *buf, void *p);

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

enum camera_queue_item_type {
	CAMERA_QUEUE_ITEM_CAPTURE = 0,
	CAMERA_QUEUE_ITEM_HOST_ALLOC,
	CAMERA_QUEUE_ITEM_LOCAL_PROCESS,
};

struct camera_queue_item {
	uint8_t type;
	uint8_t pad[3];
	union {
		struct {
			struct camera_buffer *buf;
			camera_frame_cb frame_cb;
			void *cb_data;
		} capture;
		struct {
			struct camera_buffer *buf;
		} local_process;
		struct {
			uint32_t format;
		} host_alloc;
		struct {
			uint32_t pad[3];
		} body_pad;
	};
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

static void camera_buffer_free(struct camera_buffer *buf)
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

static OV7670_colorspace format_to_colorspace(uint32_t format)
{
	switch (format) {
	case FORMAT_RGB565:
		return OV7670_COLOR_RGB;
	case FORMAT_YUYV:
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
		camera->config.dma_transfers[i] = format_plane_size(format, i, width, height) / xfer_bytes,

		camera->config.sm_cfgs[i + 1] = camera_pio_get_shift_byte_sm_config(camera->pio, i + 1,
							camera->shift_byte_offset, PIN_D0,
							xfer_bytes * 8);
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
				&camera->pio->rxf[i + 1],
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
	//log_printf(&util_logger, "Host frame done");
	host_state = HOST_STATE_COMPLETE;
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
		if (cmd_in->format.format_idx == 0) {
			qitem.host_alloc.format = FORMAT_YUYV;
		} else {
			qitem.host_alloc.format = FORMAT_RGB565;
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

static int process_frame(struct camera_buffer *buf)
{
	uint32_t start = time_us_32();
	//log_printf(&util_logger, "Process start");
	uint16_t x, y;

	uint8_t min = 255;
	uint8_t max = 0;

	for (y = 23; y < buf->height; y++) {
		for (x = 3; x < (buf->width - 2) * 2; x += 4) {
			uint8_t pix = buf->data[0][buf->strides[0] * y + x];
			if (pix < min) {
				min = pix;
			}

			if (pix > max) {
				max = pix;
			}
		}
	}

	if (max - min < 30) {
		//log_printf(&util_logger, "Process end. No target");
		return -1;
	}

	min = max - 3;

	uint8_t left = 0, right = 0;

	for (y = buf->height / 2; y <= buf->height / 2; y++) {
		for (x = 3; x < (buf->width - 2) * 2; x += 4) {
			uint8_t pix = buf->data[0][buf->strides[0] * y + x];
			if (pix < min) {
				continue;
			}

			if (left == 0) {
				left = x;
			} else {
				right = x;
			}
		}
	}

	//log_printf(&util_logger, "Process end. min, max %d, %d", min, max);
	//log_printf(&util_logger, "Middle: %d", left + ((right - left) / 2));

	uint8_t mid = (left + ((right - left) / 2)) / 2;
	log_printf(&util_logger, "Process done. %d us, mid %d", time_us_32() - start, mid);
	return mid;
}

void run_camera(void)
{
	log_printf(&util_logger, "run_camera()");
	queue_init(&camera_queue, sizeof(struct camera_queue_item), 8);

	// 125 MHz / 8 = 15.625 MHz
	clock_gpio_init(GPIO_XCLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 8);

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
	const uint32_t format = FORMAT_YUYV;

	camera_init(&camera, CAMERA_PIO, CAMERA_DMA_CHAN_BASE);
	camera_configure(&camera, format, width, height);

	host_buf = camera_buffer_alloc(FORMAT_YUYV, width, height);
	assert(host_buf);
	host_state = HOST_STATE_IDLE;

	int ret;
	int pos;
	struct camera_queue_item qitem;

	struct camera_buffer *local_buf = camera_buffer_alloc(FORMAT_YUYV, width, height);
	qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
	qitem.capture.buf = local_buf;
	qitem.capture.frame_cb = local_capture_cb;
	queue_add_blocking(&camera_queue, &qitem);

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
				sleep_ms(5);
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
			break;
			}
		case CAMERA_QUEUE_ITEM_LOCAL_PROCESS: {
			struct camera_buffer *buf = qitem.local_process.buf;
			pos = process_frame(buf);
			log_printf(&util_logger, "Middle: %d", pos);
			sleep_ms(100);
			qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
			qitem.capture.buf = buf;
			qitem.capture.frame_cb = local_capture_cb;
			queue_add_blocking(&camera_queue, &qitem);
			break;
			}
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
