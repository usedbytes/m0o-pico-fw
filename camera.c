/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
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

#define CMD_SNAP    (('S' << 0) | ('N' << 8) | ('A' << 16) | ('P' << 24))
#define CMD_SNAPGET (('S' << 0) | ('G' << 8) | ('E' << 16) | ('T' << 24))

uint8_t img[IMG_W * IMG_H] __attribute__ ((aligned (4)));
volatile bool go = false;
volatile bool done = false;

static uint32_t handle_snap(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

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
	.resp_nargs = 4,
	.size = NULL,
	.handle = &handle_snapget,
};

static uint32_t handle_snap(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	go = true;
	done = false;

	return COMM_RSP_OK;
}

static uint32_t handle_snapget(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	if (done) {
		resp_args_out[0] = IMG_W;
		resp_args_out[1] = IMG_H;
		resp_args_out[2] = (uint32_t)img;
		resp_args_out[3] = sizeof(img);
	} else {
		resp_args_out[0] = 0;
		resp_args_out[1] = 0;
		resp_args_out[2] = 0;
		resp_args_out[3] = 0;
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

	PIO pio = pio0;
	uint frame_sm = 0;
	uint data_sm = 1;

	int chan = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(pio, data_sm, false));

	uint offset = pio_add_program(pio, &camera_shift_byte_program);
	camera_shift_byte_program_init(pio, data_sm, offset, PIN_D0, 32);
	offset = pio_add_program(pio, &camera_frame_oneplane_program);
	camera_frame_oneplane_program_init(pio, frame_sm, offset, PIN_D0);

	while (1) {
		while (!go);
		go = false;

		dma_channel_configure(chan,
				&c,
				img,
				&pio->rxf[data_sm],
				IMG_W * IMG_H / 4,
				true);

		log_printf(&util_logger, "Start frame");
		pio_sm_put_blocking(pio, frame_sm, IMG_H - 1);
		pio_sm_put_blocking(pio, frame_sm, (IMG_W / 4) - 1);

		log_printf(&util_logger, "Waiting");
		dma_channel_wait_for_finish_blocking(chan);
		done = true;
		log_printf(&util_logger, "Frame done");
	}
}
