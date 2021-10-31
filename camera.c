/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "log.h"
#include "util.h"
#include "ov7670_reg.h"

static int ov7670_write(uint8_t addr, uint8_t value) {
	int ret;
	static uint8_t data[2];

	data[0] = addr;
	data[1] = value;
	ret = i2c_write_blocking(i2c0, OV7670_ADDR, data, 2, false);
	if (ret != 2) {
		log_printf(&util_logger, "ov7670_write 0x%02x %d: %d", addr, value, ret);
		return -1;
	}

	return 0;
}

static int ov7670_read(uint8_t addr, uint8_t *value) {
	int ret = i2c_write_blocking(i2c0, OV7670_ADDR, &addr, 1, false);
	if (ret != 1) {
		log_printf(&util_logger, "ov7670_read W 0x%02x 1: %d", addr, ret);
		return -1;
	}

	ret = i2c_read_blocking(i2c0, OV7670_ADDR, value, 1, false);
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

#define PIN_VSYNC 2
#define PIN_HREF  3
#define PIN_PXCLK 4
#define PIN_D0    5

#define IMG_W 163
#define IMG_H 72

uint8_t img[IMG_W * IMG_H];

void run_camera(void)
{
	// 125 MHz / 5 = 25 MHz
	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 5);

	i2c_init(i2c0, 100000);
	gpio_set_function(0, GPIO_FUNC_I2C);
	gpio_set_function(1, GPIO_FUNC_I2C);
	gpio_pull_up(0);
	gpio_pull_up(1);

	gpio_set_dir_in_masked((0xff << PIN_D0) | (0x7 << PIN_VSYNC));

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

	while (!gpio_get(PIN_VSYNC));

	int x = 0, y = 0, eof, idx;
	while (1) {
		y = 0;
		eof = 0;
		idx = 0;

		sleep_ms(1000);
		// Wait for vsync
		while (!gpio_get(PIN_VSYNC));
		while (gpio_get(PIN_VSYNC));

		while (!eof) {
			// Wait for href or vsync
			while (1) {
				uint32_t pins = gpio_get_all();
				if (pins & (1 << PIN_VSYNC)) {
					eof = 1;
					break;
				}

				if (pins & (1 << PIN_HREF)) {
					break;
				}
			}

			if (eof) {
				// vsync started
				break;
			}


			// Count pixels
			x = 0;
			idx = y * IMG_W;
			while (gpio_get(PIN_HREF)) {
				while (gpio_get(PIN_PXCLK));
				while (!gpio_get(PIN_PXCLK));
				x++;
				if (!(x & 1)) {
					uint32_t pins = gpio_get_all();
					img[idx] = (pins >> PIN_D0) & 0xff;
					idx++;
				}
				//while (gpio_get(PIN_PXCLK));
			}
			y++;
		}
		log_printf(&util_logger, "Frame %d,%d", x, y);

		/*
		int i, j;
		for (i = 0; i < IMG_H; i++) {
			for (j = 0; j < IMG_W; j++) {
				printf("%d ", img[i * IMG_W + j]);
			}
			printf("\n");
		}
		*/
	}
}
