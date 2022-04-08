/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdlib.h>

#include "camera/camera.h"
#include "camera/format.h"
#include "log.h"
#include "util.h"

#include "platform_camera.h"

#define PIN_D0        16
#define GPIO_XCLK     21

#define CAMERA_PIO           pio0
#define CAMERA_DMA_CHAN_BASE 0

struct platform_camera {
	struct camera camera;
	struct camera_platform_config platcfg;
	struct platform *platform;

	scheduled_func_t completion_cb;
	void *cb_data;
};

static int __i2c_write_blocking(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len)
{
	return i2c_bus_write_blocking((struct i2c_bus *)i2c_handle, addr, src, len);
}

static int __i2c_read_blocking(void *i2c_handle, uint8_t addr, uint8_t *dst, size_t len)
{
	return i2c_bus_read_blocking((struct i2c_bus *)i2c_handle, addr, dst, len);
}

int platform_camera_do_capture(struct platform_camera *cam, struct camera_buffer *into, camera_frame_cb cb, void *cb_data)
{
	//log_printf(&util_logger, "capture into %p", into);
	int ret = camera_capture_with_cb(&cam->camera, into, true, cb, cb_data);
	if (ret) {
		log_printf(&util_logger, "platform_camera_capture failed: %d", ret);

		// TODO: Need a status code to pass to the callback?
		// Just call the callback here to signal "completion"
		if (cb) {
			cb(into, cb_data);
		}
	}

	return ret;
}

struct platform_camera *platform_camera_init(struct platform *platform, struct i2c_bus *bus)
{
	struct platform_camera *cam = calloc(1, sizeof(struct platform_camera));
	if (!cam) {
		return NULL;
	}

	cam->platcfg = (struct camera_platform_config){
		.i2c_write_blocking = __i2c_write_blocking,
		.i2c_read_blocking = __i2c_read_blocking,
		.i2c_handle = bus,

		.pio = CAMERA_PIO,
		.xclk_pin = GPIO_XCLK,
		.xclk_divider = 9,
		.base_pin = PIN_D0,
		.base_dma_channel = -1,
	};

	int ret = camera_init(&cam->camera, &cam->platcfg);
	if (ret) {
		log_printf(&util_logger, "camera_init failed: %d", ret);
		free(cam);
		return NULL;
	}

	return cam;
}
