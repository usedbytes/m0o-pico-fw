/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLATFORM_CAMERA_H__
#define __PLATFORM_CAMERA_H__

#include "camera/camera.h"
#include "platform/platform.h"

#include "i2c_bus.h"

struct platform_camera;

struct platform_camera *platform_camera_init(struct platform *platform, struct i2c_bus *bus);

int platform_camera_do_capture(struct platform_camera *cam, struct camera_buffer *into, camera_frame_cb cb, void *cb_data);

#endif /* __PLATFORM_CAMERA_H__ */
