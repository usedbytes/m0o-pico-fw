/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __APPLES_H__
#define __APPLES_H__
#include "camera/camera.h"

struct planner_task *apples_get_task(struct camera_buffer *buf);

#endif /* __APPLES_H__ */
