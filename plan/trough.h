/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __TROUGH_H__
#define __TROUGH_H__

#include "camera/camera.h"

struct planner_task *trough_get_task(struct platform *platform, struct camera_buffer *buf);

#endif /* __TROUGH_H__ */
