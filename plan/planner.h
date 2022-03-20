/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLANNER_H__
#define __PLANNER_H__

#include "input.h"
#include "platform/platform.h"

struct planner_task {
	void (*on_start)(const struct planner_task *task);
	void (*handle_input)(const struct planner_task *task, struct platform *platform, struct input_state *input);
	void (*tick)(const struct planner_task *task, struct platform *platform, struct platform_status_report *status);
};

#endif /* __PLANNER_H__ */
