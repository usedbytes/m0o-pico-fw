/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __PLAN_BOOM_H__
#define __PLAN_BOOM_H__

#include <stdint.h>

#include "pico/time.h"

struct boom_planner {
	struct v2 requested;
	struct v2 set;
	bool done;
	bool invalidate;
};

void boom_tick(struct boom_planner *bp, struct platform *platform, struct platform_status_report *status);
bool boom_done(struct boom_planner *bp);
void boom_set(struct boom_planner *bp, struct v2 abs_target);
void boom_invalidate(struct boom_planner *bp);

#endif /* __PLAN_BOOM_H__ */
