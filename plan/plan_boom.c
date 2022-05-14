/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <math.h>

#include "pico/stdlib.h"

#include "camera/camera.h"
#include "camera/format.h"
#include "log.h"
#include "planner.h"
#include "plan_boom.h"
#include "util.h"

#include "platform/platform.h"

void boom_tick(struct boom_planner *bp, struct platform *platform, struct platform_status_report *status)
{
	if (bp->invalidate || (bp->requested.x != bp->set.x) || (bp->requested.y != bp->set.y)) {
		bp->set = bp->requested;
		bp->invalidate = false;
		log_printf(&util_logger, "update boom target: (%3.2f, %3.2f)", bp->set.x, bp->set.y);
		platform_boom_trajectory_controller_adjust_target(platform,
			bp->set,
			TRAJECTORY_ADJUST_SET_ABSOLUTE);
		platform_boom_trajectory_controller_set_enabled(platform, true);
	} else {
		if (status->status & PLATFORM_STATUS_BOOM_TARGET_REACHED) {
			bp->done = true;
		}
	}
}

bool boom_done(struct boom_planner *bp)
{
	return bp->done;
}

void boom_set(struct boom_planner *bp, struct v2 abs_target)
{
	bp->requested = abs_target;
	log_printf(&util_logger, "boom_set: (%3.2f, %3.2f)", bp->requested.x, bp->requested.y);
	bp->done = false;
	/*
	if ((abs_target.x != bp->set.x) || (abs_target.y != bp->set.y)) {
		bp->done = false;
	}
	*/
}

void boom_invalidate(struct boom_planner *bp)
{
	bp->invalidate = true;
}
