/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "comm.h"
#include "input.h"
#include "log.h"
#include "platform.h"
#include "util.h"

// This list is ordered to try and put the most frequent messages near the start
const struct comm_command *const cmds[] = {
	&input_cmd,
	&util_sync_cmd,
	&util_logs_cmd,
	&util_reboot_cmd,
	&util_read_cmd,
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

static void core1_main(void)
{
	int ret;
	struct platform platform;

	ret = platform_init(&platform);
	log_printf(&util_logger, "platform_init: %d", ret);

	multicore_fifo_push_blocking(platform.status);
	multicore_fifo_push_blocking((uint32_t)&platform);

	platform_run(&platform);
}

struct heading_result {
	absolute_time_t timestamp;
	int16_t heading;
};

struct heading_closure {
	struct platform *platform;
	struct heading_result *result;

	volatile bool done;
};

static void __get_heading_cb(absolute_time_t t, void *data)
{
	struct heading_closure *closure = (struct heading_closure *)data;

	closure->result->timestamp = closure->platform->heading_timestamp;
	closure->result->heading = closure->platform->heading;

	closure->done = true;
}

static void get_heading(struct platform *platform, struct heading_result *out)
{
	struct heading_closure closure = {
		.platform = platform,
		.result = out,
		.done = false,
	};

	platform_run_function(platform, __get_heading_cb, &closure);

	while (!closure.done);
}

static int8_t clamp8(int16_t value) {
        if (value > 127) {
                return 127;
        } else if (value < -128) {
                return -128;
        }

        return value;
}

int main()
{
	struct platform *platform;
	uint32_t platform_status;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	input_init();
	util_init();
	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	// Start platform thread
	multicore_launch_core1(core1_main);
	platform_status = multicore_fifo_pop_blocking();
	platform = (struct platform *)multicore_fifo_pop_blocking();

	log_printf(&util_logger, "platform_status: 0x%08x", platform_status);

	struct input_event ev;

	while (1) {
		input_get_event_blocking(&ev);
		do {
			log_printf(&util_logger, "L: %d,%d R: %d,%d, Hat: %1x, Buttons: %04x/%04x",
			           ev.lx, ev.ly, ev.rx, ev.ry, ev.hat, ev.btn_down, ev.btn_up);

			if (ev.btn_down) {
				gpio_put(PICO_DEFAULT_LED_PIN, 1);
			} else {
				gpio_put(PICO_DEFAULT_LED_PIN, 0);
			}

			int8_t linear = clamp8(-ev.ly);
			int8_t rot = clamp8(-ev.rx);
			platform_set_velocity(platform, linear, rot);
		} while (input_try_get_event(&ev));

		/*
		struct heading_result heading;
		get_heading(platform, &heading);
		log_printf(&util_logger, "heading @%"PRIu64": %3.2f", heading.timestamp, heading.heading / 16.0);
		*/
	}
}
