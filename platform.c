/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/sync.h"

#include "boom.h"
#include "log.h"
#include "platform.h"
#include "util.h"

#define PLATFORM_MESSAGE_QUEUE_SIZE 32
#define PLATFORM_HEADING_UPDATE_US  10000

#define BNO055_ADDR 0x28

#define CHASSIS_PIN_L_A 14
#define CHASSIS_PIN_L_B 15
#define CHASSIS_PIN_R_A 12
#define CHASSIS_PIN_R_B 13

#define I2C_MAIN_BUS      i2c0
#define I2C_MAIN_PIN_SDA  0
#define I2C_MAIN_PIN_SCL  1

#define I2C_AUX_BUS       i2c1
#define I2C_AUX_PIN_SDA   2
#define I2C_AUX_PIN_SCL   3

static struct platform_alarm_slot *__find_free_alarm_slot(struct platform *platform)
{
	int i;
	struct platform_alarm_slot *slot = NULL;

	critical_section_enter_blocking(&platform->slot_lock);
	for (i = 0; i < PLATFORM_ALARM_POOL_SIZE; i++) {
		slot = &platform->slots[i];
		if (slot->platform == NULL) {
			slot->platform = platform;
			break;
		}
	}
	critical_section_exit(&platform->slot_lock);

	return slot;
}

static void __alarm_slot_release(struct platform_alarm_slot *slot)
{
	slot->platform = NULL;
}

static int64_t __scheduled_message_cb(alarm_id_t id, void *user_data) {
	struct platform_alarm_slot *slot = (struct platform_alarm_slot *)user_data;
	if (!slot) {
		// Erk?
		return 0;
	}

	if (!queue_try_add(&slot->platform->queue, &slot->msg)) {
		log_printf(&util_logger, "scheduled message dropped");
	}
	__alarm_slot_release(slot);

	return 0;
}

static alarm_id_t platform_schedule_message(struct platform *platform, struct platform_message *msg, absolute_time_t at)
{
	struct platform_alarm_slot *slot = __find_free_alarm_slot(platform);
	if (!slot) {
		return -1;
	}

	memcpy(&slot->msg, msg, sizeof(*msg));
	alarm_id_t id = alarm_pool_add_alarm_at(platform->alarm_pool, at, __scheduled_message_cb, slot, false);
	if (id > 0) {
		return id;
	}

	__alarm_slot_release(slot);
	if (id == 0 && !queue_try_add(&platform->queue, msg)) {
		return -1;
	}

	return 0;
}

static int platform_send_message(struct platform *platform, struct platform_message *msg) {
	if (!queue_try_add(&platform->queue, msg)) {
		return -1;
	}

	return 0;
}

alarm_id_t platform_schedule_function(struct platform *platform, scheduled_func_t func, void *data, absolute_time_t at)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_RUN,
		.run = {
			.scheduled = at,
			.func = func,
			.arg = data,
		},
	};

	return platform_schedule_message(platform, &msg, at);
}

int platform_run_function(struct platform *platform, scheduled_func_t func, void *data)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_RUN,
		.run = {
			.scheduled = at_the_end_of_time,
			.func = func,
			.arg = data,
		},
	};

	return platform_send_message(platform, &msg);
}

int platform_set_velocity(struct platform *platform, int8_t linear, int8_t angular)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_VELOCITY,
		.velocity = {
			.linear = linear,
			.angular = angular,
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_update_heading(absolute_time_t scheduled, void *data)
{
	struct platform *platform = data;

	int16_t heading;
	int ret = bno055_get_heading(&platform->bno055, &heading);
	if (!ret) {
		platform->heading_timestamp = get_absolute_time();
		platform->heading = heading;

		platform_schedule_function(platform, platform_update_heading, platform, scheduled + PLATFORM_HEADING_UPDATE_US);
	} else {
		platform->status &= ~(PLATFORM_STATUS_BNO055_OK);

		log_printf(&util_logger, "platform_update_heading failed: %d", ret);

		// Schedule re-initialisation
	}
}

static void platform_boom_extend_home(absolute_time_t scheduled, void *data)
{
	const int8_t home_speed = 90;
	const uint32_t poll_us = 100000;
	const uint32_t max_extend = 2000;
	struct platform *platform = data;

	switch (platform->boom_extend_state) {
	case BOOM_EXTEND_HOME_START:
		boom_reset_count();
		boom_extend_set(home_speed);
		platform->boom_extend_state = BOOM_EXTEND_HOME_EXTENDING;
		break;
	case BOOM_EXTEND_HOME_EXTENDING:
		if (!boom_extend_at_limit()) {
			if (boom_extend_set(-home_speed) == 0) {
				platform->boom_extend_state = BOOM_EXTEND_HOME_RETRACTING;
			}
		} else if (boom_update_count() >= max_extend) {
			// Guard for if the switch is broken, so we don't plough
			// into the far end
			boom_extend_set(0);
			platform->boom_extend_state = BOOM_EXTEND_HOME_ERROR;
			log_printf(&util_logger, "boom homing failed");
		}
		break;
	case BOOM_EXTEND_HOME_RETRACTING:
		if (boom_extend_at_limit()) {
			boom_extend_set(0);
			platform->boom_extend_state = BOOM_EXTEND_HOME_STOPPED;
		}
		break;
	case BOOM_EXTEND_HOME_STOPPED:
		if (boom_extend_set(0) == 0) {
			boom_reset_count();
			platform->boom_extend_state = BOOM_EXTEND_HOME_DONE;
		}
		break;
	default:
		break;
	}

	if (platform->boom_extend_state < BOOM_EXTEND_HOME_DONE) {
		platform_schedule_function(platform, platform_boom_extend_home, platform, scheduled + poll_us);
	}
}

int platform_init(struct platform *platform /*, platform_config*/)
{
	int ret = 0;

	memset(platform, 0, sizeof(*platform));

	queue_init(&platform->queue, sizeof(struct platform_message), PLATFORM_MESSAGE_QUEUE_SIZE);

	int lock_num = spin_lock_claim_unused(true);
	critical_section_init_with_lock_num(&platform->slot_lock, lock_num);
	memset(platform->slots, 0, sizeof(platform->slots[0]) * PLATFORM_ALARM_POOL_SIZE);

	platform->alarm_pool = alarm_pool_create(PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM - 1, PLATFORM_ALARM_POOL_SIZE);
	// Note: alarm_pool_create doesn't handle allocation failure, and will
	// hard_assert if the hardware alarm is already claimed
	hard_assert(platform->alarm_pool);

        i2c_bus_init(&platform->i2c_main, I2C_MAIN_BUS, 100000);
        gpio_set_function(I2C_MAIN_PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_MAIN_PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_MAIN_PIN_SDA);
        gpio_pull_up(I2C_MAIN_PIN_SCL);

        i2c_bus_init(&platform->i2c_aux, I2C_AUX_BUS, 100000);
        gpio_set_function(I2C_AUX_PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_AUX_PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_AUX_PIN_SDA);
        gpio_pull_up(I2C_AUX_PIN_SCL);

	boom_init(&platform->i2c_aux);

	chassis_init(&platform->chassis, CHASSIS_PIN_L_A, CHASSIS_PIN_R_A);

	ret = bno055_init(&platform->bno055, &platform->i2c_main, BNO055_ADDR);
	if (!ret) {
		platform->status |= PLATFORM_STATUS_BNO055_PRESENT | PLATFORM_STATUS_BNO055_OK;
	}

	return ret;
}

static void platform_start_boom_homing(struct platform *platform)
{
	platform->boom_extend_state = BOOM_EXTEND_HOME_START;
	platform_schedule_function(platform, platform_boom_extend_home, platform, get_absolute_time() + 1000000);
}

void platform_run(struct platform *platform) {
	// Kick off heading updates
	if (platform->status & PLATFORM_STATUS_BNO055_PRESENT) {
		platform_update_heading(get_absolute_time(), platform);
	}

	platform_start_boom_homing(platform);

	while (1) {
		struct platform_message msg;

		queue_remove_blocking(&platform->queue, &msg);
		do {
			switch (msg.type) {
			case PLATFORM_MESSAGE_RUN:
				msg.run.func(msg.run.scheduled, msg.run.arg);
				break;
			case PLATFORM_MESSAGE_VELOCITY:
				chassis_set(&platform->chassis, msg.velocity.linear, msg.velocity.angular);
				break;
			}
		} while (queue_try_remove(&platform->queue, &msg));
	}
}
