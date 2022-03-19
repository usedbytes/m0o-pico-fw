/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <inttypes.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/float.h"
#include "pico/sync.h"

#include "boom.h"
#include "controller.h"
#include "kinematics.h"
#include "log.h"
#include "platform.h"
#include "util.h"

#define PLATFORM_MESSAGE_QUEUE_SIZE 32
#define PLATFORM_HEADING_UPDATE_US  10000

#define BOOM_LIFT_CONTROLLER_TICK 10000
#define BOOM_EXTEND_CONTROLLER_TICK 5000
#define BOOM_SERVO_CONTROLLER_TICK 20000

#define BOOM_POS_TARGET_DELTA 2
#define BOOM_POS_TARGET_MAX_SEGMENT 20
#define BOOM_TARGET_CONTROLLER_TICK 20000

#define BOOM_TRAJECTORY_CONTROLLER_TICK 50000
#define MAX_TRAJECTORY_STEP 10

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

static inline bool __controllers_are_enabled(struct platform *platform, uint32_t controller_mask)
{
	return (platform->controllers_enabled & controller_mask) == controller_mask;
}

static inline void __controllers_set_enabled(struct platform *platform, uint32_t controller_mask)
{
	platform->controllers_enabled |= controller_mask;
}

static inline void __controllers_set_disabled(struct platform *platform, uint32_t controller_mask)
{
	platform->controllers_enabled &= ~controller_mask;
}

static int16_t get_servo_val(int16_t angle);

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
		log_printf(&util_logger, "No alarm slot!");
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
		log_printf(&util_logger, "platform message dropped");
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

static void __platform_boom_home_run(absolute_time_t scheduled, void *data)
{
	const int8_t home_speed = 90;
	const uint32_t poll_us = 100000;
	const uint32_t max_lift = 20;
	const uint32_t max_extend = 2000;
	struct platform *platform = data;
	int ret;

	switch (platform->boom_home_state) {
	case BOOM_HOME_START:
		boom_reset_count();
		boom_extend_set(home_speed);
		platform->boom_home_state = BOOM_HOME_EXTENDING;
		break;
	case BOOM_HOME_EXTENDING:
		if (!boom_extend_at_limit()) {
			if (boom_extend_set(-home_speed) == 0) {
				platform->boom_home_state = BOOM_HOME_RETRACTING;
			}
		} else if (boom_update_count() >= max_extend) {
			// Guard for if the switch is broken, so we don't plough
			// into the far end
			boom_extend_set(0);
			platform->boom_home_state = BOOM_HOME_ERROR;
			log_printf(&util_logger, "boom homing failed - extend timeout");
		}
		break;
	case BOOM_HOME_RETRACTING:
		if (boom_extend_at_limit()) {
			boom_extend_set(0);
			platform->boom_home_state = BOOM_HOME_RETRACT_STOPPED;
		}
		break;
	case BOOM_HOME_RETRACT_STOPPED:
		if (boom_extend_set(0) == 0) {
			boom_reset_count();

			ret = boom_lift_reset_angle();
			if (ret != 0) {
				platform->boom_home_state = BOOM_HOME_ERROR;
				break;
			}

			boom_lift_set(home_speed);
			platform->boom_home_state = BOOM_HOME_RAISING;
		}
		break;
	case BOOM_HOME_RAISING:
		if (!boom_lift_at_limit()) {
			if (boom_lift_set(-home_speed) == 0) {
				platform->boom_home_state = BOOM_HOME_LOWERING;
			}
		} else {
			int16_t angle = 0;
			ret = boom_lift_get_angle(&angle);
			if (ret != 0) {
				platform->boom_home_state = BOOM_HOME_ERROR;
				break;
			}

			// Guard for if the switch is broken, so we don't plough
			// into the far end
			if (angle >= max_lift) {
				boom_lift_set(0);
				platform->boom_home_state = BOOM_HOME_ERROR;
				log_printf(&util_logger, "boom homing failed - raise timeout");
				break;
			}
		}
		break;
	case BOOM_HOME_LOWERING:
		if (boom_lift_at_limit()) {
			boom_lift_set(0);
			ret = boom_lift_reset_angle();
			if (ret != 0) {
				platform->boom_home_state = BOOM_HOME_ERROR;
				break;
			}
			platform->boom_home_state = BOOM_HOME_DONE;
			platform->status |= PLATFORM_STATUS_BOOM_HOMED;
		}
		break;
	default:
		break;
	}

	if (platform->boom_home_state < BOOM_HOME_DONE) {
		platform_schedule_function(platform, __platform_boom_home_run, platform, scheduled + poll_us);
	} else {
		boom_extend_set(0);
		boom_lift_set(0);
	}
}

static inline int __i2c_write_blocking(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len)
{
	return i2c_bus_write_blocking((struct i2c_bus *)i2c_handle, addr, src, len);
}

static inline int __i2c_read_blocking(void *i2c_handle, uint8_t addr, uint8_t *dst, size_t len)
{
	return i2c_bus_read_blocking((struct i2c_bus *)i2c_handle, addr, dst, len);
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

	i2c_bus_init(&platform->i2c_aux, I2C_AUX_BUS, 500000);
	gpio_set_function(I2C_AUX_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_AUX_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_AUX_PIN_SDA);
	gpio_pull_up(I2C_AUX_PIN_SCL);

	boom_init(&platform->i2c_aux);

	ret = ioe_init(&platform->ioe, __i2c_write_blocking, __i2c_read_blocking, &platform->i2c_aux);
	if (!ret) {
		platform->status |= PLATFORM_STATUS_IOE_PRESENT | PLATFORM_STATUS_IOE_OK;

		ret = ioe_set_pwm_period(&platform->ioe, 60000);
		ret |= ioe_set_pwm_divider(&platform->ioe, IOE_PWM_DIVIDER_8);
		ret |= ioe_set_pin_mode(&platform->ioe, 1, IOE_PIN_MODE_PWM);
		if (ret) {
			platform->status &= ~PLATFORM_STATUS_IOE_OK;
		}
	}

	chassis_init(&platform->chassis, CHASSIS_PIN_L_A, CHASSIS_PIN_R_A);

	ret = bno055_init(&platform->bno055, &platform->i2c_main, BNO055_ADDR);
	if (!ret) {
		platform->status |= PLATFORM_STATUS_BNO055_PRESENT | PLATFORM_STATUS_BNO055_OK;
	}

	struct fcontroller *c = &platform->boom_lift_controller;
	c->out_min = -127;
	c->out_max = 128;
	fcontroller_set_tunings(c, 0.9, 0.004, 0, BOOM_LIFT_CONTROLLER_TICK);

	c = &platform->boom_extend_pos_controller;
	c->out_min = -127;
	c->out_max = 128;
	fcontroller_set_tunings(c, 0.25, 0.002, 0, BOOM_EXTEND_CONTROLLER_TICK);

	return ret;
}

static void __platform_boom_lift_controller_set(struct platform *platform, int16_t angle)
{
	struct fcontroller *c = &platform->boom_lift_controller;
	fcontroller_set(c, angle);
}

int platform_boom_lift_controller_set(struct platform *platform, float degrees)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_SET,
		.boom_set = {
			.angle = boom_lift_degrees_to_angle(degrees),
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_lift_controller_run(absolute_time_t scheduled, void *data);

static void __platform_boom_lift_controller_set_enabled(struct platform *platform, bool enabled)
{
	if (enabled) {
		if (!(platform->status & PLATFORM_STATUS_BOOM_HOMED)) {
			log_printf(&util_logger, "must home before enabling controller");
			return;
		}

		if (__controllers_are_enabled(platform, CONTROLLER_BOOM_LIFT)) {
			log_printf(&util_logger, "already enabled");
			return;
		}

		boom_lift_set(0);
		int16_t current;
		int ret = boom_lift_get_angle(&current);
		if (ret) {
			log_printf(&util_logger, "error getting angle");
			return;
		}

		struct fcontroller *c = &platform->boom_lift_controller;
		fcontroller_set(c, current);
		fcontroller_reinit(c, current, 0);

		__controllers_set_enabled(platform, CONTROLLER_BOOM_LIFT);
		platform_schedule_function(platform, platform_boom_lift_controller_run, platform, get_absolute_time());
	} else {
		__controllers_set_disabled(platform, CONTROLLER_BOOM_LIFT);
		boom_lift_set(0);
		log_printf(&util_logger, "boom_lift_controller disabled");
	}
}

int platform_boom_lift_controller_set_enabled(struct platform *platform, bool enabled)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_SET_ENABLED,
		.boom_enable = {
			.enabled = enabled,
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_lift_controller_run(absolute_time_t scheduled, void *data)
{
	struct platform *platform = data;
	if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_LIFT)) {
		return;
	}

	int16_t current;
	int ret = boom_lift_get_angle(&current);
	if (ret) {
		log_printf(&util_logger, "error getting angle");
		__platform_boom_lift_controller_set_enabled(platform, false);
	}

	struct fcontroller *c = &platform->boom_lift_controller;
	float output = fcontroller_tick(c, current);
	int8_t set = clamp8(output);
	boom_lift_set(set);

	//log_printf(&util_logger, "lift_controller: in: %d, set: %d, out: %d", current, (int)c->setpoint, set);

	platform_schedule_function(platform, platform_boom_lift_controller_run, platform, scheduled + BOOM_LIFT_CONTROLLER_TICK);
}

static void __platform_boom_extend_controller_set(struct platform *platform, int16_t count)
{
	struct fcontroller *c = &platform->boom_extend_pos_controller;
	fcontroller_set(c, count);
}

int platform_boom_extend_controller_set(struct platform *platform, float mm)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_EXTEND_SET,
		.boom_extend_set = {
			.count = boom_extend_mm_to_count(mm),
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_extend_controller_run(absolute_time_t scheduled, void *data);

static void __platform_boom_extend_controller_set_enabled(struct platform *platform, bool enabled)
{
	if (enabled) {
		if (!(platform->status & PLATFORM_STATUS_BOOM_HOMED)) {
			log_printf(&util_logger, "must home before enabling controller");
			return;
		}

		if (__controllers_are_enabled(platform, CONTROLLER_BOOM_EXTEND)) {
			log_printf(&util_logger, "already enabled");
			return;
		}

		boom_extend_set(0);
		int16_t current = boom_update_count();

		struct fcontroller *c = &platform->boom_extend_pos_controller;
		//fcontroller_set(c, current);
		fcontroller_reinit(c, current, 0);

		__controllers_set_enabled(platform, CONTROLLER_BOOM_EXTEND);
		platform_schedule_function(platform, platform_boom_extend_controller_run, platform, get_absolute_time());
	} else {
		__controllers_set_disabled(platform, CONTROLLER_BOOM_EXTEND);
		boom_extend_set(0);
		log_printf(&util_logger, "boom_extend_controller disabled");
	}
}

int platform_boom_extend_controller_set_enabled(struct platform *platform, bool enabled)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_EXTEND_SET_ENABLED,
		.boom_extend_enable = {
			.enabled = enabled,
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_extend_controller_run(absolute_time_t scheduled, void *data)
{
	struct platform *platform = data;
	if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_EXTEND)) {
		return;
	}

	int16_t current = boom_update_count();
	struct fcontroller *c = &platform->boom_extend_pos_controller;
	float output = fcontroller_tick(c, current);
	int8_t set = clamp8(output);
	boom_extend_set(set);

	//log_printf(&util_logger, "extend_controller: in: %d, set: %d, out: %d", current, (int)c->setpoint, set);

	platform_schedule_function(platform, platform_boom_extend_controller_run, platform, scheduled + BOOM_EXTEND_CONTROLLER_TICK);
}

static void __platform_all_stop(struct platform *platform);

static void __platform_boom_home(struct platform *platform)
{
	if ((platform->boom_home_state > 0) && (platform->boom_home_state < BOOM_HOME_DONE)) {
		log_printf(&util_logger, "homing in progress. can't start");
		return;
	}

	__platform_all_stop(platform);
	platform->status &= ~PLATFORM_STATUS_BOOM_HOMED;

	int16_t servo_val = get_servo_val(0);
	ioe_set_pin_mode(&platform->ioe, 1, IOE_PIN_MODE_PWM);
	ioe_set_pwm_duty(&platform->ioe, 1, servo_val);

	platform->boom_home_state = BOOM_HOME_START;
	platform_schedule_function(platform, __platform_boom_home_run, platform, get_absolute_time());
}

int platform_boom_home(struct platform *platform)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_HOME,
	};

	return platform_send_message(platform, &msg);
}

// magenc value to PWM value to keep the forks level
static int16_t get_servo_val(int16_t angle)
{
	static const int16_t fork_servo_cal[][2] = {
		{   0, 6450 },
		{  86, 5750 },
		{ 161, 5100 },
		{ 252, 4450 },
		{ 373, 3700 },
		{ 481, 3200 },
		{ 586, 2650 },
		{ 681, 2200 },
		{ 791, 1550 },
		{ 833, 1150 },
	};
	static const unsigned int n_servo_cal = sizeof(fork_servo_cal) / sizeof(fork_servo_cal[0]);
	unsigned int idx = 0;

	if (angle < 0) {
		angle = 0;
	}

	for (idx = 0; idx < n_servo_cal - 1; idx++) {
		if (fork_servo_cal[idx + 1][0] <= angle) {
			continue;
		}

		int16_t angle_offs = angle - fork_servo_cal[idx][0];
		int16_t angle_diff = fork_servo_cal[idx + 1][0] - fork_servo_cal[idx][0];
		float angle_frac = (float)angle_offs / angle_diff;

		int16_t pwm_diff = fork_servo_cal[idx + 1][1] - fork_servo_cal[idx][1];

		return fork_servo_cal[idx][1] + (int16_t)(pwm_diff * angle_frac);
	}

	return fork_servo_cal[n_servo_cal - 1][1];
}

static float clamp(float v, float min, float max)
{
	if (v > max) {
		return max;
	}

	if (v < min) {
		return min;
	}

	return v;
}

static void __platform_boom_update_position(struct platform *platform)
{
	float radians;
	float mm;

	int16_t count = boom_update_count();
	int16_t angle;
	int ret = boom_lift_get_angle(&angle);
	if (ret != 0) {
		log_printf(&util_logger, "failed to get position");
		return;
	}

	absolute_time_t now = get_absolute_time();

	mm = boom_extend_count_to_mm(count);
	radians = boom_lift_angle_to_radians(angle);

	struct v2 p = forward_kinematics(radians, mm);

	platform->boom_timestamp = now;
	platform->boom_current = p;
}

int platform_boom_update_position(struct platform *platform)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_UPDATE,
	};

	return platform_send_message(platform, &msg);
}

static void __platform_boom_trajectory_controller_set(struct platform *platform, struct v2 start, struct v2 target)
{
	platform->trajectory.start = start;
	platform->trajectory.target = target;
	platform->trajectory.mag = vec2_normalise(vec2_sub(target, start), &platform->trajectory.unit);

	log_printf(&util_logger, "trajectory set. start: %3.2f,%3.2f, target: %3.2f,%3.2f, mag: %3.2f",
			platform->trajectory.start.x, platform->trajectory.start.y,
			platform->trajectory.target.x, platform->trajectory.target.y,
			platform->trajectory.mag);
}

int platform_boom_trajectory_controller_set(struct platform *platform, struct v2 start, struct v2 target)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET,
		.trajectory = {
			.start = start,
			.target = target,
		},
	};

	return platform_send_message(platform, &msg);
}

static void __platform_boom_trajectory_controller_adjust_target(struct platform *platform, struct v2 amount, enum trajectory_adjust relative_to)
{
	switch (relative_to) {
	case TRAJECTORY_ADJUST_RELATIVE_TO_START:
		__platform_boom_trajectory_controller_set(platform,
				platform->trajectory.start,
				vec2_add(platform->trajectory.start, amount));
		break;
	case TRAJECTORY_ADJUST_RELATIVE_TO_TARGET:
		__platform_boom_trajectory_controller_set(platform,
				platform->trajectory.start,
				vec2_add(platform->trajectory.target, amount));
		break;
	case TRAJECTORY_ADJUST_RELATIVE_TO_CURRENT:
		if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_TRAJECTORY)) {
			__platform_boom_update_position(platform);
		}
		__platform_boom_trajectory_controller_set(platform,
				platform->trajectory.start,
				vec2_add(platform->boom_current, amount));
		break;
	case TRAJECTORY_ADJUST_SET_ABSOLUTE:
		__platform_boom_trajectory_controller_set(platform,
				platform->trajectory.start,
				amount);
		break;
	case TRAJECTORY_ADJUST_SET_BOTH_TO_CURRENT:
		if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_TRAJECTORY)) {
			__platform_boom_update_position(platform);
		}
		__platform_boom_trajectory_controller_set(platform,
				platform->boom_current,
				platform->boom_current);
		break;
	}
}

int platform_boom_trajectory_controller_adjust_target(struct platform *platform, struct v2 amount, enum trajectory_adjust relative_to)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_TRAJECTORY_ADJUST,
		.trajectory_adjust = {
			.amount = amount,
			.relative_to = relative_to,
		},
	};

	return platform_send_message(platform, &msg);
}

static void __platform_boom_set_raw(struct platform *platform, int8_t lift, int8_t extend)
{
	boom_lift_set(lift);
	boom_extend_set(extend);
}

int platform_boom_set_raw(struct platform *platform, int8_t lift, int8_t extend)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_SET_RAW,
		.boom_set_raw = {
			.lift = lift,
			.extend = extend,
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_trajectory_controller_run(absolute_time_t scheduled, void *data);

static void __platform_boom_trajectory_controller_set_enabled(struct platform *platform, bool enabled)
{
	if (enabled) {
		if (!(platform->status & PLATFORM_STATUS_BOOM_HOMED)) {
			log_printf(&util_logger, "must home before enabling controller");
			return;
		}

		__platform_boom_extend_controller_set_enabled(platform, true);
		__platform_boom_lift_controller_set_enabled(platform, true);

		__controllers_set_enabled(platform, CONTROLLER_BOOM_TRAJECTORY);
		platform_schedule_function(platform, platform_boom_trajectory_controller_run, platform, get_absolute_time());
	} else {
		__controllers_set_disabled(platform, CONTROLLER_BOOM_TRAJECTORY);
		__platform_boom_lift_controller_set_enabled(platform, false);
		__platform_boom_extend_controller_set_enabled(platform, false);
	}
}

int platform_boom_trajectory_controller_set_enabled(struct platform *platform, bool enabled)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET_ENABLED,
		.trajectory_enable = {
			.enabled = enabled,
		},
	};

	return platform_send_message(platform, &msg);
}

static void platform_boom_trajectory_controller_run(absolute_time_t scheduled, void *data)
{
	struct platform *platform = data;

	if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_TRAJECTORY)) {
		return;
	}

	float radians;
	float mm;
	int16_t count = boom_update_count();
	int16_t angle;
	int ret = boom_lift_get_angle(&angle);
	if (ret != 0) {
		log_printf(&util_logger, "failed to get position");
		__platform_boom_trajectory_controller_set_enabled(platform, false);
		return;
	}

	// Skip the i2c traffic for timing purposes
	// At  100kHz it adds 500 us!
	absolute_time_t now = get_absolute_time();
	uint32_t start_time = to_us_since_boot(now);

	// Find the current position
	mm = boom_extend_count_to_mm(count);
	radians = boom_lift_angle_to_radians(angle);

	struct v2 current_pos = forward_kinematics(radians, mm);
	struct v2 dp = vec2_sub(platform->trajectory.target, current_pos);

	platform->boom_timestamp = now;
	platform->boom_current = current_pos;

	// Find out if we already arrived at the target
	float distance = vec2_magnitude(dp);
	if (distance <= BOOM_POS_TARGET_DELTA) {
		log_printf(&util_logger, "trajectory target reached");
		__platform_boom_trajectory_controller_set_enabled(platform, false);
		return;
	}

	// Find the closest point along the trajectory
	struct v2 current_vec = vec2_sub(current_pos, platform->trajectory.start);
	distance = vec2_dot(current_vec, platform->trajectory.unit);

	// Move a little further ahead, clamped to the target point
	distance += MAX_TRAJECTORY_STEP;
	distance = clamp(distance, 0.0, platform->trajectory.mag);

	// Calculate the new target
	struct v2 new_target = vec2_add(platform->trajectory.start, vec2_mul(platform->trajectory.unit, distance));
	dp = vec2_sub(new_target, current_pos);

	// Run the kinematics to get the movement vector
	struct m2 j = get_jacobian(radians, mm);
	struct m2 j_inv = m2_inverse(&j);
	struct v2 q_dot = m2_multvect(&j_inv, &dp);

	// TODO: This should be unnecessary now that "MAX_TRAJECTORY_STEP" is used
	//float rad_step = clamp_abs(q_dot.x, 0.8 * M_PI / 180.0);
	//float mm_step = clamp_abs(q_dot.y, 20);
	float rad_step = q_dot.x;
	float mm_step = q_dot.y;

	log_printf(&util_logger, "rad_step: %3.5f, mm_step: %3.2f", rad_step, mm_step);

	int16_t new_angle = boom_lift_radians_to_angle(radians + rad_step);
	int16_t new_count = boom_extend_mm_to_count(mm + mm_step);

	uint32_t end_time = time_us_32();

	__platform_boom_extend_controller_set(platform, new_count);
	__platform_boom_lift_controller_set(platform, new_angle);

	log_printf(&util_logger, "%d, %d -> %d, %d, took %d us", angle, count,
			new_angle, new_count, end_time - start_time);
	log_printf(&util_logger, "Current: %d, %d -> Target: %3.2f,%3.2f", (int)current_pos.x, (int)current_pos.y,
			platform->trajectory.target.x, platform->trajectory.target.y);

	platform_schedule_function(platform, platform_boom_trajectory_controller_run,
	                           platform, get_absolute_time() + BOOM_TRAJECTORY_CONTROLLER_TICK);
}

static void platform_level_servo_run(absolute_time_t scheduled, void *data)
{
	struct platform *platform = data;

	if (!__controllers_are_enabled(platform, CONTROLLER_BOOM_FORK_LEVEL)) {
		return;
	}

	int16_t angle;
	int ret = boom_lift_get_angle(&angle);
	if (ret != 0) {
		log_printf(&util_logger, "failed to get position");
		return;
	} else {
		int16_t servo_val = get_servo_val(angle);
		ioe_set_pwm_duty(&platform->ioe, 1, servo_val);
	}

	platform_schedule_function(platform, platform_level_servo_run,
	                           platform, get_absolute_time() + BOOM_SERVO_CONTROLLER_TICK);
}

static void __platform_servo_level_set_enabled(struct platform *platform, bool enabled)
{
	if (enabled) {
		if (!(platform->status & PLATFORM_STATUS_BOOM_HOMED)) {
			log_printf(&util_logger, "must home before enabling controller");
			return;
		}

		if (__controllers_are_enabled(platform, CONTROLLER_BOOM_FORK_LEVEL)) {
			log_printf(&util_logger, "already enabled");
			return;
		}

		__controllers_set_enabled(platform, CONTROLLER_BOOM_FORK_LEVEL);
		ioe_set_pin_mode(&platform->ioe, 1, IOE_PIN_MODE_PWM);
		platform_schedule_function(platform, platform_level_servo_run, platform, get_absolute_time());
	} else {
		__controllers_set_disabled(platform, CONTROLLER_BOOM_FORK_LEVEL);
		ioe_set_pin_mode(&platform->ioe, 1, IOE_PIN_MODE_OD);
		log_printf(&util_logger, "servo_level disabled");
	}
}

int platform_servo_level(struct platform *platform, bool enabled)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_SERVO_LEVEL_SET_ENABLED,
		.servo_level_enable = {
			.enabled = enabled,
		},
	};

	return platform_send_message(platform, &msg);
}

int platform_ioe_set(struct platform *platform, uint8_t pin, uint16_t val)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_IOE_SET,
		.ioe_set = {
			.pin = pin,
			.val = val,
		},
	};

	return platform_send_message(platform, &msg);
}

static void __platform_ioe_set(struct platform *platform, uint8_t pin, uint16_t val)
{
	ioe_set_pwm_duty(&platform->ioe, pin, val);
}

int platform_all_stop(struct platform *platform)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_ALL_STOP,
	};

	return platform_send_message(platform, &msg);
}

static void __platform_all_stop(struct platform *platform)
{
	__platform_boom_trajectory_controller_set_enabled(platform, false);
	__platform_boom_lift_controller_set_enabled(platform, false);
	__platform_boom_extend_controller_set_enabled(platform, false);
	__platform_servo_level_set_enabled(platform, false);
}

int platform_set_pid_coeffs(struct platform *platform, enum pid_controller_id id, float kp, float ki, float kd)
{
	struct platform_message msg = {
		.type = PLATFORM_MESSAGE_PID_SET,
		.pid_set = {
			.id = id,
			.kp = kp,
			.ki = ki,
			.kd = kd,
		},
	};

	return platform_send_message(platform, &msg);
}

static void __platform_set_pid_coeffs(struct platform *platform, enum pid_controller_id id, float kp, float ki, float kd)
{
	switch (id) {
	case PID_CONTROLLER_ID_LIFT:
		fcontroller_set_tunings(&platform->boom_lift_controller, kp, ki, kd, BOOM_LIFT_CONTROLLER_TICK);
		break;
	case PID_CONTROLLER_ID_EXTEND:
		fcontroller_set_tunings(&platform->boom_extend_pos_controller, kp, ki, kd, BOOM_EXTEND_CONTROLLER_TICK);
		break;
	}
}

void platform_run(struct platform *platform) {
	// Kick off heading updates
	if (platform->status & PLATFORM_STATUS_BNO055_PRESENT) {
		platform_update_heading(get_absolute_time(), platform);
	}

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
			case PLATFORM_MESSAGE_BOOM_HOME:
				__platform_boom_home(platform);
				break;
			case PLATFORM_MESSAGE_BOOM_SET:
				__platform_boom_lift_controller_set(platform, msg.boom_set.angle);
				break;
			case PLATFORM_MESSAGE_BOOM_SET_ENABLED:
				__platform_boom_lift_controller_set_enabled(platform, msg.boom_enable.enabled);
				break;
			case PLATFORM_MESSAGE_BOOM_EXTEND_SET:
				__platform_boom_extend_controller_set(platform, msg.boom_extend_set.count);
				break;
			case PLATFORM_MESSAGE_BOOM_EXTEND_SET_ENABLED:
				__platform_boom_extend_controller_set_enabled(platform, msg.boom_extend_enable.enabled);
				break;
			case PLATFORM_MESSAGE_IOE_SET:
				__platform_ioe_set(platform, msg.ioe_set.pin, msg.ioe_set.val);
				break;
			case PLATFORM_MESSAGE_SERVO_LEVEL_SET_ENABLED:
				__platform_servo_level_set_enabled(platform, msg.servo_level_enable.enabled);
				break;
			case PLATFORM_MESSAGE_PID_SET:
				__platform_set_pid_coeffs(platform, msg.pid_set.id, msg.pid_set.kp, msg.pid_set.ki, msg.pid_set.kd);
				break;
			case PLATFORM_MESSAGE_BOOM_UPDATE:
				__platform_boom_update_position(platform);
				break;
			case PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET:
				__platform_boom_trajectory_controller_set(platform, msg.trajectory.start, msg.trajectory.target);
				break;
			case PLATFORM_MESSAGE_BOOM_TRAJECTORY_SET_ENABLED:
				__platform_boom_trajectory_controller_set_enabled(platform, msg.trajectory_enable.enabled);
				break;
			case PLATFORM_MESSAGE_ALL_STOP:
				__platform_all_stop(platform);
				break;
			case PLATFORM_MESSAGE_BOOM_TRAJECTORY_ADJUST:
				__platform_boom_trajectory_controller_adjust_target(platform, msg.trajectory_adjust.amount,
						msg.trajectory_adjust.relative_to);
				break;
			case PLATFORM_MESSAGE_BOOM_SET_RAW:
				__platform_boom_set_raw(platform, msg.boom_set_raw.lift, msg.boom_set_raw.extend);
				break;
			}
		} while (queue_try_remove(&platform->queue, &msg));
	}
}
