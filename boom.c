/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "boom.h"
#include "chassis.h"
#include "i2c_bus.h"
#include "log.h"
#include "util.h"

#define BOOM_EXTEND_MOTOR_A_PIN    8
#define BOOM_EXTEND_MOTOR_B_PIN    9
#define BOOM_EXTEND_LIMIT_PIN     10
#define BOOM_EXTEND_ENC_PIN       11
#define BOOM_EXTEND_RETRACT_DIR    1
#define BOOM_EXTEND_EXTEND_DIR    -1
// 100ms cooldown when changing direction, to wait for motor to stop
#define BOOM_EXTEND_COOLDOWN_US   100000

// count, abs mm, rel mm, counts/mm
//     0,   44.2,      0,
//  6796,   93.7,   49.5, 137.292929292929
// 12887, 138.24,  94.04, 137.037430880476
//  6131,     89,   44.8, 136.852678571429
#define BOOM_EXTEND_COUNTS_PER_MM 137

#define BOOM_LIFT_MOTOR_A_PIN    6
#define BOOM_LIFT_MOTOR_B_PIN    7
#define BOOM_LIFT_LIMIT_PIN     22
#define BOOM_LIFT_RAISE_DIR      1
#define BOOM_LIFT_LOWER_DIR     -1
#define BOOM_LIFT_HOME_OFFSET   10.0 // Mostly a guesstimate

#define AS5600_ADDR 0x36

struct boom {
	struct {
		uint motor_slice;
		uint encoder_slice;
		volatile int8_t raw_val;
		bool cooldown;
		uint32_t cooldown_ts;
		int16_t count;
		uint16_t last_count;
	} extend;
	struct {
		uint motor_slice;
		struct i2c_bus *i2c;
		int16_t zero_angle;
		volatile int8_t raw_val;
	} lift;
} boom;

static inline bool __is_retracting(int8_t raw_val)
{
	return (raw_val * BOOM_EXTEND_RETRACT_DIR) > 0;
}

static inline bool __extend_limit_pressed()
{
	return gpio_get(BOOM_EXTEND_LIMIT_PIN);
}

bool boom_extend_at_limit()
{
	return __extend_limit_pressed();
}

void boom_handle_extend_limit(uint32_t events)
{
	if ((events & GPIO_IRQ_EDGE_RISE) && __extend_limit_pressed() && __is_retracting(boom.extend.raw_val)) {
		// Stop!
		boom_extend_set(0);
	}
}

static inline bool __is_lowering(int8_t raw_val)
{
	return (raw_val * BOOM_LIFT_LOWER_DIR) > 0;
}

static inline bool __lift_limit_pressed()
{
	return gpio_get(BOOM_LIFT_LIMIT_PIN);
}

bool boom_lift_at_limit()
{
	return __lift_limit_pressed();
}

void boom_handle_lift_limit(uint32_t events)
{
	if ((events & GPIO_IRQ_EDGE_RISE) && __lift_limit_pressed() && __is_lowering(boom.lift.raw_val)) {
		// Stop!
		boom_lift_set(0);
	}
}

void boom_gpio_irq_cb(uint gpio, uint32_t events)
{
	switch (gpio) {
	case BOOM_EXTEND_LIMIT_PIN:
		boom_handle_extend_limit(events);
		break;
	case BOOM_LIFT_LIMIT_PIN:
		boom_handle_lift_limit(events);
		break;
	}

	gpio_acknowledge_irq(gpio, events);
}

// Returns:
// -1: Cooldown
// 0: OK
// 1: At limit
int boom_extend_set(int8_t val)
{
	int8_t raw_val = clamp8(val * BOOM_EXTEND_EXTEND_DIR);

	if (boom.extend.cooldown) {
		// If still cooling down, early-out
		uint32_t t = time_us_32();
		if ((t - boom.extend.cooldown_ts) < BOOM_EXTEND_COOLDOWN_US) {
			return -1;
		}

		// Otherwise, cooldown is finished
		boom_update_count();
		boom.extend.cooldown = false;
		boom.extend.raw_val = 0;
	}

	// If we're crossing zero, then first stop completely.
	// Keep the current extend.raw_val value until cooldown is complete, so
	// any calls to boom_update_count() use the right direction.
	if (((boom.extend.raw_val < 0) && (raw_val > 0)) ||
	    ((boom.extend.raw_val > 0) && (raw_val < 0)) ||
	    ((boom.extend.raw_val != 0) && (raw_val == 0))) {
		boom.extend.cooldown = true;
		boom.extend.cooldown_ts = time_us_32();
		slice_set_with_brake(boom.extend.motor_slice, 0, true);
		return -1;
	}

	// If we're at the limit and trying to retract, that's a no.
	if (__is_retracting(raw_val) && __extend_limit_pressed()) {
		return 1;
	}

	boom.extend.raw_val = raw_val;
	slice_set_with_brake(boom.extend.motor_slice, raw_val, true);

	return 0;
}

void boom_reset_count()
{
	boom_update_count();
	boom.extend.count = 0;
}

int16_t boom_update_count()
{
	uint16_t count = pwm_get_counter(boom.extend.encoder_slice);
	uint16_t diff = count - boom.extend.last_count;
	boom.extend.last_count = count;

	if (__is_retracting(boom.extend.raw_val)) {
		boom.extend.count -= diff;
	} else {
		boom.extend.count += diff;
	}

	return boom.extend.count;
}

float boom_extend_count_to_mm(int16_t count)
{
	return count / (float)BOOM_EXTEND_COUNTS_PER_MM;
}

int16_t boom_extend_mm_to_count(float mm)
{
	return mm * BOOM_EXTEND_COUNTS_PER_MM;
}

uint16_t __lift_get_angle_raw()
{
	uint8_t buf[2];

	int ret = i2c_bus_write_blocking(boom.lift.i2c, AS5600_ADDR, (uint8_t[]){ 0x0C }, 1);
	if (ret != 1) {
		log_printf(&util_logger, "write error: %d\n", ret);
		return 0x8000;
	}

	ret = i2c_bus_read_blocking(boom.lift.i2c, AS5600_ADDR, buf, 2);
	if (ret != 2) {
		log_printf(&util_logger, "read error: %d\n", ret);
		return 0x8000;
	}

	return buf[0] << 8 | buf[1];
}

int boom_lift_get_angle(int16_t *angle)
{
	uint16_t raw_angle = __lift_get_angle_raw();
	if (raw_angle & 0x8000) {
		return -1;
	}

	*angle = (int16_t)raw_angle - boom.lift.zero_angle;

	return 0;
}

float boom_lift_angle_to_degrees(int16_t angle)
{
	return (angle * 90.0 / 1024.0) - BOOM_LIFT_HOME_OFFSET;
}

int16_t boom_lift_degrees_to_angle(float degrees)
{
	return (degrees + BOOM_LIFT_HOME_OFFSET) * 1024.0 / 90.0;
}

int boom_lift_reset_angle()
{
	uint16_t raw_angle = __lift_get_angle_raw();
	if (raw_angle & 0x8000) {
		return -1;
	}

	boom.lift.zero_angle = raw_angle;

	return 0;
}

// Returns:
// -1: Cooldown
// 0: OK
// 1: At limit
int boom_lift_set(int8_t val)
{
	int8_t raw_val = clamp8(val * BOOM_LIFT_RAISE_DIR);

	// If we're at the limit and trying to lower, that's a no.
	if (__is_lowering(raw_val) && __lift_limit_pressed()) {
		return 1;
	}

	boom.lift.raw_val = raw_val;
	slice_set_with_brake(boom.lift.motor_slice, raw_val, true);

	return 0;
}

void boom_init(struct i2c_bus *i2c)
{
	gpio_init(BOOM_EXTEND_LIMIT_PIN);
	gpio_set_pulls(BOOM_EXTEND_LIMIT_PIN, true, false);
	gpio_set_irq_enabled_with_callback(BOOM_EXTEND_LIMIT_PIN, GPIO_IRQ_EDGE_RISE, true, boom_gpio_irq_cb);

	boom.extend.motor_slice = pwm_gpio_to_slice_num(BOOM_EXTEND_MOTOR_A_PIN);
	boom.extend.encoder_slice = pwm_gpio_to_slice_num(BOOM_EXTEND_ENC_PIN);

	init_slice(boom.extend.motor_slice, BOOM_EXTEND_MOTOR_A_PIN);

	gpio_set_function(BOOM_EXTEND_ENC_PIN, GPIO_FUNC_PWM);
	pwm_config c = pwm_get_default_config();
	pwm_config_set_clkdiv_mode(&c, PWM_DIV_B_RISING);
	pwm_init(boom.extend.encoder_slice, &c, true);

	gpio_init(BOOM_LIFT_LIMIT_PIN);
	gpio_set_pulls(BOOM_LIFT_LIMIT_PIN, true, false);
	gpio_set_irq_enabled(BOOM_LIFT_LIMIT_PIN, GPIO_IRQ_EDGE_RISE, true);

	boom.lift.i2c = i2c;
	boom.lift.motor_slice = pwm_gpio_to_slice_num(BOOM_LIFT_MOTOR_A_PIN);
	init_slice(boom.lift.motor_slice, BOOM_LIFT_MOTOR_A_PIN);
}
