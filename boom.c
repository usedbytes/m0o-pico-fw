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
#include "util.h"

#define BOOM_EXTEND_MOTOR_A_PIN  8
#define BOOM_EXTEND_MOTOR_B_PIN  9
#define BOOM_EXTEND_LIMIT_PIN   10
#define BOOM_EXTEND_ENC_PIN     11
#define BOOM_EXTEND_RETRACT_DIR  1
#define BOOM_EXTEND_EXTEND_DIR  -1
// 100ms cooldown when changing direction, to wait for motor to stop
#define BOOM_EXTEND_COOLDOWN_US 100000

#define BOOM_LIFT_MOTOR_A_PIN    6
#define BOOM_LIFT_MOTOR_B_PIN    7
#define BOOM_LIFT_LIMIT_PIN     22

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
} boom;

static inline bool __is_retracting(int8_t raw_val)
{
	return (raw_val * BOOM_EXTEND_RETRACT_DIR) > 0;
}

static inline bool __extend_limit_pressed()
{
	return gpio_get(BOOM_EXTEND_LIMIT_PIN);
}

void boom_handle_extend_limit(uint32_t events)
{
	if ((events & GPIO_IRQ_EDGE_RISE) && __extend_limit_pressed() && __is_retracting(boom.extend.raw_val)) {
		// Stop!
		boom_extend_set_protected(0);
	}
}

void boom_handle_lift_limit(uint32_t events)
{

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
int boom_extend_set_protected(int8_t val)
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

void boom_init()
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
}
