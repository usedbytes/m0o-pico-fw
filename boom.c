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

#define BOOM_EXTEND_MOTOR_A_PIN  8
#define BOOM_EXTEND_MOTOR_B_PIN  9
#define BOOM_EXTEND_LIMIT_PIN   10
#define BOOM_EXTEND_ENC_PIN     11
#define BOOM_EXTEND_RETRACT_DIR  1
#define BOOM_EXTEND_EXTEND_DIR  -1

#define BOOM_LIFT_MOTOR_A_PIN    6
#define BOOM_LIFT_MOTOR_B_PIN    7
#define BOOM_LIFT_LIMIT_PIN     22

struct boom {
	uint extend_motor_slice;
	uint extend_encoder_slice;
	volatile int8_t extend_val;
	uint16_t extend_count;
	uint16_t last_count;
} boom;

static inline bool __is_retracting(int8_t val)
{
	return (val * BOOM_EXTEND_RETRACT_DIR) > 0;
}

static inline bool __extend_limit_pressed()
{
	return gpio_get(BOOM_EXTEND_LIMIT_PIN);
}

void boom_handle_extend_limit(uint32_t events)
{
	if ((events & GPIO_IRQ_EDGE_RISE) && __extend_limit_pressed() && __is_retracting(boom.extend_val)) {
		// Stop!
		boom_extend_set_raw(0);
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

void boom_extend_set_protected(int8_t val)
{
	if (__is_retracting(val) && __extend_limit_pressed()) {
		return;
	}

	boom_extend_set_raw(val);
}

void boom_reset_count()
{
	boom.extend_count = 0;
	boom.last_count = 0;
}

uint16_t boom_update_count()
{
	uint16_t count = pwm_get_counter(boom.extend_encoder_slice);
	uint16_t diff = count - boom.last_count;
	boom.last_count = count;

	if (__is_retracting(boom.extend_val)) {
		boom.extend_count -= diff;
	} else {
		boom.extend_count += diff;
	}

	return boom.extend_count;
}

void boom_extend_set_raw(int8_t val)
{
	// Stop first and reset counter if changing direction or stopping
	if (((boom.extend_val < 0) && (val > 0)) ||
	    ((boom.extend_val > 0) && (val < 0)) ||
	    (val == 0)) {
		slice_set_with_brake(boom.extend_motor_slice, 0, true);
		sleep_us(1000);

		boom_update_count();
		pwm_set_counter(boom.extend_encoder_slice, 0);
		boom.last_count = 0;
	}

	boom.extend_val = val;
	slice_set_with_brake(boom.extend_motor_slice, val, true);
}

void boom_init()
{
	gpio_init(BOOM_EXTEND_LIMIT_PIN);
	gpio_set_pulls(BOOM_EXTEND_LIMIT_PIN, true, false);
	gpio_set_irq_enabled_with_callback(BOOM_EXTEND_LIMIT_PIN, GPIO_IRQ_EDGE_RISE, true, boom_gpio_irq_cb);

	boom.extend_motor_slice = pwm_gpio_to_slice_num(BOOM_EXTEND_MOTOR_A_PIN);
	boom.extend_encoder_slice = pwm_gpio_to_slice_num(BOOM_EXTEND_ENC_PIN);

	init_slice(boom.extend_motor_slice, BOOM_EXTEND_MOTOR_A_PIN);

	gpio_set_function(BOOM_EXTEND_ENC_PIN, GPIO_FUNC_PWM);
	pwm_config c = pwm_get_default_config();
	pwm_config_set_clkdiv_mode(&c, PWM_DIV_B_RISING);
	pwm_init(boom.extend_encoder_slice, &c, true);

	gpio_init(BOOM_LIFT_LIMIT_PIN);
	gpio_set_pulls(BOOM_LIFT_LIMIT_PIN, true, false);
}
