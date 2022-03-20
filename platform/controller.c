/**
 * This is mostly a port of Brett Beauregard's Arduino library
 * https://github.com/br3ttb/Arduino-PID-Library
 * Which has some excellent accompanying theory write-up:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
 * Copyright (c) 2017 Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "controller.h"

static inline float clamp(float a, float min, float max)
{
	if (a < min) {
		return min;
	}

	if (a > max) {
		return max;
	}

	return a;
}

void fcontroller_set_tunings(struct fcontroller *c, float kp, float ki, float kd, uint32_t step_us)
{
	float step_sec = step_us / (float)1e6;
	c->kp = kp;
	c->ki = ki * step_sec;
	c->kd = kd / step_sec;
}

void fcontroller_set(struct fcontroller *c, float setpoint)
{
	c->setpoint = setpoint;
}

float fcontroller_tick(struct fcontroller *c, float input)
{
	float error = c->setpoint - input;
	float dinput = input - c->last_input;

	// Calculate and clamp integral
	c->output_sum += c->ki * error;
	c->output_sum = clamp(c->output_sum, c->out_min, c->out_max);

	// TODO: Only supports proportional-on-error
	float output = c->kp * error;

	output += c->output_sum - (c->kd * dinput);

	c->last_input = input;

	return output;
}

void fcontroller_reinit(struct fcontroller *c, float input, float output)
{
	c->output_sum = clamp(output, c->out_min, c->out_max);
	c->last_input = input;
}
