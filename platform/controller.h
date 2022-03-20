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
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include <stdint.h>

struct fcontroller {
	float last_input;
	float output_sum;

	float setpoint;

	float out_min, out_max;
	float kp;
	float ki;
	float kd;

};

// This must be called at the interval specified by step_up in
// fcontroller_set_tunings
float fcontroller_tick(struct fcontroller *c, float input);

void fcontroller_set(struct fcontroller *c, float setpoint);

void fcontroller_set_tunings(struct fcontroller *c, float kp, float ki, float kd, uint32_t step_us);

// Use this whenever the controller gets "enabled"
void fcontroller_reinit(struct fcontroller *c, float input, float output);

#endif /* __CONTROLLER_H__ */
