/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

struct v2 {
	float x, y;
};

float vec2_magnitude(struct v2 v);

float vec2_normalise(struct v2 v, struct v2 *out);

struct v2 vec2_sub(struct v2 a, struct v2 b);

float vec2_dot(struct v2 a, struct v2 b);

struct v2 forward_kinematics(float q1, float q2);

struct m2 {
	float a, b, c, d;
};

struct m2 m2_inverse(struct m2 *m);

struct v2 m2_multvect(struct m2 *m, struct v2 *v);

struct m2 get_jacobian(float q1, float q2);

#endif /* __KINEMATICS_H__ */
