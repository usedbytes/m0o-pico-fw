/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "kinematics.h"

#include "pico/float.h"

float vec2_magnitude(struct v2 v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}

float vec2_normalise(struct v2 v, struct v2 *out)
{
	float mag = vec2_magnitude(v);
	out->x = v.x / mag;
	out->y = v.y / mag;
	return mag;
}

struct v2 vec2_sub(struct v2 a, struct v2 b)
{
	return (struct v2){
		.x = a.x - b.x,
		.y = a.y - b.y,
	};
}

float vec2_dot(struct v2 a, struct v2 b)
{
	return a.x * b.x + a.y * b.y;
}

struct v2 forward_kinematics(float q1, float q2)
{
	float x = (q2 + 248.0)*cosf(q1) + 28.5518868536033*sinf(q1) + 35.2722182146761*cosf(q1) - 263.7;
	float y = (q2 + 248.0)*sinf(q1) + 35.2722182146761*sinf(q1) - 28.5518868536033*cosf(q1) + 30.0;

	return (struct v2){
		.x = x,
		.y = y,
	};
}

struct m2 m2_inverse(struct m2 *m)
{
	float det = 1.0 / (m->a * m->d - m->b * m->c);

	return (struct m2){
		.a = m->d * det,
		.b = -m->b * det,
		.c = -m->c * det,
		.d = m->a * det,
	};
}

struct v2 m2_multvect(struct m2 *m, struct v2 *v)
{
	return (struct v2){
		m->a * v->x + m->b * v->y,
		m->c * v->x + m->d * v->y,
	};
}

struct m2 get_jacobian(float q1, float q2)
{
	return (struct m2){
		.a = -(q2 + 248.0)*sinf(q1) - 35.2722182146761*sinf(q1) + 28.5518868536033*cosf(q1),
		.b = cosf(q1),
		.c = (q2 + 248.0)*cosf(q1) + 28.5518868536033*sinf(q1) + 35.2722182146761*cosf(q1),
		.d = sinf(q1),
	};
}
