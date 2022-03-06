#include "pico/float.h"

struct point {
	float x, y;
};

float point_magnitude(struct point p)
{
	return sqrtf(p.x * p.x + p.y * p.y);
}

struct point point_sub(struct point a, struct point b)
{
	return (struct point){
		.x = a.x - b.x,
		.y = a.y - b.y,
	};
}

float d2r(float degrees)
{
	return degrees * M_PI / 180.0;
}

float r2d(float radians)
{
	return radians * 180.0 / M_PI;
}

struct point forward_kinematics(float q1, float q2)
{
	float x = (q2 + 248.0)*cosf(q1) + 28.5518868536033*sinf(q1) + 35.2722182146761*cosf(q1) - 263.7;
	float y = (q2 + 248.0)*sinf(q1) + 35.2722182146761*sinf(q1) - 28.5518868536033*cosf(q1) + 30.0;

	return (struct point){
		.x = x,
		.y = y,
	};
}

struct m2 {
	float a, b, c, d;
};

struct v2 {
	float a, b;
};

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
		m->a * v->a + m->b * v->b,
		m->c * v->a + m->d * v->b,
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
