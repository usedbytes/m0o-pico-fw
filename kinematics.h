
struct point {
	float x, y;
};

float point_magnitude(struct point p);

struct point point_sub(struct point a, struct point b);

struct point forward_kinematics(float q1, float q2);

struct m2 {
	float a, b, c, d;
};

struct v2 {
	float a, b;
};

struct m2 m2_inverse(struct m2 *m);

struct v2 m2_multvect(struct m2 *m, struct v2 *v);

struct m2 get_jacobian(float q1, float q2);
