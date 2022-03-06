from sympy import *

def rotation_3d(x, y, z):
    return Matrix([
        [ cos(z)*cos(y), cos(z)*sin(y)*sin(x) - sin(z)*cos(x), cos(z)*sin(y)*cos(x) + sin(z)*sin(x), 0 ],
        [ sin(z)*cos(y), sin(z)*sin(y)*sin(x) + cos(z)*cos(x), sin(z)*sin(y)*cos(x) - cos(z)*sin(x), 0 ],
        [       -sin(y),                        cos(y)*sin(x),                        cos(y)*cos(x), 0 ],
        [             0,                                    0,                                    0, 1 ],
    ])

def translation_3d(x, y, z):
    return Matrix([
        [ 1, 0, 0, x ],
        [ 0, 1, 0, y ],
        [ 0, 0, 1, z ],
        [ 0, 0, 0, 1 ],
    ])

# Note: Angles in radians
def make_X(d, t, r, al):
    return Matrix([
        [ cos(t), -sin(t)*cos(al),  sin(t)*sin(al), r*cos(t) ],
        [ sin(t),  cos(t)*cos(al), -cos(t)*sin(al), r*sin(t) ],
        [      0,         sin(al),         cos(al),        d ],
        [      0,               0,               0,        1 ],
    ])

#lift_theta, extend = symbols('lt e', real=True)
#
#joint_params = [
#    [                    0, (pi/2)+q1, 16, pi/2 ],
#    [ 3+BASE_EXTEND_LEN+q2,         0,  0,    0 ],
#]
#
#Aj = [make_X(p[0], p[1], p[2], p[3]) for p in joint_params]
#
#A0_3d = translation_3d(0, 0, 30) * rotation_3d(pi / 2, 0, 0)
#
#A02_3d = A0_3d * Aj[0] * Aj[1]
#
#x_expr_3d = A02_3d.col(3)[0]
#z_expr_3d = A02_3d.col(3)[2]

def rotation(theta):
    return Matrix([
        [ cos(theta), -sin(theta), 0 ],
        [ sin(theta),  cos(theta), 0 ],
        [          0,           0, 1 ],
    ])

def translation(x, y):
    return Matrix([
        [ 1, 0, x ],
        [ 0, 1, y ],
        [ 0, 0, 1 ],
    ])

BASE_EXTEND_LEN=248
AXLE_FROM_FRONT_OF_TIRES = (271.4 - 25.7) + 18

q1, q2 = symbols('q1 q2')

A0 = translation(-AXLE_FROM_FRONT_OF_TIRES, 30)
A1 = rotation(q1) * translation(0, 16)
A2 = translation(BASE_EXTEND_LEN + q2, 0)
A3 = rotation(10 * pi / 180) * translation(30, -48.5)

A03 = A0 * A1 * A2 * A3

# x_expr and y_expr can be used for forward kinematics
x_expr = A03.col(2)[0]
y_expr = A03.col(2)[1]

# vector_to_target*(J**-1) gives joint velocities
J = Matrix([
    [ diff(x_expr, q1), diff(x_expr, q2) ],
    [ diff(y_expr, q1), diff(y_expr, q2) ],
])

print("x_expr: {}".format((x_expr.evalf(6))))
print("y_expr: {}".format((y_expr.evalf(6))))
print("J: {}".format((J.evalf(6))))
