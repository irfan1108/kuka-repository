# Contains kinematic functions
# Provides functions for solving the forward and inverse kinematics

import numpy as np
from sympy.matrices import Matrix
from sympy import symbols, atan2, sqrt, pi, cos, sin, atan2, simplify

# Radians to degrees, and vice versa multipliers
rads = 180.0/pi
deg = pi/180

# Euler Angle functions
# beta = atan2(-R_xyz[2, 0], sqrt(R_xyz[0,0]**2 + R_xyz[1, 0]**2)) * rads
# gamma = atan2(R_xyz[2, 1], R_xyz[2, 2])*rads
# alpha = atan2(R_xyz[1, 0], R_xyz[0, 0])*rads


# Define functions for rotation matrices about x, y and z given specific angles

# Rotate on X
def rot_x(q):
    return Matrix([[1,      0,      0  ],
                   [0, cos(q), -sin(q) ],
                   [0, sin(q),  cos(q) ]])
                   
# Rotate on Y
def rot_y(q):
    return Matrix([[cos(q),  0, sin(q)],
                   [      0, 1,      0],
                   [-sin(q), 0, cos(q)]])

# Rotate on z
def rot_z(q):
    return Matrix([[cos(q),-sin(q), 0],
                   [sin(q), cos(q), 0],
                   [     0,      0, 1]])
                   
                   
# Creates the heterogeneous transform matrix
def htm(rot_mat, trans_mat):
	filler = Matrix([[0, 0, 0, 1]])
	htm = rot_mat.row_join(trans_mat).col_join(filler)
	return htm
	
# creates DH heterogeneous transform matrix
def dhHtm(alpha, q, a, d):
    return Matrix([[       cos(q),           -sin(q),              0,             a],
                   [sin(q)*cos(alpha), cos(q)*cos(alpha),-sin(alpha), -sin(alpha)*d],
                   [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d],
                   [                    0,                     0,          0,           1]])
