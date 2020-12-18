# Harrison Hidalgo
# ECE 5725 - Final Project
#

def ball_calc(x):
	import A_ball
	import b_ball
	import numpy as np
	import Physical_Variables as p
	## Unpack X
	X=x[0]
	Y=x[1]
	Z=x[2]
	X_dot=x[3]
	Y_dot=x[4]
	Z_dot=x[5]
	## Unpack p
	c=p.c
	g=p.g
	m=p.m
	## Calculations 
	Mass = A_ball.A_ball(m)
	Force = b_ball.b_ball(c,g,m,X_dot,Y_dot,Z_dot)
	## Pack up solution
	vels = np.array([X_dot,Y_dot,Z_dot])
	accel = np.linalg.solve(Mass,Force)
	x_dot = np.concatenate((vels,accel),axis=0)
	return x_dot
