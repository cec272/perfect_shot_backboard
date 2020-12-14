# Harrison Hidalgo and Christopher Chan
# ECE 5725 - Final Project
# This module contains all of our transforms.

def transform_baseboard_to_backboard(theta,phi,psi,x):
	#
	# Parameters:
	# 	theta: rotation about baseboard z axis
	# 	phi: rotation about baseboard y axis
	# 	psi: rotation about baseboard x axis
	# 	x: vector to transform
	# Output: 
	# 	x_new: transformed vector
	import numpy as np
	R = np.matmul(np.matmul([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]],[[cos(phi),0,sin(phi)],[0,1,0],[-sin(phi),0,cos(phi)]]),[[1,0,0],[0,cos(psi),-sin(psi)],[0,sin(psi),cos(psi)]])
	x_new = np.matmul(R,x)
	return x_new

def transform_backboard_to_baseboard(theta,phi,psi,x):
	#
	# Parameters:
	# 	theta: rotation about baseboard z axis
	# 	phi: rotation about baseboard y axis
	# 	psi: rotation about baseboard x axis
	# 	x: vector to transform
	# Output: 
	# 	x_new: transformed vector
	import numpy as np
	R = np.matmul(np.matmul([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]],[[cos(phi),0,sin(phi)],[0,1,0],[-sin(phi),0,cos(phi)]]),[[1,0,0],[0,cos(psi),-sin(psi)],[0,sin(psi),cos(psi)]])
	R = np.linalg.inv(R)
	x_new = np.matmul(R,x)
	return x_new
