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
	import math
	R = np.matmul(np.matmul([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]],[[math.cos(phi),0,math.sin(phi)],[0,1,0],[-math.sin(phi),0,math.cos(phi)]]),[[1,0,0],[0,math.cos(psi),-math.sin(psi)],[0,math.sin(psi),math.cos(psi)]])
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
	import math
	R = np.matmul(np.matmul([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]],[[math.cos(phi),0,math.sin(phi)],[0,1,0],[-math.sin(phi),0,math.cos(phi)]]),[[1,0,0],[0,math.cos(psi),-math.sin(psi)],[0,math.sin(psi),math.cos(psi)]])
	R = np.linalg.inv(R)
	x_new = np.matmul(R,x)
	return x_new

def transform_camera_to_baseboard(position_camera):
	#
	# Parameters:
	# position_camera: location in camera coordinates
	new_position = [position_camera[0],-position_camera[1],-position_camera[2]]
	return new_position
	
