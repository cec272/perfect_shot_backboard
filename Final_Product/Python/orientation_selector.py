# Harrison Hidalgo
# ECE 5725 - Final Project
# Selects orientation to move to using mean-square error.

def orientation_selector(th_1_current,th_2_current,th_3_current,possible_orientations):
	min_MSE = float('inf')
	for i in range(0,len(possible_orientations[1,:])):
		MSE_th1=(th_1_current-possible_orientations[0,i])^2
		MSE_th2=(th_2_current-possible_orientations[1,i])^2
		MSE_th3=(th_3_current-possible_orientations[2,i])^2
		if (MSE_th1+MSE_th2+MSE_th3) < min_MSE:
			theta1=possible_orientations[0,i]
			theta2=possible_orientations[1,i]
			theta3=possible_orientations[2,i]
			min_MSE=MSE_th1+MSE_th2+MSE_th3
	return theta1,theta2,theta3
