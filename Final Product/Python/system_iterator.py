# Harrison Hidalgo
# ECE 5725 - Final Project
#
#

def system_iterator(e_b,r_B0,r_GB0,h,x_ball_init,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop):
	import iteration_variables as I_V
	import numpy as np
	import transformations
	import path_tracker
	# Cycle through different theta, psi, phi, and B positions
	number_of_divisions = 5
	# Create vectors to iterate through
	theta_iter = np.linspace(I_V.theta_min,I_V.theta_max,number_of_divisions)
	phi_iter   = np.linspace(I_V.phi_min,I_V.phi_max,number_of_divisions)
	psi_iter   = np.linspace(I_V.psi_min,I_V.psi_max,number_of_divisions)
	B_iter     = np.linspace(I_V.B_min,I_V.B_max,number_of_divisions)
	# For storing results
	working_orientations = []
	for i in range(0,number_of_divisions):# theta
		for j in range(0,number_of_divisions):# phi
			for k in range(0,number_of_divisions):# psi
				for l in range(0,number_of_divisions):# B
					theta = theta_iter[i]
					phi   = phi_iter[j]
					psi   = psi_iter[k]
					r_B	  = e_b*B_iter[l]+r_B0
					new_up = transformations.transform_baseboard_to_backboard(theta,phi,psi,up)
					new_front = transformations.transform_baseboard_to_backboard(theta,phi,psi,front)
					new_backboard = r_B-transformations.transform_baseboard_to_backboard(theta,phi,psi,r_GB0)
					if path_tracker.path_tracker(h,x_ball_init,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,new_backboard):
						if len(working_orientations)<1:
							working_orientations = np.array([[theta_iter[i]],[phi_iter[j]],[psi_iter[k]],[B_iter[l]]])
						else:
							working_orientations=np.append([working_orientations],np.transpose(np.array([[theta_iter[i]],[phi_iter[j]],[psi_iter[k]],[B_iter[l]]])))
	working_orientations=np.reshape(working_orientations,(4,len(working_orientations)/4))
	return working_orientations
