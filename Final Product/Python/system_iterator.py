# Harrison Hidalgo
# ECE 5725 - Final Project
#
#

def system_iterator(e_b,r_B0,rGB0,h,x_ball_init,ffun,p,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop):
	from iteration_variables import *
	import numpy as np
	from path_tracker import *
	# Unpack iteration_vals
	names = vars(iteration_vals)
	for i in range(0,len(names)
	# Done
	# Cycle throufh different theta, psi, phi, and B positions
	number_of_divisions = 10
	# Create vectors to iterate through
	theta_iter = np.linspace(theta_min,theta_max,number_of_divisions)
	phi_iter   = np.linspace(phi_min,phi_max,number_of_divisions)
	psi_iter   = np.linspace(psi_min,psi_max,number_of_divisions)
	B_iter     = np.linspace(B_min,B_max,number_of_divisions)
	
	# For storing results
	working_orientations = []
	
	for i in range(0,number_of_divisions)# theta
	for j in range(0,number_of_divisions)# phi
	for k in range(0,number_of_divisions)# psi
	for l in range(0,number_of_divisions)# B
		theta = theta_iter[i]
		phi   = phi_iter[j]
		psi   = psi_iter[k]
		r_B	  = e_b*B_iter[l]+r_B0
		R = [np.cos(theta),-np.sin(theta),0;np.sin(theta),np.cos(theta),0;0,0,1]*[np.cos(phi),0,np.sin(phi);0,1,0;-np.sin(phi),0,np.cos(phi)]*[1,0,0;0,np.cos(psi),-np.sin(psi);0,np.sin(psi),np.cos(psi)];
		new_up = R*up; new_front = R.dot(front); new_backboard = r_B-R.dot(rGB0)
		if path_tracker(h,x_ball_init,ffun,p,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,backboard)
			working_orientations=[working_orientations,[theta_iter[i];phi_iter[j];psi_iter[k];B_iter[l]]];
	return working_orientations
