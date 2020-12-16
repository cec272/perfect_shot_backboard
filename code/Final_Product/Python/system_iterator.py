# Harrison Hidalgo
# ECE 5725 - Final Project
#

import iteration_variables as I_V
import numpy as np
import transformations
from path_tracker import *
import Geometric_Variables as GV

def system_iterator(e_b,r_B0,r_GB0,h,x_ball_init,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop):
	# Cycle through different theta, psi, phi, and B positions
	number_of_divisions = 20
	# Create vectors to iterate through
	#theta_iter = np.linspace(I_V.theta_min,I_V.theta_max,number_of_divisions)
	phi_iter   = np.linspace(I_V.phi_min,I_V.phi_max,number_of_divisions)
	psi_iter   = np.linspace(I_V.psi_min,I_V.psi_max,number_of_divisions)
	B_iter     = np.linspace(I_V.B_min,I_V.B_max,number_of_divisions)
	does_it_work=False
	hits,location,velocity_at_location = path_tracker(h,x_ball_init,GV.front,GV.up,GV.W_of_backboard,GV.H_of_backboard,GV.T_of_backboard,0.06542,GV.center_hoop,GV.r_B0)
	if hits:
		print('this may work')
		need_vel_direction = (center_hoop-location)/np.linalg.norm(center_hoop-location)
	else:
		does_it_work = False
		working_orientations = 0
		return does_it_work,working_orientations
	theta = 0
	# For storing results
	working_orientations = []
	for j in range(0,number_of_divisions):# phi
		for k in range(0,number_of_divisions):# psi
			for l in range(0,number_of_divisions):
				phi   = phi_iter[j]
				psi   = psi_iter[k]
				new_up = transformations.transform_baseboard_to_backboard(theta,phi,psi,up)
				new_front = transformations.transform_baseboard_to_backboard(theta,phi,psi,front)
				need_vel_direction = (center_hoop-location+B_iter[l]*e_b)/np.linalg.norm(center_hoop-location+B_iter[l]*e_b)
				velocity_after = collision(new_up,new_front,velocity_at_location,1)
				if np.dot(velocity_after,need_vel_direction) > 0.95*np.linalg.norm(need_vel_direction)+np.linalg.norm(velocity_after):
					working_orientations = np.array([[theta_iter[i]],[phi_iter[j]],[psi_iter[k]]])
					does_it_work=True
					break
	#working_orientations=np.reshape(working_orientations,(4,len(working_orientations)/4))
	return does_it_work,working_orientations

def collision(up,front,velocity,restitution):
    import numpy as np
    # Calculate the left component of the backboard.
    left = np.cross(front,up);
    # Decompose velocity into backboard coordinates.
    up_comp = np.dot(velocity,up)/np.dot(up,up)*up;
    front_comp = np.dot(velocity,front)/np.dot(front,front)*front;
    left_comp = np.dot(velocity,left)/np.dot(left,left)*left;
    # Put velocity back together.
    velocity_new = up_comp-front_comp*restitution+left_comp;
    return velocity_new
