# Harrison Hidalgo
# ECE 5725 
# For testing different functions.
import A_ball 
import b_ball 
import ball_calc
import numpy as np
import covariances_small_enough 
import measurement_validation 
import orientation_selector
import runge_kutta
import transformations
import math
import path_tracker
from system_iterator import *
import SR_SPF_Ball
import find_angles
import Geometric_Variables as GV

h = 0.05
ffun = 'ball_calc.ball_calc'
front = np.transpose(np.array([1,0,0]))
up = np.transpose(np.array([0,1,0]))
W_of_backboard = 5
H_of_backboard = 5
T_of_backboard = 1
r_of_ball = 0.5
center_hoop = np.array([1,1,0])
backboard = np.array([0,0,0])

theta = math.pi/2
phi = math.pi/2
psi = math.pi/2
x = np.array([[1],[0],[0]])
x_new = transformations.transform_baseboard_to_backboard(theta,phi,psi,x)

n_sig = 3

#measurement = np.array([1,1,1,1,np.random.rand*5,np.random.rand*-2])
P_x = np.identity(6)*0.1+0.0001
P_v = np.identity(6)*0.1+0.0001
P_n = np.identity(6)*0.1+0.0001

S_x0 = np.linalg.cholesky(P_x)
S_v0 = np.linalg.cholesky(P_v)
S_n0 = np.linalg.cholesky(P_n)

x_ball_init = np.array([0.229, 0.189, 0.329,0,0,0])

print(system_iterator(GV.e_b,GV.r_B0,GV.rGB0,h,x_ball_init,GV.front,GV.up,GV.W_of_backboard,GV.H_of_backboard,GV.T_of_backboard,0.06542,GV.center_hoop))
#print(path_tracker.path_tracker(h,x_ball_init,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,backboard))
#print(SR_SPF_Ball.SR_SPF_Ball(x_ball_init,S_x0,S_v0,S_n0,n_sig,measurement,h))

A = [1,2,3]
B = [4,5,6]
C = np.append(transformations.transform_camera_to_baseboard(A),transformations.transform_camera_to_baseboard(B))
#print(C) 

theta = 0 
phi = 0
psi = 0 
r_B = .2
#print(find_angles.find_angles(theta,phi,psi,r_B))

#print(np.diag(np.matmul(S_xk,np.transpose(S_xk))))
