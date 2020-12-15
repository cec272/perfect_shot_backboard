# Harrison Hidalgo and Christopher Chan
# ECE 5725 - Final Project
# 

import numpy as np
from runge_kutta import *
import Physical_Variables as p

def EKF_filter(x_hat_p,Pkk,measurement,Q,R,del_t,t0):
	F = np.array([[x_hat_p[3, 0]*np.asscalar(del_t),0,0,0,0,0],[0,x_hat_p[4, 0]*np.asscalar(del_t),0,0,0,0],[0,0,x_hat_p[5, 0]*np.asscalar(del_t),0,0,0],[0,0,0,0,0,0],[0,0,0,0,-p.g,0],[0,0,0,0,0,0]])
	G = np.identity(6)
	H = np.identity(6)
	# Predict
	t, x_hat_predict = runge_kutta(np.asscalar(del_t),t0,x_hat_p)
	Pk2k = np.matmul(np.matmul(F,Pkk),np.transpose(F)) + np.matmul(np.matmul(G,Q),np.transpose(G))
	# Kalman Gain
	K = np.matmul(np.matmul(Pk2k,H),np.linalg.inv(np.matmul(np.matmul(H,Pk2k),np.transpose(H))+R))
	# Update
	x_hat = x_hat_predict + np.matmul(K,(measurement-x_hat_predict))
	Pk2k2 = np.matmul(np.matmul((np.identity(6)-np.matmul(K,H)),Pk2k),np.transpose(np.identity(6))-np.matmul(K,H)) + np.matmul(np.matmul(K,R),np.transpose(K))
	return x_hat, Pk2k2
	
'''
Q = np.identity(6)*.5        # covariance of state noise
R = np.identity(6)*1       # covariance of measurement noise
Pk2k2 = np.identity(6)*100  # initialize the state covariance matrix
x_hat_p = np.transpose(np.array([3, 4, 4, .1, .2, -9]))
del_t = 0.01
t0 = 0
measurement = np.transpose(np.array([3.2, 4.1, 4.5, .3, .2, -10]))

test = EKF_filter(x_hat_p,Pk2k2,measurement,Q,R,del_t,t0)
print(test)
'''
