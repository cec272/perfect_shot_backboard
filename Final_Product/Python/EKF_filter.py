# Harrison Hidalgo and Christopher Chan
# ECE 5725 - Final Project
# 

import numpy as np
from runge_kutta import *

def EKF_filter(x_hat_p,Pkk,measurement,Q,R,del_t,t0):
	F = [[x_hat_p[4]*del_t,0,0,0,0,0],[0,x_hat_p[5]*del_t,0,0,0,0],[0,0,x_hat_p[6]*del_t,0,0,0],[0,0,0,0,0,0],[0,0,0,0,-p.g,0],[0,0,0,0,0,0]]
	G = np.identity(6)
	H = np.identity(6)
	# Predict
	x_hat_predict = runge_kutta(del_t,t0,x_hat_p)
	Pk2k = np.matmul(np.matmul(F,Pkk),np.transpose(F)) + np.matmul(np.matmul(G,Q),np.transpose(G))
	# Kalman Gain
	K = np.matmul(np.matmul(Pk2k,H),np.linalg.inv(np.matmul(np.matmul(H,Pk2k),np.transpose(H))+R))
	# Update
	x_hat = x_hat_predict + np.matmul(K,(measurement-x_hat_predict))
	Pk2k2 = np.matmul(np.matmul((np.identity(6)-np.matmul(K,H)),Pk2k),np.transpose(np.identity(6))-np.matmul(K,H)) + np.matmul(np.matmul(K,R),np.transpose(K))
	return x_hat, Pk2k2
	


