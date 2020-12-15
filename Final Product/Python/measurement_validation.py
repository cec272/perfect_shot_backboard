# Harrison Hidalgo
# ECE 5725 - Final Project
# This program confirms or rejects measurements. Returns true if accepts, false if it rejects.

def measurement_validation(measurement,P,del_t,lam0,R,x_hat):
    import numpy as np
    # Find innovation and covariance
    H = np.identity(6)
    innovation = measurement - np.matmul(H,x_hat)
    S = np.matmul(np.matmul(H,P),np.transpose(H))+R
    # Find normalized innovation squared
    lam = np.matmul(np.matmul(np.transpose(innovation),np.linalg.inv(S)),innovation)
    # Accept or reject measurement
    if lam < lam0:
        acceptance = True
    else:
        acceptance = False
    return acceptance
