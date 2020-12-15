# Harrison Hidalgo
# ECE 5725 - Final Project
# This program confirms or rejects measurements. Returns true if accepts, false if it rejects.

import numpy as np


def measurement_validation(measurement,del_t,lam0):
    # Find innovation and covariance
    H = calculate_H(del_t)
    innovation = measurement - np.matmul
    S = np.matmul(np.matmul(H,P),np.H.transpose())+R
    # Find normalized innovation squared
    lam = np.matmul(np.matmul(np.innovation.transpose(),np.linalg.inv(S)),innovation)
    # Accept or reject measurement
    if lam < lam0:
        return acceptance = True
    else:
        return acceptance = False
