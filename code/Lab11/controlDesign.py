#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 20:30:24 2020

@author: christopher
"""
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
import pendulumParam as P
import control as ctl

# get state space matrix
A = P.A
B = P.B
C = P.C
D = P.D

# calculate the eigenvalues
E = LA.eigvals(A)

# desired closed loop eigenvalues
P = np.array([-1, -2, -3, -4])
# P = np.array([-3, -4, -5, -6])

# sets the param value "K"
K = ctl.place(A, B, P)

# check for closed loop eigenvalues
Acl = A - np.matmul(B,K)
Ecl = LA.eigvals(Acl)

# create open loop system
ssol = ctl.StateSpace(A, B, C, D)

# create closed loop system
sscl = ctl.StateSpace(Acl, B, C, D)

# solve for Kr
Kdc = ctl.dcgain(sscl);
Kr = 1/Kdc[0]

# create scaled input closed loop system
syscl_scaled = ctl.StateSpace(Acl, B*Kr, C, D)

# run the sisotool
t, y = ctl.step_response(syscl_scaled)

# plot the step response
fig, axs = plt.subplots(2)
fig.suptitle('Step Response, Poles:' + str(P))
axs[0].plot(t, y[0])
axs[0].set_ylabel('Amplitude')
axs[0].set_title('Response of z(t)')
axs[1].plot(t, y[2])
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Amplitude')
axs[1].set_title('Response of theta(t)')