# RUN ME
# This file gives the initial condition to the scipy.integrat.odeint function 
# and plots the resulting state outputs at each time step in an animation and
# on a plot that compares the actual output with the reference input

import numpy as np
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import scipy.integrate
from scipy.integrate import odeint
import pendulumParam as P
from pendulumControllerDynamics import pendulumCnt
from pendulumAnimation import pendulumAn
from plotDataZ import plotData      
from signalGenerator import signalGen

#Initialize and rename for convenience
ref = signalGen(amplitude=0.5, frequency= 0.05, y_offset=0) 
ctrl = pendulumCnt(param=P,zref=ref.square)

plt.close('all')
animation = pendulumAn()

#performs Runga-Kutta to get state values at each time step (equivalent of ode45 in MATLAB)
length = int((P.t_end-P.t_start)/P.Ts)              #The number of time steps over the time interval
t_array = np.linspace(P.t_start, P.t_end, length)   #The time vector that is passed into the ode
STATES = scipy.integrate.odeint(ctrl.cartpendfunc, [P.z0, P.zdot0, P.theta0, P.thetadot0], t_array) #Solve non-linear dynamics

#wrap angle from 0--2*PI
for x in  STATES:
    x[2] = x[2]%(2*np.pi)

#animation
i = 0
reference = np.zeros((length,1))
while i < len(t_array):  
    #calculate the reference value
    t_curr = t_array[i]                 #Current time step
    z_ref = ctrl.zref(t_curr)           #Get cart z location
    reference[i:(i+100)] = z_ref        #Which reference value is used

    #update animation and data plots
    animation.drawPendulum(STATES[i,:]) #Animate
    plt.pause(0.001)                    #Pause while plot updates
    i = i + 100                         #speeds up the simulation by not plotting all the points
 
# Plot how closely the actual performance of the pendulum on a cart matched the desired performance
dataPlot = plotData()                       #initializes plot
dataPlot.Plot(t_array, reference, STATES)   #plots the data
