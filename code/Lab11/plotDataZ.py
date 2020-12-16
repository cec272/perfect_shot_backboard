# This file assumes a reference input for z (named x in the lectures) variable 
# and plots the z reference and z, as well as, the theta value over time. 
# It should not need to be changed by the students

import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing


class plotData:
    ''' 
        This class plots the time histories for the pendulum data.
    '''

    def __init__(self):
        #Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 2    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns

        #Create figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        #Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.zref_history = []  # reference position z_r
        self.z_history = []     # position z
        self.theta_history = [] # angle theta

        #create subplots
        self.ax[0].set(xlabel='time(s)', ylabel='z(m)', title='Performance')
        self.ax[1].set(xlabel='time(s)', ylabel='theta(deg)')

    def Plot(self, t, reference, states):
        '''
            Add to the time and data histories, and update the plots.
        '''
        #the time history of all plot variables
        self.time_history = t  # time
        self.zref_history = reference[:,0]  # reference base position
        self.z_history = states[:,0]  # base position
        self.theta_history = 180.0/np.pi*states[:,2]  # pendulum angle (converted to degrees)

        #the plots with associated histories 
        line1, = self.ax[0].plot(self.time_history, self.zref_history, label='Z-Reference')
        line2, = self.ax[0].plot(self.time_history, self.z_history, label='Z')
        line3, = self.ax[1].plot(self.time_history, self.theta_history, )

        plt.show()