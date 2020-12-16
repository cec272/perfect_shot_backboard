# This file contains the scipy.integrate function called in the runSimulation.py file. 
# I also added the force contraint function here and in the kalman filter (when it's being called)

import numpy as np
from signalGenerator import signalGen

class pendulumCnt:

    def __init__(self, u_min=None, u_max=None, m1=None, m2=None, ell=None, b=None, g=None,
                 Ts=None,angle_limit=None, K=None, param=None, zref=None):
        #Initial state conditions
        
        # Sensor noise characteristics
        if not param is None:
            self.sig_z = param.sig_z
            self.sig_zdot = param.sig_zdot
            self.sig_theta = param.sig_theta
            self.sig_thetadot = param.sig_thetadot
        
        # Min motor constants, N
        if not u_min is None:
            self.u_min = u_min
        elif not param is None:
            self.u_min = param.u_min
            
        # Max motor constants, N
        if not u_max is None:
            self.u_max = u_max
        elif not param is None:
            self.u_max = param.u_max
        
        # Mass of the pendulum, kg
        if not m1 is None:
            self.m1 = m1          
        elif not param is None:
            self.m1 = param.m1
        
        # Mass of the cart, kg
        if not m2 is None:            
            self.m2 = m2
        elif not param is None:
            self.m2 = param.m2
       
         # Length of the rod, m
        if not ell is None:    
            self.ell = ell
        elif not param is None:
            self.ell = param.ell       
            
        # Damping coefficient, Ns 
        if not b is None:
            self.b = b                   
        elif not param is None:
            self.b = param.b            
                
        # Gravity constant
        if not g is None:
            self.g = g            
        elif not param is None:
            self.g = param.g          
            
        # sample rate of controller
        if not Ts is None:    
            self.Ts = Ts          
        elif not param is None:
            self.Ts = param.Ts        
            
        if not K is None:
            self.K = K
        elif not param is None:
            if 'K' in param.__dict__.keys():
                self.K = param.K
            else:
                self.K= None
        else:
            self.K = None
            
        self.angle_limit = np.pi*2.0
        
        if not zref is None:
            self.zref=zref
        else:
            self.reference = signalGen(amplitude=.5, frequency=0.05, y_offset=0)
            self.zref= self.reference.square

    ####################################################
    #               scipy.integrate
    ####################################################
    def cartpendfunc(self,y,t):
        #unpack the state
        z, zdot, theta, thetadot = y

        #don't allow theta to exceed 2pi
        #need this so that equivalnent angles don't cause a difference in 
        #in the reference input
        theta = self.limitTheta(theta)

        #Get reference inputs from signal generators
        zref = self.zref(t)      #control cart z location
        
        #Add sensor noise to current state values
        curr_state = np.array([[z], [zdot], [theta], [thetadot]])   #current state
        n = np.array([[np.random.normal(0, self.sig_z)], [np.random.normal(0, self.sig_zdot)], [np.random.normal(0, self.sig_theta)], [np.random.normal(0, self.sig_thetadot)]])
        n_curr_state = np.add(curr_state, n)
        
        #calculting the new control force
        des_state = np.array([zref, [0.0], [np.pi], [0.0]])         #desired state
        error = des_state - n_curr_state

        #Feedback control. If there's no gain it assigns the control to be 0
        if not self.K is None: # a controller has been implemented
            u_des = np.dot(self.K, error) # multiply the error by the control gain
            if abs(u_des) <= self.u_min:
                self.u = 0
            elif abs(u_des) >= self.u_max:
                self.u = np.sign(u_des)*self.u_max
            else:
                self.u = u_des
        else:
            self.u = 0
        
        #simplifications for the calculations - constants
        Sy = np.sin(theta)
        Cy = np.cos(theta)
        D = self.m1*self.ell*self.ell*(self.m2+self.m1*(1.0-Cy*Cy))

        #calculating state values at the current time step
        ydot0 = zdot
        ydot1 = (1.0/D)*(-self.m1*self.m1*self.ell*self.ell*self.g*Cy*Sy + self.m1*self.ell*self.ell*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) + self.m1*self.ell*self.ell*(1.0/D)*self.u
        ydot2 = thetadot
        ydot3 = (1.0/D)*((self.m1+self.m2)*self.m1*self.g*self.ell*Sy    - self.m1*self.ell*Cy*      (self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) - self.m1*self.ell*Cy*(1.0/D)*self.u
        dydt = [ydot0, ydot1, ydot2, ydot3]
         
        return dydt

    ####################################################
    #              Extra Functions
    ####################################################
    def limitTheta(self, angle):
        if abs(angle) > self.angle_limit:
            rem = (abs(angle) % self.angle_limit)*np.sign(angle)
        elif abs(angle) <= self.angle_limit:
            rem = angle
        return rem
    