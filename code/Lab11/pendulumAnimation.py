# This file contains everything they need for the animation and shouldn't need to be changed at all

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import pendulumParam as P


class pendulumAn:
    '''
        Create pendulum animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        plt.axis([-3*P.ell,3*P.ell, -0.1, 3*P.ell]) # Change the x,y axis limits
        plt.plot([-2*P.ell,2*P.ell],[0,0],'b--')    # Draw a base line
        plt.xlabel('z(m)')
        plt.title('Animation')

        # Draw pendulum is the main function that will call the functions:
        # drawCart, drawCircle, and drawRod to create the animation.
    def drawPendulum(self, state_input):
        # Process inputs to function
        z = state_input[0]      # Horizontal position of cart [m]
        theta = state_input[2]  # Angle of pendulum [radians]

        self.drawCart(z)
        self.drawCircle(z, theta)
        self.drawRod(z, theta)
        self.ax.axis('equal') # This will ensure that the image does not distort
        # After each function has been called, initialization is over.
        plt.show()
        if self.flagInit == True:
            self.flagInit = False

    def drawCart(self, z):
        x = z-P.w/2.0   # x coordinate
        y = P.gap       # y coordinate
        xy = (x, y)     # Bottom left corner of rectangle

        # When the class is initialized, a rectangle patch object will be
        # created and added to the axes. After initialization, only that 
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(xy,
                P.w,P.h, fc = 'blue', ec = 'black'))
            self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        else:
            self.handle[0].set_xy(xy)         # Update patch

    def drawRod(self, z, theta):
        X = [z, z+P.ell*np.sin(theta)]                      # X data points
        Y = [(P.gap+P.h), (P.gap+P.h)-P.ell*np.cos(theta)]  # Y data points

        # When the class is initialized, a line object is created and
        # added to the axes.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X,Y,lw = 1, c = 'black')
            self.handle.append(line)
        else:
            self.handle[2].set_xdata(X)                 # Update the line
            self.handle[2].set_ydata(Y)
            
    def drawCircle(self, z, theta):
        x = z+(P.ell+P.radius)*np.sin(theta)            # x coordinate
        y = (P.gap+P.h)-(P.ell+P.radius)*np.cos(theta)  # y coordinate
        xy = (x,y)                                      # Center of circle

        # When the class is initialized, a CirclePolygon patch object is
        # created and added to the axes. 
        if self.flagInit == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(mpatches.CirclePolygon(xy,
                radius = P.radius, resolution = 15,
                fc = 'limegreen', ec = 'black'))
            self.ax.add_patch(self.handle[1])  # Add the patch to the axes
        else:
            self.handle[1]._xy=xy



# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = pendulumAn()     # Create Animate object
    z = 0.0                         # Position of cart, m
    theta = np.pi                   # Angle of pendulum, rads
    simAnimation.drawPendulum([z, 0, theta, 0])  # Draw the pendulum
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()
