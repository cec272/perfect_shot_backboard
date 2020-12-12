# Harrison Hidalgo
# ECE 5725 - Final Project
# This function will update the current interface.

import pygame
from pygame.locals import *
import os

def interface_update(X,Y,Z,X_d,Y_d,Z_d,th1,th2,th3,theta,phi,psi,cov_x,cov_y,cov_z,cov_x_d,cov_y_d,cov_z_d,cov_th1,cov_th2,cov_th3,cov_theta,cov_phi,cov_psi):
    ### Move these to their own module
    size_x = 320
    size_y = 240
    black  = 0,0,0
    white  = 255,255,255
    red    = 255,0,0
    
    vars_1 = size_x/5
    vars_2 = size_x*3/5

    mean_1 = size_x*3/10
    cov_1  = size_x*4/10
    mean_2 = size_x*4/5
    cov_2  = size_x*9/10

    # Perminent labels
    my_labels_1={'Mean':(mean_1,size_y/5),'Covariance':(cov_1,size_y/5),'X:'(),'Y:':(),'Z:':(),'X_d:','Y_d:':(),'Z_d:':(),'STOP':(),'Mean  Covariance':(),'th1:':(),'th2:':(),'th3:':(),'theta:':(),'phi:':(),'psi:':()}
    my_labels_large={'BALL':(),'BACKBOARD':(),'STOP':(),'STATUS:':(),str(state):()}
    # Inputs
    my_values={str(X):(),str(Y):(),str(Z):(),str(X_d):(),str(Y_d):(),str(Z_d):(),str(th1):(),str(th2):(),str(th3):(),str(theta):(),str(phi):(),str(psi):()}
    
    ###
    
    
    stop_bar = pygame.Rect(size_x/10,size_y*8/10,sixe_x*4/10,size_y/10)
    
    screen.fill(black)
    pygame.draw.rect(screen,red,stop_bar)
    
    for my_text,text_pos in my_labels.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)


    

