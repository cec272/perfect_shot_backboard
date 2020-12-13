# Harrison Hidalgo
# ECE 5725 - Final Project
# This function will update the current interface.

import pygame
from pygame.locals import *
import os
from interface_variables import *


def interface_creator(X,Y,Z,X_d,Y_d,Z_d,th1,th2,th3,theta,phi,psi,cov_x,cov_y,cov_z,cov_x_d,cov_y_d,cov_z_d,cov_th1,cov_th2,cov_th3,cov_theta,cov_phi,cov_psi,state):
   # Inputs
    my_values={str(X):(mean_1,row_4),str(Y):(mean_1,row_5),str(Z):(mean_1,row_6),str(X_d):(mean_1,row_7),str(Y_d):(mean_1,row_8),str(Z_d):(mean_1,row_9),str(th1):(mean_1,row_4),str(th2):(mean_1,row_5),str(th3):(mean_1,row_6),str(theta):(mean_1,row_7),str(phi):(mean_1,row_8),str(psi):(mean_1,row_9)}
    my_labels_large={'BALL':(size_x/3,row_2),'BACKBOARD':(size_x*2/3,row_2),'STOP':(size_x/3,row_10),'STATUS:':(size_x*4/6,row_10),str(state):(size_x*5/6,row_10)}
    # Check for button press
    for event in pygame.event.get():
        if(event.type is MOUSEBUTTONUP):
            pos = pygame.mouse.get_pos()
            x,y=pos
            if (y>size_y*4/5) and (y<size_y*9/10) and (x>size_x/10) and (x<size_x/2):
                run = False
    ## Plotting
    screen.fill(black)
    pygame.draw.rect(screen,red,stop_bar)
    
    # Labels_1
    for my_text,text_pos in my_labels_1.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    # Labels_2
    for my_text,text_pos in my_labels_2.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    # Values
    for my_text,text_pos in my_values.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    # Labels_Large
    for my_text,text_pos in my_labels_large.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    pygame.display.flip()
    run = True
    return run
