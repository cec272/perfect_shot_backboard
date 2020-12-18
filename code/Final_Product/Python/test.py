
import time
import os
import pygame
from pygame.locals import *
from interface_variables import *

#os.putenv('SDL_VIDEODRIVER','fbcon')
os.putenv('SDL_FBDEV','/dev/fb0')
os.putenv('SDL_MOUSEDRV','TSLIB')
os.putenv('SDL_MOUSEDEV','/dev/input/touchscreen')

start_time = time.time()
current_time = time.time()
X=1
Y=2
Z=3
X_d=4
Y_d=5
Z_d=6
th1=7
th2=8
th3=9
theta=10
phi=11
psi=12
cov_x=13
cov_y=14
cov_z=15
cov_x_d=16
cov_y_d=17
cov_z_d=18
cov_th1=19
cov_th2=20
cov_th3=21
cov_theta=22
cov_phi=23
cov_psi=24
state = 25
run = True

pygame.init()
pygame.mouse.set_visible(False)
screen = pygame.display.set_mode((320,240))
my_font = pygame.font.Font(None,20)

while ((current_time-start_time) < 10) and run:
    # Inputs
    my_values_1={str(X):(mean_1,row_4),str(Y):(mean_1,row_5),str(Z):(mean_1,row_6),str(X_d):(mean_1,row_7),str(Y_d):(mean_1,row_8),str(Z_d):(mean_1,row_9),str(th1):(mean_2,row_4),str(th2):(mean_2,row_5),str(th3):(mean_2,row_6),str(theta):(mean_2,row_7),str(phi):(mean_2,row_8),str(psi):(mean_2,row_9)}
    my_values_2={str(cov_x):(cov_1,row_4),str(cov_y):(cov_1,row_5),str(cov_z):(cov_1,row_6),str(cov_x_d):(cov_1,row_7),str(cov_y_d):(cov_1,row_8),str(cov_z_d):(cov_1,row_9),str(cov_th1):(cov_2,row_4),str(cov_th2):(cov_2,row_5),str(cov_th3):(cov_2,row_6),str(cov_theta):(cov_2,row_7),str(cov_phi):(cov_2,row_8),str(cov_psi):(cov_2,row_9)}
    my_labels_large={'BALL':(size_x/3,row_2),'BACKBOARD':(size_x*17/24,row_2),'STOP':(size_x*3/10,row_10),'STATUS:':(size_x*4/6,row_10),str(state):(size_x*5/6,row_10)}
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
    for my_text,text_pos in my_values_1.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    for my_text,text_pos in my_values_2.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    # Labels_Large
    for my_text,text_pos in my_labels_large.items():
        text_surface = my_font.render(my_text,True,white)
        rect = text_surface.get_rect(center=text_pos)
        screen.blit(text_surface,rect)
    pygame.display.flip()
    current_time = time.time()
