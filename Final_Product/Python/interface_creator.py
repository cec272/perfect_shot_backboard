# Harrison Hidalgo
# ECE 5725 - Final Project
# This function will update the current interface.

import pygame
from pygame.locals import *
import os
from interface_variables import *
import numpy as np
import motor_control
import board
import digitalio
from adafruit_motor import stepper
import time
import os
import csv

#os.putenv('SDL_VIDEODRIVER', 'fbcon') # Display on piTFT
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV', 'TSLIB') # Track mouse clicks on piTFT
os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

### Run fix touchscreen just in case ###
os.system('/home/pi/lab2_files_f20/fix_touchscreen.sh')

## Pygame initialization
pygame.init()
pygame.mouse.set_visible(False)
screen = pygame.display.set_mode((320,240))
my_font = pygame.font.Font(None,20)

# motor variables
coils = motor_control.coils
motor1 = motor_control.motor1
motor2 = motor_control.motor2
motor3 = motor_control.motor3

# loop variables
run = True
start_time = time.time()
current_time = time.time()
end_time = 30

# writing data
csv_interface = 'interface_states.csv'
csv_run = 'run.csv'

# main loop
while run and ((current_time-start_time) < end_time):
    holder = np.zeros((1,13))
    with open(csv_interface,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            holder = row[:]
    X,Y,Z,X_d,Y_d,Z_d = holder[0, 0:6]
    P = np.matmul(np.transpose(holder[0, 6:12]), holder[0, 6:12])
    cov_x,cov_y,cov_z,cov_x_d,cov_y_d,cov_z_d = np.diag(P)
    th1=0
    th2=0
    th3=0
    theta=0
    phi=0
    psi=0
    cov_th1=0
    cov_th2=0
    cov_th3=0
    cov_theta=0
    cov_phi=0
    cov_psi=0
    state = state_calc(holder[0, 12])
    # Update interface
    my_values_1={str(X):(mean_1,row_4),str(Y):(mean_1,row_5),str(Z):(mean_1,row_6),str(X_d):(mean_1,row_7),str(Y_d):(mean_1,row_8),str(Z_d):(mean_1,row_9),str(th1):(mean_2,row_4),str(th2):(mean_2,row_5),str(th3):(mean_2,row_6),str(theta):(mean_2,row_7),str(phi):(mean_2,row_8),str(psi):(mean_2,row_9)}
    my_values_2={str(cov_x):(cov_1,row_4),str(cov_y):(cov_1,row_5),str(cov_z):(cov_1,row_6),str(cov_x_d):(cov_1,row_7),str(cov_y_d):(cov_1,row_8),str(cov_z_d):(cov_1,row_9),str(cov_th1):(cov_2,row_4),str(cov_th2):(cov_2,row_5),str(cov_th3):(cov_2,row_6),str(cov_theta):(cov_2,row_7),str(cov_phi):(cov_2,row_8),str(cov_psi):(cov_2,row_9)}
    my_labels_large={'BALL':(size_x/3,row_2),'BACKBOARD':(size_x*17/24,row_2),'STOP':(size_x*3/10,row_10),'STATUS:':(size_x*4/6,row_10),state:(size_x*5/6,row_10)}
    # Check for button press
    for event in pygame.event.get():
        if(event.type is MOUSEBUTTONUP):
            pos = pygame.mouse.get_pos()
            x,y=pos
        if (y>size_y*4/5) and (y<size_y*9/10) and (x>size_x/10) and (x<size_x/2): # stop button pressed
            run = False
            motor1.release() # release all motors
            motor2.release()
            motor3.release()
        
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
    with open(csv_run,'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(run)
        
    # Update time
    current_time = time.time()

def state_calc(state):
    if state==0:
        script = 'RESET'
    elif state==1:
        script = 'TRACKING'
    elif state==2:
        script = 'CALCULATING'
    elif state==3:
        script = 'MOVING'
    elif script==4:
        script = 'HOLDING'
    return script
