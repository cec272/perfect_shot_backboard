# Harrison Hidalgo and Chris Chan
# ECE 5725 - Final Project
# This is the main program for our system.

######## BEFORE WHILE LOOP #########

## Import modules
import time
import RPi.GPIO
import pygame
from pygame.locals import *
import os
import math
import Geometric_Variables
import Physcial_Variables as p
from interface_variables import *
from system_iterator import *
import weights
from scipy.stats.distributions import chi2
import csv
import covariance_small_enough import *

## Create constants
current_time = time.time()
start_time   = time.time()
run_time     = 30
state        = 0
motor_1_reset= False
motor_2_reset= False
motor_3_reset= False
ball_radius = ########
min_covariances=[ball_radius/4,ball_radius/4,ball_radius/4,.076,.076,.076]

lam0 = chi2.ppf(0.95,0)
N = 10 # Number of samples
run = True
n_sig = 3; # Number of sigma points

with open('run.csv','w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(run)

# Image addresses
image1 = 'image1.csv'
time1  = 'time1.csv'

image2 = 'image2.csv'
time2  = 'time2.csv'

image3 = 'image3.csv'
time3  = 'time3.csv'

# GPIO Pins
MOTOR_11 = 
MOTOR_12 = 
MOTOR_21 = 
MOTOR_22 = 
MOTOR_31 = 
MOTOR_32 = 

# Limit switch pins
LS_1 = 
LS_2 = 
LS_3 = 

## Pygame initialization
pygame.init()
pygame.mouse.set_visible(False)
screen = pygame.display.set_mode((320,240))
my_font = pygame.font.Font(None,20)

## Interrupts
def LIMIT_SWITCH_1(channel):
    global motor_1_reset
    motor_1_reset = True

def LIMIT_SWITCH_2(channel):
    global motor_2_reset
    motor_2_reset = True

def LIMIT_SWITCH_3(channel):
    global motor_3_reset
    motor_3_reset = True

GPIO.add_event_detect(LS_1,GPIO.RISING,callback=LIMIT_SWITCH_1,bouncetime=300)
GPIO.add_event_detect(LS_2,GPIO.RISING,callback=LIMIT_SWITCH_2,bouncetime=300)
GPIO.add_event_detect(LS_3,GPIO.RISING,callback=LIMIT_SWITCH_3,bouncetime=300)

######## START WHILE LOOP ##########
while (current_time-start_time) < run_time and run:
    ## Collect measurements
    # Collect image from csv
    I1 = np.loadtxt(open(image1, "rb"), delimiter=",")
    X_state_1 = I1[0:6]
    t_1 = I1[6]
    I2 = np.loadtxt(open(image2, "rb"), delimiter=",")
    T2 = np.loadtxt(open(time2, "rb"), delimiter=",")
    I3 = np.loadtxt(open(image3, "rb"), delimiter=",")
    T3 = np.loadtxt(open(time3, "rb"), delimiter=",")
    # Collect imu data from csv

    ## Process measurements
    for i in range(0,N):
        SR_SPF_Ball(x_0,S_x0,S_v0,S_n0,n_sig,weights,p,measurement)
        # Process imu data

    ## Check covariances
    if covariances_small_enough(S,min_covariances) and (state == 1):
        state = 2

    # Update interface
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
    
    ## Update state of system
    if covariance_small_enough and (state == 1):
        state = 2

## Process according to state
# State 0: Reset system.
# State 1: Collect measurements.
# State 2: Find position to move to.
# State 3: Move backboard to position.
# State 4: Pause until hit.

    if state == 0:
        # Move motors back
        if motor_1_reset == False:
            #####Move motor 1 back one step ####
        if motor_2_reset == False:
            #####Move motor 2 back one step ####
        if motor_3_reset == False:
            #####Move motor 3 back one step ####
        if motor_1_reset and motor_2_reset and motor_3_reset:
            state = 1
    elif state == 1:
        # Do nothing
    elif state == 2:
        x_ball_init = locate_ball()
        working_orientations = system_iterator(iteration_vals,e_b,r_B0,rGB0,h,x_ball_init,'ball_calc',p,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop)
        theta1_goal,theta2_goal,theta3_goal = orientation_selector(th_1_current,th_2_current,th_3_current,working_orientations)
        state = 3
    elif state == 3:
        # Use controller to find backboard movement
        # Move backboard
    elif state == 4:
        # Do nothing until ball hits
    with open('run.csv','w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(run)
