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
import weights
from scipy.stats.distributions import chi2

## Create constants
current_time = time.time()
start_time   = time.time()
run_time     = 30
state        = 0
motor_1_reset= False
motor_2_reset= False
motor_3_reset= False
lam0 = chi2.ppf(0.95,0)

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
while (current_time-start_time) < run_time:
## Collect measurements
# Collect image from fifo
# Collect imu data from fifo

## Reject bad measurements



## Process measurements



## Check covariances
if covariance small enough:
    ball_state = []

## Update state of system
if covariance small enough:
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
    working_orientations = system_iterator()
    [theta1_goal,theta2_goal,theta3_goal] = orientation_selector()
elif state == 3:
    # Use controller to find backboard movenment
    # Move backboard
elif state == 4:
    # Do nothing until ball hits
    

