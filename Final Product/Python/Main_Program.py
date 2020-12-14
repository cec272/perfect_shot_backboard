# Harrison Hidalgo and Chris Chan
# ECE 5725 - Final Project
# This is the main program for our system.

######## BEFORE WHILE LOOP #########

## Import modules
import time
import board
import digitalio
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
from covariance_small_enough import *
from motor_control import moveStepper
import camera_transmitter

## Start other scripts
os.system('sudo python3 camera_transmiter.py &')
os.system('sudo python3 imu_transmiter.py &')
os.system('sudo python3 interface_creator.py &')

## Create constants
current_time = time.time()
start_time   = time.time()
run_time     = 30
state        = 0
motor_1_reset= False
motor_2_reset= False
motor_3_reset= False
ball_radius  = camera_transmitter.r_ball/1000
min_covariances=[ball_radius/4,ball_radius/4,ball_radius/4,.076,.076,.076]

lam0 = chi2.ppf(0.95,0)
N = 10 # Number of samples
run = True
n_sig = 3; # Number of sigma points


with open('run.csv','w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(run)

# Addresses
image = 'image.csv'

interface_states = 'interface_states.csv'

imu = 'image3.csv' #### make these for imus
time_imu  = 'time3.csv'

# Motor GPIO pins and coils
coils = (
    # motor 1
    digitalio.DigitalInOut(board.D19),  # A1
    digitalio.DigitalInOut(board.D26),  # A2
    digitalio.DigitalInOut(board.D6),   # B1
    digitalio.DigitalInOut(board.D13),  # B2
    # motor 2
    digitalio.DigitalInOut(board.D12),  # A1
    digitalio.DigitalInOut(board.D16),  # A2
    digitalio.DigitalInOut(board.D20),  # B1
    digitalio.DigitalInOut(board.D21),  # B2
    # motor 3
    digitalio.DigitalInOut(board.D27),  # A1
    digitalio.DigitalInOut(board.D17),  # A2
    digitalio.DigitalInOut(board.D22),  # B1
    digitalio.DigitalInOut(board.D23),  # B2
)

for coil in coils:
    coil.direction = digitalio.Direction.OUTPUT
    
motor1 = stepper.StepperMotor(coils[0], coils[1], coils[2], coils[3], microsteps=None)
motor2 = stepper.StepperMotor(coils[4], coils[5], coils[6], coils[7], microsteps=None)
motor3 = stepper.StepperMotor(coils[8], coils[9], coils[10], coils[11], microsteps=None)

# Limit switch GPIO pins
LS_1 = 5
LS_2 = 14
LS_3 = 15

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
    with open('run.csv','r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            camera_measurements[i,:] = row[:6]
            t_camera[i] = row[6]
    # Collect imu data from csv

    ## Process measurements
    for i in range(0,3):
        measurement = camera_measurements[i,:]
        eval('t_' + str(i) + '=t_camera[i]')
        eval('x_hat,S_xk = SR_SPF_Ball(x_hat,S_xk,S_v0,S_n0,n_sig,weights,p,measurement,t_' str(i) '-t_last)')
        eval('t_last = t' + str(i))
    for i in range(0,N):
        #Process imu measurements
    
    # Send measurements to interface
    with open(interface_states,'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(np.concatenate(x_hat,S_xk,state))
        
    ## Check covariances
    if covariances_small_enough(S,min_covariances) and (state == 1):
        state = 2

## Process according to state
# State 0: Reset system.
# State 1: Collect measurements.
# State 2: Find position to move to.
# State 3: Move backboard to position.
# State 4: Pause until hit.

    if state == 0:
        # Check which limit switches have been reached
        motors = []
        direc = []
        if motor_1_reset == False:
            motors.append(motor1)
            direc.append(stepper.BACKWARD)
        if motor_2_reset == False:
            motors.append(motor2)
            direc.append(stepper.FORWARD)
        if motor_3_reset == False:
            motors.append(motor3)
            direc.append(stepper.FORWARD)
        # Move motors that haven't hit their limit
        moveStepper(motors, [1]*len(motors), direc)
        # Move onto next state once all motorrs have hit their limits
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
        
        state = 4
        end_state_3_time = time.time()
    elif state == 4:
        # Do nothing until ball hits
        if (time.time()-end_state_3_time) > 5:
            state = 0
    with open('run.csv','r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            run = row[0]
            
