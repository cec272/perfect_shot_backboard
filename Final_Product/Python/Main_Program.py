# Harrison Hidalgo and Chris Chan
# ECE 5725 - Final Project
# This is the main program for our system.

######## BEFORE WHILE LOOP #########

## Import modules
import time
import board
import digitalio
import RPi.GPIO as GPIO
import pygame
from pygame.locals import *
import os
import math
import Geometric_Variables as g
import Physical_Variables as p
from interface_variables import *
from system_iterator import *
import weights
import csv
from SR_SPF_Ball import SR_SPF_Ball
from covariances_small_enough import *
from motor_control import gearToStep
from motor_control import moveStepper
from adafruit_motor import stepper
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import imutils
import transformations
import numpy as np

## Addresses
image_data = 'image.csv'
data = np.zeros((3,7))
with open(image_data,'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(data)

interface_states = 'interface_states.csv'

imu_data = 'imu.csv' #### make these for imus
data = np.zeros((3,4))
with open(imu_data,'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(data)

run_status = 'run.csv'

## Output to this file about the run status
run = True
with open(run_status,'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow([run])


## Start other scripts
os.system('python3 camera_transmitter.py &')
#os.system('python3 imu_transmitter.py &')
#os.system('python3 interface_creator.py &')

## Create constants
current_time = time.time()
start_time   = time.time()
run_time     = 30
state        = 0
motor_1_reset= False
motor_2_reset= False
motor_3_reset= False
ball_radius  = 0.06542 # CHANGE TO BALL LATER!!!!!!

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

degPerStep = 1.8 # number of degrees per step of the motor

# Limit switch GPIO pins
LS_1 = 5
LS_2 = 14
LS_3 = 15

# Gear Specs
motorGearTeeth = 16
leverGearTeeth = 36

## Interrupts
GPIO.setmode(GPIO.BCM)  
GPIO.setup(LS_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LS_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LS_3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
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

## Ball Measurement Variables
x_hat = 0
S_x0 = np.linalg.cholesky(np.identity(6))*10
S_v0 = np.linalg.cholesky(np.identity(6))
S_n0 = np.linalg.cholesky(np.identity(6))
n_sig = 3
t_last = None
min_covariances=[ball_radius/4,ball_radius/4,ball_radius/4,.076,.076,.076]
lam0 = 3.8415 # chi2.ppf(0.95,0)
N = 10 # Number of samples
n_sig = 3; # Number of sigma points
R = 1
delT = 0.1 # Discritizing time step for ball sim

######## START WHILE LOOP ##########
while (current_time-start_time) < run_time and run:
    ## Update loop variable
    current_time = time.time()
    
    ## Collect measurements
    # Collect image from csv
    camera_measurements = np.zeros((3,6))
    t_camera = np.zeros((3,1))
    with open(image_data,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        row_num = 0
        for row in csvreader:
            camera_measurements[row_num,:] = row[:6]
            t_camera[row_num] = row[6]
            row_num = row_num + 1
    '''        
    # Collect imu data from csv
    imu_measurements = np.zeros((3, 4))
    t_imu = np.zeros((3,1))
    with open(imu_data,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        row_num = 0
        for row in csvreader:
            camera_measurements[row_num,:] = row[:6]
            t_camera[row_num] = row[6]
            row_num = row_num + 1
    '''
    
    '''
    ## Process measurements
    for i in range(0,3):
        # Check if there's a ball
        print(x_hat)
        print(camera_measurements)
        if (np.all(x_hat == 0) and not np.all(camera_measurements == 0)):
            x_hat = camera_measurements [i,:]
            t_last = t_camera[i]
            S_xk = S_x0
        elif (not np.all(camera_measurements == 0)):
            measurement = camera_measurements[i,:]
            x_pos,S_xk_pos = SR_SPF_Ball(x_hat,S_xk,S_v0,S_n0,n_sig,measurement,t_camera[i]-t_last)
            P = np.matmul(S_xk_pos,np.transpose(S_xk_pos))
            if measurement_validation(measurement,P,t_camera[i]-t_last,lam0,R,x_hat):
                t_last = t_camera[i]
                x_hat = x_pos 
                S_xk = S_xk_pos
    #for i in range(0,N):
        #Process imu measurements
    
    # Check if there's a ball
    if (not np.all(camera_measurements == 0)):
        # Send measurements to interface
        with open(interface_states,'w') as csvfile:
            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(np.concatenate(x_hat,np.diag(np.matmul(S_xk,np.transpose(S_xk))),state))
        ## Check covariances
        if covariances_small_enough(S_xk,min_covariances) and (state == 1):
            state = 2
    '''    
    

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
        print('resetting motors!!')
        # Move onto next state once all motorrs have hit their limits
        if motor_1_reset and motor_2_reset and motor_3_reset:
            state = 1
    elif state == 1: ## CHANGE THIS !!!!!!!!!!!!!!!!!
        print('Woohooo PRETEND STATE 1 WAS REACHED')
        state = 2
    elif state == 2:
        x_ball_init = g.r_cam + x_hat
        working_orientations = system_iterator(g.e_b,g.r_B0,g.rGB0,delT,x_ball_init,g.front,g.up,g.W_of_backboard,g.H_of_backboard,g.T_of_backboard,ball_radius,g.center_hoop)
        for i in range(0,len(working_orientations)):
            does_it_work,theta_1,theta_2,theta_3 = find_angles(working_orientations[1,i],working_orientations[2,i],working_orientations[3,i],working_orientations[4,i])
            if does_it_work:
                break
        theta1_goal,theta2_goal,theta3_goal = orientation_selector(g.th1_10,g.th1_20,g.th3_30,np.array([theta_1,theta_2,theta_3]))
        state = 3
    elif state == 3:
        # Use controller to find backboard movement
        # Move backboard
        steps = []
        direc = []
        motorNumber = 1
        for angle in [theta1_goal, theta2_goal, theta3_goal]:
            # find desired angle based on gearing
            des_step = gearToStep(leverGearTeeth, motorGearTeeth, angle, degPerStep)
            # determine motor direction
            if motorNumber == 1:
                if des_step < 0:
                    steps.append(abs(des_step))
                    direc.append(stepper.BACKWARD)
                else:
                    steps.append(des_step)
                    direc.append(stepper.FORWARD)
            if motorNumber == 2 or motorNumber == 3:
                if des_step < 0:
                    steps.append(abs(des_step))
                    direc.append(stepper.FORWARD)
                else:
                    steps.append(des_step)
                    direc.append(stepper.BACKWARD)
            motorNumber = motorNumber + 1
        moveStepper([motor1, motor2, motor3], steps, direc)
        state = 4
        end_state_3_time = time.time()
    elif state == 4:
        # Do nothing until ball hits
        if (time.time()-end_state_3_time) > 5:
            state = 0
    with open(run_status,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            run = row[0]
            
    # Print stuff diagnosing
    print(state)

GPIO.cleanup()
