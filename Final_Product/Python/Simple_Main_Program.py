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
from measurement_validation import *
import weights
import csv
from SR_SPF_Ball import SR_SPF_Ball
from covariances_small_enough import *
from motor_control import gearToStep
from motor_control import moveStepper
from adafruit_motor import stepper
from picamera.array import PiRGBArray
from picamera import PiCamera
from EKF_filter import *
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

run_status = 'run.csv'

## Output to this file about the run status
run = True
with open(run_status,'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow([run])

## Start other scripts
os.system('python3 camera_transmitter.py &')
#os.system('python3 interface_creator.py &')

## Create constants
current_time = time.time()
start_time   = time.time()
reset_time   = 10 # time allowed between detected and system reset
run_time     = 600
state        = 0
motor_1_reset= False
motor_2_reset= False
motor_3_reset= False

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

motor1Angle = 0
motor2Angle = 0
motor3Angle = 0

angleStepSize = math.pi/10

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
t_last = None
ball_seen = False
thresh = 0.2

## EKF Variables 
Q = np.identity(6)*1        # covariance of state noise
R = np.identity(6)*10       # covariance of measurement noise
Pk2k2 = np.identity(6)*100  # initialize the state covariance matrix

######## START WHILE LOOP ##########
while (current_time-start_time) < run_time and run:
    ## Update loop variable
    current_time = time.time()
    
    ## Collect measurements
    # Collect image from csv
    camera_measurements = np.zeros((3,7))
    with open(image_data,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        row_num = 0
        for row in csvreader:
            if row_num >= 3:
                row_num = 0
            camera_measurements[row_num,:] = row
            row_num = row_num + 1
    ## Process measurements
    for i in range(0,3):
        # Check if there's a ball
        if (np.all(x_hat == 0) and not np.all(camera_measurements == 0)):
            ball_seen = True
    
    x_hat = np.array([[camera_measurements[1,0]]])
    ball_in_sight = np.array([camera_measurements[1,6]])
    
    # Check if there's a ball
    if ball_seen and state == 1:
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
        #print('resetting motors!!')
        # Move onto next state once all motorrs have hit their limits
        if motor_1_reset and motor_2_reset and motor_3_reset:
            motor1Angle = 0
            motor2Angle = 0
            motor3Angle = 0
            state = 1
            time.sleep(1)
    elif state == 2:
        theta1_goal = 0
        theta2_goal = 0
        theta3_goal = 0
        if x_hat[0,0] >= g.W_of_backboard/3:
            if motor3Angle <= math.pi/4-angleStepSize:
                theta3_goal = angleStepSize
                motor3Angle = motor3Angle + angleStepSize
            if motor1Angle >= math.pi/12+angleStepSize:
                theta1_goal = -angleStepSize
                motor1Angle = motor1Angle - angleStepSize
        elif x_hat[0,0] < g.W_of_backboard/3:
            if motor1Angle <= math.pi/4-angleStepSize:
                theta1_goal = angleStepSize
                motor1Angle = motor1Angle + angleStepSize
            if motor3Angle >= math.pi/12+angleStepSize:
                theta3_goal = -angleStepSize
                motor3Angle = motor3Angle - angleStepSize
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
        if (time.time()-end_state_3_time) > reset_time:
            motor1.release()
            motor2.release()
            motor3.release()
            state = 0
        else:
            state = 1
        
    # Check if a stop signal has been flagged from other programs
    with open(run_status,'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            run = row[0]
GPIO.cleanup()
