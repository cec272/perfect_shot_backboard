# Harrison Hidalgo and Chris Chan
# ECE 5725 - Final Project
# This is the main program for our system.

######## BEFORE WHILE LOOP #########

## Import libraries
import time

## Create constants
current_time = time.time()
start_time   = time.time()
run_time     = 30
state = 0


######## START WHILE LOOP ##########
while (current_time-start_time) < run_time:
## Collect measurements


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
    

