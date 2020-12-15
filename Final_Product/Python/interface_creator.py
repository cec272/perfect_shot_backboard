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
size_x = 320
size_y = 240
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
end_time = 600

# writing data
csv_interface = 'interface_states.csv'
csv_run = 'run.csv'

# main loop
while run and ((current_time-start_time) < end_time):
    my_labels_large={'BALL':(size_x/3,row_2),'BACKBOARD':(size_x*17/24,row_2),'STOP':(size_x*3/10,row_10),'STATUS:':(size_x*4/6,row_10)}
    # Check for button press
    for event in pygame.event.get():
        if(event.type is MOUSEBUTTONDOWN):
            pos = pygame.mouse.get_pos()
        elif(event.type is MOUSEBUTTONUP):
            pos = pygame.mouse.get_pos()
            x,y = pos
            if (y>size_y*4/5) and (y<size_y*9/10) and (x>size_x/10) and (x<size_x/2): # stop button pressed
                run = False
                motor1.release() # release all motors
                motor2.release()
                motor3.release()
        
    ## Plotting
    screen.fill(black)
    pygame.draw.rect(screen,red,stop_bar)
    
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
