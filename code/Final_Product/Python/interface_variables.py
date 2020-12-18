# Harrison Hidalgo
# ECE 5725 - Final Project
# This contains the vaiables used in interface_creator.py

import pygame

### Move these to their own module
size_x = 320
size_y = 240
black  = 0,0,0
white  = 255,255,255
red    = 255,0,0
vars_1 = size_x/5
vars_2 = size_x*3/5
mean_1 = size_x*3/10
cov_1  = size_x*4/10
mean_2 = size_x*4/5
cov_2  = size_x*9/10
    
row_0 = 0
row_1 = size_y/12
row_2 = size_y*2/12
row_3 = size_y*3/12
row_4 = size_y*4/12
row_5 = size_y*5/12
row_6 = size_y*6/12
row_7 = size_y*7/12
row_8 = size_y*8/12
row_9 = size_y*9/12
row_10 = size_y*17/20
row_11 = size_y*11/12
row_12 = size_y*12/12

stop_bar = pygame.Rect(size_x/10,size_y*8/10,size_x*4/10,size_y/10)

# Perminent labels
my_labels_1={'Mean':(mean_1,row_3),'Cov':(cov_1,row_3),'X:':(vars_1,row_4),'Y:':(vars_1,row_5),'Z:':(vars_1,row_6),'X_d:':(vars_1,row_7),'Y_d:':(vars_1,row_8),'Z_d:':(vars_1,row_9),'th1:':(vars_2,row_4),'th2:':(vars_2,row_5),'th3:':(vars_2,row_6),'theta:':(vars_2,row_7),'phi:':(vars_2,row_8),'psi:':(vars_2,row_9)}
my_labels_2={'Mean':(mean_2,row_3),'Cov':(cov_2,row_3)}
 
