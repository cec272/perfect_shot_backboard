# Harrison Hidalgo and Christopher Chan
# ECE 5725 - Final Project
# 


## Positions on backboard wrt baseboard
# Number of points: 3
# Number of constriants: 1
# Number of CoMs: 1

import numpy as np

# X positions
lam1 = 0.04001
lam2 = 0.22543
lam3 = 0.41085
beta1= 0.2286
Gx   = 0.2286
# Y positions
mu1   = 0.015875
mu2   = 0.180975
mu3   = 0.015875
alpha1= 0.098425
Gy    = 0.098425

#### Other dimensions
front = np.array([0,0,1])
up = np.array([0,1,0])
left = np.cross(front,up)
e_b = np.array([0,0,1])
W_of_backboard =0.4572
H_of_backboard = 0.1969
T_of_backboard = 0.0032

### Gears 
## Positions wrt baseboard
# Gear 1
gear2_11 = 0.3810
gear2_12 = 0.1270
gear2_13 = 0.0381
gear2_1 = np.array([gear2_11,gear2_12,gear2_13])

# Gear 2
gear2_21 = 0.1955
gear2_22 = 0.2921
gear2_23 = 0.0381
gear2_2 = np.array([gear2_21,gear2_22,gear2_23])

# Gear 3
gear2_31 = 0.0762
gear2_32 = 0.127
gear2_33 = 0.0381
gear2_3 = np.array([gear2_31,gear2_32,gear2_33])

# Sizes and effieciency
r1  = 0.0100075
r2  = 0.0227075
r3  = 0.03302
eta = 1

### Motors
## Unit vectors for motors wrt baseboard
# Motor 1
motor_up11 = 0
motor_up12 = -1
motor_up13 = 0
motor_up1 = np.array([motor_up11,motor_up12,motor_up13])

# Motor 2
motor_up21 = 0
motor_up22 = 1
motor_up23 = 0
motor_up2 = np.array([motor_up21,motor_up22,motor_up23])

# Motor 3
motor_up31 = 0
motor_up32 = -1
motor_up33 = 0
motor_up3 = np.array([motor_up31,motor_up32,motor_up33])

## Initial thetas
th1_10 = 0
th1_20 = 0
th3_30 = 0

### Camera
## Unit vectors for camera wrt baseboard
camera_up11 = 1
camera_up12 = -1
camera_up13 = -1

# Camera translation wrt baseboard
camera_dx = 0.44558
camera_dy = 0.40005
camera_dz = 0.00820
r_cam = np.transpose(np.array([[camera_dx,camera_dy,camera_dz,0,0,0]]))

rGB0  = -e_b*T_of_backboard/2

r_B0 = np.array([0.2286,0.2070,0.1809])

rGP10 = -e_b*T_of_backboard+left*(lam1-Gx)+up*(mu1-Gy)
rGP20 = -e_b*T_of_backboard+left*(lam2-Gx)+up*(mu2-Gy)
rGP30 = -e_b*T_of_backboard+left*(lam3-Gx)+up*(mu3-Gy)

motor_rod10 = np.array([16.6103,5,1.6859])/39.37
motor_rod20 = np.array([9.3198,11.5,1.5635])/39.37
motor_rod30 = np.array([0.03747516,0.127,0.04286504])

L1 = 0.11811
L2 = 0.11811
L3 = 0.11811

### Hoop
# center of hoop wrt baseboard
center_hoop = np.array([0.229, 0.089, 0.329])
