# Harrison Hidalgo
# ECE 5725 - Final Project
# 


## Positions on backboard wrt baseboard
# Number of points: 3
# Number of constriants: 1
# Number of CoMs: 1

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
### Gears 
## Positions wrt baseboard
# Gear 1
gear2_11 = 0.3810
gear2_12 = 0.1270
gear2_13 = 0.0381

# Gear 2
gear2_21 = 0.1955
gear2_22 = 0.2921
gear2_23 = 0.0381

# Gear 3
gear2_31 = 0.0762
gear2_32 = 0.127
gear2_33 = 0.0381

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

# Motor 2
motor_up21 = 0
motor_up22 = 1
motor_up23 = 0

# Motor 3
motor_up31 = 0
motor_up32 = -1
motor_up33 = 0

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
