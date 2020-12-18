# Use this example for digital pin control of an H-bridge driver
# like a DRV8833, TB6612 or L298N.

import time
import board
import digitalio
from adafruit_motor import stepper
import math

DELAY = 0.02

# configure pins and motors
coils = (
    # motor 1
    digitalio.DigitalInOut(board.D19),  # A1
    digitalio.DigitalInOut(board.D26),  # A2
    digitalio.DigitalInOut(board.D6),  # B1
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

def moveStepper(motor, steps, direc):
    '''
    Steps the motor a set number of steps in the defined direction
    Parameters:
        motor       list of motor names to move, e.g. [motor1, motor2, motor3]
        steps       list of number of steps to move, corresponding to MOTOR, e.g. [30, 60, 90]
        direc         list of directions to move, corresponding to MOTOR, e.g. [stepper.FORWARD, stepper.BACKWARD, stepper.FORWARD]
    '''
    for s in range(max(steps)): # step one step at a time
        for m in range(len(motor)): # loop through each motor
            if s <= steps[m]: # check if step number is valid
                motor[m].onestep(direction=direc[m], style=stepper.DOUBLE)
        time.sleep(DELAY) # pause for motors to complete step

def gearToStep(gearTeeth1, gearTeeth2, angle, degPerStep):
	'''
	Converts the angle provided at gear 1 to steps at the motors, where the motor is connected to gear 2
	Parameters:
		gearTeeth1      number of teeth at gear 1, e.g. 24 [nondimm]
		gearTeeth2      number of teeth at gear 2, e.g. 18 [nondimm]
		angle			desired angle at gear 1, e.g. 1 [rad]
		degPerStep		degrees per step of the stepper motor, e.g. 1.8 [deg]
	Outputs
		steps			number of steps needed at the motor, e.g. 20 [nondimm]
	'''
	theta2 = gearTeeth1/gearTeeth2*angle*180/math.pi
	steps = round(theta2/degPerStep)
	
	return steps
    
motors_released = False
t_start = time.time()
print('Motors Moving')
#moveStepper([motor3], [20], [stepper.FORWARD])
moveStepper([motor1, motor2, motor3], [5, 5, 5], [stepper.FORWARD, stepper.BACKWARD, stepper.BACKWARD])
print('Motors Locked')

'''
# Use controller to find backboard movement
# Move backboard
# Gear Specs
motorGearTeeth = 16
leverGearTeeth = 36
degPerStep = 1.8 # number of degrees per step of the motor
theta1_goal = 0#math.pi/4
theta2_goal = 0#math.pi/4
theta3_goal = math.pi/4
steps = []
direc = []
motorNumber = 1
for angle in [theta1_goal, theta2_goal, theta3_goal]:
    # find desired angle based on gearing
    des_step = gearToStep(leverGearTeeth, motorGearTeeth, angle, degPerStep)
    print(des_step)
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
print('motors moved!')
'''        
run = True
while run:
    t = time.time()
    if (t - t_start) >= 1 and not motors_released:
        print('Motors Have Been Released')
        motor1.release()
        motor2.release()
        motor3.release()
        motors_released = True
        run = False
