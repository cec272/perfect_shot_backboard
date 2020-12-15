import time
import board
import busio
import math
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS # accel + gyro
from adafruit_lis3mdl import LIS3MDL # magnetometer
import numpy as np
import csv

run = True

### *** Set-UP *** ###
# set-up sensors
accel_gyro = LSM6DS(board.I2C())

# sensor global variables
accel = (0, 0, 0)
gyro = (0, 0, 0)
accel_angles = [0, 0] # pitch, roll
gyro_angles = [0, 0, 0] # pitch, roll, yaw
comp_filter_angles = [0, 0] # pitch, roll
time_imu_read = time.time()
time_imu_read_0 = time.time()
a_x = []
a_y = []
a_z = []
pitch_accel = []
roll_accel = []
pitch_gyro = []
roll_gyro = []
yaw_gyro = []
pitch_filt = []
roll_filt = []

# Writing Data
imu_csv = 'imu.csv'
data_to_plot = 3
data = np.zeros((data_to_plot,4))
data_count = 0

### *** Helper Functions *** ###
# read raw values from the IMU
def getIMU():
	global accel
	global gyro
	global time_imu_read
	global time_imu_read_0
	accel = accel_gyro.acceleration
	gyro = accel_gyro.gyro
	time_imu_read_0 = time_imu_read
	time_imu_read = time.time()
	
# get pitch and roll in degrees using the accel data
def getAccelAngles():
	global accel
	global accel_angles
	aX = accel[0]
	aY = accel[1]
	aZ = accel[2]
	aMag = math.sqrt(math.pow(aX, 2) + math.pow(aY, 2) + math.pow(aZ, 2))
	theta = math.asin(aX/aMag)*57.295779513
	phi = math.asin(aY/aMag)*57.295779513
	accel_angles = [theta, phi]
	
# get pitch, roll, yaw in degrees using the gyroscope
def getGyroAngles():
	global gyro
	global accel_angles
	global gyro_angles
	omegaX = gyro[0]*57.295779513
	omegaY = gyro[1]*57.295779513
	omegaZ = gyro[2]*57.295779513
	# set pitch and roll to values from accel on start or if they differ too much
	if gyro_angles[0] == 0 or gyro_angles[1] == 0 or (gyro_angles[0]-accel_angles[0] >= 40) or (gyro_angles[1]-accel_angles[1] >= 40):
		gyro_angles = [accel_angles[0], accel_angles[1], 0]
	# integrate angular rate
	pitchGyro = gyro_angles[0]-omegaY*(time_imu_read-time_imu_read_0)
	rollGyro = gyro_angles[1]+omegaX*(time_imu_read-time_imu_read_0)
	yawGyro = gyro_angles[2]+omegaZ*(time_imu_read-time_imu_read_0)
	# update variables
	gyro_angles = [pitchGyro, rollGyro, yawGyro]
	
# fuse angles from accelerometer and gyro using a complementary filter
def compFilter(alpha):
	global comp_filter_angles
	global accel_angles
	global gyro_angles
	# initialize pitch and roll on start
	if comp_filter_angles[0] == 0 or comp_filter_angles[1] == 0:
		comp_filter_angles = [accel_angles[0], accel_angles[1]]
	# fuse the angles
	pitchFilt = (comp_filter_angles[0] + gyro_angles[0]*(time_imu_read-time_imu_read_0))*(1-alpha) + accel_angles[0]*alpha
	rollFilt = (comp_filter_angles[1] + gyro_angles[1]*(time_imu_read-time_imu_read_0))*(1-alpha) + accel_angles[1]*alpha
	# update stuff
	comp_filter_angles = [pitchFilt, rollFilt]
	
# used periodically from FuncAnimation
def animate(i, a_x, a_y, a_z, pitch_accel, roll_accel, pitch_gyro, roll_gyro, yaw_gyro, pitch_filt, roll_filt):
	# get raw accel values
	global accel
	getIMU()
	# compute angular values
	getAccelAngles()
	getGyroAngles()
	compFilter(0.4)
	# add y to list
	a_x.append(accel[0])
	a_y.append(accel[1])
	a_z.append(accel[2])
	pitch_accel.append(accel_angles[0])
	roll_accel.append(accel_angles[1])
	pitch_gyro.append(gyro_angles[0])	
	roll_gyro.append(gyro_angles[1])
	yaw_gyro.append(gyro_angles[2])
	pitch_filt.append(comp_filter_angles[0])
	roll_filt.append(comp_filter_angles[1])
	# limit y list to the set number of items
	a_x = a_x[-x_len:]
	a_y = a_y[-x_len:]
	a_z = a_z[-x_len:]
	pitch_accel = pitch_accel[-x_len:]
	roll_accel = roll_accel[-x_len:]
	pitch_gyro = pitch_gyro[-x_len:]
	roll_gyro = roll_gyro[-x_len:]
	yaw_gyro = yaw_gyro[-x_len:]
	pitch_filt = pitch_filt[-x_len:]
	roll_filt = roll_filt[-x_len:]
	# update line with new y values
	line1.set_ydata(a_x)
	line2.set_ydata(a_y)
	line3.set_ydata(a_z)
	line4.set_ydata(pitch_accel)
	line5.set_ydata(roll_accel)
	line6.set_ydata(pitch_gyro)
	line7.set_ydata(roll_gyro)
	line8.set_ydata(yaw_gyro)
	line9.set_ydata(pitch_filt)
	line10.set_ydata(roll_filt)
	return line1, line2, line3, line4, line5, line6, line7, line8, line9, line10,

while run:
	# get raw accel values
	getIMU()
	# compute angular values
	getAccelAngles()
	getGyroAngles()
	compFilter(0.4)
	# add y to list
	a_x.append(accel[0])
	a_y.append(accel[1])
	a_z.append(accel[2])
	pitch_accel.append(accel_angles[0])
	roll_accel.append(accel_angles[1])
	pitch_gyro.append(gyro_angles[0])	
	roll_gyro.append(gyro_angles[1])
	yaw_gyro.append(gyro_angles[2])
	pitch_filt.append(comp_filter_angles[0])
	roll_filt.append(comp_filter_angles[1])	
	
	print(pitch_filt)
	print(roll_filt)
	### *** WRITE TO CSV FILE *** ###
	data[data_count,:] = [pitch_filt[-1], roll_filt[-1], None, time_imu_read]
	with open(imu_csv,'w') as csvfile:
		csvwriter = csv.writer(csvfile)
		csvwriter.writerows(data)
	if data_count < 2:
		data_count = data_count+1
	else:
		data_count = 0
	print(data)
	
	
	### *** CHECK IF FUNCTION SHOULD QUIT *** ###
	with open('run.csv','r') as csvfile:
		csvreader = csv.reader(csvfile)
		for row in csvreader:
			run = row[0]

