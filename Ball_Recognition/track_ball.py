# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import imutils
import time

# parameters
r_ball = 139.7 # radius of ball (mm)
f_lens = 3.04 # camera lens focal length (mm)
h_sensor = 2.76 # camera sensor height (mm)

# create a timer
current_time = time.time()
start_time = time.time()
time_out = 120 # number of seconds before timing out

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
framesToPlot = 8
 
# define the lower and upper boundaries of the ball in the HSV color space
orangeLower = (151, 97, 18)
orangeUpper = (190, 207, 199)

# keep track of bounding box locations in a dictionary
ball_dict = {'location': [], 'time': [], 'velocity': [[0, 0, 0]]}

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	### *** RECOGNIZE AND RECORD LOCATION OF OBJECT *** ###
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	# Resize the frame, blur it, and convert to HSV color space
	blurred = cv2.GaussianBlur(image, (11, 11), 0)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# Construct a mask for the color "orange", then perform a series of
	# dilations and erosions to remove any small blobs left in the mask
	mask = cv2.inRange(hsv, orangeLower, orangeUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# Find contours in the mask and initialize the current (x,y) center
	# of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	# Only proceed if at least one contour was found
	if len(cnts) > 0:
		# Find the largest contour in the mask, then use it to compute
		# the minimum enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# Only proceed if the radius meets a minimum size
		ballDetected = False
		if radius > 10:
			# draw the circle and centroid on the frame, then update the
			# list of tracked points
			ballDetected = True
			cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			cv2.circle(image, center, 5, (0, 0, 255), -1)
			ball_dict['location'].append([x, y, radius])
			ball_dict['time'].append(time.time())
	# Connect last __ frames with a blue line
	if len(ball_dict['location']) >= framesToPlot:
		center_pts = np.zeros((framesToPlot, 2), np.int32)
		for i in range(framesToPlot):
			center_pts[i][0] = ball_dict['location'][len(ball_dict['location'])-i-1][0]
			center_pts[i][1] = ball_dict['location'][len(ball_dict['location'])-i-1][1]
		center_pts = center_pts.reshape((-1,1,2))
		cv2.polylines(image,[center_pts],False,(255,0,0),5)
	# Show the frame
	cv2.imshow('Frame',image)
	# Wait for key
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	### *** GET VELOCITY *** ###
	if len(ball_dict['location']) > 1 and ballDetected:
		# get measurments of last 2 frames
		[x, y, r] = [ball_dict['location'][-1][0], ball_dict['location'][-1][1], ball_dict['location'][-1][2]]
		t = ball_dict['time'][-1]
		[x0, y0, r0] = [ball_dict['location'][-2][0], ball_dict['location'][-2][1], ball_dict['location'][-1][2]]
		t0 = ball_dict['time'][-2]
		# calculate x
		x_m = x*r_ball/r # (mm)
		x0_m = x0*r_ball/r0 # (mm)
		v_x = (x_m-x0_m)/((t-t0)*1000) # (m/s)
		# calculate y
		y_m = y*r_ball/r # (mm)
		y0_m = y0*r_ball/r0 # (mm)
		v_y = (y_m-y0_m)/((t-t0)*1000) # (m/s)
		# calculate z
		z_m = (f_lens*r_ball*camera.resolution[1])/(r*h_sensor*1000) # (m)
		z0_m = (f_lens*r_ball*camera.resolution[1])/(r0*h_sensor*1000) # (m)
		v_z = (z0_m-z_m)/(t-t0) # (m/s)
		#print(z_m*39.3701)
		#print(x_m*0.03937)
		#print(y_m*0.03937)
		#print(v_x)
		ball_dict['velocity'].append([v_x, v_y, v_z])
		
	### *** TIME-OUT AFTER A CERTAIN TIME *** ###
	current_time = time.time()
	if current_time-start_time > time_out:
		break

# Print for debugging
for key, value in ball_dict.items():
	print(key, '->', value)

cv2.destroyAllWindows()
