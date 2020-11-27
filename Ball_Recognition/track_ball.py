# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time

# create a timer
current_time = time.time()
start_time = time.time()
time_out = 20 # number of seconds before timing out

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
framesToPlot = 8
 
# Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml");

# keep track of bounding box locations in a dictionary
face_dict = {'location': [], 'time': [], 'velocity': [[0, 0]]}

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	### *** RECOGNIZE AND RECORD LOCATION OF OBJECT *** ###
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array	
	# Convert to grayscale
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	# Look for faces in the image using the loaded cascade file
	faces = face_cascade.detectMultiScale(gray, 1.1, 5)
	# Draw a rectangle around every found face
	faceDetected = False
	for (x,y,w,h) in faces:
		faceDetected = True
		# Create rectangle around the face
		cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
		# Save this in the dictionary
		cx = x + w/2
		cy = y + h/2
		face_dict['location'].append([x, y, w, h, cx, cy])
		face_dict['time'].append(time.time())
	# Connect last __ frames with a blue line
	if len(face_dict['location']) >= framesToPlot:
		center_pts = np.zeros((framesToPlot, 2), np.int32)
		for i in range(framesToPlot):
			center_pts[i][0] = face_dict['location'][len(face_dict['location'])-i-1][4]
			center_pts[i][1] = face_dict['location'][len(face_dict['location'])-i-1][5]
		center_pts = center_pts.reshape((-1,1,2))
		cv2.polylines(image,[center_pts],False,(255,0,0),5)
	# Show the frame
	cv2.imshow('Frame',image)
	# Wait for key
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	### *** GET VELOCITY *** ###
	if len(face_dict['location']) > 1 and faceDetected:
		[x, y] = [face_dict['location'][-1][4], face_dict['location'][-1][5]]
		t = face_dict['time'][-1]
		[x0, y0] = [face_dict['location'][-2][4], face_dict['location'][-2][5]]
		t0 = face_dict['time'][-2]
		v_x = (x-x0)/(t-t0)
		v_y = (y-y0)/(t-t0)
		face_dict['velocity'].append([v_x, v_y])
	
	### *** TIME-OUT AFTER A CERTAIN TIME *** ###
	current_time = time.time()
	if current_time-start_time > time_out:
		break

# Print for debugging
for key, value in face_dict.items():
	print(key, '->', value)

cv2.destroyAllWindows()
