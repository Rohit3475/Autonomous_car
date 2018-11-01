# import RPi.GPIO as GPIO
# import time
# # import l239d.driver as l239d

# # GPIO.cleanup()
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(21,GPIO.OUT)
# #sa GPIO.setup(26,GPIO.OUT)
# GPIO.setup(20,GPIO.OUT)
# # GPIO.setup(12,GPIO.OUT)

# # while True:
# # 	print("print")
# # 	GPIO.output(21,GPIO.HIGH)
# # 	time.sleep(5)
# # 	GPIO.output(21,GPIO.LOW)
# # 	GPIO.output(26,GPIO.HIGH)
# # 	time.sleep(5)
# 	# GPIO.output(26,GPIO.LOW)
# GPIO.output(21,GPIO.HIGH)
# time.sleep(5)
# GPIO.output(21,GPIO.LOW)

# GPIO.output(20,GPIO.HIGH)
# time.sleep(5)
# GPIO.output(20,GPIO.LOW)
# # GPIO.output(16,GPIO.HIGH)
# time.sleep(1000)

from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import cv2
import numpy as np
import math
GPIO.setmode(GPIO.BCM)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
mypwm = GPIO.PWM(21,10)
mypwm2 = GPIO.PWM(20,10)
mypwm.start(0)
mypwm2.start(0)
theta=0
minLineLength = 5
maxLineGap = 10
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	image.setflags(write=1)
	image[np.where((image>=[0,0,60]).all(axis=2))] = [255,255,255]
	image[np.where((image>=[0,60,0]).all(axis=2))] = [255,255,255]
	image[np.where((image>=[60,0,0]).all(axis=2))] = [255,255,255]
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(blurred, 85, 85)
	lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
	if(lines !=None):
	   for x in range(0, len(lines)):
	       for x1,y1,x2,y2 in lines[x]:
	           # cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
	           theta=theta+math.atan2((y2-y1),(x2-x1))
	#print(theta)GPIO pins were connected to arduino for servo steering control
	threshold=6
	if(theta>threshold*1.5):
		#GPIO.output(7,True)
		#GPIO.output(8,False)
		print("left")
		mypwm.ChangeDutyCycle(25)
		# GPIO.output(21,True)
		# GPIO.output(26,False)
		mypwm2.ChangeDutyCycle(0)
		# GPIO.output(20,False)
		# GPIO.output(12,True)
	if(theta<-threshold*1.5):
	   # GPIO.output(8,True)
	   # GPIO.output(7,False)
		print("right")
		mypwm.ChangeDutyCycle(0)
		# GPIO.output(21,False)
		#GPIO.output(26,True)
		# GPIO.output(20,True)
		mypwm2.ChangeDutyCycle(25)
		#GPIO.output(12,False)
	if(abs(theta)<threshold*1.5):
	  # GPIO.output(8,False)
	  # GPIO.output(7,False)
		mypwm.ChangeDutyCycle(25)
		print ("straight")
		# GPIO.output(21,True)
		#GPIO.output(26,False)
		mypwm2.ChangeDutyCycle(25)
		# GPIO.output(20,True)
		#GPIO.output(12,False)
	theta=0
	# cv2.imshow("Frame",image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	if key == ord("q"):
	   break

