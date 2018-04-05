from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
import cv2.cv as cv

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (160,120)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(160,120))

# allow the camera to warm up
time.sleep(0.1)

# initialize publisher
rospy.init_node('ball_position',anonymous=True)
pub = rospy.Publisher('ball_pos',Float64MultiArray,queue_size = 10)

ball_pos = Float64MultiArray()

for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    image = frame.array

    # convert color to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
    cv2.imshow("frame", gray)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream for the next frame
    rawCapture.truncate(0)

    # if the 'q' key is pressed, break the loop
    if key==ord("q"):
        break

    # publish ball position
    ball_pos.data = [1,2]
    pub.publish(ball_pos)
