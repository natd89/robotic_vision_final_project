from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import rospy
from pdb import set_trace as pause
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

flag=1

for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    image = frame.array

    if flag:
        # setup initial location of window
        r,h,c,w = 80-40,80+40,64-30,64+30  # simply hardcoded the values
        track_window = (c,r,w,h)
        
        # set up the ROI for tracking
        # roi = image[r:r+h, c:c+w]
        hsv_roi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([0,0,0])
        upper_blue = np.array([10,255,255])

        # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        flag=0
    else:
        # convert color to grayscale
        hsv_new = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        # apply meanshift to get the new location
        mask = cv2.inRange(cv2.GaussianBlur(hsv_new,(5,5),0), lower_blue, upper_blue)
        #mask = cv2.dilate(mask,np.ones((6,6),np.uint8),iterations=1)
        mask = cv2.erode(mask,np.ones((2,2),np.uint8),iterations=1)
        mask[0:,0:17]=0
        mask[0:,142:]=0

        ret, track_window = cv2.CamShift(mask, track_window, term_crit)
        
        # Draw it on image
        pts=cv2.cv.BoxPoints(ret)
        pts = np.int0(pts)
        center = np.mean(pts,axis=0)
        cntr = (int(center[0]),int(center[1]))
        cv2.polylines(image,[pts],True, 255,2)
        cv2.circle(image,cntr,3,(255,0,0),2)

        cv2.imshow("frame",image)
        cv2.imshow("mask",mask)
        key = cv2.waitKey(1) & 0xFF
        
        # if the 'q' key is pressed, break the loop
        if key==ord("q"):
            break
        
        # publish ball position
        center = [(center[1]-64.)*5.25/128.,(center[0]-80.)*5.25/128.]
        ball_pos.data = center
        pub.publish(ball_pos)
    
    # clear the stream for the next frame
    rawCapture.truncate(0)
