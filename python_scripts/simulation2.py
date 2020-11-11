#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np 
from sensor_msgs.msg import Image


# includes optical flow
class feature_detector(object):

    def __init__(self):
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
        self.bridge  = CvBridge()
        self.image_pub = rospy.Publisher("corner_detected",Image)
        self.flag = False
        self.ifInitialized = False
        self.mask = None
        self.p0 = None
        self.old_gray = None
        self.p1 = None
        self.st = None
        
    def image_callback(self,data):
        
        try:
            cv_image  = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        
        # params for ShiTomasi corner detection
        feature_params = dict(maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Create some random colors
        color = np.random.randint(0,255,(100,3))
        if(not self.ifInitialized):
          # Take first frame and find corners in it
          old_frame = cv_image
          self.old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
          self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, **feature_params)
          # Create a mask image for drawing purposes
          self.mask = np.zeros_like(old_frame)
        self.flag = True
        if(self.flag):
     
             frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

             # calculate optical flow
             self.p1, self.st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **lk_params)

             # Select good points
             good_new = self.p1[st==1]
             good_old = self.p0[st==1]

             # draw the tracks
             for i,(new,old) in enumerate(zip(good_new,good_old)):
                 a,b = new.ravel()
                 c,d = old.ravel()
                 self.mask = cv2.line(self.mask, (a,b),(c,d), color[i].tolist(), 2)
                 cv_image = cv2.circle(cv_image,(a,b),5,color[i].tolist(),-1)
             cv_image = cv2.add(cv_image,self.mask)
             # Now update the previous frame and previous points
             self.old_gray = frame_gray.copy()
             self.p0 = good_new.reshape(-1,1,2)    
        self.ifInitialized = True
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)    

if __name__== "__main__":
        f = feature_detector()
        rospy.init_node('image_converter',anonymous=True)
        rospy.loginfo('node is starting')
        rospy.spin()
       
      

        




