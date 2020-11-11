#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np 
from sensor_msgs.msg import Image


class feature_detector(object):

    def __init__(self):
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
        self.bridge  = CvBridge()
        self.image_pub = rospy.Publisher("corner_detected",Image)

    def image_callback(self,data):

        try:
            cv_image  = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
        corners = np.int0(corners)
        for i in corners:
            x,y = i.ravel()
            cv2.circle(cv_image,(x,y),3,255,-1)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)    

if __name__== "__main__":
        f = feature_detector()
        rospy.init_node('image_converter',anonymous=True)
        rospy.loginfo('node is starting')
        rospy.spin()
       
      

        




