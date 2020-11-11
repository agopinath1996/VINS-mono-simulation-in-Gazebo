#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Point,Quaternion

class Ground_truth_publisher(object):

    def __init__(self):

        self.ground_truth_sub = rospy.Subscriber('/ground_truth/state',Odometry,self.odom_callback)
        self.Pose = []
     
    def odom_callback(self,data):

        msg1 = Path()
        msg1.header.seq = data.header.seq
        msg1.header.frame_id = "world"
        msg = PoseStamped()
        msg.header.seq = data.header.seq 
        msg.header.frame_id = "world"
        msg.pose.position = Point(data.pose.pose.position.x, data.pose.pose.position.y,data.pose.pose.position.z)
        msg.pose.orientation = Quaternion(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        self.Pose.append(msg)
        msg1.poses = self.Pose 
        path_pub = rospy.Publisher('/ground_truth_path',Path, queue_size=10)
        path_pub.publish(msg1)
        
if __name__ == "__main__":
    rospy.init_node('ground_truth_publisher',anonymous = True)
    rospy.loginfo('node is starting')
    g = Ground_truth_publisher()
    rospy.spin()  

