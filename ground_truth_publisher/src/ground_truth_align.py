#!/usr/bin/env python
import message_filters
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Point,Quaternion

class Ground_truth_publisher(object):

    def __init__(self):

        self.ground_truth_sub = message_filters.Subscriber('/ground_truth/state',Odometry)
        self.Pose = []
        self.vins_sub = message_filters.Subscriber('/vins_estimator/odometry',Odometry)
        self.ts = message_filters.TimeSynchronizer([self.ground_truth_sub,self.vins_sub],10)
        self.ts.registerCallback(self.ground_align_callback)
        

    def ground_align_callback(self,odom,path):

        msg1 = Path()
        msg1.header.seq = path.header.seq
        msg1.header.frame_id = "world"
        msg = PoseStamped()
        msg.header.seq = path.header.seq 
        msg.header.frame_id = "world"
        msg.pose.position =  Point(odom.pose.pose.position.x, odom.pose.pose.position.y,odom.pose.pose.position.z) 
        msg.pose.orientation = Quaternion(path.pose.pose.orientation.x,path.pose.pose.orientation.y,path.pose.pose.orientation.z,path.pose.pose.orientation.w)
        self.Pose.append(msg)
        msg1.poses = self.Pose 
        path_pub = rospy.Publisher('/vins_aligned',Path, queue_size=10)
        path_pub.publish(msg1)

        
if __name__ == "__main__":
    rospy.init_node('ground_truth_align',anonymous = True)
    rospy.loginfo('node is starting')
    g = Ground_truth_publisher()
    rospy.spin()  

