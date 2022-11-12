#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Quaternion, Twist, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int64 , Int32, Float32, Float64


class Alphasense_odometry(object):
    def __init__(self):
        
        self.base_frame_id = rospy.get_param('base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('odom_frame_id', 'alphasense/odom') # the name of the odometry reference frame

        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0

        self.seq=0

        self.msg = TwistStamped()

        self.pub = rospy.Publisher("alphasense/odom", TwistStamped, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_cb)       
        self.loop_rate = rospy.Rate(5)

    def odom_cb(self, msg):
        
        self.odom_frame_id = msg.header.frame_id
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs 
        self.Vx = msg.twist.twist.linear.x
        self.Vy = msg.twist.twist.linear.y
        self.Wz = msg.twist.twist.angular.z
        # print(self.Vx)

        
        self.msg.twist.linear.x =  self.Vx
        self.msg.twist.linear.y =  self.Vy
        self.msg.twist.angular.z = self.Wz
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.odom_frame_id
        self.msg.header.seq = self.seq
        self.seq += 1
        self.pub.publish(self.msg)
        # self.loop_rate.sleep()    
	    

if __name__ == "__main__": 
    rospy.init_node('odom_listener', anonymous=True)
    Alphasense_odometry()
    rospy.spin()
    