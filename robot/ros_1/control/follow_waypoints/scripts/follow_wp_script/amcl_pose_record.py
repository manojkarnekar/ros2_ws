#!/usr/bin/env python

from numpy.lib.financial import ipmt
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped


class Pose_save(object):
    def __init__(self):

        self.pose_data = open("pose_data.csv", mode='w+')
        self.pose_data.write("{},{},{},{}\n".format("P_x","P_y", "O_z", "O_w"))
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback)
        self.t0=rospy.Time.now().to_sec()
        self.time_lapse = 5.0

    def odom_callback(self,msg):
        self.t1=rospy.Time.now().to_sec()

        self.p_y_0 = msg.pose.pose.position.x
        self.p_x_0 = msg.pose.pose.position.y

        if (self.t1 - self.t0)>self.time_lapse:

            #self.pose_data.write("{},{},{},{},{}\n".format((self.t1 - self.t0),(msg.pose.pose.position.x-self.p_x_0), (msg.pose.pose.position.y-self.p_y_0), msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            self.pose_data.write("{},{},{},{}\n".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            # print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            # print(math.sqrt((msg.pose.pose.position.x-self.p_x_0)**2 + (msg.pose.pose.position.y-self.p_y_0)**2))
            # print("x------------------>",(msg.pose.pose.position.x-self.p_x_0), "y------------------>",(msg.pose.pose.position.y-self.p_y_0))
            # print("time->",(self.t1 - self.t0),msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            self.t0=rospy.Time.now().to_sec()
            self.p_y_0 = msg.pose.pose.position.x
            self.p_x_0 = msg.pose.pose.position.y
        

if __name__ == '__main__':
    rospy.init_node("odom_data_save")
    Pose_save()
    rospy.spin()

