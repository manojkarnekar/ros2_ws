#!/usr/bin/env python
from __future__ import print_function


import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys, select, termios, tty

class Pose_save(object):
    def __init__(self):

        self.pose_data = open("pose_data.csv", mode='w+')
        self.pose_data.write("{},{},{},{}\n".format("P_x","P_y", "O_z", "O_w"))

        listener = tf.TransformListener()

        map_frame_id = rospy.get_param('~goal_frame_id','map')
        odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        base_frame_id = rospy.get_param('~base_frame_id','base_footprint')

        self.old_x = 0
        self.old_y = 0
        self.yaw_old = 0
        self.cnt = 0
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(map_frame_id, base_frame_id, rospy.Time(0))
                # print("translations", trans, "rotations", rot)
                curr_dis = abs(math.sqrt((trans[0] - self.old_x)**2 + (trans[1] - self.old_y)**2))
                # print("translations-->",trans[0], trans[1],"rotations-->", rot[2], rot[3])

                orientation_list = [rot[0], rot[1], rot[2], rot[3]]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                self.yaw_curr = yaw* 180.0 / math.pi
                # print("YAW-->",(self.yaw_curr-self.yaw_old))
                
                # if self.cnt<1:
                #     print("translations-->",trans[0], trans[1],"rotations-->", rot[2], rot[3])
                #     self.pose_data.write("{},{},{},{}\n".format(trans[0], trans[1], rot[2], rot[3]))
                # self.cnt+=1
                # if abs(self.yaw_curr-self.yaw_old)>25.0:
                #     print("old_to_new_yaw-->", self.yaw_curr)
                #     self.yaw_old = self.yaw_curr
                # print("GOAL angle->",math.atan2((trans[1] - self.old_y),(trans[0] - self.old_x)))

                if abs(self.yaw_curr-self.yaw_old)>20.0 or curr_dis > 0.2:
                    self.pose_data.write("{},{},{},{}\n".format(trans[0], trans[1], rot[2], rot[3]))
                    print("translations-->",trans[0], trans[1],"rotations-->", rot[2], rot[3])
                    # print("old_to_new_yaw-->", self.yaw_curr)
                    self.old_x = trans[0]
                    self.old_y = trans[1]
                    self.yaw_old = self.yaw_curr
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        
        self.pose_data.close()
        

if __name__ == '__main__':
    rospy.init_node("odom_data_save")
    try:
        Pose_save()
    except:
        pass
    # rospy.spin()