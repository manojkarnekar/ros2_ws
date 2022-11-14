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

import sys, select, termios, tty

class Pose_save(object):
    def __init__(self):

        self.pose_data = open("pose_data.csv", mode='w+')
        self.pose_data.write("{},{},{},{}\n".format("P_x","P_y", "O_z", "O_w"))

        listener = tf.TransformListener()

        map_frame_id = rospy.get_param('~goal_frame_id','map')
        odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        base_frame_id = rospy.get_param('~base_frame_id','base_footprint')

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(map_frame_id, base_frame_id, rospy.Time(0))
                print("translations", trans, "rotations", rot)
                self.pose_data.write("{},{},{},{}\n".format(trans[0], trans[1], rot[2], rot[3]))
                print(trans[0], trans[1], rot[2], rot[3])
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        

if __name__ == '__main__':
    rospy.init_node("odom_data_save")
    Pose_save()
    # rospy.spin()