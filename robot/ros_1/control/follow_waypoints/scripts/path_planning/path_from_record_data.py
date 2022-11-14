#!/usr/bin/env python
from __future__ import print_function


import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped, PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys, select, termios, tty

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

class Pose_Path(object):
    def __init__(self):

        map_frame_id = rospy.get_param('~goal_frame_id','map')
        odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        base_frame_id = rospy.get_param('~base_frame_id','base_footprint')

        df = pd.read_csv("pose_data.csv")

        path_pub = rospy.Publisher('/saved_path', Path, queue_size=10)

        pose = PoseStamped()
        pose.header.frame_id = map_frame_id
        pose.pose.position.x = df["P_x"].values
        pose.pose.position.y = df["P_y"].values
        pose.pose.position.z = [0 for i in range(len(df))]
        pose.pose.orientation.x = [0 for i in range(len(df))]
        pose.pose.orientation.y = [0 for i in range(len(df))]
        pose.pose.orientation.z = df["O_z"].values
        pose.pose.orientation.w = df["O_w"].values

        path = Path()
        path.header.frame_id = "main"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        print("ok")

        while not rospy.is_shutdown():
            path_pub.publish(path)


if __name__ == '__main__':
    rospy.init_node("path_data_show")
    try:
        Pose_Path()
    except:
        pass
    # rospy.spin()