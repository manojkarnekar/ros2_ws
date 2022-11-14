#!/usr/bin/env python

import sys
import time
import rospy
import move_arm
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    move_arm = move_arm.MoveArm()
    pose = geometry_msgs.msg.Pose()

    move_arm.reach_joint_angles(tolerance=0.001)
    rospy.loginfo("Planning without any constraints")
    quaternion = quaternion_from_euler(-pi, 0.0, 0.0)
    pose.position.x = 0.50
    pose.position.y = -0.25
    pose.position.z = 0.50
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    for i in range(10):
        pose.position.y += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    pose.position.y = 0.00
    quaternion = quaternion_from_euler(0.0, pi/2, 0.0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    for i in range(5):
        pose.position.z += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    for i in range(5):
        pose.position.x += 0.025
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
