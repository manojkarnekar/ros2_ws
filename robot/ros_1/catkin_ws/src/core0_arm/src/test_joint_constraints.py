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
    pose.position.z = 0.25
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    for i in range(5):
        pose.position.y += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
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

    move_arm.reach_joint_angles(tolerance=0.001)
    rospy.loginfo("Planning with 1 constraint")
    move_arm.init_path_constraints(num = 1)
    move_arm.enable_path_constraints()
    quaternion = quaternion_from_euler(-pi, 0.0, 0.0)
    pose.position.x = 0.50
    pose.position.y = -0.25
    pose.position.z = 0.25
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.y += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    quaternion = quaternion_from_euler(0.0, pi/2, 0.0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.z += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.x += 0.025
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)

    move_arm.reach_joint_angles(tolerance=0.001)
    rospy.loginfo("Planning with 2 constraints")
    move_arm.init_path_constraints(num = 2)
    move_arm.enable_path_constraints()
    quaternion = quaternion_from_euler(-pi, 0.0, 0.0)
    pose.position.x = 0.50
    pose.position.y = -0.25
    pose.position.z = 0.25
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.y += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    quaternion = quaternion_from_euler(0.0, pi/2, 0.0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.z += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.x += 0.025
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)

    move_arm.reach_joint_angles(tolerance=0.001)
    rospy.loginfo("Planning with 3 constraints")
    move_arm.init_path_constraints(num = 3)
    move_arm.enable_path_constraints()
    quaternion = quaternion_from_euler(-pi, 0.0, 0.0)
    pose.position.x = 0.50
    pose.position.y = -0.25
    pose.position.z = 0.25
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.y += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    quaternion = quaternion_from_euler(0.0, pi/2, 0.0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.z += 0.05
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)
    for i in range(5):
        pose.position.x += 0.025
        move_arm.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=move_arm.constraints)

    rospy.loginfo("Removing all constraints")
    move_arm.disable_path_constraints()
