#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

class MoveArm():

    def __init__(self):
      # Initialize the node
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('moveit_joint_constraints')

      try:
        self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
        if self.is_gripper_present:
          gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
          self.gripper_joint_name = gripper_joint_names[0]
        else:
          gripper_joint_name = ""
        self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

        # Create the MoveItInterface necessary objects
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                      moveit_msgs.msg.DisplayTrajectory,
                                                      queue_size=20)

        if self.is_gripper_present:
          gripper_group_name = "gripper"
          self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
      except Exception as e:
        print (e)
        self.is_init_success = False
      else:
        self.is_init_success = True

    def reach_named_position(self, target):
      arm_group = self.arm_group

      # Going to one of those targets
      rospy.loginfo("Going to named target " + target)
      # Set the target
      arm_group.set_named_target(target)
      # Plan the trajectory
      planned_path1 = arm_group.plan()
      # Execute the trajectory and block while it's not finished
      return arm_group.execute(planned_path1, wait=True)

    def reach_joint_angles(self, tolerance):
      arm_group = self.arm_group
      success = True

      # Get the current joint positions
      joint_positions = arm_group.get_current_joint_values()
      # rospy.loginfo("Printing current joint positions before movement :")
      # for p in joint_positions: rospy.loginfo(p)

      rospy.loginfo("Going to starting pose")

      # Set the goal joint tolerance
      self.arm_group.set_goal_joint_tolerance(tolerance)

      # Set the joint target configuration
      if self.degrees_of_freedom == 7:
        joint_positions[0] = 0
        joint_positions[1] = 0
        joint_positions[2] = 0
        joint_positions[3] = pi/2
        joint_positions[4] = 0
        joint_positions[5] = 0
        joint_positions[6] = 0
      elif self.degrees_of_freedom == 6:
        joint_positions[0] = 0
        joint_positions[1] = 0
        joint_positions[2] = pi/2
        joint_positions[3] = pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2
      arm_group.set_joint_value_target(joint_positions)

      # Plan and execute in one command
      success &= arm_group.go(wait=True)

      # Show joint positions after movement
      # new_joint_positions = arm_group.get_current_joint_values()
      # rospy.loginfo("Printing current joint positions after movement :")
      # for p in new_joint_positions: rospy.loginfo(p)
      return success

    def get_cartesian_pose(self):
      arm_group = self.arm_group

      # Get the current pose and display it
      pose = arm_group.get_current_pose()
      rospy.loginfo("Actual cartesian pose is : ")
      rospy.loginfo(pose.pose)

      return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints=None):
      arm_group = self.arm_group

      # Set the tolerance
      arm_group.set_goal_position_tolerance(tolerance)

      # Set the trajectory constraint
      arm_group.set_path_constraints(constraints)

      # Get the current Cartesian Position
      arm_group.set_pose_target(pose)

      # Plan and execute
      rospy.loginfo("Planning and going to the Cartesian Pose")
      return arm_group.go(wait=True)

    def init_path_constraints(self, num=1):
      self.constraints = moveit_msgs.msg.Constraints()
      self.constraints.name = "joint_constraint"
      joint_constraint_3 = moveit_msgs.msg.JointConstraint()
      joint_constraint_3.position = 0
      joint_constraint_3.tolerance_above = 0.01
      joint_constraint_3.tolerance_below = 0.01
      joint_constraint_3.weight = 1
      joint_constraint_3.joint_name = "joint_3"
      joint_constraint_5 = moveit_msgs.msg.JointConstraint()
      joint_constraint_5.position = 0
      joint_constraint_5.tolerance_above = 0.01
      joint_constraint_5.tolerance_below = 0.01
      joint_constraint_5.weight = 1
      joint_constraint_5.joint_name = "joint_5"
      joint_constraint_7 = moveit_msgs.msg.JointConstraint()
      joint_constraint_7.position = 0
      joint_constraint_7.tolerance_above = 0.01
      joint_constraint_7.tolerance_below = 0.01
      joint_constraint_7.weight = 1
      joint_constraint_7.joint_name = "joint_7"
      if num == 1:
          self.constraints.joint_constraints.append(joint_constraint_7)
      elif num == 2:
          self.constraints.joint_constraints.append(joint_constraint_7)
          self.constraints.joint_constraints.append(joint_constraint_5)
      elif num == 3:
          self.constraints.joint_constraints.append(joint_constraint_7)
          self.constraints.joint_constraints.append(joint_constraint_5)
          self.constraints.joint_constraints.append(joint_constraint_3)

    def enable_path_constraints(self):
      self.arm_group.set_path_constraints(self.constraints)

    def disable_path_constraints(self):
      self.arm_group.set_path_constraints(None)
