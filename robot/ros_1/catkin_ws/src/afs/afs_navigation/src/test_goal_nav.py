#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
rospy.init_node('movebase_client_py')
rate = rospy.get_param('~rate', 100)
r = rospy.Rate(self.rate)
while not rospy.is_shutdown():
    print("going to A")
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 3.331
    goal.target_pose.pose.position.y = 0.191
    goal.target_pose.pose.orientation.z = -0.061
    goal.target_pose.pose.orientation.w = 0.998
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to B")
    goal=MoveBaseGoal()
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 3.560
    goal.target_pose.pose.position.y = 2.984
    goal.target_pose.pose.orientation.z = 0.996
    goal.target_pose.pose.orientation.z = -0.086
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to C")
    goal=MoveBaseGoal()
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.619
    goal.target_pose.pose.position.y = 0.815
    goal.target_pose.pose.orientation.z = 0.075
    goal.target_pose.pose.orientation.z = 0.997
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to D")
    goal=MoveBaseGoal()
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.619
    goal.target_pose.pose.position.y = 0.815
    goal.target_pose.pose.orientation.z = 0.075
    goal.target_pose.pose.orientation.z = 0.997
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    r.sleep()
