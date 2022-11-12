#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
rospy.init_node('movebase_client_py')
rate = rospy.get_param('~rate', 100)
r = rospy.Rate(rate)
while not rospy.is_shutdown():
    print("going to A")
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -2.58293366432
    goal.target_pose.pose.position.y = -0.593510270119
    goal.target_pose.pose.orientation.z = -0.00356950458226
    goal.target_pose.pose.orientation.w = 0.999993629298
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
    goal.target_pose.pose.position.x = 1.51143193245
    goal.target_pose.pose.position.y = -0.517457425594
    goal.target_pose.pose.orientation.z = -0.00191127334307
    goal.target_pose.pose.orientation.w = 0.999998173515
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
    goal.target_pose.pose.position.x = 2.24036884308
    goal.target_pose.pose.position.y = 0.226059913635
    goal.target_pose.pose.orientation.z = 0.710855570952
    goal.target_pose.pose.orientation.w = 0.703338010666
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
    goal.target_pose.pose.position.x = 2.66347289085
    goal.target_pose.pose.position.y = 0.998387753963
    goal.target_pose.pose.orientation.z = 0.999998335683
    goal.target_pose.pose.orientation.w = 0.00182445345303
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to E")
    goal=MoveBaseGoal()
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -2.53125286102
    goal.target_pose.pose.position.y = 0.998387753963
    goal.target_pose.pose.orientation.z = 0.999998335683
    goal.target_pose.pose.orientation.w = 0.00182445345303
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to F")
    goal=MoveBaseGoal()
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -3.25000
    goal.target_pose.pose.position.y = 0.226059913635
    goal.target_pose.pose.orientation.z = -0.70859717782
    goal.target_pose.pose.orientation.w = 0.705613236544
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    print("going to A")
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -2.58293366432
    goal.target_pose.pose.position.y = -0.593510270119
    goal.target_pose.pose.orientation.z = -0.00356950458226
    goal.target_pose.pose.orientation.w = 0.999993629298
    client.send_goal(goal)
    wait = client.wait_for_result()
    print(wait)
    # r.sleep()
