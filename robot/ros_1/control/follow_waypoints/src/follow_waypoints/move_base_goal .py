#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import pandas as pd
import os
class Move_base_goal(object):
    def __init__(self):
        self.df = pd.read_csv("pose_data.csv")
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        os.system("rosservice call /StartLocalization")
        
        # self.client.wait_for_server()


        # for t, p_x, p_y, o_z, o_w in zip(self.df["time"], self.df["P_x"], self.df["P_y"], self.df["O_z"], self.df["O_w"]):
        #     print(t, p_x, p_y, o_z, o_w)
        #     result = self.movebase_client(p_x, p_y, o_z, o_w)
        #     if result:
        #         rospy.loginfo("Goal execution done!")

    def movebase_client(self, p_x ,p_y, o_z, o_w):

        goal = MoveBaseGoal()
        os.system("rosservice call /move_base/clear_costmaps")
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p_x
        goal.target_pose.pose.position.y = p_y
        # goal.target_pose.pose.orientation.z = o_z
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        Move_base_goal()
        # result = movebase_client()
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")