#!/usr/bin/env python
import rospy, tf, math, time, os,rospy, actionlib ,pandas as pd
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
import tf
class Move_base_goal(object):
    def __init__(self):
        self.df = pd.read_csv("pose_data.csv")
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.5)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        # os.system("rosservice call /StartLocalization")
        # os.system("rosservice call /move_base/clear_costmaps")
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        # self.tf = tf.TransformListener()
        self.listener = tf.TransformListener()
        
        # for p_x, p_y, p_z, o_x, o_y, o_z, o_w in zip(self.df["P_x"], self.df["P_y"], self.df["P_z"],self.df["O_x"], self.df["O_y"],self.df["O_z"], self.df["O_w"]):
        #     print(p_x, p_y, p_z, o_x, o_y, o_z, o_w)
        #     rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
        #     result = self.movebase_client(p_x, p_y, o_z, o_w)
        #     if result:
        #         rospy.loginfo("Goal execution done!")
        
        for p_x, p_y, o_z, o_w in zip(self.df["P_x"], self.df["P_y"], self.df["O_z"], self.df["O_w"]):
            try:
                print(p_x, p_y, o_z, o_w)
                rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
                result = self.movebase_client(p_x, p_y, o_z, o_w)
                if result:
                    rospy.loginfo("Goal execution done!")
            except:
                pass
            

    def movebase_client(self, p_x ,p_y, o_z, o_w):
        goal = MoveBaseGoal()
        # os.system("rosservice call /move_base/clear_costmaps")
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p_x
        goal.target_pose.pose.position.y = p_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = o_z
        goal.target_pose.pose.orientation.w = o_w
        self.client.send_goal(goal)

        if not self.distance_tolerance > 0.0:
            # self.client.wait_for_result()
            finished_within_time = self.client.wait_for_result(rospy.Duration(10.0))
            if not finished_within_time:
                self.client.cancel_goal()
                rospy.loginfo("Timed out achieving goal and goal canceled")
                
            # rospy.loginfo("Waiting for %f sec..." % self.duration)
            # print("---------------if------------------")
            time.sleep(self.duration)
            
        else:
            distance = 10
            while(distance > self.distance_tolerance):
                # now = rospy.Time.now()
                try:
                    self.listener.waitForTransform(self.frame_id, self.base_frame_id, rospy.Time(), rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.frame_id,self.base_frame_id, rospy.Time())
                    distance = math.sqrt(pow(p_x-trans[0],2)+pow(p_y-trans[1],2))
                    
                    # print(distance, "-----------------------------------------------")
                except:
                    self.client.cancel_goal()
                    rospy.loginfo("Timed out achieving goal and goal canceled")
                # print(distance, "-----------------------------------------------")

        return 'success'
        # wait = self.client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return self.client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        Move_base_goal()
        # result = movebase_client()
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")