#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int8
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Trigger, TriggerResponse

class MotionStateController():

    def __init__(self):
        try:
            rospy.init_node('airbot_motion_state_controller')
            rospy.Subscriber('/front_camera/ar_pose_marker', AlvarMarkers, self.front_ar_cb, queue_size=1)
            rospy.Subscriber('/rear_camera/ar_pose_marker', AlvarMarkers, self.rear_ar_cb, queue_size=1)
            rospy.Service('airbot/start_autonomous_motion', Trigger, self.start_am_handler)
            rospy.Service('airbot/stop_autonomous_motion', Trigger, self.stop_am_handler)
            rospy.Service('airbot/teleop_mode', Trigger, self.teleop_mode_handler)
            self.pub_motion_state = rospy.Publisher('airbot_motion_state', Int8, queue_size=1)
            self.threshold_distance = rospy.get_param('threshold_distance', 1.0)
            self.num_filter = rospy.get_param('num_filter', 3)
            self.front_marker_id = rospy.get_param('front_marker_id', 1)
            self.rear_marker_id = rospy.get_param('rear_marker_id', 2)
            self.rate = rospy.get_param('rate', 15.0)
            self.states = {'stop': 0, 'forward': 1, 'reverse': -1,'cycle_complete': 2, 'teleop': 10}
            self.motion_state = self.states['stop']
            self.num_dist = 0
        except Exception as e:
            rospy.loginfo(e)
        else:
            rospy.loginfo("{0} node initialized successfully".format(rospy.get_name()))

    def switch_state(self, state):
        try:
            self.motion_state = self.states['stop']
            rospy.sleep(0.5)
            self.num_dist = 0
            self.motion_state = state
        except:
            return False
        else:
            return True

    def front_ar_cb(self, ar_marker_data):
        if self.motion_state == self.states['forward']:
            try:
                if self.num_dist < self.num_filter:
                    marker_id = ar_marker_data.markers[0].id
                    marker_position = ar_marker_data.markers[0].pose.pose.position
                    marker_distance  = math.hypot(marker_position.x, marker_position.y)
                    if marker_id == self.front_marker_id:
                        if marker_distance <= self.threshold_distance:
                            self.num_dist += 1
                    if self.num_dist >= self.num_filter:
                        self.switch_state(self.states['cycle_complete'])
            except IndexError:
                pass

    def rear_ar_cb(self, ar_marker_data):
        if self.motion_state == self.states['reverse']:
            try:
                if self.num_dist < self.num_filter:
                    marker_id = ar_marker_data.markers[0].id
                    marker_position = ar_marker_data.markers[0].pose.pose.position
                    marker_distance  = math.hypot(marker_position.x, marker_position.y)
                    if marker_id == self.rear_marker_id:
                        if marker_distance <= self.threshold_distance:
                            self.num_dist += 1
                    if self.num_dist >= self.num_filter:
                        self.switch_state(self.states['stop'])
            except IndexError:
                pass

    def start_am_handler(self, req):
        res = TriggerResponse()
        if self.motion_state == self.states['forward'] or self.motion_state == self.states['reverse']:
            res.success = True
            res.message = "Robot is already in Autonomous Motion"
        else:
            switch_status = self.switch_state(self.states['forward'])
            if switch_status:
                res.success = True
                res.message = "Autonomous Motion started successfully"
            else:
                res.success = False
                res.message = "Failed to start Autonomous Motion"
        return res

    def stop_am_handler(self, req):
        res = TriggerResponse()
        if self.motion_state == self.states['stop']:
            res.success = True
            res.message = "Robot is already in Stopped state"
        else:
            switch_status = self.switch_state(self.states['stop'])
            if switch_status:
                res.success = True
                res.message = "Autonomous Motion stopped successfully"
            else:
                res.success = False
                res.message = "Failed to stop Autonomous Motion"
        return res

    def teleop_mode_handler(self, req):
        res = TriggerResponse()
        if self.motion_state == self.states['teleop']:
            res.success = True
            res.message = "Robot is already in Teleop Mode"
        else:
            switch_status = self.switch_state(self.states['teleop'])
            if switch_status:
                res.success = True
                res.message = "Teleop Mode activated successfully"
            else:
                res.success = False
                res.message = "Failed to switch to Teleop Mode"
        return res

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                self.pub_motion_state.publish(self.motion_state)
                rate.sleep()
            except rospy.ROSInterruptException:
                return

if __name__ == '__main__':
    motion_state_controller = MotionStateController()
    motion_state_controller.spin()
