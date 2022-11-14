#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32, Int16, Float64
from geometry_msgs.msg import Twist

class TwistToMotor():
    def __init__(self):
        rospy.init_node("motor_control_gazebo")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.pub_steer_angle = rospy.Publisher('steer_joint_position_controller/command', Float64, queue_size=10)
        self.pub_drive_motor = rospy.Publisher('drive_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('tab_control', String, self.brakecb)
        rospy.Subscriber('front_blocked', String, self.sonarcb)
        self.bw = rospy.get_param("base_width", 0.3646)
        self.bl = rospy.get_param("base_length", 0.892)
        self.rate = rospy.get_param("rate", 50)
        self.timeout_ticks = rospy.get_param("timeout_ticks", 20)
        self.steer = 0.0
        self.drive = 0
        self.brakes = 0
        self.frontBlocked = 0
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0
        self.K_steer = rospy.get_param('steer_twist_factor',1)
        self.K_drive = rospy.get_param('drive_twist_factor',8)
    def spin(self):
        r = rospy.Rate(self.rate)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        while not rospy.is_shutdown():
            if self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
            self.ticks_since_target += 1
            r.sleep()
    def spinOnce(self):
        self.steer = self.K_steer * (math.atan2(self.bl*self.Wz, self.Vx))
        self.drive = self.K_drive * (math.hypot(self.bl*self.Wz, self.Vx))
        # if self.steer>1.3:
        #     self.steer = 1.3
        # elif self.steer<-1.3:
        #     self.steer = -1.3
        if self.brakes == 1:
            self.steer = self.drive = 0
        if self.frontBlocked == 1:
            if self.steer>0:
                self.steer = 0
            if self.drive>0:
                self.drive = 0
        self.pub_steer_angle.publish(self.steer)
        self.pub_drive_motor.publish(self.drive)
    def twistCallback(self,msg):
        self.ticks_since_target = 0
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.Wz = msg.angular.z
    def brakecb(self, msg):
        self.brakes = int(msg.data)
    def sonarcb(self, msg):
        self.frontBlocked = int(msg.data)

if __name__ == "__main__":
    """ main """
    twist_to_motor = TwistToMotor()
    twist_to_motor.spin()
