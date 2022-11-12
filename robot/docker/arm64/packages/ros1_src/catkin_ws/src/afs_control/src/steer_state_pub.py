#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32, UInt16, Int32
from geometry_msgs.msg import Twist

class TwistToMotor():
    def __init__(self):
        rospy.init_node("steer_vel_pub")
        self.pub_steer_angle = rospy.Publisher('heading', Float32, queue_size=10)
        self.steer = rospy.get_param("steer", 1)
        self.rate = rospy.get_param("rate", 20)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.pub_steer_angle.publish(self.steer)
            print("ok pub steer 1")
            r.sleep()

if __name__ == "__main__":
    """ main """
    twist_to_motor = TwistToMotor()
    twist_to_motor.spin()
