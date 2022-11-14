#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def head_joint_controller_pub():

    rospy.init_node('airbot_head_joint_controller_pub')

    joint_speed = 0.25 #radians/sec
    cycle_time = 1.0 #sec
    cycle_frequency = 1.0/cycle_time #Hz

    pub = rospy.Publisher('/head_joint_velocity_controller/command', Float64, queue_size=1)

    #Initialize cycle
    rospy.sleep(2.0)
    joint_vel = Float64(joint_speed)
    pub.publish(joint_vel)
    rospy.sleep(cycle_time/2.0)

    rate = rospy.Rate(cycle_frequency)

    while not rospy.is_shutdown():
        joint_speed *= -1
        joint_vel = Float64(joint_speed)
        pub.publish(joint_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        head_joint_controller_pub()
    except rospy.ROSInterruptException:
        pass
