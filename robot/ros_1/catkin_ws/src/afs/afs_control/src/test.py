#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

rospy.init_node('test_afs')
pub_drive_motor = rospy.Publisher('throttle_state', UInt16, queue_size=10)
sp = UInt16()
rospy.sleep(1)
sp.data = 255
pub_drive_motor.publish(sp)
rospy.sleep(2)
sp.data = 0
pub_drive_motor.publish(sp)
