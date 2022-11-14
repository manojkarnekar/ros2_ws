#!/usr/bin/env python

import rospy
import time as r
from geometry_msgs.msg import Twist

rospy.init_node('teleop_twist_keyboard')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

twist = Twist()
speed = 0.203
turn = 0.5

r.sleep(3)
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0


def myhook():
  	twist.linear.x = 0
 	print("Program END")
	pub.publish(twist)
	exit()



while not rospy.is_shutdown():
	#Forward movement
	twist.linear.x = 1 * speed

	# Publish.
	print("forward")
	pub.publish(twist)
	r.sleep(5)

	# Publish stop message when thread exits.
	twist.linear.x = 0

	# Publish.
	print("stop")
	pub.publish(twist)
	r.sleep(1)

	#Backward movement
	twist.linear.x = -1 * speed

	# Publish.
	print("backward")
	pub.publish(twist)
	r.sleep(5)

	# Publish stop message when thread exits.
	twist.linear.x = 0

	# Publish.
	print("stop")
	pub.publish(twist)
	r.sleep(1)

rospy.on_shutdown(myhook)
