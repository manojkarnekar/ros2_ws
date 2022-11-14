#!/usr/bin/env python2

import sys
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, copysign, exp

x = 0.0
y = 0.0
theta = 0.0

def callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node ("air_nav_controller")
sub = rospy.Subscriber("/tricycle_controller/odom", Odometry, callback)
pub = rospy.Publisher("/tricycle_controller/cmd_vel", Twist, queue_size=1)

speed = Twist()
r = rospy.Rate(4)

goal = Point()
goal.x = 4.0
goal.y = 0.0

rospy.sleep(0.5)

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2 (inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.03:
        #speed.linear.x = 0.0
        speed.angular.z = copysign(0.1, angle_to_goal - theta)
    else:
        speed.linear.x = 1.0
        speed.angular.z = 0.0
    if inc_x < 0.2:
        print(inc_x)
        speed.linear.x = abs(exp(-inc_x)-1)
        #speed.angular.z = 0.0
        if inc_x < 0.01:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            sys.exit()

    pub.publish(speed)
    r.sleep()

print("END")
