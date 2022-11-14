
#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int16, Int64 , Int32
from geometry_msgs.msg import Point, Twist
from math import atan2, copysign, exp, pi

DRIVE_ENCODER_PPR = 1024
DRIVE_GEAR_RATIO = 18
DRIVE_PPR = DRIVE_ENCODER_PPR * DRIVE_GEAR_RATIO
wheel_D = 0.25
wheel_circum = pi * wheel_D
x = 0.0
y = 0.0
enc = 0

def callback(msg):
    
    global enc
    enc = msg.data

rospy.init_node ("afs_nav_controller")
sub = rospy.Subscriber("/drive_encoder", Int32, callback)
pub = rospy.Publisher("/throttle_state", Int16, queue_size=1)

speed = Int16()
r = rospy.Rate(10)

goal = Point()
goal.x = 6.0
goal.y = 0.0
total_rev = goal.x/wheel_circum
goal_ticks = -1 * int(total_rev * DRIVE_PPR)

rospy.sleep(0.5)

while not rospy.is_shutdown():
   
    if (goal_ticks > enc):
        speed.data = 2300
    else:
        speed.data = 2500
    
    pub.publish(speed)
    r.sleep()

print("END")
