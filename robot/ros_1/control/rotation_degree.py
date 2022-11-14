#!/usr/bin/env python


import tf
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

rospy.init_node('rotation_degree')
twist=Twist()
now = rospy.Time(0)
rate = rospy.Rate(10.0)
pub = rospy.Publisher('rotation_degree', Int16, queue_size = 1)
listener = tf.TransformListener()
while not rospy.is_shutdown():
    listener.waitForTransform("/map", "/base_link", now, rospy.Duration(30.0))
    (trans,rot) = listener.lookupTransform('/map', '/base_link', now)
    rot=tf.transformations.euler_from_quaternion(rot)
    b=int(rot[2]*57)
    pub.publish(b)
#    print b
    #rate.sleep()
