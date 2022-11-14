#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('bot_tf_listener')
    listener = tf.TransformListener()

    map_frame_id = rospy.get_param('~goal_frame_id','map')
    odom_frame_id = rospy.get_param('~odom_frame_id','odom')
    base_frame_id = rospy.get_param('~base_frame_id','base_footprint')

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(map_frame_id, base_frame_id, rospy.Time(0))
            print("translations", trans, "rotations", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    
    # rospy.spin()