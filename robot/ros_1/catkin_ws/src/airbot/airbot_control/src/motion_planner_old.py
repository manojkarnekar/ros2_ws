#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

search_length = 2.0
search_width_max = 0.25
search_width_min = 0.15
search_height = 1.0
ground_clearance = 0.05
linear_velocity = 0.1
num_filter = 3
gain = 0.1

rospy.init_node('airbot_motion_planner')
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
velocity = Twist()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.sleep(1)
camera_bf_tf = tf_buffer.lookup_transform('camera_depth_optical_frame', 'base_footprint', rospy.Time())

x_bf = camera_bf_tf.transform.translation.x
y_bf = camera_bf_tf.transform.translation.y
z_bf = camera_bf_tf.transform.translation.z

sl_range = [0.0, search_length]
sw_range_right = [search_width_min+x_bf, search_width_max+x_bf]
sw_range_left = [-search_width_max+x_bf, -search_width_min+x_bf]
sh_range = [y_bf-ground_clearance-search_height, y_bf-ground_clearance]

def pc2_cb(pc2_data):
    motion_state = rospy.get_param('airbot_motion_state')
    if motion_state == 0:
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        cmd_pub.publish(velocity)
    else:
        st = rospy.get_time()
        dist_r = sl_range[1]
        dist_l = sl_range[1]
        num_r = 0
        num_l = 0
        points_gen = pc2.read_points(pc2_data, field_names = ("x", "y", "z"), skip_nans=True)
        for point in points_gen:
            if (sw_range_right[0] <= point[0] <= sw_range_right[1]) and (sh_range[0] <= point[1] <= sh_range[1]) and (sl_range[0] <= point[2] <= dist_r):
                dist_r = point[2]
                num_r += 1
            elif (sw_range_left[0] <= point[0] <= sw_range_left[1]) and (sh_range[0] <= point[1] <= sh_range[1]) and (sl_range[0] <= point[2] <= dist_l):
                dist_l = point[2]
                num_l += 1
        if(num_l < num_filter): dist_l = sl_range[1]
        if(num_r < num_filter): dist_r = sl_range[1]
        correction_factor = (dist_l - dist_r)/(dist_l + dist_r)
        velocity.linear.x = motion_state * linear_velocity
        velocity.angular.z = gain * correction_factor
        cmd_pub.publish(velocity)
        et = rospy.get_time()
        print(correction_factor, "R:", dist_r, "L:", dist_l, "T:", et-st)

rospy.Subscriber('/camera/depth/color/points', PointCloud2, pc2_cb, queue_size=1)

rospy.spin()
