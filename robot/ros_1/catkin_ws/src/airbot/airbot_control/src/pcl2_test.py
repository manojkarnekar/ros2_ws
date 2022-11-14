#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

rospy.init_node('point_cloud2_test')

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    count = 0
    st = rospy.get_time()
    gen = point_cloud2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
    for point in gen:
        count += 1
      # if (-0.25 <= point[0] <= 0.0) and (-0.75 <= point[1] <= 0.25) and (point[2] < 2.0): print point
    et = rospy.get_time()
    print("Time:", et-st, count)

rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_pointcloud, queue_size=1)

rospy.spin()
