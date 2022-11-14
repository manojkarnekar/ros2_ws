#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty

rospy.init_node('airbot_2d_obstacle_search')

footprint_l = 2.
footprint_w = 0.45

pub = rospy.Publisher('pose_correction', Float32, queue_size=1)

rospy.wait_for_service('/octomap_server/reset')
print("Subscribed to octomap reset server")
reset_map = rospy.ServiceProxy('/octomap_server/reset', Empty)

def map_cb(map_data):
    try:
      reset_map()
      print("Map Cleared")
    except rospy.ServiceException as e:
      print("Service did not process request: " + str(e))
    resolution = map_data.info.resolution
    grid_width = map_data.info.width
    grid_height = map_data.info.height
    grid_origin = map_data.info.origin.position
    grid_data = map_data.data
    x_o = grid_origin.x
    y_o = grid_origin.y
    dist_r = get_obstacle_length(0., -footprint_w/2., x_o, y_o, resolution, grid_width, grid_data)
    dist_l = get_obstacle_length(0., footprint_w/2., x_o, y_o, resolution, grid_width, grid_data)
    correction_factor = (dist_l - dist_r)/footprint_l
    print(correction_factor, "R:", dist_r, "L:", dist_l)

def get_obstacle_length(x_p, y_p, x_o, y_o, resolution, grid_width, grid_data):
    while x_p <= footprint_l:
        x_m = ((-x_o + x_p))
        y_m = ((-y_o + y_p))
        G_x = math.floor(x_m / resolution)
        G_y = math.floor(y_m / resolution)
        G_num = int(G_y * grid_width + G_x)
        G_occ = grid_data[G_num]
        if G_occ == 100:
            break
        x_p += resolution
    return x_p

rospy.Subscriber('/projected_map', OccupancyGrid, map_cb, queue_size=1)

rospy.spin()
