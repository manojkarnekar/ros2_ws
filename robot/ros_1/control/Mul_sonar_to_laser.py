#!/usr/bin/env python
from operator import add
import rospy
import math
import numpy as np
import tf2_ros
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import std_msgs.msg
rospy.init_node('merged_laser')
laserscan_topics=rospy.get_param('laserscan_topics','/scan1')
scan_topics=laserscan_topics.split(" ")  
print(scan_topics)
size=len(scan_topics)
#rospy.loginfo("number of subscribers {}".format(size))
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
class Laser():
	def __init__(self):
		self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
		#self.scan = LaserScan()
		rospy.Subscriber('scan1', LaserScan, self.callback)
		for i in range(1, 6):
			rospy.Subscriber("sensor/us_m_{}_link".format(i), Range, self.callback1)
			#rospy.Subscriber("sensor/us_m_" + i + "_link",Range,self.callback1)
		#rospy.Subscriber("sensor/us_m_1_link",Range,self.callback1)
		#rospy.Subscriber("sensor/us_m_3_link", Range, self.callback1)
		#rospy.Subscriber("sensor/us_m_4_link",Range,self.callback1)
		#rospy.Subscriber("sensor/us_m_5_link",Range,self.callback1)
		rospy.spin()

		#self.num_readings = 20
                #self.laser_frequency = 40
		#self.sonar_frequency = 40
		#self.r = rospy.Rate(5.0)

	def callback(self,msg):
		self.laser = msg
       
	def callback1(self,msg):
    		ranges = msg.range
		l_range = math.tan(10) * ranges
		trans = tfBuffer.lookup_transform( msg.header.frame_id, 'laser', rospy.Time(0))
		#self.trans = tfBuffer.lookup_transform('us_m_1_link', 'base_link', rospy.Time(0))
    		tf_dist = math.sqrt((trans.transform.translation.x**2)+(trans.transform.translation.y**2))
		yaw = 2 * math.asin(trans.transform.rotation.z)
		yaw_in_degree = int((180/math.pi) * yaw)
		trans_range = add(ranges, tf_dist)
		theta = math.atan((l_range)/(ranges + tf_dist))
		theta_in_degree = int((180/math.pi) * theta)
		min_theta = 170 - theta_in_degree + yaw_in_degree
		print min_theta
		max_theta = 190 + theta_in_degree + yaw_in_degree
		print max_theta

		sranges = list(self.laser.ranges)
                for j in range(0, 360):
			if j in range(min_theta, max_theta):
				if trans_range < sranges[j]:            	      		
					sranges[j] = trans_range     
                self.laser.ranges = tuple(sranges)                 
		self.scan_pub.publish(self.laser)
		
if __name__=='__main__':
	Laser()
