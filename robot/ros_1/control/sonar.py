#!/usr/bin/env python
import rospy
from std_msgs.msg  import UInt8MultiArray, String
from sensor_msgs.msg import Range
import std_msgs.msg
class Sonar():
	def __init__(self):
		rospy.init_node('sonars')
		self.sonars = []
#		self.sonars.append(rospy.Publisher('sonar_1', Range , queue_size=10))
		self.sonars.append(rospy.Publisher('sonar_2', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_3', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_4', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_5', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_6', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_7', Range , queue_size=10))
#		self.sonars.append(rospy.Publisher('sonar_8', Range , queue_size=10))
		rospy.Subscriber("sensors",UInt8MultiArray,self.callback)

		self.num_readings = len(self.sonars)
		self.sonar_frequency = 40
		self.r = rospy.Rate(5.0)
       
	def callback(self,msg):
		for i in range(self.num_readings):

			val =  float(ord(msg.data[i]))/100
			if val==0.0:
				val = 4.1

			h = std_msgs.msg.Header()
			h.stamp = rospy.Time.now()
		 	h.frame_id = 'us_sensor{}_link'.format(i+1)
			rang = Range()
			rang.header = h
			rang.field_of_view=0.30
			rang.min_range=0.05
			rang.max_range=4.1
			rang.range = val
			self.sonars[i].publish(rang)
		
if __name__=='__main__':
	Sonar()
	rospy.spin()
