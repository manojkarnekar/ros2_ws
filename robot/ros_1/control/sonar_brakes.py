#!/usr/bin/env python
import rospy
from std_msgs.msg  import UInt8MultiArray, String
import std_msgs.msg
class Sonar():
        def __init__(self):
                rospy.init_node("sonar_brakes")
                self.frontBrakePub = rospy.Publisher('front_blocked', String, queue_size=10, latch=True)
                rospy.Subscriber("sensors",UInt8MultiArray,self.sonar_cb)
        def sonar_cb(self, msg):
                sonar2 = ord(msg.data[2])
                sonar3 = ord(msg.data[3])
                if sonar2 in range(10, 70) or sonar3 in range(10, 70):
                        self.frontBrakePub.publish("1")
                else:
                        self.frontBrakePub.publish("0")
if __name__=='__main__':
        Sonar()
        rospy.spin()
