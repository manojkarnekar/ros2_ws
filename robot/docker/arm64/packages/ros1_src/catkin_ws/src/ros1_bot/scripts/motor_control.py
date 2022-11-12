#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg  import  String
from std_msgs.msg import Float32 , Int16
from geometry_msgs.msg import Twist 
class TwistToPID():
    def __init__(self):
        rospy.init_node("motor_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.w = rospy.get_param("base_width", 0.465)
        self.pub_lmotor = rospy.Publisher('lmotor_cmd', Int16,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rmotor_cmd', Int16,queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('tab_control', String, self.brakecb)
        rospy.Subscriber('front_blocked', String, self.sonarcb)
        self.rate = rospy.get_param("rate", 50)
        self.timeout_ticks = rospy.get_param("timeout_ticks", 20)
        self.left = 0
        self.right = 0
        self.brakes = 0
        self.frontBlocked = 0
        self.K = rospy.get_param('twist_factor',542)
    def spin(self):
        r = rospy.Rate(self.rate)
        #idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        while not rospy.is_shutdown():
            if self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
            self.ticks_since_target += 1
            r.sleep()
            #idle.sleep()
    def spinOnce(self):
        self.right = self.K * ((1.0 * self.dx) + (self.dr * self.w / 2 ))
        self.left = self.K * ((1.0 * self.dx) - (self.dr * self.w / 2))
	    
        if self.brakes == 1:
            self.left = self.right = 0

        if self.frontBlocked == 1:
            if self.left>0:
                self.left = 0
            if self.right>0:
                self.right = 0

        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        
    def twistCallback(self,msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    def brakecb(self, msg):
        self.brakes = int(msg.data)
    def sonarcb(self, msg):
        self.frontBlocked = int(msg.data)

if __name__ == '__main__':
    """ main """
    twistToMoto = TwistToPID()
    twistToMoto.spin()

