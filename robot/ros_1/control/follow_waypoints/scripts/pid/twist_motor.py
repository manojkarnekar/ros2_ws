#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

class TwistToMotors():
    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.2)
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 5)
        self.left = 0
        self.right = 0

        self.K = rospy.get_param('twist_factor',1)

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
                
    def spinOnce(self):
        self.right = self.K * ((1.0 * self.dx) + (self.dr * self.w / 2 ))
        self.left = self.K * ((1.0 * self.dx) - (self.dr * self.w / 2))
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1

    def twistCallback(self,msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
