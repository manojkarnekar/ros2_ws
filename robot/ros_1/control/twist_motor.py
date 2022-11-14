#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Float32,Int32, UInt8MultiArray, Bool, String
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Range
class TwistToMotors():
    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.34)
        
        #rospy.Subscriber("sonar_6",Range, self.sonar6_callback)
        #rospy.Subscriber("sonar_7",Range, self.sonar7_callback)
        #rospy.Subscriber("sonar_8",Range, self.sonar8_callback)

        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        #self.pub_lmotor = rospy.Publisher('lwheel', Float32,queue_size=10)
        #self.pub_lmotor = rospy.Publisher('rwheel', Float32,queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('tab_control',String,self.ekillCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 100)
        self.left = 0
        self.right = 0
        self.switch_status = 0

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(100)
        #then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()


    def spinOnce(self):

        # dx = (l + r) / 2
        # dr = (r - l) / w

        #print self.switch_status
        if (self.switch_status == "1"):
            self.right = 0.0
            self.left = 0.0

            self.pub_lmotor.publish(self.left)
            self.pub_rmotor.publish(self.right)

        else:

            self.right = 1.0 * self.dx + self.dr * self.w / 2 
            self.left = 1.0 * self.dx - self.dr * self.w / 2
            # rospy.loginfo("publishing: (%d, %d)", left, right) 
                        
            self.pub_lmotor.publish(self.left)
            self.pub_rmotor.publish(self.right)

            #print "left :",self.left, "right :", self.right
                
            self.ticks_since_target += 1

    def twistCallback(self,msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

  

    def ekillCallback(self,msg):
        self.switch_status = msg.data
        

        
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
