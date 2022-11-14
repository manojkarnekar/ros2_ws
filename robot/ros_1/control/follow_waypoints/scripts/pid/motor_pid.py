#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg  import  String
from std_msgs.msg import Float32 , Int16
from geometry_msgs.msg import Twist
import numpy as np

class TwistToPID():
    def __init__(self):
        rospy.init_node("motor_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        self.Kp = rospy.get_param('Kp',13)
        self.Ki = rospy.get_param('Ki',0)
        self.Kd = rospy.get_param('Kd',0)

        self.lprev_encoder   = 0
        self.rprev_encoder   = 0

        self.l_prev_encoder = 0
        self.r_prev_encoder = 0

        self.l_curr_encoder = 0
        self.r_curr_encoder = 0
        self.wheel_mult = 0

        self.l_feedback_vel_arr = []
        self.r_feedback_vel_arr = []

        self.w = rospy.get_param("base_width", 0.3646)
        self.ticks_per_meter = rospy.get_param('ticks_meter', 20)
        self.rate = rospy.get_param("rate", 50)
        self.timeout_ticks = rospy.get_param("timeout_ticks", 20)
        self.left = 0
        self.right = 0
        self.brakes = 0
        self.frontBlocked = 0

        self.e_prev = 0.0
        self.i = 0.0
        self.e = 0.0
        self.d = 0.0
        self.dt = 0.0

        self.K = rospy.get_param('twist_factor',1)


        rospy.Subscriber("lwheel_encoder", Int16, self.lwheelCallback)
        rospy.Subscriber("rwheel_encoder", Int16, self.rwheelCallback)

        self.pub_lmotor = rospy.Publisher('lmotor_cmd', Int16,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rmotor_cmd', Int16,queue_size=10)


        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )


        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('tab_control', String, self.brakecb)
        rospy.Subscriber('front_blocked', String, self.sonarcb)

        self.prev_pid_time = rospy.Time.now()

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
            # self.left_vel_u = 0
            # self.right_vel_u = 0
            #idle.sleep()

    def spinOnce(self):
        self.left_vel_u =   self.K*((1.0 * self.dx) - (self.dr * self.w / 2))
        self.right_vel_u =  self.K*((1.0 * self.dx) + (self.dr * self.w / 2 ))
        
        
        self.dt = rospy.Time.now() - self.prev_pid_time
        self.dt = self.dt.to_sec()
        self.l_feedback_vel = (self.l_curr_encoder - self.l_prev_encoder) / self.dt
        self.r_feedback_vel = (self.r_curr_encoder - self.r_prev_encoder) / self.dt

        self.l_feedback_vel_arr.append(self.l_feedback_vel)
        self.r_feedback_vel_arr.append(self.r_feedback_vel)

        # print("l_feedback_vel",self.l_feedback_vel,"r_feedback_vel",self.r_feedback_vel)
        # print("l_vel_u",self.left_vel_u,"r_vel_u",self.right_vel_u)
        # self.left_vel_u = 0
        # self.right_vel_u = 0
        try:
            self.left_vel_u = self.PID(self.Kp, self.Ki, self.Kd, self.left_vel_u, np.mean(self.l_feedback_vel_arr))
            self.right_vel_u = self.PID(self.Kp, self.Ki, self.Kd, self.right_vel_u, np.mean(self.l_feedback_vel_arr))
            print(self.left_vel_u , self.right_vel_u)

        except:
            self.left_vel_u = 0
            self.right_vel_u = 0

        self.l_prev_encoder = self.l_curr_encoder
        self.r_prev_encoder = self.r_curr_encoder
        self.prev_pid_time = rospy.Time.now()

        # if self.brakes == 1:
        #     self.left = self.right = 0

        # if self.frontBlocked == 1:
        #     if self.left>0:
        #         self.left = 0
        #     if self.right>0:
        #         self.right = 0

        self.pub_lmotor.publish(int(self.left_vel_u))
        self.pub_rmotor.publish(int(self.right_vel_u))

    def twistCallback(self,msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

        # if self.dx == 0:
        #     self.left_vel_u = 0
        #     self.right_vel_u = 0

    def brakecb(self, msg):
        self.brakes = int(msg.data)

    def sonarcb(self, msg):
        self.frontBlocked = int(msg.data)
    
    def lwheelCallback(self, msg):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.lprev_encoder   > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.lprev_encoder   < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
        
        self.l_curr_encoder = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.lprev_encoder   = enc
    
    def rwheelCallback(self, msg):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.rprev_encoder   > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.rprev_encoder   < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
        
        self.r_curr_encoder = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.rprev_encoder   = enc

    def PID(self, Kp, Ki, Kd, dis, pred):
        self.e = dis - pred
        self.i += self.e*self.dt
        
        self.d = (self.e - self.e_prev)/self.dt
        self.pid = Kp*self.e + Ki*self.i + Kd*self.d
        self.e_prev = self.e
        return self.pid


if __name__ == '__main__':
    """ main """
    twistToMoto = TwistToPID()
    twistToMoto.spin()


