#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32, UInt16, Int32
from geometry_msgs.msg import Twist
from collections import deque
import numpy as np
import os

class TwistToMotor():
    def __init__(self):
        rospy.init_node("motor_pid_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        self.pub_steer_angle = rospy.Publisher('heading', Float32, queue_size=10)
        self.pub_steer_angle.publish(0)
        self.pub_drive_motor = rospy.Publisher('throttle_state', UInt16, queue_size=10)

        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber('tab_control', String, self.brakecb)
        rospy.Subscriber('front_blocked', String, self.sonarcb)
        rospy.Subscriber('dreamvu/pal/persons/get/afs_state', Int32, self.pal_cb)
        rospy.Subscriber("heading_fb", Float32, self.heading_cb)
        rospy.Subscriber("drive_encoder", Int32, self.drive_enc_cb)


        self.bw = rospy.get_param("base_width", 0.3646)
        self.bl = rospy.get_param("base_length", 0.892)
        self.rate = rospy.get_param("rate", 20)
        self.timeout_ticks = rospy.get_param("timeout_ticks", 20)

        self.rate = rospy.get_param('~rate',50)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 23468))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('base_width', 0.3646)) # The wheel base width in meters
        self.base_length = float(rospy.get_param('base_length', 0.892)) # The wheel base width in meters

        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )

        self.dt = 1.0/self.rate
        self.motor_direction = rospy.get_param('motor_direction', 1)
        self.encoder_direction = rospy.get_param('encoder_direction', -1)

        self.error_buffer = deque(maxlen=10)
        self.err = 0
        self.heading = 0.0
        self.angle = 0.0
        self.angle_prev = 0.0
        self.dr_enc_prev = 0
        self.drive_enc = 0
        self.drive_enc_prev = 0
        self.drive_dist = 0.0
        self.dr_mult = 0

        self.steer = 0.0
        self.drive = 0
        self.brakes = 0
        self.frontBlocked = 0
        self.pal_blocked = 2
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0
        self.K_steer = rospy.get_param('steer_twist_factor',1)
        self.K_drive = rospy.get_param('drive_twist_factor',728)

        self.Kp = 0.8
        self.Ki = 0.0001
        self.Kd = 1.00



    def spin(self):
        r = rospy.Rate(self.rate)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        while not rospy.is_shutdown():
            # if self.ticks_since_target < self.timeout_ticks:
            self.spinOnce()
            # self.ticks_since_target += 1
            r.sleep()
    def spinOnce(self):
        self.steer = self.K_steer * (math.atan2(self.bl*self.Wz, self.Vx))
        self.drive = self.K_drive * (math.hypot(self.bl*self.Wz, self.Vx))

        self.steer = np.clip(self.steer, -1.45, 1.45)


        if self.brakes == 1:
            self.steer = self.drive = 0
        
        # self.drive_dist = self.encoder_direction * ((self.drive_enc - self.drive_enc_prev)/self.ticks_meter)
        # self.angle = self.angle_prev + (self.drive_dist * math.sin(self.heading)/self.base_length)
        # self.angle_mean = 0.5 * (self.angle + self.angle_prev)
        
        self.target_steer = self.steer
        self.actual_steer = self.heading
        
        self.steer = self.PID(self.Kp, self.Ki, self.Kd, self.target_steer, self.actual_steer)

        self.angle_prev = self.angle
        self.drive_enc_prev = self.drive_enc

        self.pub_steer_angle.publish(self.steer)
        self.pub_drive_motor.publish(self.drive)

    def twistCallback(self,msg):
        self.ticks_since_target = 0
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.Wz = msg.angular.z

    def brakecb(self, msg):
        self.brakes = int(msg.data)
    def sonarcb(self, msg):
        self.frontBlocked = int(msg.data)
    def pal_cb(self, msg):
        self.pal_blocked = msg.data
    
    def heading_cb(self, msg):
        self.heading = msg.data

    def drive_enc_cb(self, msg):
        dr_enc = msg.data
        if(dr_enc < self.encoder_low_wrap and self.dr_enc_prev > self.encoder_high_wrap):
            self.dr_mult = self.dr_mult + 1
        if(dr_enc > self.encoder_high_wrap and self.dr_enc_prev < self.encoder_low_wrap):
            self.dr_mult = self.dr_mult - 1
        self.drive_enc = 1.0 * (dr_enc + self.dr_mult * (self.encoder_max - self.encoder_min))
        self.dr_enc_prev = dr_enc
    
    def PID(self, Kp, Ki, Kd, target, actual):
        self.err = target - actual
        self.error_buffer.append(self.err)

        if len(self.error_buffer) >= 2:
            self.d = (self.error_buffer[-1] - self.error_buffer[-2])/self.dt
            self.i = sum(self.error_buffer)*self.dt
        else:
            self.d = 0.0
            self.i = 0.0
        
        self.pid = Kp*self.err + Ki*self.i + Kd*self.d
        self.pid = np.clip(self.pid, -1.45, 1.45)
        return self.pid
        

if __name__ == "__main__":
    """ main """
    twist_to_motor = TwistToMotor()
    twist_to_motor.spin()
