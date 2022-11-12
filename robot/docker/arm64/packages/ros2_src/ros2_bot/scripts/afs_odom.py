#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16, Int64 , Int32, Float32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomTF:
    def __init__(self):
        rospy.init_node("odometry")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = rospy.get_param('~rate',50)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 23468))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('base_width', 0.3646)) # The wheel base width in meters
        self.base_length = float(rospy.get_param('base_length', 0.892)) # The wheel base width in meters

        self.base_frame_id = rospy.get_param('base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('odom_frame_id', 'odom') # the name of the odometry reference frame

        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )

        self.dt = 1.0/self.rate
        self.motor_direction = rospy.get_param('motor_direction', 1)
        self.encoder_direction = rospy.get_param('encoder_direction', 1)

        self.heading = 0.0
        self.angle = 0.0
        self.angle_prev = 0.0
        self.dr_enc_prev = 0
        self.drive_enc = 0
        self.drive_enc_prev = 0
        self.drive_dist = 0.0
        self.dr_mult = 0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.quaternion = None
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0
        self.Vd = 0.0

        rospy.Subscriber("heading_fb", Float32, self.heading_cb)
        rospy.Subscriber("drive_encoder", Int32, self.drive_enc_cb)
        self.pub_odom = rospy.Publisher("afs_odom", Odometry, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        self.drive_dist = self.encoder_direction * ((self.drive_enc - self.drive_enc_prev)/self.ticks_meter)
        self.angle = self.angle_prev + (self.drive_dist * math.sin(self.heading)/self.base_length)
        self.angle_mean = 0.5 * (self.angle + self.angle_prev)
        self.X += self.drive_dist*math.cos(self.heading)*math.cos(self.angle_mean)
        self.Y += self.drive_dist*math.cos(self.heading)*math.sin(self.angle_mean)
        self.quaternion = quaternion_from_euler(0.0, 0.0, self.angle)
        self.Vd = self.drive_dist/self.dt
        self.Vx = self.Vd * math.cos(self.heading)
        self.Wz = self.Vd * math.sin(self.heading) / self.base_length
        self.angle_prev = self.angle
        self.drive_enc_prev = self.drive_enc

        # odom_tf = TransformStamped()
        # odom_tf.header.stamp = now
        # odom_tf.header.frame_id = self.odom_frame_id
        # odom_tf.child_frame_id = self.base_frame_id
        # odom_tf.transform.translation.x = self.X
        # odom_tf.transform.translation.y = self.Y
        # odom_tf.transform.translation.z = self.Z
        # odom_tf.transform.rotation.x = self.quaternion[0]
        # odom_tf.transform.rotation.y = self.quaternion[1]
        # odom_tf.transform.rotation.z = self.quaternion[2]
        # odom_tf.transform.rotation.w = self.quaternion[3]
        # self.tf_broadcaster.sendTransform(odom_tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.X
        odom.pose.pose.position.y = self.Y
        odom.pose.pose.position.z = self.Z
        odom.pose.pose.orientation.x = self.quaternion[0]
        odom.pose.pose.orientation.y = self.quaternion[1]
        odom.pose.pose.orientation.z = self.quaternion[2]
        odom.pose.pose.orientation.w = self.quaternion[3]
        odom.twist.twist.linear.x = self.Vx
        odom.twist.twist.linear.y = self.Vy
        odom.twist.twist.angular.z = self.Wz
        self.pub_odom.publish(odom)

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

if __name__ == "__main__":
    """ main """
    odometry = OdomTF()
    odometry.spin()
