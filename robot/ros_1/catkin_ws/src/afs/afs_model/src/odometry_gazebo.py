#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16, Int64 , Int32, Float32
from sensor_msgs.msg import Imu, JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomTF:
    def __init__(self):
        rospy.init_node("odometry_gazebo")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = rospy.get_param('~rate',50)  # the rate at which to publish the transform
        self.base_length = float(rospy.get_param('base_length', 0.892)) # The wheel base length in meters
        self.drive_wheel_diameter = float(rospy.get_param('drive_wheel_diameter', 0.25)) # The diameter of drive wheel in meters
        self.base_frame_id = rospy.get_param('base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('odom_frame_id', 'odom') # the name of the odometry reference frame
        self.axis_direction = rospy.get_param('axis_direction', 1)

        self.dt = 1.0/self.rate
        self.steer_joint_name = "steer_joint"
        self.drive_joint_name = "drive_wheel_joint"

        self.heading = 0.0
        self.angle = 0.0
        self.angle_prev = 0.0
        self.drive_angle = 0.0
        self.drive_angle_prev = 0.0
        self.drive_dist = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.quaternion = None
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0
        self.Vd = 0.0

        rospy.Subscriber("joint_states", JointState, self.joint_states_cb, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pub_diff_twist = rospy.Publisher("diff_twist", Twist, queue_size=10)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        self.drive_dist = self.axis_direction * (0.5 * self.drive_wheel_diameter * (self.drive_angle - self.drive_angle_prev))
        self.angle = self.angle_prev + (self.drive_dist * math.sin(self.heading)/self.base_length)
        self.angle_mean = 0.5 * (self.angle + self.angle_prev)
        self.X += self.drive_dist*math.cos(self.heading)*math.cos(self.angle_mean)
        self.Y += self.drive_dist*math.cos(self.heading)*math.sin(self.angle_mean)
        self.quaternion = quaternion_from_euler(0.0, 0.0, self.angle)
        self.Vd = self.drive_dist/self.dt
        self.Vx = self.Vd * math.cos(self.heading)
        self.Wz = self.Vd * math.sin(self.heading) / self.base_length
        self.angle_prev = self.angle

        odom_tf = TransformStamped()
        odom_tf.header.stamp = now
        odom_tf.header.frame_id = self.odom_frame_id
        odom_tf.child_frame_id = self.base_frame_id
        odom_tf.transform.translation.x = self.X
        odom_tf.transform.translation.y = self.Y
        odom_tf.transform.translation.z = self.Z
        odom_tf.transform.rotation.x = self.quaternion[0]
        odom_tf.transform.rotation.y = self.quaternion[1]
        odom_tf.transform.rotation.z = self.quaternion[2]
        odom_tf.transform.rotation.w = self.quaternion[3]
        self.tf_broadcaster.sendTransform(odom_tf)

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

        diff_twist = odom.twist.twist
        self.pub_diff_twist.publish(diff_twist)

    def joint_states_cb(self, msg):
        if self.steer_joint_name in msg.name:
            i = msg.name.index(self.steer_joint_name)
            self.heading = msg.position[i]
        if self.drive_joint_name in msg.name:
            i = msg.name.index(self.drive_joint_name)
            self.drive_angle_prev = self.drive_angle
            self.drive_angle = msg.position[i]

if __name__ == "__main__":
    """ main """
    odometry = OdomTF()
    odometry.spin()
