#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

class IMU_Fusion(object):
    def __init__(self):
        
        degrees2rad = math.pi/180.0
        imu_yaw_calibration = 0.0
        
        roll=0
        pitch=0
        yaw=0
        self.seq=0
        
        self.imuMsg = Imu()

        self.imuMsg.orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]

        self.imuMsg.angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]

        self.imuMsg.linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]
        
        self.pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        rospy.Subscriber("/imu", Float32MultiArray, self.Imu_callback)

    def Imu_callback(self,data):

        imu_data = data.data

        self.imuMsg.linear_acceleration.x = float(imu_data[0])
        self.imuMsg.linear_acceleration.y = float(imu_data[1])
        self.imuMsg.linear_acceleration.z = float(imu_data[2])

        self.imuMsg.angular_velocity.x = float(imu_data[3])
        self.imuMsg.angular_velocity.y = float(imu_data[4])
        self.imuMsg.angular_velocity.z = float(imu_data[5])

        self.imuMsg.header.stamp= rospy.Time.now()
        self.imuMsg.header.frame_id = "imu_link"
        self.imuMsg.header.seq = self.seq
        self.seq += 1
        self.pub.publish(self.imuMsg)

        

if __name__ == '__main__':
    rospy.init_node('imu_listener', anonymous=True)
    rospy.loginfo("-I- %s started" % rospy.get_name())
    IMU_Fusion()
    rospy.spin()