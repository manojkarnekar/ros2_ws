#include <math.h>
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32MultiArray.h"

class IMU_raw
{
public:
	IMU_raw();
private:
    ros::NodeHandle n;
	ros::Subscriber imu_sub;
    ros::Publisher imu_pub ;
    void imu_raw_Cb(const std_msgs::Float32MultiArray& msg);

    

};

IMU_raw::IMU_raw()
{
    ROS_INFO("Started imu_raw_sub_pub node");
    imu_sub = n.subscribe("/imu",10, &IMU_raw::imu_raw_Cb, this);
    imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    double ax, ay, az, gx, gy, gz;
    float roll, pitch, yaw;
    double dt;
}

void IMU_raw::imu_raw_Cb(const std_msgs::Float32MultiArray& msg)
{
    double ax, ay, az, gx, gy, gz;
    ax = msg.data.at(0);
    ay = msg.data.at(1);
    az = msg.data.at(2);
    gx = msg.data.at(3);
    gy = msg.data.at(4);
    gz = msg.data.at(5);

    sensor_msgs::Imu imu_msg;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;

    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;

    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;

    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;

    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_pub.publish(imu_msg);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"IMU_raw");
	IMU_raw obj;
	ros::spin();
    return 0;
}