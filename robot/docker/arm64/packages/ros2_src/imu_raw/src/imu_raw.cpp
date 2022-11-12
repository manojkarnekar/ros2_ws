#include "imu_raw/imu_raw.h"

void IMU_raw::imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Duration elapsed = now - then;
	  dt = elapsed.seconds();

    std::cout << "imu_data" <<msg->data.at(0)<<msg->data.at(1)<<msg->data.at(2)<<msg->data.at(3)<<msg->data.at(4)<<msg->data.at(5)<< std::endl;

    ax = msg->data.at(0);
    ay = msg->data.at(1);
    az = msg->data.at(2);
    gx = msg->data.at(3);
    gy = msg->data.at(4);
    gz = msg->data.at(5);


    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = this->get_clock()->now();
    imu_msg->header.frame_id = "imu_link";

    imu_msg->linear_acceleration.x = ax;
    imu_msg->linear_acceleration.y = ay;
    imu_msg->linear_acceleration.z = az;

    imu_msg->linear_acceleration_covariance[0] = 0.04;
    imu_msg->linear_acceleration_covariance[1] = 0;
    imu_msg->linear_acceleration_covariance[2] = 0;

    imu_msg->linear_acceleration_covariance[3] = 0;
    imu_msg->linear_acceleration_covariance[4] = 0.04;
    imu_msg->linear_acceleration_covariance[5] = 0;

    imu_msg->linear_acceleration_covariance[6] = 0;
    imu_msg->linear_acceleration_covariance[7] = 0;
    imu_msg->linear_acceleration_covariance[8] = 0.04;

    imu_msg->angular_velocity.x = gx;
    imu_msg->angular_velocity.y = gy;
    imu_msg->angular_velocity.z = gz;

    imu_msg->angular_velocity_covariance[0] = 0.02;
    imu_msg->angular_velocity_covariance[1] = 0;
    imu_msg->angular_velocity_covariance[2] = 0;

    imu_msg->angular_velocity_covariance[3] = 0;
    imu_msg->angular_velocity_covariance[4] = 0.02;
    imu_msg->angular_velocity_covariance[5] = 0;

    imu_msg->angular_velocity_covariance[6] = 0;
    imu_msg->angular_velocity_covariance[7] = 0;
    imu_msg->angular_velocity_covariance[8] = 0.02;

    imu_raw->publish(*imu_msg);
    then = now;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU_raw>());
  rclcpp::shutdown();
  return 0;
}