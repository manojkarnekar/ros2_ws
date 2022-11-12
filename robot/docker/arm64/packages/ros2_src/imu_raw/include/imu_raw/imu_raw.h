#ifndef imu_raw_H
#define imu_raw_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class IMU_raw : public rclcpp::Node
{
  public:
    IMU_raw()
    : Node("IMU_raw")
    {
      imu_mic = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/imu", 10, std::bind(&IMU_raw::imu_mic_cb, this, _1));

      imu_raw = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
      
    }

  private:
    void imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_mic;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw;
    float data = 0;
    float offset[3];
    int i = 0;
    double ax = 0;
    double ay = 0;
    double az = 0;
    double gx = 0;
    double gy = 0; 
    double gz = 0;
    rclcpp::Time then = this->get_clock()->now();
    double dt = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

};



#endif