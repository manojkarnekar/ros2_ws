#include <memory>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "AHRS.hpp"

AHRS ahrs;
using std::placeholders::_1;

class IMU_raw_node : public rclcpp::Node
{
  public:
    IMU_raw_node()
    : Node("IMU_raw_node")
    {
      imu_mic = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/imu", 10, std::bind(&IMU_raw_node::imu_mic_cb, this, _1));

      imu_raw = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
      
    }

  private:
    void imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_mic;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw;
    float data = 0;
    float offset[3];
    int i = 0;
    double ax, ay, az, gx, gy, gz;
    rclcpp::Time then = this->get_clock()->now();
    double dt;
    float roll, pitch, yaw;

};

void IMU_raw_node::imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
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

    if(i<100)
    {
      offset[0] += (-gx*0.0175);
      offset[1] += (-gy*0.0175);
      offset[2] += (-gz*0.0175);
      if(i==99)
      {
        offset[0]/=100.0;
        offset[1]/=100.0;
        offset[2]/=100.0;
        ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
      }
      i+=1;
    }
    
    if(fabs(ax)<0.05) ax = 0.0;
    if(fabs(ay)<0.05) ay = 0.0;

    if(fabs(gx)<2.0) gx = 0.0;
    if(fabs(gy)<2.0) gy = 0.0;
    if(fabs(gz)<2.0) gz = 0.0;

    ahrs.updateIMU(ax, ay, az, gx*0.02, gy*0.02, gz*0.02, dt);

    ahrs.getEuler(&roll, &pitch, &yaw);

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

    imu_msg->orientation.w = ahrs.getW();
    imu_msg->orientation.x = ahrs.getX();
    imu_msg->orientation.y = ahrs.getY();
    imu_msg->orientation.z = ahrs.getZ();

    imu_msg->orientation_covariance[0] = 0.0025;
    imu_msg->orientation_covariance[1] = 0;
    imu_msg->orientation_covariance[2] = 0;

    imu_msg->orientation_covariance[3] = 0;
    imu_msg->orientation_covariance[4] = 0.0025;
    imu_msg->orientation_covariance[5] = 0;

    imu_msg->orientation_covariance[6] = 0;
    imu_msg->orientation_covariance[7] = 0;
    imu_msg->orientation_covariance[8] = 0.0025;
    
    imu_raw->publish(*imu_msg);
    then = now;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU_raw_node>());
  rclcpp::shutdown();
  return 0;
}