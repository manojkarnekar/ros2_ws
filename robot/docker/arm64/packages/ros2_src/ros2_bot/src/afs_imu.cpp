#include <memory>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

class AFS_IMU_raw_node : public rclcpp::Node
{
  public:
    AFS_IMU_raw_node()
    : Node("AFS_IMU_raw_node")
    {
      imu_mic = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/imu", 10, std::bind(&AFS_IMU_raw_node::afs_imu_mic_cb, this, _1));

      imu_raw = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
      
    }

  private:
    void afs_imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_mic;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw;
    double ax, ay, az, gx, gy, gz;  

};

void AFS_IMU_raw_node::afs_imu_mic_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

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

    imu_msg->angular_velocity.x = gx;
    imu_msg->angular_velocity.y = gy;
    imu_msg->angular_velocity.z = gz;


    imu_msg->orientation_covariance[0] =  0.0025;
    imu_msg->orientation_covariance[1] =  0.0;
    imu_msg->orientation_covariance[2] =  0.0;

    imu_msg->orientation_covariance[3] =  0.0;
    imu_msg->orientation_covariance[4] =  0.0025;
    imu_msg->orientation_covariance[5] =  0.0;

    imu_msg->orientation_covariance[6] =  0.0;
    imu_msg->orientation_covariance[7] =  0.0;
    imu_msg->orientation_covariance[8] =  0.0025;


    imu_msg->angular_velocity_covariance[0] = 0.02;
    imu_msg->angular_velocity_covariance[1] = 0.0;
    imu_msg->angular_velocity_covariance[2] = 0.0;

    imu_msg->angular_velocity_covariance[3] = 0.0;
    imu_msg->angular_velocity_covariance[4] = 0.02;
    imu_msg->angular_velocity_covariance[5] = 0.0;

    imu_msg->angular_velocity_covariance[6] = 0.0;
    imu_msg->angular_velocity_covariance[7] = 0.0;
    imu_msg->angular_velocity_covariance[8] = 0.02;


    imu_msg->linear_acceleration_covariance[0] = 0.04;
    imu_msg->linear_acceleration_covariance[1] = 0.0;
    imu_msg->linear_acceleration_covariance[2] = 0.0;

    imu_msg->linear_acceleration_covariance[3] = 0.0;
    imu_msg->linear_acceleration_covariance[4] = 0.04;
    imu_msg->linear_acceleration_covariance[5] = 0.0;

    imu_msg->linear_acceleration_covariance[6] = 0.0;
    imu_msg->linear_acceleration_covariance[7] = 0.0;
    imu_msg->linear_acceleration_covariance[8] = 0.04;

    imu_raw->publish(*imu_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AFS_IMU_raw_node>());
  rclcpp::shutdown();
  return 0;
}