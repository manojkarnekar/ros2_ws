#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "depth_zed2_scan.hpp"
#include <iostream>
#include <vector>
#include "rclcpp/logger.hpp"
#include <bits/stdc++.h>
#include <math.h>


using namespace std;
zed2_laser_scan_header zed2_laser_scan_header;

using std::placeholders::_1;
using namespace std::chrono_literals;

class depth_zed2_scan : public rclcpp::Node
{
  public:
    depth_zed2_scan()
    : Node("depth_zed2_scan")
    {
         zed2_range_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/cv_pose", rclcpp::SensorDataQoS(), std::bind(&depth_zed2_scan::zed2_range_callback, this, _1)); 

      // timer_ = this->create_wall_timer(71.42ms, std::bind(&depth_zed2_scan::update, this));

      zed2_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("zed2_scan", rclcpp::QoS(rclcpp::KeepLast(10)));



    }

  private:
    void zed2_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr zed2_range_sub;

    void update();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub;
    

};




void depth_zed2_scan::update()
{
  
  zed2_laser_scan_header.zed2_range_scan(this->get_clock()->now(),"camera_link",zed2_laser_scan_header.zed2_to_laser(zed2_laser_scan_header.d2, zed2_laser_scan_header.th1 , zed2_laser_scan_header.th2),zed2_scan_pub,zed2_laser_scan_header.th1,zed2_laser_scan_header.th2);
}

void depth_zed2_scan::zed2_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)    
{    

    // for(int i=0; i<10; i++)
    // {
    //   zed2_laser_scan_header.raw[i] = msg->data[i];
    // }
    // zed2_laser_scan_header.th1=atan(zed2_laser_scan_header.raw[5]/zed2_laser_scan_header.raw[6]);
    // zed2_laser_scan_header.th2=atan(zed2_laser_scan_header.raw[7]/zed2_laser_scan_header.raw[6]);
    zed2_laser_scan_header.th1=atan(msg->data[6]/sqrt(msg->data[2]*msg->data[2]+msg->data[3]*msg->data[3]));
    zed2_laser_scan_header.th2=atan(msg->data[8]/sqrt(msg->data[2]*msg->data[2]+msg->data[3]*msg->data[3]));
    RCLCPP_INFO(get_logger(), "th1--->{%f}, th2--->{%f}, d2--->{%f}", zed2_laser_scan_header.th1 , zed2_laser_scan_header.th2, zed2_laser_scan_header.d2);
    zed2_laser_scan_header.d2 = sqrt(msg->data[2]*msg->data[2]+msg->data[3]*msg->data[3]);
    update();


}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<depth_zed2_scan>());
  rclcpp::shutdown();
  return 0;
}