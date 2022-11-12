#ifndef US_LASER_SCAN
#define US_LASER_SCAN
#include <chrono>
#include <memory>
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
using namespace std;

class US_fusion_header 
{
public:
    rclcpp::Time time_1;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float time_increment_;
    float scan_time_;
    float range_min_;
    float range_max_;
    float field_of_view;
    bool radiation_type;
    // unsigned int raw[12];
    float raw[12];
    vector<float> uf;
    void ultrasonic_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr us_scan_pub);
}; 

void US_fusion_header::ultrasonic_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr us_scan_pub)
{
  sensor_msgs::msg::LaserScan us_scan_msg;
  us_scan_msg.header.stamp = now;
  us_scan_msg.header.frame_id = frame_id;
  us_scan_msg.angle_min = 0.0;  //-40*(M_PI/180);
  us_scan_msg.angle_max =  360*(M_PI/180);
  us_scan_msg.angle_increment = 0.5*(M_PI/180);
  us_scan_msg.time_increment = 0;
  us_scan_msg.scan_time = 0.0;
  us_scan_msg.range_min = 0.005;
  us_scan_msg.range_max = 3.00;
  us_scan_msg.ranges.resize(ranges.size());
  us_scan_msg.intensities.resize(ranges.size());
  us_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(long unsigned int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  us_scan_msg.intensities=inten;

  us_scan_pub->publish(us_scan_msg);

  inten.clear();
  ranges.clear();
}

#endif