#ifndef zed2_bed_LASER_SCAN
#define zed2_bed_LASER_SCAN
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include "sensor_msgs/msg/laser_scan.hpp" 
#include <rclcpp/rclcpp.hpp>

using namespace std;

class zed2_bed_laser_scan_header
{
public:
    
    std::string frame_id_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float time_increment_;
    float scan_time_;
    float range_min_;
    float range_max_;
    float theta1;
    float theta2;
    float th1;
    float th2;
    float dis;
    float bdx;
    float bdx_1;
    float bdx_2;
    float p1;
    float p2;
    float d1;
    float d2;
    float d3;
    float d_1;
    float d_2;
    float psi1;
    float psi2;
    float phi1;
    float phi2;
    float div;
    float alpha_2;
    float alpha_1;
    float divphi1;
    float divphi2;
    int insert;
    int insertphi1;
    int insertphi2;
    float id;
    float th_i_d;
    float i_d;
    float lth;
    float rth;

    vector<float> zed2;


   
    void zed2_bed_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float theta1,float theta2,float incr);
    
    
};




void zed2_bed_laser_scan_header::zed2_bed_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float theta1,float theta2,float incr)
{
  sensor_msgs::msg::LaserScan zed2_bed_scan_msg;
  zed2_bed_scan_msg.header.stamp = now;
  zed2_bed_scan_msg.header.frame_id = frame_id;
  zed2_bed_scan_msg.angle_min = -theta2;  //-40*(M_PI/180);
  zed2_bed_scan_msg.angle_max = theta1;
  zed2_bed_scan_msg.angle_increment = incr;
  zed2_bed_scan_msg.time_increment = 0;
  zed2_bed_scan_msg.scan_time = 0.0;
  zed2_bed_scan_msg.range_min = 0.5;
  zed2_bed_scan_msg.range_max = 6.00;
  zed2_bed_scan_msg.ranges.resize(ranges.size());
  zed2_bed_scan_msg.intensities.resize(ranges.size());
  zed2_bed_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  zed2_bed_scan_msg.intensities=inten;

  zed2_scan_pub->publish(zed2_bed_scan_msg);

  inten.clear();
  ranges.clear();
}





#endif