#ifndef zed2_LASER_SCAN
#define zed2_LASER_SCAN
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include "sensor_msgs/msg/laser_scan.hpp" 
#include <rclcpp/rclcpp.hpp>

using namespace std;

class zed2_laser_scan_header
{
public:
    unsigned int raw[10];
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
    float d2;

    vector<float> zed2_to_laser(float d2,float theta1,float theta2);
    void zed2_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float theta1,float theta2);
    vector<float> zed2;
    
    

    
    
      
      // vector<float> us_to_laser();
    // vector<float> us_to_laser(float d1,float d2,float d3,float d4,float d5,float d6,float d7,float d8,float d9);
   
};


vector<float> zed2_laser_scan_header::zed2_to_laser(float d2,float theta1,float theta2)
{
    vector<float> zed2;
    for (float t = -theta2; t <= theta1; t+=0.011)
    {
      float y;
        y=d2/cos(t);
    

      if (y < 4.00 && y >= 0.5)
      {
        zed2.push_back(y);
      }
      else
      {
        zed2.push_back(std::numeric_limits<double>::infinity());
      }

    }
  return zed2;
  cout<<"zed2"<<endl;
}

void zed2_laser_scan_header::zed2_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float theta1,float theta2)
{
  sensor_msgs::msg::LaserScan us_scan_msg;
  us_scan_msg.header.stamp = now;
  us_scan_msg.header.frame_id = frame_id;
  us_scan_msg.angle_min = -theta2;  //-40*(M_PI/180);
  us_scan_msg.angle_max = theta1;
  us_scan_msg.angle_increment = 0.011;
  us_scan_msg.time_increment = 0;
  us_scan_msg.scan_time = 0.0;
  us_scan_msg.range_min = 0.5;
  us_scan_msg.range_max = 4.00;
  us_scan_msg.ranges.resize(ranges.size());
  us_scan_msg.intensities.resize(ranges.size());
  us_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  us_scan_msg.intensities=inten;

  zed2_scan_pub->publish(us_scan_msg);

  inten.clear();
  ranges.clear();
}





#endif