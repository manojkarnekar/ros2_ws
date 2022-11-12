#ifndef US_LASER_SCAN
#define US_LASER_SCAN
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include "sensor_msgs/msg/laser_scan.hpp" 
#include <rclcpp/rclcpp.hpp>

using namespace std;

class US_laser_scan_header 
{
public:
    //  header_; 
    float us_1,us_2,us_3,us_4,us_5,us_6,us_7,us_8,us_9,us_10,us_11,us_12;
    // float32 us1,us2,us3[1],us4[1],us5[1],us6[1],us7[1],us8[1],us9[1],us10[1],us11[1],us12[1];
    rclcpp::Time time_1,time_2,time_3,time_4,time_5,time_6,time_7,time_8,time_9,time_10,time_11,time_12;
    std::string frame_id_,frame_id_1,frame_id_2,frame_id_3,frame_id_4,frame_id_5,frame_id_6,frame_id_7,frame_id_8,frame_id_9,frame_id_10,frame_id_11,frame_id_12;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float time_increment_;
    float scan_time_;
    float range_min_;
    float range_max_;
    float field_of_view;
    bool radiation_type;
    float max_range=1.0;
    
    
    vector<float> s;
    
    

    
    
      
      // vector<float> us_to_laser();  
    // vector<float> us_to_laser(float d1,float d2,float d3,float d4,float d5,float d6,float d7,float d8,float d9);
    void ultrasonic_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr us_scan_pub);
    void us_range(rclcpp::Time now, float d,std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us);

}; 

// vector<float> US_laser_scan_header::us_to_laser(float d1,float d2,float d3,float d4,float d5,float d6,float d7,float d8,float d9)


void US_laser_scan_header::ultrasonic_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr us_scan_pub)
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
  for(int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  us_scan_msg.intensities=inten;

  us_scan_pub->publish(us_scan_msg);

  inten.clear();
  ranges.clear();
}

void US_laser_scan_header::us_range(rclcpp::Time now, float d,std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us)
{
  sensor_msgs::msg::Range rangeMsg;
  float distance = d;
  // rclcpp::Time now = this->get_clock()->now();
  rangeMsg.header.stamp = now;
  rangeMsg.header.frame_id = frame_id;  
  rangeMsg.radiation_type = 0,                     //0=ultrasonic, 1=IR
  rangeMsg.field_of_view = 0.6981;
  rangeMsg.min_range = 0.0;
  rangeMsg.max_range = 1.00;
  rangeMsg.range = distance;

  us->publish(rangeMsg);
}



#endif