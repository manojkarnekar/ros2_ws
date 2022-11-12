#ifndef zed2_bed_LASER_SCAN
#define zed2_bed_LASER_SCAN
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include "sensor_msgs/msg/laser_scan.hpp" 
#include <rclcpp/rclcpp.hpp>

using namespace std;

class zed2_full_bed_laser_scan_header
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
    float divphi1;
    float divphi2;
    int insert;
    int insertphi=1;
    float th_i_d;
    float i_d;
    float lth;
    float rth;
    float gpx;
    float gpy;
    float gpz;
    float gpth;
    float m1;
    float m2;
    float th_i_d_;



    // vector<float> zed2_bed_to_laser(float dis,float theta1,float theta2);
    void zed2_full_bed_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_full_scan_pub,float theta1,float theta2,float incr);
    vector<float> zed2;
    vector<float> zed2_bed;

    
    

    
    
      
      // vector<float> us_to_laser();
    // vector<float> us_to_laser(float d1,float dis,float d3,float d4,float d5,float d6,float d7,float d8,float d9);
   
};


// vector<float> zed2_full_bed_laser_scan_header::zed2_bed_to_laser(float dis,float theta1,float theta2)
// {
//     vector<float> zed2;
//     for (float t = -theta2; t <= theta1; t+=0.11)
//     {
//       float y;
//         y=dis/cos((M_PI/180)*t);
    

//       if (y < 4.00 && y >= 0.5)
//       {
//         zed2.push_back(y);
//       }
//       else
//       {
//         zed2.push_back(std::numeric_limits<double>::infinity());
//       }

//     }
//   return zed2;
// }

void zed2_full_bed_laser_scan_header::zed2_full_bed_range_scan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_full_scan_pub,float theta1,float theta2,float incr)
{
  sensor_msgs::msg::LaserScan zed2_full_bed_scan_msg;
  zed2_full_bed_scan_msg.header.stamp = now;
  zed2_full_bed_scan_msg.header.frame_id = frame_id;
  zed2_full_bed_scan_msg.angle_min = theta2;  //-40*(M_PI/180);
  zed2_full_bed_scan_msg.angle_max = theta1;
  zed2_full_bed_scan_msg.angle_increment = incr;
  zed2_full_bed_scan_msg.time_increment = 0;
  zed2_full_bed_scan_msg.scan_time = 0.0;
  zed2_full_bed_scan_msg.range_min = 0.5;
  zed2_full_bed_scan_msg.range_max = 4.00;
  zed2_full_bed_scan_msg.ranges.resize(ranges.size());
  zed2_full_bed_scan_msg.intensities.resize(ranges.size());
  zed2_full_bed_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  zed2_full_bed_scan_msg.intensities=inten;

  zed2_full_scan_pub->publish(zed2_full_bed_scan_msg);

  inten.clear();
  ranges.clear();
}





#endif