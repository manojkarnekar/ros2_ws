#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "core0_us_sim.hpp"
#include <iostream>
#include "rclcpp/logger.hpp"
#include <bits/stdc++.h>

using namespace std;

Us_sim_header us_sim_header;

using std::placeholders::_1;
using namespace std::chrono_literals;



class us_range_pub_sub_sim : public rclcpp::Node
{
  public:
    us_range_pub_sub_sim()
    : Node("us_range_pub_sub_sim")
    {
      us1_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us1", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us1_range_callback, this, _1));
      us2_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us2", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us2_range_callback, this, _1));
      us3_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us3", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us3_range_callback, this, _1));
      us4_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us4", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us4_range_callback, this, _1));
      us5_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us5", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us5_range_callback, this, _1));
      us6_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us6", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us6_range_callback, this, _1));
      us7_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us7", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us7_range_callback, this, _1));
      us8_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us8", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us8_range_callback, this, _1));
      us9_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us9", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us9_range_callback, this, _1));
      us10_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us10", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us10_range_callback, this, _1));
      us11_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us11", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us11_range_callback, this, _1));
      us12_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us12", rclcpp::SensorDataQoS(), std::bind(&us_range_pub_sub_sim::us12_range_callback, this, _1));

      timer_ = this->create_wall_timer(50ms, std::bind(&us_range_pub_sub_sim::update, this));
      
      US_range_pub_1 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S1", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_2 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S2", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_3 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S3", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_4 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S4", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_5 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S5", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_6 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S6", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_7 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S7", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_8 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S8", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_9 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S9", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_10 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S10", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_11 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S11", rclcpp::QoS(rclcpp::KeepLast(10)));
      US_range_pub_12 = this->create_publisher<sensor_msgs::msg::LaserScan>("U_S12", rclcpp::QoS(rclcpp::KeepLast(10)));

     float us_1=0.0;

      us_range_pub_1 = this->create_publisher<sensor_msgs::msg::Range>("US_1", 10);
      us_range_pub_2 = this->create_publisher<sensor_msgs::msg::Range>("US_2", 10);
      us_range_pub_3 = this->create_publisher<sensor_msgs::msg::Range>("US_3", 10);
      us_range_pub_4 = this->create_publisher<sensor_msgs::msg::Range>("US_4", 10);
      us_range_pub_5 = this->create_publisher<sensor_msgs::msg::Range>("US_5", 10);
      us_range_pub_6 = this->create_publisher<sensor_msgs::msg::Range>("US_6", 10);
      us_range_pub_7 = this->create_publisher<sensor_msgs::msg::Range>("US_7", 10);
      us_range_pub_8 = this->create_publisher<sensor_msgs::msg::Range>("US_8", 10);
      us_range_pub_9 = this->create_publisher<sensor_msgs::msg::Range>("US_9", 10);
      us_range_pub_10 = this->create_publisher<sensor_msgs::msg::Range>("US_10", 10);
      us_range_pub_11 = this->create_publisher<sensor_msgs::msg::Range>("US_11", 10);
      us_range_pub_12 = this->create_publisher<sensor_msgs::msg::Range>("US_12", 10);

      // float us_1=0;
      
   
   // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html    
 //    sensor_msgs/Range.msg
//Raw Message Definition
//uint8 data
// Header header 
//uint8 ULTRASOUND=0   uint8 INFRARED=1    uint8 radiation_type   
// float32 field_of_view  float32 min_range       # minimum range value [m]
// float32 max_range       # maximum range value [m]
// float32 range           # range data [m] # (Note: values < range_min or > range_max

//uint8 ULTRASOUND=0 ,uint8 INFRARED=1, std_msgs/Header header, uint8 radiation_type, float32 field_of_view, float32 min_range, float32 max_range, float32 range

      
    }
    
private:
    float us_1=0.0;

    void us1_range_callback(const sensor_msgs::msg::Range::SharedPtr us1_msg);
    
    void us2_range_callback(const sensor_msgs::msg::Range::SharedPtr us2_msg);
    void us3_range_callback(const sensor_msgs::msg::Range::SharedPtr us3_msg);
    void us4_range_callback(const sensor_msgs::msg::Range::SharedPtr us4_msg);
    void us5_range_callback(const sensor_msgs::msg::Range::SharedPtr us5_msg);
    void us6_range_callback(const sensor_msgs::msg::Range::SharedPtr us6_msg);
    void us7_range_callback(const sensor_msgs::msg::Range::SharedPtr us7_msg);
    void us8_range_callback(const sensor_msgs::msg::Range::SharedPtr us8_msg);
    void us9_range_callback(const sensor_msgs::msg::Range::SharedPtr us9_msg);
    void us10_range_callback(const sensor_msgs::msg::Range::SharedPtr us10_msg);
    void us11_range_callback(const sensor_msgs::msg::Range::SharedPtr us11_msg);
    void us12_range_callback(const sensor_msgs::msg::Range::SharedPtr us12_msg);


    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us1_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us2_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us3_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us4_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us5_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us6_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us7_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us8_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us9_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us10_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us11_range_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us12_range_sub;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_1;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_2;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_3;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_4;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_5;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_6;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_7;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_8;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_9;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_10;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_11;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_pub_12;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_1;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_2;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_3;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_4;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_5;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_6;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_7;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_8;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_9;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_10;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_11;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_12;
    // uint8_t d;
    // std::string frame_id;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub;

    // void us_range(unsigned int d,std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us);
    // void us_range(unsigned int d,std::string frame_id);
    // void us_range(uint8_t d,std::string frame_id);
    // void us_range(unsigned int d);
    void update();
    rclcpp::TimerBase::SharedPtr timer_;



    
  //  https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_node.html
    
    // float32 data ;
    int i = 0;
    uint8_t U_S1, U_S2, U_S3, U_S4, U_S5, U_S6, U_S7, U_S8, U_S9, U_S10, U_S11, U_S12;
//     uint8_t US_1,US_2,US_3,US_4,US_5,US_6,US_7,US_8,US_9,US_10,US_11,US_12;
    // float us_1;
    rclcpp::Time then = this->get_clock()->now();
    
    

    
    
};
// cout<<"check"<<us_1<<endl;

void us_range_pub_sub_sim::us1_range_callback(const sensor_msgs::msg::Range::SharedPtr us1_msg)    
{     
  
  // double distance = d;
  // rclcpp::Time now = this->get_clock()->now();
  us_sim_header.time_1 = us1_msg->header.stamp;
  us_sim_header.frame_id_1= us1_msg->header.frame_id;  
  // us_sim_header.radiation_type = us1_msg->radiation_type;                  //0=ultrasonic, 1=IR
  // us_sim_header.field_of_view = us1_msg->field_of_view;
  // us_sim_header.angle_min_ = us1_msg->min_range;
  // us_sim_header.angle_max_ = us1_msg->max_range;
  us_sim_header.us_1 = us1_msg->range;
  // cout<<us_sim_header.frame_id_<<endl;
  // cout<<us_sim_header.us_1<<endl;
      if (us_sim_header.us_1 > us_sim_header.max_range)
      {
        us_sim_header.us_1= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_1=us_sim_header.us_1;
      }

  
}
void us_range_pub_sub_sim::us2_range_callback(const sensor_msgs::msg::Range::SharedPtr us2_msg)    
{       us_sim_header.time_2 = us2_msg->header.stamp;
        us_sim_header.frame_id_2= us2_msg->header.frame_id;
        us_sim_header.us_2 = us2_msg->range;
      // us_sim_header.cout<<us_sim_header.us_2;
        if (us_sim_header.us_2 >  us_sim_header.max_range)
      {
        us_sim_header.us_2= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_2=us_sim_header.us_2;
      }

}
void us_range_pub_sub_sim::us3_range_callback(const sensor_msgs::msg::Range::SharedPtr us3_msg)    
{     
        us_sim_header.time_3 = us3_msg->header.stamp;
        us_sim_header.frame_id_3= us3_msg->header.frame_id;
        us_sim_header.us_3 = us3_msg->range;

      // us_sim_header.cout<<us_sim_header.us_3;
        if (us_sim_header.us_3 >  us_sim_header.max_range)
      {
        us_sim_header.us_3= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_3=us_sim_header.us_3;
      }
}
void us_range_pub_sub_sim::us4_range_callback(const sensor_msgs::msg::Range::SharedPtr us4_msg)    
{     
        us_sim_header.time_4 = us4_msg->header.stamp;
        us_sim_header.frame_id_4= us4_msg->header.frame_id;
        us_sim_header.us_4 = us4_msg->range;
      // us_sim_header.cout<<us_sim_header.us_4;
        if (us_sim_header.us_4 >  us_sim_header.max_range)
      {
        us_sim_header.us_4= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_4=us_sim_header.us_4;
      }
}
void us_range_pub_sub_sim::us5_range_callback(const sensor_msgs::msg::Range::SharedPtr us5_msg)    
{     
        us_sim_header.time_5 = us5_msg->header.stamp;
        us_sim_header.frame_id_5= us5_msg->header.frame_id;
        us_sim_header.us_5 = us5_msg->range;
        if (us_sim_header.us_5 >  us_sim_header.max_range)
      {
        us_sim_header.us_5= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_5=us_sim_header.us_5;
      }
        // cout<<us_sim_header.us_5;
}
void us_range_pub_sub_sim::us6_range_callback(const sensor_msgs::msg::Range::SharedPtr us6_msg)    
{     
        us_sim_header.time_6 = us6_msg->header.stamp;
        us_sim_header.frame_id_6= us6_msg->header.frame_id;
        us_sim_header.us_6 = us6_msg->range;
        if (us_sim_header.us_6 >  us_sim_header.max_range)
      {
        us_sim_header.us_6= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_6=us_sim_header.us_6;
      }
        // cout<<us_sim_header.us_6;
}
void us_range_pub_sub_sim::us7_range_callback(const sensor_msgs::msg::Range::SharedPtr us7_msg)    
{     
        us_sim_header.time_7 = us7_msg->header.stamp;
        us_sim_header.frame_id_7= us7_msg->header.frame_id;
        us_sim_header.us_7 = us7_msg->range;
        if (us_sim_header.us_7 >  us_sim_header.max_range)
      {
        us_sim_header.us_7= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_7=us_sim_header.us_7;
      }
        // cout<<us_sim_header.us_7;
}
void us_range_pub_sub_sim::us8_range_callback(const sensor_msgs::msg::Range::SharedPtr us8_msg)    
{     
        us_sim_header.time_8 = us8_msg->header.stamp;
        us_sim_header.frame_id_8= us8_msg->header.frame_id;
        us_sim_header.us_8 = us8_msg->range;
        if (us_sim_header.us_8 >  us_sim_header.max_range)
      {
        us_sim_header.us_8= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_8=us_sim_header.us_8;
      }
        // cout<<us_sim_header.us_8;
}
void us_range_pub_sub_sim::us9_range_callback(const sensor_msgs::msg::Range::SharedPtr us9_msg)    
{     
        us_sim_header.time_9 = us9_msg->header.stamp;
        us_sim_header.frame_id_9= us9_msg->header.frame_id;
        us_sim_header.us_9 = us9_msg->range;
        if (us_sim_header.us_9 >  us_sim_header.max_range)
      {
        us_sim_header.us_9= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_9=us_sim_header.us_9;
      }
        // cout<<us_sim_header.us_9;
}
void us_range_pub_sub_sim::us10_range_callback(const sensor_msgs::msg::Range::SharedPtr us10_msg)    
{     
        us_sim_header.time_10 = us10_msg->header.stamp;
        us_sim_header.frame_id_10= us10_msg->header.frame_id;
        us_sim_header.us_10 = us10_msg->range;
        if (us_sim_header.us_10 >  us_sim_header.max_range)
      {
        us_sim_header.us_10= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_10=us_sim_header.us_10;
      }
        // cout<<us_sim_header.us_10;
}
void us_range_pub_sub_sim::us11_range_callback(const sensor_msgs::msg::Range::SharedPtr us11_msg)    
{     
        us_sim_header.time_11 = us11_msg->header.stamp;
        us_sim_header.frame_id_11= us11_msg->header.frame_id;
        us_sim_header.us_11 = us11_msg->range;
        if (us_sim_header.us_11 > us_sim_header.max_range)
      {
        us_sim_header.us_11=us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_11=us_sim_header.us_11;
      }
        // cout<<us_sim_header.us_11;
}
void us_range_pub_sub_sim::us12_range_callback(const sensor_msgs::msg::Range::SharedPtr us12_msg)    
{     
        us_sim_header.time_12 = us12_msg->header.stamp;
        us_sim_header.frame_id_12= us12_msg->header.frame_id;
        us_sim_header.us_12 = us12_msg->range;
        if (us_sim_header.us_12 >  us_sim_header.max_range)
      {
        us_sim_header.us_12= us_sim_header.max_range;
      }
      else
      {
        us_sim_header.us_12=us_sim_header.us_12;
      }
        // cout<<us_sim_header.us_12<<endl;
}


// {   float us_1;
//         sensor_msgs::msg::Range us_scan1;
//         us_scan1.frame_id_=us1_msg->header.frame_id;
//         us_scan1.us_1=us1_msg->range;

//         // us_sim_header.us_1 = us1_msg->ranges;
//         // cout<<us_1<< endl;
//         // cout<<"check"<<us1_msg->range<<endl;
//     }

void us_range_pub_sub_sim::update()
{
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_1, us_sim_header.frame_id_1,us_sim_header.us_to_laser(us_sim_header.us_1),US_range_pub_1);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_2, us_sim_header.frame_id_2,us_sim_header.us_to_laser(us_sim_header.us_2),US_range_pub_2);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_3, us_sim_header.frame_id_3,us_sim_header.us_to_laser(us_sim_header.us_3),US_range_pub_3);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_4, us_sim_header.frame_id_4,us_sim_header.us_to_laser(us_sim_header.us_4),US_range_pub_4);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_5, us_sim_header.frame_id_5,us_sim_header.us_to_laser(us_sim_header.us_5),US_range_pub_5);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_6, us_sim_header.frame_id_6,us_sim_header.us_to_laser(us_sim_header.us_6),US_range_pub_6);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_7, us_sim_header.frame_id_7,us_sim_header.us_to_laser(us_sim_header.us_7),US_range_pub_7);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_8, us_sim_header.frame_id_8,us_sim_header.us_to_laser(us_sim_header.us_8),US_range_pub_8);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_9, us_sim_header.frame_id_9,us_sim_header.us_to_laser(us_sim_header.us_9),US_range_pub_9);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_10, us_sim_header.frame_id_10,us_sim_header.us_to_laser(us_sim_header.us_10),US_range_pub_10);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_11, us_sim_header.frame_id_11,us_sim_header.us_to_laser(us_sim_header.us_11),US_range_pub_11);
  us_sim_header.ultrasonic_range_scan(us_sim_header.time_12, us_sim_header.frame_id_12,us_sim_header.us_to_laser(us_sim_header.us_12),US_range_pub_12);
  
  us_sim_header.us_range(us_sim_header.time_1,us_sim_header.us_1,us_sim_header.frame_id_1,us_range_pub_1);
  us_sim_header.us_range(us_sim_header.time_2,us_sim_header.us_2,us_sim_header.frame_id_2,us_range_pub_2);
  us_sim_header.us_range(us_sim_header.time_3,us_sim_header.us_3,us_sim_header.frame_id_3,us_range_pub_3);
  us_sim_header.us_range(us_sim_header.time_4,us_sim_header.us_4,us_sim_header.frame_id_4,us_range_pub_4);
  us_sim_header.us_range(us_sim_header.time_5,us_sim_header.us_5,us_sim_header.frame_id_5,us_range_pub_5);
  us_sim_header.us_range(us_sim_header.time_6,us_sim_header.us_6,us_sim_header.frame_id_6,us_range_pub_6);
  us_sim_header.us_range(us_sim_header.time_7,us_sim_header.us_7,us_sim_header.frame_id_7,us_range_pub_7);
  us_sim_header.us_range(us_sim_header.time_8,us_sim_header.us_8,us_sim_header.frame_id_8,us_range_pub_8);
  us_sim_header.us_range(us_sim_header.time_9,us_sim_header.us_9,us_sim_header.frame_id_9,us_range_pub_9);
  us_sim_header.us_range(us_sim_header.time_10,us_sim_header.us_10,us_sim_header.frame_id_10,us_range_pub_10);
  us_sim_header.us_range(us_sim_header.time_11,us_sim_header.us_11,us_sim_header.frame_id_11,us_range_pub_11);
  us_sim_header.us_range(us_sim_header.time_12,us_sim_header.us_12,us_sim_header.frame_id_12,us_range_pub_12);
}
 





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<us_range_pub_sub_sim>());
  rclcpp::shutdown();
  return 0;
}





