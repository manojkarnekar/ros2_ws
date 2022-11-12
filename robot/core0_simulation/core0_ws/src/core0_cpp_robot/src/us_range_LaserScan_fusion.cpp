#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "us_range_LaserScan_fusion.hpp"
#include <iostream>
#include <vector>
#include "rclcpp/logger.hpp"
#include <bits/stdc++.h>
#include <math.h>
// #include <matrix.h>

using namespace std;

US_laser_scan_header us_laser_scan_header; 

using std::placeholders::_1;
using namespace std::chrono_literals;

class us_range_LaserScan_fusion : public rclcpp::Node
{
  public:
    us_range_LaserScan_fusion()
    : Node("us_range_LaserScan_fusion")
    {
         us1_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us1", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us1_range_callback, this, _1));
      us2_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us2", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us2_range_callback, this, _1));
      us3_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us3", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us3_range_callback, this, _1));
      us4_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us4", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us4_range_callback, this, _1));
      us5_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us5", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us5_range_callback, this, _1));
      us6_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us6", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us6_range_callback, this, _1));
      us7_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us7", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us7_range_callback, this, _1));
      us8_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us8", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us8_range_callback, this, _1));
      us9_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us9", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us9_range_callback, this, _1));
      us10_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us10", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us10_range_callback, this, _1));
      us11_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us11", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us11_range_callback, this, _1));
      us12_range_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/us12", rclcpp::SensorDataQoS(), std::bind(&us_range_LaserScan_fusion::us12_range_callback, this, _1));

      timer_ = this->create_wall_timer(50ms, std::bind(&us_range_LaserScan_fusion::update, this)); 
      
      US_range_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("Us_scan", rclcpp::QoS(rclcpp::KeepLast(10)));

      // us_range_pub_1 = this->create_publisher<sensor_msgs::msg::Range>("US_1", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_2 = this->create_publisher<sensor_msgs::msg::Range>("US_2", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_3 = this->create_publisher<sensor_msgs::msg::Range>("US_3", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_4 = this->create_publisher<sensor_msgs::msg::Range>("US_4", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_5 = this->create_publisher<sensor_msgs::msg::Range>("US_5", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_6 = this->create_publisher<sensor_msgs::msg::Range>("US_6", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_7 = this->create_publisher<sensor_msgs::msg::Range>("US_7", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_8 = this->create_publisher<sensor_msgs::msg::Range>("US_8", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_9 = this->create_publisher<sensor_msgs::msg::Range>("US_9", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_10 = this->create_publisher<sensor_msgs::msg::Range>("US_10", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_11 = this->create_publisher<sensor_msgs::msg::Range>("US_11", rclcpp::QoS(rclcpp::KeepLast(10)));
      // us_range_pub_12 = this->create_publisher<sensor_msgs::msg::Range>("US_12", rclcpp::QoS(rclcpp::KeepLast(10)));
    

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

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_range_scan_pub;

    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_1;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_2;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_3;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_4;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_5;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_6;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_7;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_8;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_9;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_10;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_11;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_12;
    
    void update();
    rclcpp::TimerBase::SharedPtr timer_;
    
    vector<float> us_to_laser();
    
    vector<float> cordinate_transformation(float x, float dx, float dy, float theta);

    void range_replace();

    float d_,x_,y_;

    int phic_,phi_,us_,ue_;

    
  //  https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_node.html
    
   
    int i = 0;

    rclcpp::Time then = this->get_clock()->now();
    
};

void us_range_LaserScan_fusion::us1_range_callback(const sensor_msgs::msg::Range::SharedPtr us1_msg)    
{     
  
  // double distance = d;
  // rclcpp::Time now = this->get_clock()->now();
  us_laser_scan_header.time_1 = us1_msg->header.stamp;
  us_laser_scan_header.frame_id_1= us1_msg->header.frame_id;  
  // us_laser_scan_header.radiation_type = us1_msg->radiation_type;                  //0=ultrasonic, 1=IR
  // us_laser_scan_header.field_of_view = us1_msg->field_of_view;
  // us_laser_scan_header.angle_min_ = us1_msg->min_range;
  // us_laser_scan_header.angle_max_ = us1_msg->max_range;
  us_laser_scan_header.us_1 = us1_msg->range;
  
  
  // cout<<us_laser_scan_header.us_1<<endl;
      if (us_laser_scan_header.us_1 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_1= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_1=us_laser_scan_header.us_1;
        
        // d1=us_laser_scan_header.us_1;
        
      }
    
  // cout<<us_laser_scan_header.frame_id_1<<endl;
  // cout <<"us_laser_scan_header.us_1 Test"<<endl; 
  // cout<<us_laser_scan_header.us_1<<endl;
  // cout<<
}

void us_range_LaserScan_fusion::us2_range_callback(const sensor_msgs::msg::Range::SharedPtr us2_msg)    
{       us_laser_scan_header.time_2 = us2_msg->header.stamp;
        us_laser_scan_header.frame_id_2= us2_msg->header.frame_id;
        us_laser_scan_header.us_2 = us2_msg->range;
      // us_laser_scan_header.cout<<us_laser_scan_header.us_2;
        if (us_laser_scan_header.us_2 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_2= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_2=us_laser_scan_header.us_2;
        // d2=us_laser_scan_header.us_2;
      }

}
void us_range_LaserScan_fusion::us3_range_callback(const sensor_msgs::msg::Range::SharedPtr us3_msg)    
{     
        us_laser_scan_header.time_3 = us3_msg->header.stamp;
        us_laser_scan_header.frame_id_3= us3_msg->header.frame_id;
        us_laser_scan_header.us_3 = us3_msg->range;

      // us_laser_scan_header.cout<<us_laser_scan_header.us_3;
        if (us_laser_scan_header.us_3 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_3= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_3=us_laser_scan_header.us_3;
      }
}
void us_range_LaserScan_fusion::us4_range_callback(const sensor_msgs::msg::Range::SharedPtr us4_msg)    
{     
        us_laser_scan_header.time_4 = us4_msg->header.stamp;
        us_laser_scan_header.frame_id_4= us4_msg->header.frame_id;
        us_laser_scan_header.us_4 = us4_msg->range;
      // us_laser_scan_header.cout<<us_laser_scan_header.us_4;
        if (us_laser_scan_header.us_4 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_4= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_4=us_laser_scan_header.us_4;
      }
}
void us_range_LaserScan_fusion::us5_range_callback(const sensor_msgs::msg::Range::SharedPtr us5_msg)    
{     
        us_laser_scan_header.time_5 = us5_msg->header.stamp;
        us_laser_scan_header.frame_id_5= us5_msg->header.frame_id;
        us_laser_scan_header.us_5 = us5_msg->range;
        if (us_laser_scan_header.us_5 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_5= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_5=us_laser_scan_header.us_5;
      }
        // cout<<us_laser_scan_header.us_5;
}
void us_range_LaserScan_fusion::us6_range_callback(const sensor_msgs::msg::Range::SharedPtr us6_msg)    
{     
        us_laser_scan_header.time_6 = us6_msg->header.stamp;
        us_laser_scan_header.frame_id_6= us6_msg->header.frame_id;
        us_laser_scan_header.us_6 = us6_msg->range;
        if (us_laser_scan_header.us_6 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_6= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_6=us_laser_scan_header.us_6;
      }
        // cout<<us_laser_scan_header.us_6;
}
void us_range_LaserScan_fusion::us7_range_callback(const sensor_msgs::msg::Range::SharedPtr us7_msg)    
{     
        us_laser_scan_header.time_7 = us7_msg->header.stamp;
        us_laser_scan_header.frame_id_7= us7_msg->header.frame_id;
        us_laser_scan_header.us_7 = us7_msg->range;
        if (us_laser_scan_header.us_7 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_7= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_7=us_laser_scan_header.us_7;
      }
        // cout<<us_laser_scan_header.us_7;
}
void us_range_LaserScan_fusion::us8_range_callback(const sensor_msgs::msg::Range::SharedPtr us8_msg)    
{     
        us_laser_scan_header.time_8 = us8_msg->header.stamp;
        us_laser_scan_header.frame_id_8= us8_msg->header.frame_id;
        us_laser_scan_header.us_8 = us8_msg->range;
        if (us_laser_scan_header.us_8 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_8= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_8=us_laser_scan_header.us_8;
      }
        // cout<<us_laser_scan_header.us_8;
}
void us_range_LaserScan_fusion::us9_range_callback(const sensor_msgs::msg::Range::SharedPtr us9_msg)    
{     
        us_laser_scan_header.time_9 = us9_msg->header.stamp;
        us_laser_scan_header.frame_id_9= us9_msg->header.frame_id;
        us_laser_scan_header.us_9 = us9_msg->range;
        if (us_laser_scan_header.us_9 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_9= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_9=us_laser_scan_header.us_9;
      }
        // cout<<us_laser_scan_header.us_9;
}
void us_range_LaserScan_fusion::us10_range_callback(const sensor_msgs::msg::Range::SharedPtr us10_msg)    
{     
        us_laser_scan_header.time_10 = us10_msg->header.stamp;
        us_laser_scan_header.frame_id_10= us10_msg->header.frame_id;
        us_laser_scan_header.us_10 = us10_msg->range;
        if (us_laser_scan_header.us_10 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_10= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_10=us_laser_scan_header.us_10;
      }
        // cout<<us_laser_scan_header.us_10;
}
void us_range_LaserScan_fusion::us11_range_callback(const sensor_msgs::msg::Range::SharedPtr us11_msg)    
{     
        us_laser_scan_header.time_11 = us11_msg->header.stamp;
        us_laser_scan_header.frame_id_11= us11_msg->header.frame_id;
        us_laser_scan_header.us_11 = us11_msg->range;
        if (us_laser_scan_header.us_11 > us_laser_scan_header.max_range)
      {
        us_laser_scan_header.us_11=us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_11=us_laser_scan_header.us_11;
      }
        // cout<<us_laser_scan_header.us_11;
}
void us_range_LaserScan_fusion::us12_range_callback(const sensor_msgs::msg::Range::SharedPtr us12_msg)    
{     
        us_laser_scan_header.time_12 = us12_msg->header.stamp;
        us_laser_scan_header.frame_id_12= us12_msg->header.frame_id;
        us_laser_scan_header.us_12 = us12_msg->range;
        if (us_laser_scan_header.us_12 > us_laser_scan_header.max_range + 1.0)
      {
        us_laser_scan_header.us_12= us_laser_scan_header.max_range;
      }
      else
      {
        us_laser_scan_header.us_12=us_laser_scan_header.us_12;
      }
        // cout<<us_laser_scan_header.us_12<<endl;
}


vector<float> us_range_LaserScan_fusion::cordinate_transformation(float x, float dx, float dy, float theta)
{
  // cout<<"check"<<endl;
  float y=0.0;
  float c = cos(theta);
  float s = sin(theta);
  x_ = x*c - y*s + dx ;
  // cout <<" x_ "<<x_;
  // cout <<" y_ "<<y_;
  y_ = x*s + y*c + dy;
  d_=sqrt((x_*x_)+(y_*y_));
  
  phic_=(atan2((y_),(x_)))*(180/M_PI);
  // cout<<" x "<< x;
 
  phi_=(atan2((x*tan(0.349)),(d_)))*(180/M_PI);
  
  if (phic_ < 0)
  {
    phic_=phic_+360;
  }
  us_=(phic_-phi_)*2;
  
  
  if(us_<0)
  {
    us_=0;

  }
  ue_=(phic_+phi_)*2;
  
  if(ue_>720)
  {
    ue_=719;

  }
  
  vector<float> u;
  u.push_back(d_);
  u.push_back(us_);
  u.push_back(ue_);
  return u;
}




vector<float> us_range_LaserScan_fusion::us_to_laser()
{
  //  cout<<"check"<<endl;

    for( int i=0; i<720; i+=1)
    {
      
        us_laser_scan_header.s.push_back(std::numeric_limits<double>::infinity());
        

    }
    

      if (us_laser_scan_header.us_1 < 1.0 && us_laser_scan_header.us_1 >= 0.05)
      {
        vector<float> us_scan1= cordinate_transformation(us_laser_scan_header.us_1,0.252,0.228,0.74); //dx1=x1-x'; dy1=y1-y'
        for(int i=us_scan1.at(1); i<us_scan1.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan1.at(0); 
        
        } 
      } 

      if (us_laser_scan_header.us_2 < 1.0 && us_laser_scan_header.us_2 >= 0.05)
      {
        // d_,us_,ue_ = cordinate_transformation(us_laser_scan_header.us_2,-0.2645,-0.140,0.0);
        vector<float> us_scan2=cordinate_transformation(us_laser_scan_header.us_2,0.2645,0.140,0.0);
        for(int i=us_scan2.at(1); i<us_scan2.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan2.at(0);
        
        } 
      }
    
      if (us_laser_scan_header.us_3 < 1.0 && us_laser_scan_header.us_3 >= 0.05)
      {
        vector<float> us_scan3= cordinate_transformation(us_laser_scan_header.us_3,0.2645,-0.140,0.0);
        for(int i=us_scan3.at(1); i<us_scan3.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan3.at(0);
        
        } 
      }
      

      if (us_laser_scan_header.us_4 < 1.0 && us_laser_scan_header.us_4 >= 0.05)
      {
        vector<float> us_scan4 = cordinate_transformation(us_laser_scan_header.us_4,0.251,-0.228,-0.74);
        for(int i=us_scan4.at(1); i<us_scan4.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan4.at(0);
        
        } 
      }

      if (us_laser_scan_header.us_5 < 1.0 && us_laser_scan_header.us_5 >= 0.05)
      {
        vector<float> us_scan5= cordinate_transformation(us_laser_scan_header.us_5,0.135,-0.2505,-1.48);
        for(int i=us_scan5.at(1); i<us_scan5.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan5.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_6 < 1.0 && us_laser_scan_header.us_6 >= 0.05)
      {
        vector<float> us_scan6= cordinate_transformation(us_laser_scan_header.us_6,-0.135,-0.274,-1.48);
        for(int i=us_scan6.at(1); i<us_scan6.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan6.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_7 < 1.0 && us_laser_scan_header.us_7 >= 0.05)
      {
        vector<float> us_scan7= cordinate_transformation(us_laser_scan_header.us_7,-0.248,-0.2695,-2.31159);
        for(int i=us_scan7.at(1); i<us_scan7.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan7.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_8 < 1.0 && us_laser_scan_header.us_8 >= 0.05)
      {
        vector<float> us_scan8= cordinate_transformation(us_laser_scan_header.us_8,-0.2645,-0.140,3.14);
        for(int i=us_scan8.at(1); i<us_scan8.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan8.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_9 < 1.0 && us_laser_scan_header.us_9 >= 0.05)
      {
        vector<float> us_scan9= cordinate_transformation(us_laser_scan_header.us_9,-0.2645,0.140,3.14);
        for(int i=us_scan9.at(1); i<us_scan9.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan9.at(0);
        
        } 
      }

      if (us_laser_scan_header.us_10 < 1.0 && us_laser_scan_header.us_10 >= 0.05)
      {
        vector<float> us_scan10= cordinate_transformation(us_laser_scan_header.us_10,-0.248,0.2695,2.31159);
        for(int i=us_scan10.at(1); i<us_scan10.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan10.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_11 < 1.0 && us_laser_scan_header.us_11 >= 0.05)
      {
        vector<float> us_scan11= cordinate_transformation(us_laser_scan_header.us_11,-0.155,0.274,1.48);
        for(int i=us_scan11.at(1); i<us_scan11.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan11.at(0);
        
        } 
      }
      if (us_laser_scan_header.us_12 < 1.0 && us_laser_scan_header.us_12 >= 0.05)
      {
        vector<float> us_scan12= cordinate_transformation(us_laser_scan_header.us_12,0.135,0.2505,1.48);
        for(int i=us_scan12.at(1); i<us_scan12.at(2); i+=1)
        {
        us_laser_scan_header.s.at(i)=us_scan12.at(0);
        
        } 
      }

    // for (auto i = us_laser_scan_header.s.begin(); i != us_laser_scan_header.s.end(); ++i)
    // {
    //     cout << *i << " ";
    // }   

    
  // cout<<us_laser_scan_header.s.size()<<endl;
  return us_laser_scan_header.s;
  
}

void us_range_LaserScan_fusion::update()
{  
  us_to_laser(); 
  us_laser_scan_header.ultrasonic_range_scan(us_laser_scan_header.time_1,"us_scan_link",us_laser_scan_header.s,US_range_scan_pub);
  us_laser_scan_header.s.clear(); 

  // us_laser_scan_header.us_range(us_laser_scan_header.time_1,us_laser_scan_header.us_1,us_laser_scan_header.frame_id_1,us_range_pub_1);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_2,us_laser_scan_header.us_2,us_laser_scan_header.frame_id_2,us_range_pub_2);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_3,us_laser_scan_header.us_3,us_laser_scan_header.frame_id_3,us_range_pub_3);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_4,us_laser_scan_header.us_4,us_laser_scan_header.frame_id_4,us_range_pub_4);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_5,us_laser_scan_header.us_5,us_laser_scan_header.frame_id_5,us_range_pub_5);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_6,us_laser_scan_header.us_6,us_laser_scan_header.frame_id_6,us_range_pub_6);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_7,us_laser_scan_header.us_7,us_laser_scan_header.frame_id_7,us_range_pub_7);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_8,us_laser_scan_header.us_8,us_laser_scan_header.frame_id_8,us_range_pub_8);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_9,us_laser_scan_header.us_9,us_laser_scan_header.frame_id_9,us_range_pub_9);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_10,us_laser_scan_header.us_10,us_laser_scan_header.frame_id_10,us_range_pub_10);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_11,us_laser_scan_header.us_11,us_laser_scan_header.frame_id_11,us_range_pub_11);
  // us_laser_scan_header.us_range(us_laser_scan_header.time_12,us_laser_scan_header.us_12,us_laser_scan_header.frame_id_12,us_range_pub_12);
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<us_range_LaserScan_fusion>());
  rclcpp::shutdown();
  return 0;
}

