#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "zed2_full_bed_scan.hpp"
#include <iostream>
#include <vector>
#include "rclcpp/logger.hpp" 
#include <bits/stdc++.h>
#include <math.h>


using namespace std;
zed2_full_bed_laser_scan_header zed2_full_bed_laser_scan_header;

using std::placeholders::_1;
using namespace std::chrono_literals;

class zed2_full_bed_scan : public rclcpp::Node
{
  public:
    zed2_full_bed_scan()
    : Node("zed2_full_bed_scan")
    {
         zed2_full_bed_range_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/cv_pose", rclcpp::SensorDataQoS(), std::bind(&zed2_full_bed_scan::zed2_bed_range_callback, this, _1)); 

      // timer_ = this->create_wall_timer(100ms, std::bind(&zed2_full_bed_scan::update, this));

      zed2_full_bed_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("zed2_full_bed_scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    }

  private:
    void zed2_bed_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr zed2_full_bed_range_sub;

    void update();
    vector<float> zed2_bed_to_laser();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_full_bed_scan_pub;

};






void zed2_full_bed_scan::zed2_bed_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)    
{    

    for(int i=0; i<msg->data.size(); i++)
    {
        zed2_full_bed_laser_scan_header.zed2_bed.push_back(msg->data[i]);
    }

    
    zed2_full_bed_laser_scan_header.p1= sqrt((zed2_full_bed_laser_scan_header.zed2_bed.at(2)-zed2_full_bed_laser_scan_header.zed2_bed.at(0))*(zed2_full_bed_laser_scan_header.zed2_bed.at(2)-zed2_full_bed_laser_scan_header.zed2_bed.at(0))+(zed2_full_bed_laser_scan_header.zed2_bed.at(3)-zed2_full_bed_laser_scan_header.zed2_bed.at(1))*(zed2_full_bed_laser_scan_header.zed2_bed.at(3)-zed2_full_bed_laser_scan_header.zed2_bed.at(1)));
    zed2_full_bed_laser_scan_header.p2= sqrt((zed2_full_bed_laser_scan_header.zed2_bed.at(5)-zed2_full_bed_laser_scan_header.zed2_bed.at(3))*(zed2_full_bed_laser_scan_header.zed2_bed.at(5)-zed2_full_bed_laser_scan_header.zed2_bed.at(3))+(zed2_full_bed_laser_scan_header.zed2_bed.at(4)-zed2_full_bed_laser_scan_header.zed2_bed.at(2))*(zed2_full_bed_laser_scan_header.zed2_bed.at(4)-zed2_full_bed_laser_scan_header.zed2_bed.at(2)));

    zed2_full_bed_laser_scan_header.d1= sqrt((zed2_full_bed_laser_scan_header.zed2_bed.at(0))*(zed2_full_bed_laser_scan_header.zed2_bed.at(0))+(zed2_full_bed_laser_scan_header.zed2_bed.at(1))*(zed2_full_bed_laser_scan_header.zed2_bed.at(1)));
    zed2_full_bed_laser_scan_header.d2= sqrt((zed2_full_bed_laser_scan_header.zed2_bed.at(2))*(zed2_full_bed_laser_scan_header.zed2_bed.at(2))+(zed2_full_bed_laser_scan_header.zed2_bed.at(3))*(zed2_full_bed_laser_scan_header.zed2_bed.at(3)));

    zed2_full_bed_laser_scan_header.d3= sqrt((zed2_full_bed_laser_scan_header.zed2_bed.at(4))*(zed2_full_bed_laser_scan_header.zed2_bed.at(4))+(zed2_full_bed_laser_scan_header.zed2_bed.at(5))*(zed2_full_bed_laser_scan_header.zed2_bed.at(5)));
   
   
    zed2_full_bed_laser_scan_header.i_d=abs(zed2_full_bed_laser_scan_header.zed2_bed.at(9)*1000);
    cout<<" i_d "<<zed2_full_bed_laser_scan_header.zed2_bed.at(9)*1000<<endl;

    // zed2_full_bed_laser_scan_header.th_i_d=asin(zed2_full_bed_laser_scan_header.i_d/zed2_full_bed_laser_scan_header.d2);
    
    

    
    cout<<" th_i_d "<<zed2_full_bed_laser_scan_header.th_i_d*57.324<<endl;


    // zed2_full_bed_laser_scan_header.bdx=2.5;
    zed2_full_bed_laser_scan_header.th1= acos(((zed2_full_bed_laser_scan_header.d1*zed2_full_bed_laser_scan_header.d1)+(zed2_full_bed_laser_scan_header.d2*zed2_full_bed_laser_scan_header.d2)-(zed2_full_bed_laser_scan_header.p1*zed2_full_bed_laser_scan_header.p1))/(2*zed2_full_bed_laser_scan_header.d1*zed2_full_bed_laser_scan_header.d2));
    zed2_full_bed_laser_scan_header.th2= acos(((zed2_full_bed_laser_scan_header.d2*zed2_full_bed_laser_scan_header.d2)+(zed2_full_bed_laser_scan_header.d3*zed2_full_bed_laser_scan_header.d3)-(zed2_full_bed_laser_scan_header.p2*zed2_full_bed_laser_scan_header.p2))/(2*zed2_full_bed_laser_scan_header.d2*zed2_full_bed_laser_scan_header.d3));
    
    
    cout<<" lth "<<zed2_full_bed_laser_scan_header.lth<<endl;
    cout<<" rth "<<zed2_full_bed_laser_scan_header.rth<<endl;


    // cout <<" p1 "<<zed2_full_bed_laser_scan_header.p1<<" p2 "<<zed2_full_bed_laser_scan_header.p2<<" d1 "<<zed2_full_bed_laser_scan_header.d1<<" d2 "<<zed2_full_bed_laser_scan_header.d2<<" th1 "<<zed2_full_bed_laser_scan_header.th1<<" th2 "<<zed2_full_bed_laser_scan_header.th2;
 

    

    // zed2_full_bed_laser_scan_header.div=(zed2_full_bed_laser_scan_header.th1+zed2_full_bed_laser_scan_header.th2)/(zed2_full_bed_laser_scan_header.zed2_bed.size()-10.0); 
    zed2_full_bed_laser_scan_header.div=(abs(zed2_full_bed_laser_scan_header.lth)+abs(zed2_full_bed_laser_scan_header.rth))/(zed2_full_bed_laser_scan_header.zed2_bed.size()-10.0); 
// cout<<" div "<<zed2_full_bed_laser_scan_header.div<<endl;

    for (auto i = zed2_full_bed_laser_scan_header.zed2_bed.begin(); i != zed2_full_bed_laser_scan_header.zed2_bed.end(); ++i)
    {
        cout << *i << " ";
    }  
    // cout<<zed2_full_bed_laser_scan_header.zed2_bed.size()-10<<endl;
    update();
}



vector<float> zed2_full_bed_scan::zed2_bed_to_laser()
{
  //  cout<<"check"<<endl;

    for( float i=(zed2_full_bed_laser_scan_header.rth); i<(zed2_full_bed_laser_scan_header.lth); i+=zed2_full_bed_laser_scan_header.div)
    {
      
        zed2_full_bed_laser_scan_header.zed2.push_back(std::numeric_limits<double>::infinity());
        
        
        
        

    }
    // cout<<" zed2_size "<<zed2_full_bed_laser_scan_header.zed2.size()<<endl;

    zed2_full_bed_laser_scan_header.th_i_d=(zed2_full_bed_laser_scan_header.i_d*0.05729)*(0.01745); 


    if (zed2_full_bed_laser_scan_header.zed2_bed.at(9)>0)
    {
      // zed2_full_bed_laser_scan_header.th_i_d=zed2_full_bed_laser_scan_header.th_i_d;
      zed2_full_bed_laser_scan_header.rth=-(zed2_full_bed_laser_scan_header.th_i_d+zed2_full_bed_laser_scan_header.th2);
      zed2_full_bed_laser_scan_header.lth= -(zed2_full_bed_laser_scan_header.th_i_d-zed2_full_bed_laser_scan_header.th1);


    }
    else
    {
      // zed2_full_bed_laser_scan_header.th_i_d=zed2_full_bed_laser_scan_header.th_i_d;
      zed2_full_bed_laser_scan_header.rth=(zed2_full_bed_laser_scan_header.th_i_d-zed2_full_bed_laser_scan_header.th2);
      zed2_full_bed_laser_scan_header.lth= (zed2_full_bed_laser_scan_header.th_i_d+zed2_full_bed_laser_scan_header.th1);
    }
     

      if (zed2_full_bed_laser_scan_header.d1>=0.5 && zed2_full_bed_laser_scan_header.d2>= 0.5 && zed2_full_bed_laser_scan_header.d3>= 0.5)
      {
        for (float t = (zed2_full_bed_laser_scan_header.rth); t < (zed2_full_bed_laser_scan_header.lth-zed2_full_bed_laser_scan_header.div); t+=zed2_full_bed_laser_scan_header.div)
        {
          
          if (zed2_full_bed_laser_scan_header.insert>(zed2_full_bed_laser_scan_header.zed2_bed.size()-10.0))
          {
            zed2_full_bed_laser_scan_header.insert=zed2_full_bed_laser_scan_header.insert-1;
          }
          // cout<<" insert "<<zed2_full_bed_laser_scan_header.insert<<endl;
          // cout<<" insert-10 [] "<<(zed2_full_bed_laser_scan_header.insert)+(10)<<endl;
         

          zed2_full_bed_laser_scan_header.zed2.at(zed2_full_bed_laser_scan_header.insert)=zed2_full_bed_laser_scan_header.zed2_bed.at(zed2_full_bed_laser_scan_header.insert+10); 
          zed2_full_bed_laser_scan_header.insert=zed2_full_bed_laser_scan_header.insert+1;
        }
    
      } 
//   // cout<<zed2_full_bed_laser_scan_header.zed2.size()<<endl; 
//   // cout<<zed2_full_bed_laser_scan_header.zed2_bed.size()-10<<endl; 
  // for (auto i = zed2_full_bed_laser_scan_header.zed2.begin(); i != zed2_full_bed_laser_scan_header.zed2.end(); ++i)
  //   {
  //       cout << *i << " ";
  //   }  
  
  return zed2_full_bed_laser_scan_header.zed2; 
}


void zed2_full_bed_scan::update()
{
  zed2_full_bed_laser_scan_header.insert=0;
  zed2_bed_to_laser();
  zed2_full_bed_laser_scan_header.zed2_full_bed_range_scan(this->get_clock()->now(),"camera_link",zed2_full_bed_laser_scan_header.zed2,zed2_full_bed_scan_pub,(zed2_full_bed_laser_scan_header.lth),(zed2_full_bed_laser_scan_header.rth),zed2_full_bed_laser_scan_header.div);
  zed2_full_bed_laser_scan_header.zed2.clear(); 
  zed2_full_bed_laser_scan_header.zed2_bed.clear(); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zed2_full_bed_scan>());
  rclcpp::shutdown();
  return 0;
}