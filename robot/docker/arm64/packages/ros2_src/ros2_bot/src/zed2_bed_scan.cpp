#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "zed2_bed_scan.hpp"
#include <iostream>
#include <vector>
#include "rclcpp/logger.hpp" 
#include <bits/stdc++.h>
#include <math.h>


using namespace std;
zed2_bed_laser_scan_header Zed2_bed_laser_scan_header;

using std::placeholders::_1;
using namespace std::chrono_literals;

class zed2_bed_scan : public rclcpp::Node
{
  public:
    zed2_bed_scan()
    : Node("zed2_bed_scan")
    {
         zed2_bed_range_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/cv_pose", rclcpp::SensorDataQoS(), std::bind(&zed2_bed_scan::zed2_bed_range_callback, this, _1)); 

      // timer_ = this->create_wall_timer(71.42ms, std::bind(&zed2_bed_scan::update, this));

      zed2_bed_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("zed2_bed_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
      //  zed2_bed_to_laser();
      //  Zed2_bed_laser_scan_header.zed2.clear(); 


    }

  private:
    void zed2_bed_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr zed2_bed_range_sub;

    void update();
    vector<float> zed2_bed_to_laser();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_bed_scan_pub;

};






void zed2_bed_scan::zed2_bed_range_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)    
{    

    // for(int i=0; i<10; i++)
    // {
    //   Zed2_bed_laser_scan_header.raw[i] = msg->data[i];
    // }    
    Zed2_bed_laser_scan_header.p1= sqrt((msg->data[3]-msg->data[1])*(msg->data[3]-msg->data[1])+((msg->data[2]-msg->data[0])*(msg->data[2]-msg->data[0])));
    Zed2_bed_laser_scan_header.p2= sqrt((msg->data[5]-msg->data[3])*(msg->data[5]-msg->data[3])+((msg->data[4]-msg->data[2])*(msg->data[4]-msg->data[2])));
    Zed2_bed_laser_scan_header.d1= sqrt((msg->data[0]*msg->data[0])+(msg->data[1]*msg->data[1]));
    Zed2_bed_laser_scan_header.d2= sqrt((msg->data[2]*msg->data[2])+(msg->data[3]*msg->data[3]));
    Zed2_bed_laser_scan_header.d3= sqrt((msg->data[4]*msg->data[4])+(msg->data[5]*msg->data[5]));
    Zed2_bed_laser_scan_header.bdx=2.5;
    Zed2_bed_laser_scan_header.id=msg->data[9];
    cout<<" id "<<Zed2_bed_laser_scan_header.id<<endl;;

    // cout<<" p1 "<<Zed2_bed_laser_scan_header.p1<<" p2"<<Zed2_bed_laser_scan_header.p2<<" d2 "<<Zed2_bed_laser_scan_header.d2;
    

    


    Zed2_bed_laser_scan_header.psi1=atan(Zed2_bed_laser_scan_header.p1/(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx));
    Zed2_bed_laser_scan_header.psi2=atan(Zed2_bed_laser_scan_header.p2/(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx));
    // cout <<" psi1 "<<Zed2_bed_laser_scan_header.psi1<<" psi2 "<<Zed2_bed_laser_scan_header.psi2;
    
    Zed2_bed_laser_scan_header.th1=atan(Zed2_bed_laser_scan_header.p1/Zed2_bed_laser_scan_header.d2);
    Zed2_bed_laser_scan_header.th2=atan(Zed2_bed_laser_scan_header.p2/Zed2_bed_laser_scan_header.d2); 
    // cout << " th1 "<<Zed2_bed_laser_scan_header.th1<<" th2 "<<Zed2_bed_laser_scan_header.th2;

    Zed2_bed_laser_scan_header.phi1=Zed2_bed_laser_scan_header.th1-Zed2_bed_laser_scan_header.psi1;
    Zed2_bed_laser_scan_header.phi2=Zed2_bed_laser_scan_header.th2-Zed2_bed_laser_scan_header.psi2;

    // cout <<" phi1 "<<Zed2_bed_laser_scan_header.phi1<<" phi2 "<<Zed2_bed_laser_scan_header.phi2;
     Zed2_bed_laser_scan_header.i_d=abs(Zed2_bed_laser_scan_header.id);

    Zed2_bed_laser_scan_header.th_i_d=(110/1920*1000*Zed2_bed_laser_scan_header.i_d);

    if (Zed2_bed_laser_scan_header.id>0)
    {
      // Zed2_bed_laser_scan_header.th_i_d=Zed2_bed_laser_scan_header.th_i_d;
      Zed2_bed_laser_scan_header.lth=-(Zed2_bed_laser_scan_header.th_i_d+Zed2_bed_laser_scan_header.th2);
      Zed2_bed_laser_scan_header.rth= -(Zed2_bed_laser_scan_header.th_i_d-Zed2_bed_laser_scan_header.th1);


    }
    else
    {
      // Zed2_bed_laser_scan_header.th_i_d=Zed2_bed_laser_scan_header.th_i_d;
      Zed2_bed_laser_scan_header.lth=(Zed2_bed_laser_scan_header.th_i_d-Zed2_bed_laser_scan_header.th2);
      Zed2_bed_laser_scan_header.rth= (Zed2_bed_laser_scan_header.th_i_d+Zed2_bed_laser_scan_header.th1);
    }




    Zed2_bed_laser_scan_header.div=(Zed2_bed_laser_scan_header.th1+Zed2_bed_laser_scan_header.th2)/200.00;
    



    // cout <<" div "<<Zed2_bed_laser_scan_header.div;
    // RCLCPP_INFO(get_logger(), " Zed2_bed_laser_scan_header.div--->{%f}", Zed2_bed_laser_scan_header.div);
    // Zed2_bed_laser_scan_header.divphi1=(Zed2_bed_laser_scan_header.phi1)/100;
    // Zed2_bed_laser_scan_header.divphi2=(Zed2_bed_laser_scan_header.phi2)/100;




    
    // Zed2_bed_laser_scan_header.bdx_1=Zed2_bed_laser_scan_header.bdx/(Zed2_bed_laser_scan_header.psi1/i);
    // Zed2_bed_laser_scan_header.bdx_2=Zed2_bed_laser_scan_header.bdx/(Zed2_bed_laser_scan_header.psi2/i);
    // Zed2_bed_laser_scan_header.d_1=sqrt(((Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_1)*(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_1))+ (Zed2_bed_laser_scan_header.p1*Zed2_bed_laser_scan_header.p1));
    // Zed2_bed_laser_scan_header.d_2=sqrt(((Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_2)*(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_2))+ (Zed2_bed_laser_scan_header.p2*Zed2_bed_laser_scan_header.p2));
    

    // Zed2_bed_laser_scan_header.th1=atan(Zed2_bed_laser_scan_header.raw[5]/Zed2_bed_laser_scan_header.raw[6]);
    // Zed2_bed_laser_scan_header.th2=atan(Zed2_bed_laser_scan_header.raw[7]/Zed2_bed_laser_scan_header.raw[6]);
    
    update();


}



vector<float> zed2_bed_scan::zed2_bed_to_laser()
{
  //  cout<<"check"<<endl;
  //  cout <<Zed2_bed_laser_scan_header.th1<<endl;
  // vector<float> zed2;
  // Zed2_bed_laser_scan_header.div=(Zed2_bed_laser_scan_header.th1+Zed2_bed_laser_scan_header.th2)/200;
  

    for( float i = -Zed2_bed_laser_scan_header.th2; i <= Zed2_bed_laser_scan_header.th1; i+=Zed2_bed_laser_scan_header.div)
    {
      
        Zed2_bed_laser_scan_header.zed2.push_back(std::numeric_limits<double>::infinity()); 
        
       
      //  cout <<Zed2_bed_laser_scan_header.zed2.size()<<endl;

        
        // RCLCPP_INFO(get_logger(), " th2--->{%f}", Zed2_laser_scan_header.th2);
        
      
    }
    
     
    

    
     
    

      if (Zed2_bed_laser_scan_header.d2 < 4.0 && Zed2_bed_laser_scan_header.d2 >= 0.5)
      {


        for (float t = -Zed2_bed_laser_scan_header.th2; t < (Zed2_bed_laser_scan_header.th1-Zed2_bed_laser_scan_header.div); t+=Zed2_bed_laser_scan_header.div)
        { 
          
          if(Zed2_bed_laser_scan_header.insert>200)
          {
            Zed2_bed_laser_scan_header.insert=Zed2_bed_laser_scan_header.insert-2;
          }
          // cout<<" d2' " <<Zed2_bed_laser_scan_header.d_2;



          Zed2_bed_laser_scan_header.zed2.at(Zed2_bed_laser_scan_header.insert)=Zed2_bed_laser_scan_header.d2/cos(t); 
          Zed2_bed_laser_scan_header.insert=Zed2_bed_laser_scan_header.insert + 1;
          // cout<<" insert "<<Zed2_bed_laser_scan_header.insert<<endl;
          
        }
        // cout<<" insert "<<Zed2_bed_laser_scan_header.insert<<endl;
      

        for(float i=Zed2_bed_laser_scan_header.psi1; i<(Zed2_bed_laser_scan_header.th1); i+=2*(Zed2_bed_laser_scan_header.div))
        {
          
          Zed2_bed_laser_scan_header.bdx_1=(Zed2_bed_laser_scan_header.bdx/(Zed2_bed_laser_scan_header.phi1))*(i-Zed2_bed_laser_scan_header.psi1);
          
          // Zed2_bed_laser_scan_header.d_1=sqrt(((Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_1)*(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_1))+ (Zed2_bed_laser_scan_header.p1*Zed2_bed_laser_scan_header.p1));
          
          Zed2_bed_laser_scan_header.alpha_1=atan(Zed2_bed_laser_scan_header.p1/(Zed2_bed_laser_scan_header.bdx_1+Zed2_bed_laser_scan_header.d2));
          Zed2_bed_laser_scan_header.d_1=sqrt((Zed2_bed_laser_scan_header.d1*Zed2_bed_laser_scan_header.d1)+(Zed2_bed_laser_scan_header.bdx_1*Zed2_bed_laser_scan_header.bdx_1)-(2*Zed2_bed_laser_scan_header.d1*Zed2_bed_laser_scan_header.bdx_1*cos((3.14)-(Zed2_bed_laser_scan_header.alpha_1+(Zed2_bed_laser_scan_header.phi1-i)))));


          // cout<<" d1 "<<Zed2_bed_laser_scan_header.d_1;

          
          
          if (Zed2_bed_laser_scan_header.insertphi1%2==0)
          {
            Zed2_bed_laser_scan_header.insertphi1=Zed2_bed_laser_scan_header.insertphi1+1; 
          }
          
          if (Zed2_bed_laser_scan_header.insertphi1>200)
          {
            Zed2_bed_laser_scan_header.insertphi1=Zed2_bed_laser_scan_header.insertphi1-2;
          }
          
          // cout<<" insertphi1 " << Zed2_bed_laser_scan_header.insertphi1;
          // cout <<" insertphi1 "<<Zed2_bed_laser_scan_header.insertphi1<<endl;
          Zed2_bed_laser_scan_header.zed2.at(Zed2_bed_laser_scan_header.insertphi1)=Zed2_bed_laser_scan_header.d_1; 
          Zed2_bed_laser_scan_header.insertphi1=Zed2_bed_laser_scan_header.insertphi1+2;
          
          
        
        } 
        
        
        for(float i= -Zed2_bed_laser_scan_header.th2; i<(-Zed2_bed_laser_scan_header.psi2); i+= 2*(Zed2_bed_laser_scan_header.div))
        {
          
          
          Zed2_bed_laser_scan_header.bdx_2=(Zed2_bed_laser_scan_header.bdx/(Zed2_bed_laser_scan_header.phi2))*(i-(-Zed2_bed_laser_scan_header.th2));          
          // Zed2_bed_laser_scan_header.d_2=sqrt(((Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_2)*(Zed2_bed_laser_scan_header.d2+Zed2_bed_laser_scan_header.bdx_2))+ (Zed2_bed_laser_scan_header.p2*Zed2_bed_laser_scan_header.p2));
          Zed2_bed_laser_scan_header.alpha_2=atan(Zed2_bed_laser_scan_header.p2/(Zed2_bed_laser_scan_header.bdx_2+Zed2_bed_laser_scan_header.d2));
          Zed2_bed_laser_scan_header.d_2=sqrt((Zed2_bed_laser_scan_header.d3*Zed2_bed_laser_scan_header.d3)+(Zed2_bed_laser_scan_header.bdx_2*Zed2_bed_laser_scan_header.bdx_2)-(2*Zed2_bed_laser_scan_header.d3*Zed2_bed_laser_scan_header.bdx_2*cos((3.14)-(Zed2_bed_laser_scan_header.alpha_2+i))));


          if (Zed2_bed_laser_scan_header.insertphi2 > 200)
          {
            Zed2_bed_laser_scan_header.insertphi2=Zed2_bed_laser_scan_header.insertphi2-2;
          }
          // cout <<" d_2 "<<Zed2_bed_laser_scan_header.d_2;
          // cout <<" insertphi2 "<<Zed2_bed_laser_scan_header.insertphi2<<endl;
          Zed2_bed_laser_scan_header.zed2.at(Zed2_bed_laser_scan_header.insertphi2)=Zed2_bed_laser_scan_header.d_2; 
          Zed2_bed_laser_scan_header.insertphi2=Zed2_bed_laser_scan_header.insertphi2+2;
          // cout<<" insertphi2 "<<Zed2_bed_laser_scan_header.insertphi2<<endl;
          
          
        } 


        

        // cout<<Zed2_bed_laser_scan_header.zed2.size()<<endl;

        // RCLCPP_INFO(get_logger(), " dzed2--->{%f}", Zed2_laser_scan_header.zed2);
    
      } 

   
  // cout <<Zed2_bed_laser_scan_header.zed2.size()<<endl;

  // for (auto i = Zed2_bed_laser_scan_header.zed2.begin(); i != Zed2_bed_laser_scan_header.zed2.end(); ++i)
  //   {
  //       cout << *i << " ";
  //   }  

    // cout <<Zed2_bed_laser_scan_header.zed2<<endl; 
    return Zed2_bed_laser_scan_header.zed2;
  
  
}


void zed2_bed_scan::update()
{
  
  zed2_bed_to_laser();
  Zed2_bed_laser_scan_header.insertphi2=1;
  Zed2_bed_laser_scan_header.insert=0;
  Zed2_bed_laser_scan_header.insertphi1=((200/(Zed2_bed_laser_scan_header.th1+Zed2_bed_laser_scan_header.th2))*(Zed2_bed_laser_scan_header.phi2+Zed2_bed_laser_scan_header.psi1+Zed2_bed_laser_scan_header.psi2))+2;
  
  Zed2_bed_laser_scan_header.zed2_bed_range_scan(this->get_clock()->now(),"camera_link",Zed2_bed_laser_scan_header.zed2,zed2_bed_scan_pub,Zed2_bed_laser_scan_header.lth,Zed2_bed_laser_scan_header.rth,Zed2_bed_laser_scan_header.div);
  // cout <<" zed2 "<<Zed2_bed_laser_scan_header.zed2.size()<<endl; 
  Zed2_bed_laser_scan_header.zed2.clear(); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zed2_bed_scan>());
  rclcpp::shutdown();
  return 0;
}