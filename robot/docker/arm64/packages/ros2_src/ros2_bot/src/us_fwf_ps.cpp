#include "us_fusion_ps.hpp"
US_fusion_header usfwfh;
using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

class US_fusion : public rclcpp::Node
{
  public:
    US_fusion()
    : Node("us_fwf_ps")
    {
      US_fusion_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/us_scan", 10, std::bind(&US_fusion::US_fusion_cb, this, _1));
      lmotor_cmd_sub = this->create_subscription<std_msgs::msg::Int16>("/lmotor_cmd", 10, std::bind(&US_fusion::lmotor_cmd_cb, this, _1));
      rmotor_cmd_sub = this->create_subscription<std_msgs::msg::Int16>("/rmotor_cmd", 10, std::bind(&US_fusion::rmotor_cmd_cb, this, _1));
      // timer_ = this->create_wall_timer(71.42ms, std::bind(&US_fusion::update, this)); 
      US_fusion_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/ufwf_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

    void US_fusion_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void lmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msglm);
    void rmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msgrm);
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr US_fusion_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lmotor_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr rmotor_cmd_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_fusion_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<float> us_to_laser();
    void update();
    float m_a(float l,float m,float r);
    vector<float> c_t(float x, float dx, float dy, float theta);
    rclcpp::Time then = this->get_clock()->now();
    float d_,x_,y_;
    int phic_,phi_,us_,ue_,lmData,rmData;  
    float usfr[12];
    float fov=0.2618*2; // fov 15*2 deg us
    float ufd1,ufd2,ufd3,ufd4,ufd5,ufd6,ufd7,ufd8,ufd9,ufd10,ufd11,ufd12;

    vector<vector<float>> raw
    {
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
    };
    vector<float> ur;
};

void US_fusion::lmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msglm)   
{ 
   lmData = msglm->data;

}
void US_fusion::rmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msgrm)   
{ 
   rmData = msgrm->data;

}

void US_fusion::US_fusion_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)   
{    
    for(int j=0; j<12; j++)
    {
      float ss = msg->data[j];
      
      if(ss > 105)
      {
        ss = 310;
      }
      
      if (lmData==0 && rmData==0)
      {
        ss = 200;
      }
      if(ss==0)
      {
        ss=200;
      }
      cout<<" ss "<<ss<<endl;
      ur.push_back(ss/100.00);
    }  
    
  raw.push_back(ur);
  
    // for (int i = 0; i <raw.size(); i++)
    // {
    //     for (int j = 0; j < raw[i].size(); j++)
    //     {
    //         // cout << raw[i][j] << " ";
    //     }    
    //     cout << endl;
    // }

    for (int i=0;i<12;i++)
    {
      usfr[i]=m_a(raw[0][i],raw[1][i],raw[2][i]);
      // cout<<" usfr[i] "<<usfr[i]<<" ";
    }
    // cout<<endl;
    us_to_laser();
    // cout<<" raw "<<raw.size()<<endl;
    reverse(raw.begin(), raw.end()); 
    raw.pop_back();
    reverse(raw.begin(), raw.end());
     update();    
}

float US_fusion::m_a(float l,float m,float r)
{
  float usr;
  usr=((l+m+r)/3.00);
  return usr;
}

vector<float> US_fusion::c_t(float x, float dx, float dy, float theta)
{
  float y=0.0;
  float c = cos(theta);
  float s = sin(theta);
  x_ = x*c - y*s + dx ;
  y_ = x*s + y*c + dy;
  d_=sqrt((x_*x_)+(y_*y_));
  phic_=(atan2((y_),(x_)))*(180/M_PI);
  phi_=(atan2((x*tan(fov/2)),(d_)))*(180/M_PI);
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

vector<float> US_fusion::us_to_laser()
{
    
    for( int i=0; i<720; i+=1)
    {
        usfwfh.uf.push_back(std::numeric_limits<double>::infinity());
    }
      if (usfr[0]< 1.0 && usfr[0]>= 0.05)
      {
        vector<float> us_scan1= c_t(usfr[0],0.252,0.228,0.74); //dx1=x1-x'; dy1=y1-y'
        
        for(int i=us_scan1.at(1); i<us_scan1.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan1.at(0); 
        } 
      } 

      if (usfr[1]< 1.0 && usfr[1]>= 0.05)
      {
        // cout<<usfr[]<endl;
        vector<float> us_scan2=c_t(usfr[1],0.2645,0.140,0.0);
        for(int i=us_scan2.at(1); i<us_scan2.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan2.at(0);
        
        } 
      }
    
      if (usfr[2]< 1.0 && usfr[2]>= 0.05)
      {
        vector<float> us_scan3= c_t(usfr[2],0.2645,-0.140,0.0);
        for(int i=us_scan3.at(1); i<us_scan3.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan3.at(0);
        } 
      }
    
      if (usfr[3]< 1.0 && usfr[3]>= 0.05)
      {
        vector<float> us_scan4 = c_t(usfr[3],0.251,-0.228,-0.74);
        for(int i=us_scan4.at(1); i<us_scan4.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan4.at(0);
        } 
      }

      if (usfr[4]< 1.0 && usfr[4]>= 0.05)
      {
        vector<float> us_scan5= c_t(usfr[4],0.135,-0.2505,-1.48);
        for(int i=us_scan5.at(1); i<us_scan5.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan5.at(0);
        } 
      }

      if (usfr[5]< 1.0 && usfr[5]>= 0.05)
      {
        vector<float> us_scan6= c_t(usfr[5],-0.135,-0.274,-1.48);
        for(int i=us_scan6.at(1); i<us_scan6.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan6.at(0);
        
        } 
      }

      if (usfr[6]< 1.0 && usfr[6]>= 0.05)
      {
        vector<float> us_scan7= c_t(usfr[6],-0.248,-0.2695,-2.31159);
        for(int i=us_scan7.at(1); i<us_scan7.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan7.at(0);
        
        } 
      }

      if (usfr[7]< 1.0 && usfr[7]>= 0.05)
      {
        vector<float> us_scan8= c_t(usfr[7],-0.2645,-0.140,3.14);
        for(int i=us_scan8.at(1); i<us_scan8.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan8.at(0);
        
        } 
      }

      if (usfr[8]< 1.0 && usfr[8]>= 0.05)
      {
        vector<float> us_scan9= c_t(usfr[8],-0.2645,0.140,3.14);
        for(int i=us_scan9.at(1); i<us_scan9.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan9.at(0);
        
        } 
      }

      if (usfr[9] < 1.0 && usfr[9] >= 0.05)
      {
        vector<float> us_scan10= c_t(usfr[9],-0.248,0.2695,2.31159);
        for(int i=us_scan10.at(1); i<us_scan10.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan10.at(0);
        } 
      }

      if (usfr[10] < 1.0 && usfr[10] >= 0.05)
      {
        vector<float> us_scan11= c_t(usfr[10],-0.155,0.274,1.48);
        for(int i=us_scan11.at(1); i<us_scan11.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan11.at(0);
        
        } 
      }

      if (usfr[11] < 1.0 && usfr[11] >= 0.05)
      {
        vector<float> us_scan12= c_t(usfr[11],0.135,0.2505,1.48);
        for(int i=us_scan12.at(1); i<us_scan12.at(2); i+=1)
        {
        usfwfh.uf.at(i)=us_scan12.at(0);
        } 
      }

  //   for (auto i = usfwfh.uf.begin(); i != usfwfh.uf.end(); ++i)
  //   {
  //       cout << *i << " ";
  //   }   
  // cout<<usfwfh.uf.size()<<endl; 
  
  return usfwfh.uf;
  
}

void US_fusion::update()
{  
  us_to_laser(); 
  usfwfh.ultrasonic_range_scan(this->get_clock()->now(),"us_scan_link",usfwfh.uf,US_fusion_pub);
  usfwfh.uf.clear(); 
  ur.clear();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<US_fusion>());
  rclcpp::shutdown();
  return 0;
}