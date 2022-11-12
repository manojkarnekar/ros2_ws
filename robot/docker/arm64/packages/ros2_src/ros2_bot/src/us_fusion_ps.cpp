#include "us_fusion_ps.hpp"
US_fusion_header usfh;
using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

class US_fusion : public rclcpp::Node
{
  public:
    US_fusion()
    : Node("us_fusion_ps")
    {
      US_fusion_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/us_scan", 10, std::bind(&US_fusion::US_fusion_cb, this, _1));
      timer_ = this->create_wall_timer(71.42ms, std::bind(&US_fusion::update, this)); 
      US_fusion_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/uf_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
    }
    
private:
    void US_fusion_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr US_fusion_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr US_fusion_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<float> us_to_laser();
    void update();
    vector<float> c_t(float x, float dx, float dy, float theta);
    rclcpp::Time then = this->get_clock()->now();
    float d_,x_,y_;
    int phic_,phi_,us_,ue_;   
};

void US_fusion::US_fusion_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)   
{     
    for(int i=0; i<12; i++)
    {
      usfh.raw[i] = msg->data[i]/100.00;
    }   
    // update();
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

vector<float> US_fusion::us_to_laser()
{
  //  cout<<"check"<<endl;

    for( int i=0; i<720; i+=1)
    {
        usfh.uf.push_back(std::numeric_limits<double>::infinity());
    }
      if (usfh.raw[0]< 1.0 && usfh.raw[0] >= 0.05)
      {
        vector<float> us_scan1= c_t(usfh.raw[0],0.252,0.228,0.74); //dx1=x1-x'; dy1=y1-y'
        for(int i=us_scan1.at(1); i<us_scan1.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan1.at(0); 
        } 
      } 

      if (usfh.raw[1] < 1.0 && usfh.raw[1] >= 0.05)
      {
        vector<float> us_scan2=c_t(usfh.raw[1],0.2645,0.140,0.0);
        for(int i=us_scan2.at(1); i<us_scan2.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan2.at(0);
        } 
      }
    
      if (usfh.raw[2] < 1.0 && usfh.raw[2] >= 0.05)
      {
        vector<float> us_scan3= c_t(usfh.raw[2],0.2645,-0.140,0.0);
        for(int i=us_scan3.at(1); i<us_scan3.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan3.at(0);
        } 
      }
    
      if (usfh.raw[3] < 1.0 && usfh.raw[3] >= 0.05)
      {
        vector<float> us_scan4 = c_t(usfh.raw[3],0.251,-0.228,-0.74);
        for(int i=us_scan4.at(1); i<us_scan4.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan4.at(0);
        } 
      }

      if (usfh.raw[4] < 1.0 && usfh.raw[4] >= 0.05)
      {
        vector<float> us_scan5= c_t(usfh.raw[4],0.135,-0.2505,-1.48);
        for(int i=us_scan5.at(1); i<us_scan5.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan5.at(0);
        } 
      }

      if (usfh.raw[5] < 1.0 && usfh.raw[5] >= 0.05)
      {
        vector<float> us_scan6= c_t(usfh.raw[5],-0.135,-0.274,-1.48);
        for(int i=us_scan6.at(1); i<us_scan6.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan6.at(0);
        
        } 
      }

      if (usfh.raw[6] < 1.0 && usfh.raw[6] >= 0.05)
      {
        vector<float> us_scan7= c_t(usfh.raw[6],-0.248,-0.2695,-2.31159);
        for(int i=us_scan7.at(1); i<us_scan7.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan7.at(0);
        
        } 
      }

      if (usfh.raw[7] < 1.0 && usfh.raw[7] >= 0.05)
      {
        vector<float> us_scan8= c_t(usfh.raw[7],-0.2645,-0.140,3.14);
        for(int i=us_scan8.at(1); i<us_scan8.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan8.at(0);
        
        } 
      }

      if (usfh.raw[8] < 1.0 && usfh.raw[8] >= 0.05)
      {
        vector<float> us_scan9= c_t(usfh.raw[8],-0.2645,0.140,3.14);
        for(int i=us_scan9.at(1); i<us_scan9.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan9.at(0);
        
        } 
      }

      if (usfh.raw[9] < 1.0 && usfh.raw[9] >= 0.05)
      {
        vector<float> us_scan10= c_t(usfh.raw[9],-0.248,0.2695,2.31159);
        for(int i=us_scan10.at(1); i<us_scan10.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan10.at(0);
        
        } 
      }

      if (usfh.raw[10] < 1.0 && usfh.raw[10] >= 0.05)
      {
        vector<float> us_scan11= c_t(usfh.raw[10],-0.155,0.274,1.48);
        for(int i=us_scan11.at(1); i<us_scan11.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan11.at(0);
        
        } 
      }

      if (usfh.raw[11] < 1.0 && usfh.raw[11] >= 0.05)
      {
        vector<float> us_scan12= c_t(usfh.raw[11],0.135,0.2505,1.48);
        for(int i=us_scan12.at(1); i<us_scan12.at(2); i+=1)
        {
        usfh.uf.at(i)=us_scan12.at(0);
        
        } 
      }

    for (auto i = usfh.uf.begin(); i != usfh.uf.end(); ++i)
    {
        cout << *i << " ";
    }   
  cout<<usfh.uf.size()<<endl; 
  return usfh.uf;
  
}

void US_fusion::update()
{  
  us_to_laser(); 
  usfh.ultrasonic_range_scan(this->get_clock()->now(),"us_scan_link",usfh.uf,US_fusion_pub);
  usfh.uf.clear(); 

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<US_fusion>());
  rclcpp::shutdown();
  return 0;
}