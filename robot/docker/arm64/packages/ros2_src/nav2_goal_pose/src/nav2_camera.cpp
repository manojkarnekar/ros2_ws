#include "nav2_goal_pose/nav2_goal_pose.h"

vector<float> Utility::zed2_to_laser(double rA, double aA, double rC, double aC, double incrAng,  double min_ang, double max_ang)
{
  ScanPDRatio = 1/RAD2DEG(incrAng);
  
  // float p;
  float maxAng;
  float minAng;
  float bdfl = (rA+rC)/2 - pad;
  float thpd = RAD2DEG(atan(pady/bdfl));

  maxAng = RAD2DEG(aA)-RAD2DEG(min_ang) + RAD2DEG(max_ang) + thpd;
  if ((aA < 0 && aC < 0) || (aA >0 && aC > 0))
  {
    maxAng = RAD2DEG(aA) + thpd; 
  }
  
  minAng = RAD2DEG(aC) - thpd ;
  // float NoScanData = (max_ang - min_ang)/incrAng;
  ScanPDRatio = 1/RAD2DEG(incrAng);
  maxScanDataIndex = abs(ScanPDRatio*(maxAng-RAD2DEG(min_ang)));
  minScanDataIndex = abs(ScanPDRatio*(minAng - RAD2DEG(min_ang)));
  // float p=Sqrt((rA*cos(aA) - rC*cos(aC)) , (rA*sin(aA)- rC*sin(aC)));
  
  // cout<<"maxAng-RAD2DEG(min_ang) "<<maxAng-RAD2DEG(min_ang)<<endl;
  // cout<<" (minAng - RAD2DEG(min_ang)) "<<(minAng - RAD2DEG(min_ang))<<endl;
  
  for( int i=0; i<maxScanDataIndex; i+=1)
  {
      uf.push_back(std::numeric_limits<double>::infinity());
  }

  for( int i=minScanDataIndex; i<maxScanDataIndex; i+=1)
  {
    float XA = rA*cos(aA);
    float YA = rA*sin(aA);
    // cout <<"YA"<<YA<<endl;
    float XC = rC*cos(aC);
    float YC = rC*sin(aC);
    float THpB = atan2(-(XC-XA),(YC-YA));
    XA = (pad )*cos(THpB) + XA;
    XC = (pad )*cos(THpB) + XC;
    float NOD =(maxScanDataIndex - minScanDataIndex);
    int j= (i-minScanDataIndex);
    float PX = (j*XA + (NOD-j)*XC)/(NOD);
    float PY = (j*YA + (NOD-j)*YC)/(NOD);
    float drange = Sqrt(PX,PY);
    uf.at(i)= drange;
  }
  return uf;
}

vector<float> Utility::zed2BS_to_laser(float r_A, float a_A, float r_C, float a_C, float incr_Ang,  float minang, float maxang)
{
  vector<float> zfb;

  // float p;
  float maxAng;
  float minAng;

  ScanPDRatio = 1/RAD2DEG(incr_Ang);

  if ( a_A < 0 && a_C > 0 )
  {
    maxBDIndex=(RAD2DEG(maxang - minang) + RAD2DEG(a_A)- RAD2DEG(minang))*ScanPDRatio;
    minScanDataIndex=RAD2DEG(a_C - minang)*ScanPDRatio;
  }

  
  maxAng = RAD2DEG(a_A)-RAD2DEG(minang) + RAD2DEG(maxang);
  if ((a_A < 0 && a_C < 0) || (a_A >0 && a_C > 0))
  {
    maxAng = RAD2DEG(a_A);
  }
  
  minAng = RAD2DEG(a_C) ;
  // float NoScanData = (maxang - minang)/incr_Ang;
  ScanPDRatio = 1/RAD2DEG(incr_Ang);
  maxBDIndex = abs(ScanPDRatio*(maxAng-RAD2DEG(minang)));
  minScanDataIndex = abs(ScanPDRatio*(minAng - RAD2DEG(minang)));

  
  for( int i=0; i<maxBDIndex; i+=1)
  {
      zfb.push_back(std::numeric_limits<double>::infinity());
  }

  float X_A = r_A*cos(a_A);
  float Y_A = r_A*sin(a_A);
  float X_C = r_C*cos(a_C);
  float Y_C = r_C*sin(a_C);
  float THAmr = atan2(-(X_C-X_A),(Y_C - Y_A)); // mp =-1/m;
  // float THAmr_ = atan2((X_C-X_A),-(Y_C - Y_A)); // mp =-1/m;

  float XRA = X_A - bed_length*cos(THAmr);
  float YRA = Y_A - bed_length*sin(THAmr);
  float XRC = X_C - bed_length*cos(THAmr);
  float YRC = Y_C - bed_length*sin(THAmr);

  float aRC = atan2(YRC,XRC);
  float aRA = atan2(YRA,XRA);

  float RCScanDataIndex = minScanDataIndex + abs(RAD2DEG(aRC-a_C)*ScanPDRatio);
  float RAScanDataIndex = maxBDIndex - abs(RAD2DEG(aRA-a_A)*ScanPDRatio);

  if (abs(RAD2DEG(aRA-a_A))>300)
  {
    RAScanDataIndex = maxBDIndex-((RAD2DEG(a_A-minang)+RAD2DEG(maxang-aRA))*ScanPDRatio);
    // cout<<" RAScanDataIndex --- "<<RAScanDataIndex<<endl;
  }
      
    
  for( int i=RAScanDataIndex; i<maxBDIndex; i+=1)
  {
    float NOD =(maxBDIndex - RAScanDataIndex);
    int j= (i-RAScanDataIndex);

    float PX = (j*X_A + (NOD-j)*XRA)/(NOD);
    float PY = (j*Y_A + (NOD-j)*YRA)/(NOD);
    float drange = Sqrt(PX,PY);
    zfb.at(i)= drange;
  }

  for( int i=minScanDataIndex; i<RCScanDataIndex; i+=1)
  {
    float NOD =(RCScanDataIndex - minScanDataIndex);
    int j= (i-minScanDataIndex);

    float PX = (j*XRC + (NOD-j)*X_C)/(NOD);
    float PY = (j*YRC + (NOD-j)*Y_C)/(NOD);
    float drange = Sqrt(PX,PY);
    zfb.at(i)= drange;
  }
    
  return zfb;
}

void Utility::Zed2S2RangeScan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2scanpub,float incr, float minlAng, float maxlang)
{
  sensor_msgs::msg::LaserScan us_scan_msg; 
  us_scan_msg.header.stamp = now;
  us_scan_msg.header.frame_id = frame_id;
  us_scan_msg.angle_min = minlAng;  //-40*(M_PI/180);
  us_scan_msg.angle_max =  maxlang;
  us_scan_msg.angle_increment = incr;
  us_scan_msg.time_increment = 0;
  us_scan_msg.scan_time = 0.0;
  us_scan_msg.range_min = 0.005;
  us_scan_msg.range_max = 10.00;
  us_scan_msg.ranges.resize(ranges.size());
  us_scan_msg.intensities.resize(ranges.size());
  us_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(size_t i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  us_scan_msg.intensities=inten;

  zed2scanpub->publish(us_scan_msg);

  inten.clear();
  ranges.clear();
}

