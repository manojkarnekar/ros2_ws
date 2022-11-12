#include "nav2_goal_pose/nav2_goal_pose.h"

vector<double> Utility::cor_lidar_cam(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double gpx, double gpz)
{
  vector<double> ranges {};
  for (long unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    range = scan_msg->ranges[i];
    angle = scan_msg->angle_min + i*scan_msg->angle_increment;
    l_y = range * sin(angle);
    l_x = -range * cos(angle);
    if(range > scan_msg->range_min && range < scan_msg->range_max)
    {
      ranges.push_back(Sqrt((l_x-gpz), (l_y-gpx)));
    }
    else
    {
      ranges.push_back(std::numeric_limits<float>::infinity());
    }
  }
  return ranges;
}

vector<double> Utility::cor_lidar_cam_tf(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> ranges, std::string parent_frame, std::string child_frame)
{
  vector<double> vec = minmax_idx(ranges);
  // double min = vec.at(0);
  int i = vec.at(1);
  
  range = scan_msg->ranges[i];
  angle = scan_msg->angle_min + i*scan_msg->angle_increment;

  l_y = range * sin(angle);
  l_x = range * cos(angle);

  // rclcpp::Time now = this->get_clock()->now();
  // send_static_target_tf(now, parent_frame, child_frame, l_x, l_y, 0.0, 0.0, 0.0, atan(l_y/l_x));

  try
  {
    rclcpp::Time now = this->get_clock()->now();
    send_static_target_tf(now, parent_frame, child_frame, l_x, l_y, 0.0, 0.0, 0.0, 0.0);

  }
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", 
    parent_frame.c_str(), child_frame.c_str(), ex.what());
  }
  vector<double> range_angle {};
  range_angle.push_back(range); // [0]
  range_angle.push_back(angle); // [1]
  range_angle.push_back(l_x);  //  [2]
  range_angle.push_back(l_y);  //  [3]
  range_angle.push_back(scan_msg->angle_increment);  // [4]
  range_angle.push_back(scan_msg->angle_min);   // [5]
  range_angle.push_back(scan_msg->angle_max);   // [6]

  ranges.clear();
  return range_angle;
}

vector<double> Utility::lidar_cluster_l(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame)
{
  vector<double> ranges_ {};
  vector<double> vals_ {};
  vector<double> angles_ {};
  for(double angle_=range_angle.at(1)+1.0; angle_> range_angle.at(1)+0.1 ; angle_ -= scan_msg->angle_increment)
  {
    long unsigned int i_= (angle_ - scan_msg->angle_min)/(scan_msg->angle_increment);
    float range_ = scan_msg->ranges[i_];
    float l_x_ = range_ * cos(angle_);
    float l_y_ = range_ * sin(angle_);

    if((range_ != std::numeric_limits<float>::infinity()) && (range_ > 0.5))
    {
      ranges_.push_back(Sqrt((l_x_- lxt), (l_y_- lyt)));
      vals_.push_back(range_);
      angles_.push_back(angle_);
    }

  }

  vector<double> vec_ = minmax_idx(ranges_);
  // double val_ = vec_.at(0);
  int idx_ = vec_.at(1);

  range = vals_[idx_];
  angle = angles_[idx_];

  l_y = range * sin(angle);
  l_x = range * cos(angle);

  try
  {
    rclcpp::Time now = this->get_clock()->now();
    send_static_target_tf(now, parent_frame, child_frame, l_x, l_y, 0.0, 0.0, 0.0, 0.0);

  }
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", 
    parent_frame.c_str(), child_frame.c_str(), ex.what());
  }
  
  vec_.clear();
  vals_.clear();
  angles_.clear();
  ranges_.clear();
  range_angle.clear();

  vector<double> ra {};
  ra.push_back(l_x);
  ra.push_back(l_y);

  return ra;
}

vector<double> Utility::lidar_cluster_r(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame)
{
  vector<double> ranges_ {};
  vector<double> vals_ {};
  vector<double> angles_ {};
  for(double angle_ = range_angle.at(1)-1.0; angle_<range_angle.at(1)-0.1; angle_ += scan_msg->angle_increment)
  {
    long unsigned int i_= (angle_ - scan_msg->angle_min)/(scan_msg->angle_increment);
    float range_ = scan_msg->ranges[i_];
    float l_x_ = range_ * cos(angle_);
    float l_y_ = range_ * sin(angle_);

    if((range_ != std::numeric_limits<float>::infinity()) && (range_ > 0.5))
    {
      ranges_.push_back(Sqrt((l_x_- lxt), (l_y_- lyt)));
      vals_.push_back(range_);
      angles_.push_back(angle_);
    }

  }

  vector<double> vec_ = minmax_idx(ranges_);
  // double val_ = vec_.at(0);
  int idx_ = vec_.at(1);

  range = vals_[idx_];
  angle = angles_[idx_];

  l_y = range * sin(angle);
  l_x = range * cos(angle);

  try
  {
    rclcpp::Time now = this->get_clock()->now();
    send_static_target_tf(now, parent_frame, child_frame, l_x, l_y, 0.0, 0.0, 0.0, 0.0);

  }
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", 
    parent_frame.c_str(), child_frame.c_str(), ex.what());
  }
  
  vec_.clear();
  vals_.clear();
  angles_.clear();
  ranges_.clear();
  range_angle.clear();

  vector<double> ra {};
  ra.push_back(l_x);
  ra.push_back(l_y);

  return ra;
}
vector<double> Utility::Coor_find(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  vector<double> A {msg->data.at(0), msg->data.at(1)};
  vector<double> B {msg->data.at(2), msg->data.at(3)};
  vector<double> C {msg->data.at(4), msg->data.at(5)};
  vector<double> D {0.0, msg->data.at(11)};
  vector<double> M {(A.at(0)+C.at(0))/2, (A.at(1)+C.at(1))/2};

  Id = Sqrt(D.at(0), D.at(1));

  d1 = Sqrt(A.at(0), A.at(1));
  dm = Sqrt(M.at(0), M.at(1));
  d3 = Sqrt(C.at(0), C.at(1));

  pa = Sqrt((D.at(0)-A.at(0)), (D.at(1)-A.at(1)));
  pm = Sqrt((D.at(0)-M.at(0)), (D.at(1)-M.at(1)));
  pc = Sqrt((D.at(0)-C.at(0)), (D.at(1)-C.at(1)));


  gamaA = Cosin(Id, d1, pa);
  gamaM = Cosin(Id, dm, pm);
  gamaC = Cosin(Id, d3, pc);

  tha=abs(gamaM-gamaA);
  thc=abs(gamaM-gamaC);
  ipx=msg->data.at(9);

  if (A.at(0)<0 && M.at(0)>0 && C.at(0)>0)
  {
    gamaA = -gamaA;
    gamaM = gamaM;
    gamaC = gamaC;
  }
  if (A.at(0)<0 && M.at(0)<0 && C.at(0)>0)
  {
    gamaA = -gamaA;
    gamaM = -gamaM;
    gamaC = gamaC;

  }
  if (A.at(0)<0 && M.at(0)<0 && C.at(0)<0)
  {
    gamaA = -gamaA;
    gamaM = -gamaM;
    gamaC = -gamaC;
  }

  gpxA = d1*sin(gamaA);
  gpzA = d1*cos(gamaA);

  gpxM = dm*sin(gamaM);
  gpzM = dm*cos(gamaM);

  gpxC = d3*sin(gamaC);
  gpzC = d3*cos(gamaC);

  vector<double> line_ver {gpxA, gpzA, gpxM, gpzM, gpxC,gpzC};
  
  return line_ver;
}

vector<double> Utility::minmax_idx(vector<double> v)
{
  auto it = std::minmax_element(v.begin(), v.end());
	double min = *it.first;
	// double max = *it.second;
  double i = it.first-v.begin();

  vector<double> vec {};
  vec.push_back(min);
  vec.push_back(i);
  return vec;
}

void Utility::scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  vector<double> A_ranges = cor_lidar_cam(scan_msg, gpxA, gpzA);
  vector<double> C_ranges = cor_lidar_cam(scan_msg, gpxC, gpzC);
  // std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
  acummulated_poses_.clear();

  try
  {
    vector<double> l = cor_lidar_cam_tf(scan_msg, A_ranges, laser_frame, leg_A_frame);
    vector<double> r = cor_lidar_cam_tf(scan_msg, C_ranges, laser_frame, leg_C_frame);

    double l_x = l.at(2);
    double l_y = l.at(3);
    double r_x = r.at(2);
    double r_y = r.at(3);

    double th_m_h = atan2((l_y - r_y), (l_x - r_x));
    double th_m_v = atan2(-(l_x - r_x), (l_y - r_y));

    double k = 0.7;
    double x_h = k*cos(th_m_h);
    double y_h = k*sin(th_m_h);

    double x_v = k*cos(th_m_v);
    double y_v = k*sin(th_m_v);
    
    // RCLCPP_INFO(get_logger(),"A_range->{%.2lf}, A_angle->{%.2lf}, C_range->{%.2lf}, C_angle->{%.2lf}",
    //     l.at(0), 
    //     RAD2DEG(l.at(1)),
    //     r.at(0), 
    //     RAD2DEG(r.at(1)) );

    vector<float> ls = zed2_to_laser(l.at(0),l.at(1),r.at(0), r.at(1), l.at(4), l.at(5), l.at(6));
    vector<float> lsf = zed2BS_to_laser(l.at(0),l.at(1),r.at(0), r.at(1), l.at(4), l.at(5), l.at(6));
    float maxSAng = DEG2RED(maxScanDataIndex/ScanPDRatio);
    // int maxBDAng = DEG2RED(maxBDIndex/ScanPDRatio);
    // cout <<"maxSAng = "<<RAD2DEG(maxSAng)<<" maxScanDataIndex = "<<maxScanDataIndex<<" l.at(5) = "<<RAD2DEG((l).at(5))<<"maxBDAng = "<< RAD2DEG(maxBDAng) <<endl;

    Zed2S2RangeScan(this->get_clock()->now(),"laser_link",ls,zed2S2pub,l.at(4),l.at(5) ,maxSAng );
    Zed2S2RangeScan(this->get_clock()->now(),"laser_link",lsf,zed2S2BSpub,l.at(4),l.at(5) ,maxSAng );

    rclcpp::Time now = this->get_clock()->now();

    send_static_target_tf(now, laser_frame, bed_frame_mm, ((l_x + r_x)/2) - (2.0*cos(th_m_v)), ((l_y + r_y)/2) - (2.0*sin(th_m_v)), 0.0, 0.0, 0.0, ((M_PI/2.0)-th_m_h));

    send_static_target_tf(now, laser_frame, bed_frame_l, (l_x + x_h) , (l_y + y_h), 0.0, 0.0, 0.0, th_m_h);
    send_static_target_tf(now, laser_frame, bed_frame_lf, (l_x + x_h + x_v), (l_y + y_h + y_v), 0.0, 0.0, 0.0, (M_PI+th_m_h));
    send_static_target_tf(now, laser_frame, bed_frame_lb_u, (l_x + x_h - x_v), (l_y + y_h - y_v), 0.0, 0.0, 0.0, M_PI/2.0-th_m_h);
    send_static_target_tf(now, laser_frame, bed_frame_lb_d, (l_x + x_h - x_v), (l_y + y_h - y_v), 0.0, 0.0, 0.0, M_PI/2.0+th_m_h);

    send_static_target_tf(now, laser_frame, bed_frame_r, (r_x - x_h), (r_y - y_h) , 0.0, 0.0, 0.0, th_m_h);
    send_static_target_tf(now, laser_frame, bed_frame_rf, (r_x - x_h + x_v), (r_y - y_h + y_v) , 0.0, 0.0, 0.0, th_m_h);
    send_static_target_tf(now, laser_frame, bed_frame_rb_u, (r_x - x_h - x_v), (r_y - y_h - y_v) , 0.0, 0.0, 0.0, M_PI/2.0-th_m_h);
    send_static_target_tf(now, laser_frame, bed_frame_rb_d, (r_x - x_h - x_v), (r_y - y_h - y_v) , 0.0, 0.0, 0.0, M_PI/2.0+th_m_h);

    tf_lf = tf_sub(map_frame, bed_frame_lf);
    tf_lb_u = tf_sub(map_frame, bed_frame_lb_u);
    tf_lb_d = tf_sub(map_frame, bed_frame_lb_d);

    tf_mm = tf_sub(map_frame, bed_frame_mm);

    tf_rf = tf_sub(map_frame, bed_frame_rf);
    tf_rb_u = tf_sub(map_frame, bed_frame_rb_u);
    tf_rb_d = tf_sub(map_frame, bed_frame_rb_d);

    acummulated_poses_.push_back(nav2_pose(tf_lf));
    acummulated_poses_.push_back(nav2_pose(tf_rf));

    updateWpNavigationMarkers(acummulated_poses_);
    startWaypointFollowing(acummulated_poses_);

    acummulated_poses_.clear();
    uf.clear(); 
    ls.clear();
    lsf.clear();
    
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

}


