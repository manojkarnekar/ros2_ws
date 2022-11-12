#include "utility.cpp"

// Utility u;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public Utility
{
  public:
    int p, h, phi;

};


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

    

    // acummulated_poses_.push_back(nav2_pose(tf_mm));
    // acummulated_poses_.push_back(nav2_pose(tf_lb_u));
    acummulated_poses_.push_back(nav2_pose(tf_lf));
    // acummulated_poses_.push_back(nav2_pose(tf_lb_d));

    // acummulated_poses_.push_back(nav2_pose(tf_mm));

    // acummulated_poses_.push_back(nav2_pose(tf_rb_u));
    acummulated_poses_.push_back(nav2_pose(tf_rf));
    // acummulated_poses_.push_back(nav2_pose(tf_rb_d));
    // acummulated_poses_.push_back(nav2_pose(tf_mm));

    // RCLCPP_INFO(get_logger(),"th_m_h->{%.2lf}, th_m_v->{%.2lf}, quaternion_to_yaw->{%.2lf}",
    //     RAD2DEG(th_m_h), 
    //     RAD2DEG(th_m_v),
    //     RAD2DEG(quaternion_to_yaw(tf_lf)));

    // if(cnt<1)
    // {
    updateWpNavigationMarkers(acummulated_poses_);
    startWaypointFollowing(acummulated_poses_);
      // RCLCPP_INFO(
      // this->get_logger(), "-----------------------------------%s","ok");
    // }
    // cnt +=1;

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

void Utility::nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  bed_ai(msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
