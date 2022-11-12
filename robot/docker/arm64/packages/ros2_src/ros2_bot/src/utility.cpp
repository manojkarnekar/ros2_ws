#include "utility.hpp"
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RED(x) ((x)*M_PI/180.0)

class Utility : public rclcpp::Node
{
  public:
    explicit Utility() : Node("Utility")
    {
      wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "waypoints", 
        rclcpp::QoS(1).transient_local());

      navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose");

      waypoint_follower_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
        this,
        "follow_waypoints");

      nav_through_poses_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this,
        "navigate_through_poses");
      
      navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
      waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
      nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();

      target_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      cv_pose_arr = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cv_pose", 10, std::bind(&Utility::nav2_goal_pose_callback, this, _1));
      
      lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 
      rclcpp::SensorDataQoS(), 
      std::bind(&Utility::scanCb, this, _1));

      zed2S2pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/zed2_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
      zed2S2BSpub = this->create_publisher<sensor_msgs::msg::LaserScan>("/zed2BothSide_scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    }

  public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cv_pose_arr;

    int unique_id {0};
    std::unique_ptr<tf2_ros::TransformBroadcaster> target_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

    geometry_msgs::msg::TransformStamped tf_lf, tf_lb, tf_lb_u , tf_lb_d, tf_mm, tf_rf, tf_rb, tf_rb_u, tf_rb_d;

    // double d1, d2, d3, Id, p1, p2, Ip, gama, th1, th2, 
    // m, c, m_p, c_p, th_p, k, m_l, m_r, g, gpx, gpy, gpz, 
    // wp_x0, wp_z0, del_p, phi_1, phi_2, psi, gpx_l, gpz_l, gpx_r, gpz_r, gpx_m, 
    // gpz_m, m_m, gama_m, d_m, p_m, phi_m, psi_m, mic, idx, gamaA, gamaM, gamaC, 
    // dm, pa, pm, pc, gpxA, gpzA, gpxM, gpzM, gpxC, gpzC, tha, thc, ipx, l_x, l_y, l_z, 
    // dis, range, angle, Gap_L, Gap_R;

    std::string camera_frame ="camera_link";
    std::string map_frame ="map";
    std::string laser_frame = "laser_link";
    std::string bed_frameA = "bed_linkA";
    std::string bed_frameM = "bed_linkM";
    std::string bed_frameC = "bed_linkC";
    std::string leg_A_frame = "leg_A_frame";
    std::string leg_C_frame = "leg_C_frame";
    std::string bed_frame_mm = "bed_link_mm";
    std::string bed_frame_l = "bed_link_l";
    std::string bed_frame_lf = "bed_link_lf";
    std::string bed_frame_lb = "bed_link_lb";
    std::string bed_frame_lb_u = "bed_link_lb_u";
    std::string bed_frame_lb_d = "bed_link_lb_d";

    std::string bed_frame_r = "bed_link_r";
    std::string bed_frame_rf = "bed_link_rf";
    std::string bed_frame_rb = "bed_link_rb";
    std::string bed_frame_rb_u = "bed_link_rb_u";
    std::string bed_frame_rb_d = "bed_link_rb_d";

    geometry_msgs::msg::TransformStamped tf_sub(std::string parent_frame, std::string child_frame);
    void send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);

    // occ_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    // "/global_costmap/costmap",
    // rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    // std::bind(&Utility::Occupancy_cb, this, _1));

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2S2pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2S2BSpub;
    // void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    geometry_msgs::msg::PoseStamped nav2_pose(geometry_msgs::msg::TransformStamped tf_msg);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void resetUniqueId();
    int getUniqueId();
    void nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void bed_ai(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan);
    vector<double> Coor_find(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    vector<float> tanrule(float a, float b, float c);
    double Cosin(double a, double b, double c);
    double Sqrt(double x, double y);

    vector<double> minmax_idx(vector<double> v);
    vector<double> cor_lidar_cam(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double gpx, double gpz);
    vector<double> cor_lidar_cam_tf(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> ranges, std::string parent_frame,  std::string child_frame);
    vector<double> lidar_cluster_l(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame);
    vector<double> lidar_cluster_r(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame);
    void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
    void startNavigation(geometry_msgs::msg::PoseStamped pose);
    void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);

    using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using WaypointFollowerGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
    using NavThroughPosesGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
    
    // The NavigateToPose action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr 
      navigation_action_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
      waypoint_follower_action_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
      nav_through_poses_action_client_;

    // Goal-related state
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
    NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

    rclcpp::Node::SharedPtr client_node_;
    std::chrono::milliseconds server_timeout_;

    double d1, d2, d3, Id, p1, p2, Ip, gama, th1, th2, m, c, m_p, c_p, th_p, k, m_l, m_r, g, 
    gpx, gpy, gpz, wp_x0, wp_z0, del_p, phi_1, phi_2, psi, gpx_l, gpz_l, gpx_r, gpz_r, gpx_m, gpz_m, 
    m_m, gama_m, d_m, p_m, phi_m, psi_m, mic, idx;

    double gamaA, gamaM, gamaC, dm, pa, pm, pc, gpxA, gpzA, gpxM, gpzM, 
    gpxC, gpzC, tha, thc, ipx, l_x, l_y, l_z, dis, range, angle;

    int cnt = 0;
    int maxScanDataIndex,minScanDataIndex,maxBDIndex;
    float ScanPDRatio;


    vector<double> lidar_z, lidar_x;
    vector<float> uf;
    float grid_resolution;
    float bed_length=1.5;
    float pad = 0.50;
    float pady = 0.15;
    unsigned int grid_width, grid_height;
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;
    vector<float> zed2_to_laser(double rA, double aA, double rC, double aC, double incrAng,  double min_ang, double max_ang);
    vector<float> zed2BS_to_laser(float r_A, float a_A, float r_C, float a_C, float incr_Ang,  float minang, float maxang);
    void Zed2S2RangeScan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2scanpub,float incr, float minlAng, float maxlang);
    double quaternion_to_yaw(geometry_msgs::msg::TransformStamped tf_msg);


};

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

geometry_msgs::msg::PoseStamped Utility::nav2_pose(geometry_msgs::msg::TransformStamped tf_msg)
{   
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = tf_msg.transform.translation.x;
    goal_pose.pose.position.y = tf_msg.transform.translation.y;
    goal_pose.pose.position.z = tf_msg.transform.translation.z;
    // goal_pose.pose.orientation = orientationAroundZAxis(theta);
    goal_pose.pose.orientation.x = tf_msg.transform.rotation.x;
    goal_pose.pose.orientation.y = tf_msg.transform.rotation.y;
    goal_pose.pose.orientation.z = tf_msg.transform.rotation.z;
    goal_pose.pose.orientation.w = tf_msg.transform.rotation.w;
    return goal_pose;
}

void Utility::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
{
    resetUniqueId();
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    for (size_t i = 0; i < poses_.size(); i++)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header = poses_[i].header;
        arrow_marker.id = getUniqueId();
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose = poses_[i].pose;
        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.05;
        arrow_marker.scale.z = 0.02;
        arrow_marker.color.r = 0;
        arrow_marker.color.g = 255;
        arrow_marker.color.b = 0;
        arrow_marker.color.a = 1.0f;
        arrow_marker.lifetime = rclcpp::Duration(0s);
        arrow_marker.frame_locked = false;
        marker_array->markers.push_back(arrow_marker);

        // Draw a red circle at the waypoint pose
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header = poses_[i].header;
        circle_marker.id = getUniqueId();
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose = poses_[i].pose;
        circle_marker.scale.x = 0.05;
        circle_marker.scale.y = 0.05;
        circle_marker.scale.z = 0.05;
        circle_marker.color.r = 255;
        circle_marker.color.g = 0;
        circle_marker.color.b = 0;
        circle_marker.color.a = 1.0f;
        circle_marker.lifetime = rclcpp::Duration(0s);
        circle_marker.frame_locked = false;
        marker_array->markers.push_back(circle_marker);

        // Draw the waypoint number
        visualization_msgs::msg::Marker marker_text;
        marker_text.header = poses_[i].header;
        marker_text.id = getUniqueId();
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.pose = poses_[i].pose;
        marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
        marker_text.scale.x = 0.07;
        marker_text.scale.y = 0.07;
        marker_text.scale.z = 0.07;
        marker_text.color.r = 0;
        marker_text.color.g = 255;
        marker_text.color.b = 0;
        marker_text.color.a = 1.0f;
        marker_text.lifetime = rclcpp::Duration(0s);
        marker_text.frame_locked = false;
        marker_text.text = "wp_" + std::to_string(i + 1);
        marker_array->markers.push_back(marker_text);
    }

    if (marker_array->markers.empty()) 
    {
    visualization_msgs::msg::Marker clear_all_marker;
    clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_all_marker);
    }

    wp_navigation_markers_pub_->publish(std::move(marker_array));
    // poses_.clear();

}

void Utility::resetUniqueId()
{
  unique_id = 0;
}

int Utility::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}


void Utility::send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  // rclcpp::Time now = this->get_clock()->now();
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(th_x, th_y, th_z);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  target_tf->sendTransform(tf_msg);
}

void Utility::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  // rclcpp::Time now = this->get_clock()->now();
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(th_x, th_y, th_z);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  static_target_tf->sendTransform(tf_msg);
}

geometry_msgs::msg::TransformStamped Utility::tf_sub(std::string parent_frame, std::string child_frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped = tf_buffer_->lookupTransform(
    parent_frame.c_str(), child_frame.c_str(),
    tf2::TimePointZero);

    return transformStamped;
}

vector<float> Utility::tanrule(float a, float b, float c)
{
  double th;
  if (a !=0)
  {
    th = Cosin(a, b, c);
  }
  else
  {
    th = asin(c/b);
  }
  c = a*tan(th);
  vector<float> xy{a,c};
  return xy;
}

double Utility::Cosin(double a, double b, double c)
{
  return acos(((a*a)+(b*b)-(c*c))/(2*a*b));
}

double Utility::Sqrt(double x, double y)
{
  double h = hypot(x, y);
  return h;
}

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


void Utility::bed_ai(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  rclcpp::Time now = this->get_clock()->now();
  vector<double> co = Coor_find(msg);

  send_static_target_tf(now, camera_frame, bed_frameA, co.at(0),0.0,co.at(1), 0.0, 0.0, 0.0);
  send_static_target_tf(now, camera_frame, bed_frameM, co.at(2),0.0,co.at(3), 0.0, 0.0, 0.0);
  send_static_target_tf(now, camera_frame, bed_frameC, co.at(4),0.0,co.at(5), 0.0, 0.0, 0.0);
}


void Utility::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      navigation_goal_handle_.reset();
    };

  auto future_goal_handle =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  navigation_goal_handle_ = future_goal_handle.get();
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }
}


void Utility::startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  
  if (!is_action_server_ready) 
  {
    RCLCPP_ERROR(
      get_logger(), "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(
    get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());

  for (auto waypoint : waypoint_follower_goal_.poses) 
  {
    RCLCPP_DEBUG(
      get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      waypoint_follower_goal_handle_.reset();
    };

  auto future_goal_handle =
    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) 
  {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return;
  }



}


void Utility::startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint : nav_through_poses_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      nav_through_poses_goal_handle_.reset();
    };

  auto future_goal_handle =
    nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }
}

double Utility::quaternion_to_yaw(geometry_msgs::msg::TransformStamped tf_msg)
{
  tf2::Quaternion q(
    tf_msg.transform.rotation.x,
    tf_msg.transform.rotation.y,
    tf_msg.transform.rotation.z,
    tf_msg.transform.rotation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}