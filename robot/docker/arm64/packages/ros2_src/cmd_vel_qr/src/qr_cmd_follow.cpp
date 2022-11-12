#include "qr_code_follow.hpp"

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

class QR_code_follow : public rclcpp::Node
{
  public:
    QR_code_follow()
    : Node("QR_code_follow")
    { 
        qr_pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "qr_pose", 10, std::bind(&QR_code_follow::qr_poseCb, this, _1));

        img_pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "img_pose", 10, std::bind(&QR_code_follow::img_poseCb, this, _1));

        qr_data_sub = this->create_subscription<std_msgs::msg::String>(
            "qr_data", 10, std::bind(&QR_code_follow::qr_dataCb, this, _1));
        
        static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "waypoints", 
        rclcpp::QoS(1).transient_local());

        goal_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(150ms, std::bind(&QR_code_follow::update, this));

        // cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 
        rclcpp::SensorDataQoS(), 
        std::bind(&QR_code_follow::cor_lidar_cam, this, _1));

    }
    public:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr qr_pose_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr img_pose_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_data_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        

        void qr_poseCb(const std_msgs::msg::Float32MultiArray::SharedPtr qr_pose);
        void img_poseCb(const std_msgs::msg::Float32MultiArray::SharedPtr img_pose);
        void qr_dataCb(const std_msgs::msg::String::SharedPtr qr_data);

        geometry_msgs::msg::PoseStamped nav2_pose(geometry_msgs::msg::TransformStamped tf_msg);
        void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
        void resetUniqueId();
        int getUniqueId();

        int unique_id {0};

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

        std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

        void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);
        geometry_msgs::msg::TransformStamped tf_sub(std::string parent_frame, std::string child_frame);

        void move(double x, double y);
        void stop();

        double Cosin(double a, double b, double c);
        double Sqrt(double x, double y);
        // vector<float> A;
        // vector<float> O;

        double o_x, o_z, a_x, a_z , d_po, d_pa, d_oa, gama_, gpxM, gpzM, gpxM_T, gpzM_T;
        double k = 0.8;
        double stopDistance_ = 0.8;
        double speed_= 0.75;
        double l_v = 0.0;
        double a_v = 0.0;
        double iGain_ = 3.0;
        double pGain_ = 3.0;

        std::string qr_frame ="qr_link";
        std::string qr_target_frame ="qr_target_link";
        std::string camera_frame ="camera_link";
        std::string map_frame ="map";

        geometry_msgs::msg::TransformStamped tf_target;
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::Twist cmd_vel_msg;
        rclcpp::TimerBase::SharedPtr timer_;
        string qr_code_data;
        const char *qr_code_target = "Hello :)";
        int flag = 1;

        void update();
        void slow_down();

        // rmw_qos_profile_t cmd_vel_qos_profile = rmw_qos_profile_sensor_data;
        // cmd_vel_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        // cmd_vel_qos_profile.depth = 50;
        // cmd_vel_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        // cmd_vel_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;



};

void QR_code_follow::update()
{
  if (strcmp(qr_code_data.c_str(), qr_code_target)==0)
  {
    flag = 1;
    RCLCPP_INFO(this->get_logger(), "qr_code_data: '%s'", qr_code_data.c_str());

    d_po = Sqrt(0.0, o_z);
    d_pa = Sqrt(a_x, a_z);

    d_oa = Sqrt((o_x - a_x), (o_z - a_z));

    gama_ = Cosin(d_po, d_pa, d_oa);

    if (a_x < 0)
    {
        gama_ = -abs(gama_);
    }

    if (a_x > 0)
    {
        gama_ = abs(gama_);
    }

    gpxM = d_pa*sin(gama_);
    gpzM = d_pa*cos(gama_);

    gpxM_T = gpxM - k*sin(gama_);
    gpzM_T = gpzM - k*cos(gama_);

    rclcpp::Time now = this->get_clock()->now();

    if(isfinite(gpzM) && isfinite(gpxM))
    {
      double currentDistance = Sqrt(gpzM, gpxM);
      RCLCPP_INFO(get_logger(), "qr_code_data: '%s' gpxM--> %lf gpzM--> %lf currentDistance--> %lf",qr_code_data.c_str(), gpxM, gpzM, currentDistance);
      if(currentDistance>stopDistance_ && currentDistance<1.7)
      {
        move(gpzM, gpxM);
      }
      
      else
      {
        stop();
      }
    }
    
  }

  else
  {
    while(flag < 4000)
    {
      flag ++;
      slow_down();
    }
    stop();
  }
  // qr_code_data = " ";
  
  
}

void QR_code_follow::qr_poseCb(const std_msgs::msg::Float32MultiArray::SharedPtr qr_pose)
{
    a_x = qr_pose->data.at(0);
    a_z = qr_pose->data.at(2);
    // RCLCPP_INFO(get_logger(), "a_x--> %lf a_z--> %lf", a_x, a_z);

}

void QR_code_follow::img_poseCb(const std_msgs::msg::Float32MultiArray::SharedPtr img_pose)
{
    o_x = img_pose->data.at(0);
    o_z = img_pose->data.at(2);
    // RCLCPP_INFO(get_logger(), "o_x--> %lf o_z--> %lf", o_x, o_z);
}

void QR_code_follow::qr_dataCb(const std_msgs::msg::String::SharedPtr qr_data)
{
  qr_code_data = qr_data->data;
  


}

void QR_code_follow::move(double x, double y)
{
    
    double currentDistance = Sqrt(x, y);

    l_v = (currentDistance - stopDistance_)*speed_;

    if (l_v>0.4)
    {
        l_v = 0.4;
    }

    a_v = 0.6*atan(-y/x) ;

    if (a_v > 0.8)
    {
      a_v = 0.8;
    }

    if (a_v < -0.8)
    {
      a_v = -0.8;
    }

    cmd_vel_msg.linear.x =  l_v;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = a_v;

    cmd_vel_pub->publish(cmd_vel_msg);


}

void QR_code_follow::slow_down()
{
  
    cmd_vel_msg.linear.x = l_v/20.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    cmd_vel_pub->publish(cmd_vel_msg);
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));
  
  
}

void QR_code_follow::stop()
{
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    cmd_vel_pub->publish(cmd_vel_msg);

}

vector<double> QR_code_follow::cor_lidar_cam(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  vector<double> ranges {};

    d_po = Sqrt(0.0, o_z);
    d_pa = Sqrt(a_x, a_z);

    d_oa = Sqrt((o_x - a_x), (o_z - a_z));

    gama_ = Cosin(d_po, d_pa, d_oa);

    if (a_x < 0)
    {
        gama_ = -abs(gama_);
    }

    if (a_x > 0)
    {
        gama_ = abs(gama_);
    }

    gpxM = d_pa*sin(gama_);
    gpzM = d_pa*cos(gama_);

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

vector<double> QR_code_follow::cor_lidar_cam_tf(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> ranges, std::string parent_frame, std::string child_frame)
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

double QR_code_follow::Sqrt(double x, double y)
{
  double h = hypot(x, y);
  return h;
}

double QR_code_follow::Cosin(double a, double b, double c)
{
  return acos(((a*a)+(b*b)-(c*c))/(2*a*b));
}

geometry_msgs::msg::PoseStamped QR_code_follow::nav2_pose(geometry_msgs::msg::TransformStamped tf_msg)
{   
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = tf_msg.transform.translation.x;
    goal_pose.pose.position.y = tf_msg.transform.translation.y;
    goal_pose.pose.position.z = 0.0;
    // goal_pose.pose.orientation = orientationAroundZAxis(theta);
    goal_pose.pose.orientation.x = tf_msg.transform.rotation.x;
    goal_pose.pose.orientation.y = tf_msg.transform.rotation.y;
    goal_pose.pose.orientation.z = tf_msg.transform.rotation.z;
    goal_pose.pose.orientation.w = tf_msg.transform.rotation.w;
    return goal_pose;
}

void QR_code_follow::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
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

void QR_code_follow::resetUniqueId()
{
  unique_id = 0;
}

int QR_code_follow::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}

void QR_code_follow::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
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

geometry_msgs::msg::TransformStamped QR_code_follow::tf_sub(std::string parent_frame, std::string child_frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped = tf_buffer_->lookupTransform(
    parent_frame.c_str(), child_frame.c_str(),
    tf2::TimePointZero);

    return transformStamped;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QR_code_follow>());
  rclcpp::shutdown();
  return 0;
}