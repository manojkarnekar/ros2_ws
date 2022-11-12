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

    }
    public:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr qr_pose_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr img_pose_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_data_sub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;


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

        double Cosin(double a, double b, double c);
        double Sqrt(double x, double y);
        // vector<float> A;
        // vector<float> O;

        double o_x, o_z, a_x, a_z , d_po, d_pa, d_oa, gama_, gpxM, gpzM, gpxM_T, gpzM_T;
        double k = 0.8;

        std::string qr_frame ="qr_link";
        std::string qr_target_frame ="qr_target_link";
        std::string camera_frame ="camera_link";
        std::string map_frame ="map";

        geometry_msgs::msg::TransformStamped tf_target;
        geometry_msgs::msg::PoseStamped pose;



};

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
    try
    {
      if(isfinite(gpzM) && isfinite(gpxM))
      {
        send_static_target_tf(now, camera_frame, qr_frame, gpxM, 0.0, gpzM, 0.0, 0.0, 0.0);
        if(gpxM_T < 0.0)
        {
            float beta = atan2(-gpxM_T, gpzM_T);
            send_static_target_tf(now, camera_frame, qr_target_frame, gpxM_T, 0.0, gpzM_T, 0.0, -1.57 - beta, 0.0);
        }
        else
        {
            float beta = atan2(gpxM_T, gpzM_T);
            send_static_target_tf(now, camera_frame, qr_target_frame, gpxM_T, 0.0, gpzM_T, 0.0, -1.57 + beta, 0.0);
        }
        // send_static_target_tf(now, camera_frame, qr_frame, gpzM, gpxM, 0.0, 0.0, 0.0, 0.0);
        // send_static_target_tf(now, camera_frame, qr_target_frame, gpzM_T, gpxM_T, 0.0, 0.0, -1.57, 0.0);

        // send_static_target_tf(now, camera_frame, qr_frame, gpzM, gpxM, 0.0, 0.0, 0.0, 0.0);
        // send_static_target_tf(now, camera_frame, qr_target_frame, gpzM_T, gpxM_T, 0.0, 0.0, 0.0, 0.0);

        RCLCPP_INFO(get_logger(), "gpxM--> %lf gpzM--> %lf", gpxM, gpzM);

        tf_target = tf_sub(map_frame, qr_target_frame);
        pose = nav2_pose(tf_target);
        acummulated_poses_.push_back(pose);
        updateWpNavigationMarkers(acummulated_poses_);
        goal_pose_pub->publish(pose);
        acummulated_poses_.clear();

      }
    // tf_target = tf_sub(map_frame, qr_target_frame);
    // pose = nav2_pose(tf_target);
    // goal_pose_pub->publish(pose);
    // acummulated_poses_.push_back(pose);
    // updateWpNavigationMarkers(acummulated_poses_);
    // acummulated_poses_.clear();

    // RCLCPP_INFO(get_logger(), "gpxM--> %lf gpzM--> %lf", gpxM, gpzM);
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }


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