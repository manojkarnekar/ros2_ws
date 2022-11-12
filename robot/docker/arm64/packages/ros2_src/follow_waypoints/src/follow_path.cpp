#include "path.hpp"

class path_fol : public rclcpp::Node
{
  public:
    explicit path_fol() : Node("path_fol")
    {
      wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());
      waypoints_client_ptr_  = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
      target_tf = std::make_unique<tf2_ros::TransformBroadcaster>(this);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      alpha_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ros_pose", 10, std::bind(&path_fol::pose_callback, this, _1));
    }

  public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    geometry_msgs::msg::PoseStamped nav2_pose(double tx_2, double ty_2, double qx_2, double qy_2);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void resetUniqueId();
    int getUniqueId();
    int unique_id {0};

    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoints_client_ptr_;

    void sendGoal(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void Nav2_path_fn();
    void feedbackCallback(GoalHandleFollowWaypoints::SharedPtr,const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void resultCallback(const GoalHandleFollowWaypoints::WrappedResult & result);
    std::unique_ptr<tf2_ros::TransformBroadcaster> target_tf;
    void send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double t_x, double t_y, double q_z, double q_w);
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    geometry_msgs::msg::TransformStamped tf_sub(std::string parent_frame, std::string child_frame);
    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr alpha_pose;
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void alpha_origin(double d_tx, double d_ty, double d_qz, double d_qw);
    // target_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    double qx, qy, qz, qw, d_x, d_y, d_z, r_2, p_2, y_2, r_3, p_3, y_3;
    double qx_1, qy_1, qz_1, qw_1, px, py, p_x, p_y;
    
    std::string map_frame ="map";
    std::string alpha_frame = "alpha_frame";
    geometry_msgs::msg::TransformStamped tf_Stamped;
    vector<vector<string>> read_csv(string fname);
};

void path_fol::send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double x, double y, double q_z, double q_w)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = 0.0;

  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = q_z;
  tf_msg.transform.rotation.w = q_w;

  target_tf->sendTransform(tf_msg);
}

// geometry_msgs::msg::TransformStamped path_fol::tf_sub(std::string parent_frame, std::string child_frame)
// {
//   geometry_msgs::msg::TransformStamped transformStamped;
//   transformStamped = tf_buffer_->lookupTransform(
//     parent_frame.c_str(), child_frame.c_str(),
//     tf2::TimePointZero);

//     return transformStamped;
// }

// vector<vector<string>> path_fol::read_csv(string fname)
// {
// 	vector<vector<string>> content;
// 	vector<string> row;
// 	string line, word;
 
// 	fstream file (fname, ios::in);
// 	if(file.is_open())
// 	{
// 		while(getline(file, line))
// 		{
// 			row.clear();
 
// 			stringstream str(line);
 
// 			while(getline(str, word, ','))
// 				row.push_back(word);
// 			content.push_back(row);
// 		}
// 	}
// 	else
//   {
//     cout<<"Could not open the file\n";
//   }
//   return content;
// }

geometry_msgs::msg::PoseStamped path_fol::nav2_pose(double tx_2, double ty_2, double qx_2, double qy_2)
{   
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = tx_2;
    goal_pose.pose.position.y = ty_2;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = qx_2;
    goal_pose.pose.orientation.w = qy_2;
    return goal_pose;
}

void path_fol::alpha_origin(double d_tx, double d_ty, double d_qz, double d_qw)
  {   try
     {
      rclcpp::Time now = this->get_clock()->now();
      send_target_tf(now, "map", "alpha_frame", d_tx, d_ty, d_qz, d_qw);
    }
    catch (tf2::TransformException & ex) 
    {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", 
      "map", "alpha_frame", ex.what());
     }
}

void path_fol::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    double tx, ty, tz, r, p, y, y_1, t_x, t_y, t_z, q_z, q_w;

    tx = msg->pose.position.x;
    ty = msg->pose.position.y;
    tz = msg->pose.position.z;

    tf2::Quaternion q(
              msg->pose.orientation.x,
              msg->pose.orientation.y,
              msg->pose.orientation.z,
              msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    y_1 = y;
  
    if(y<0){
      y = 2*3.14159 - fabs(y);
    }

    tx = -tx;
    ty = -ty;

    t_x = tx*cos(y) + ty*sin(y);
    t_y = -tx*sin(y) + ty*cos(y);
    t_z = tz;
    cout<<"tx = "<<tx << " ty = " << ty <<endl;
    cout<<t_x << " " << t_y << " " << t_z <<endl;
  
  
    tf2::Quaternion q2;
    q2.setRPY(r, p, -y_1);
    // q_x = q2.x();
    // q_y = q2.y();
    q_z = q2.z();
    q_w = q2.w();

    alpha_origin(t_x, t_y, q_z, q_w);
}


// void path_fol::Nav2_path_fn()
// {
//     vector<vector<string>> content = read_csv("");


//     int l = int(content.size()/12);
//     double t_x1, t_y1, q_x1, q_y1, q_z1, q_w1;
//     std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
//     for(int i=0; i<l; i++)
//     {
//     geometry_msgs::msg::TransformStamped trans = tf_sub("map", "alpha_frame");

//     t_x1 = trans.transform.translation.x;
//     t_y1 = trans.transform.translation.y;

//     tf2::Quaternion q(
//               trans.transform.rotation.x,
//               trans.transform.rotation.y,
//               trans.transform.rotation.z,
//               trans.transform.rotation.w);
//     tf2::Matrix3x3 m(q);
//     m.getRPY(r_2, p_2, y_2);

//     px = stof(content[(20 + 10*i)][2]);            // waypoints in alphasense origin frame
//     py = stof(content[(20 + 10*i)][3]);
//     p_x = px*cos(y_2) - py*sin(y_2) + t_x1;
//     p_y = px*sin(y_2) + py*cos(y_2) + t_y1;           // waypoints in map frame
//     cout<<" px = "<<px<<" py = "<<py<<endl;
//     cout<<" p_x = "<<p_x<<" p_y = "<<p_y<<endl;
        
//     q_x1 = stof(content[(20 + 10*i)][5]);
//     q_y1 = stof(content[(20 + 10*i)][6]);
//     q_z1 = stof(content[(20 + 10*i)][7]);               // waypoints orientation in quaternion
//     q_w1 = stof(content[(20 + 10*i)][8]);

//     tf2::Quaternion q2(q_x1, q_y1, q_z1, q_w1);
//     tf2::Matrix3x3 m2(q2);
//     m2.getRPY(r_3, p_3, y_3);
  
//     tf2::Quaternion q1;
//     q1.setRPY(0, 0, y_2 + y_3);
//     qx_1 = q1.x();                                
//     qy_1 = q1.y();
//     qz_1 = q1.z();
//     qw_1 = q1.w();

//     acummulated_poses_.push_back(nav2_pose(p_x, p_y, qz_1, qw_1));   
// }
//     sendGoal(acummulated_poses_);
// }

// void path_fol::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_)
// {
//     resetUniqueId();
//     auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
//     for (size_t i = 0; i < acummulated_poses_.size(); i++)
//     {
//         visualization_msgs::msg::Marker arrow_marker;
//         arrow_marker.header = acummulated_poses_[i].header;
//         arrow_marker.id = getUniqueId();
//         arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
//         arrow_marker.action = visualization_msgs::msg::Marker::ADD;
//         arrow_marker.pose = acummulated_poses_[i].pose;
//         arrow_marker.scale.x = 0.3;
//         arrow_marker.scale.y = 0.05;
//         arrow_marker.scale.z = 0.02;
//         arrow_marker.color.r = 0;
//         arrow_marker.color.g = 255;
//         arrow_marker.color.b = 0;
//         arrow_marker.color.a = 1.0f;
//         arrow_marker.lifetime = rclcpp::Duration(0s);
//         arrow_marker.frame_locked = false;
//         marker_array->markers.push_back(arrow_marker);

//         // Draw a red circle at the waypoint pose
//         visualization_msgs::msg::Marker circle_marker;
//         circle_marker.header = acummulated_poses_[i].header;
//         circle_marker.id = getUniqueId();
//         circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
//         circle_marker.action = visualization_msgs::msg::Marker::ADD;
//         circle_marker.pose = acummulated_poses_[i].pose;
//         circle_marker.scale.x = 0.05;
//         circle_marker.scale.y = 0.05;
//         circle_marker.scale.z = 0.05;
//         circle_marker.color.r = 255;
//         circle_marker.color.g = 0;
//         circle_marker.color.b = 0;
//         circle_marker.color.a = 1.0f;
//         circle_marker.lifetime = rclcpp::Duration(0s);
//         circle_marker.frame_locked = false;
//         marker_array->markers.push_back(circle_marker);

//         // Draw the waypoint number
//         visualization_msgs::msg::Marker marker_text;
//         marker_text.header = acummulated_poses_[i].header;
//         marker_text.id = getUniqueId();
//         marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker_text.action = visualization_msgs::msg::Marker::ADD;
//         marker_text.pose = acummulated_poses_[i].pose;
//         marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
//         marker_text.scale.x = 0.07;
//         marker_text.scale.y = 0.07;
//         marker_text.scale.z = 0.07;
//         marker_text.color.r = 0;
//         marker_text.color.g = 255;
//         marker_text.color.b = 0;
//         marker_text.color.a = 1.0f;
//         marker_text.lifetime = rclcpp::Duration(0s);
//         marker_text.frame_locked = false;
//         marker_text.text = "wp_" + std::to_string(i + 1);
//         marker_array->markers.push_back(marker_text);
//     }

//     if (marker_array->markers.empty()) 
//     {
//     visualization_msgs::msg::Marker clear_all_marker;
//     clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array->markers.push_back(clear_all_marker);
//     }

//     wp_navigation_markers_pub_->publish(std::move(marker_array));
//     // acummulated_poses_.clear();
// }

// void path_fol::resetUniqueId()
// {
//   unique_id = 0;
// }

// int path_fol::getUniqueId()
// {
//   int temp_id = unique_id;
//   unique_id += 1;
//   return temp_id;
// }

// // send gaol
// void path_fol::sendGoal(std::vector<geometry_msgs::msg::PoseStamped> poses_) 
// {
//     while (!waypoints_client_ptr_->wait_for_action_server()) 
//     {
//       RCLCPP_INFO(get_logger(), "Waiting for action server...");
//     }
    
//     auto goal_msg = FollowWaypoints::Goal();
//     goal_msg.poses = poses_;
    

//     auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
//     send_goal_options.feedback_callback = std::bind(&path_fol::feedbackCallback, this, _1, _2);
//     send_goal_options.result_callback = std::bind(&path_fol::resultCallback, this, _1);

//     waypoints_client_ptr_->async_send_goal(goal_msg, send_goal_options);
//     poses_.clear();
// }

// //feedback
// void path_fol::feedbackCallback(GoalHandleFollowWaypoints::SharedPtr,const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
// {
//   RCLCPP_INFO(get_logger(), "current_waypoint = %d", feedback->current_waypoint);
// }

// //result
// void path_fol::resultCallback(const GoalHandleFollowWaypoints::WrappedResult & result)
// {
//   switch (result.code) 
//   {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       RCLCPP_INFO(get_logger(), "Success!!!");
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(get_logger(), "Goal was aborted");
//       return;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(get_logger(), "Goal was canceled");
//       return;
//     default:
//       RCLCPP_ERROR(get_logger(), "Unknown result code");
//       return;
//   }
// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_fol>();
  //node->sendGoal();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}