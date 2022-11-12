#include "follow_path.cpp" 

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public path_fol
{
  public:
   // int p, h, phi;

};


void path_fol::nav_path()
{
  // std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
  //acummulated_poses_.clear();

//   try
//   {
    //rclcpp::Time now = this->get_clock()->now();

    vector<vector<string>> content = read_csv("/home/manoj/ros2_ws/src/manoj_bot/src/simulation.csv");
    //vector<double> d1 = tf_subscriber();

    int l = content.size();
    double q_x1, q_y1, q_z1, q_w1, r_3, p_3, y_3, px, py, p_x, p_y, qz_1, qw_1;
    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
    for(int i=0; i<l-1; i++)
    {
    px = stof(content[i+1][2]);            // waypoints in alphasense origin frame
    py = stof(content[i+1][3]);
    p_x = px*cos(y_2) - py*sin(y_2); //+ d1[0];
    p_y = px*sin(y_2) + py*cos(y_2); //+ d1[1];           // waypoints in map frame
    
        
    q_x1 = stof(content[i+1][5]);
    q_y1 = stof(content[i+1][6]);
    q_z1 = stof(content[i+1][7]);               // waypoints orientation in quaternion
    q_w1 = stof(content[i+1][8]);

    tf2::Quaternion q2(q_x1, q_y1, q_z1, q_w1);
    tf2::Matrix3x3 m2(q2);
    m2.getRPY(r_3, p_3, y_3);
  
    tf2::Quaternion q1;
    q1.setRPY(0, 0,  y_3);        //d1[2] +
    //qx_1 = q1.x();                                
    //qy_1 = q1.y();
    qz_1 = q1.z();
    qw_1 = q1.w();
    cout<<" px = "<<px<<" py = "<<py<<endl;
    // cout<<" p_x = "<<p_x<<" p_y = "<<p_y<<endl;

    acummulated_poses_.push_back(nav2_pose(p_x, p_y, qz_1, qw_1));
    //cout<<"acummulated_poses_ = " << acummulated_poses_[0] <<endl;
    }

//    send_static_target_tf(now, laser_frame, bed_frame_l, (l_x + x_h) , (l_y + y_h), 0.0, 0.0, 0.0, th_m_h);
    cout<<" p_x = "<<p_x<<" p_y = "<<p_y<<endl;
    cout<<" p_xx "<<endl;
    updateWpNavigationMarkers(acummulated_poses_);
    startWaypointFollowing(acummulated_poses_);

    //startNavThroughPoses(acummulated_poses_);

    //acummulated_poses_.clear();
    
//   }
//   catch(const std::exception& e)
//   {
//     std::cerr << e.what() << '\n';
//   }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->nav_path();
  rclcpp::spin(node);

  //rclcpp::shutdown();
  return 0;
}
