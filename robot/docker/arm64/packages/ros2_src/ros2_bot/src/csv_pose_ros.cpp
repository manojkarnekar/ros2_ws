#include <iostream>
#include<fstream>
#include <unistd.h>
#include <cstdlib>
#include <bits/stdc++.h>
#include <time.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std;

char* csv_path;

void read(auto pub_cv_pose)
{
  ifstream fin;
  string line;
  rclcpp::Rate loop_rate(10);

  while(rclcpp::ok())
  {
      try 
      {
          fin.open(csv_path);
          fin>>line;
          string str = line;

          stringstream ss(str);
          vector<std::string> v;
          
          while (ss.good()) {
              string substr;
              getline(ss, substr, ',');
              v.push_back(substr);
          }

          std_msgs::msg::Float32MultiArray cv_pose;
          cv_pose.data.clear();

          for(long unsigned int i=0; i<v.size();i++)
          {
            cv_pose.data.push_back(stof(v.at(i)));
          }

          pub_cv_pose->publish(cv_pose);

          std::cout << "Mid X = " << std::stof(v[2]) << " Mid Y = " << std::stof(v[3]) << " Left X = " << std::stof(v[0]) << " Left Y = " << std::stof(v[1]) << " Right X = " << std::stof(v[4]) << " Right Y = " << std::stof(v[5]) << " Bed Left Half = " << std::stof(v[6]) << " Bed Distance = " << std::stof(v[7]) << " Bed Right Half = " << std::stof(v[8]) << " Bed Y Coordinate = " << std::stof(v[9]) << std::endl;
          // cout<<"----------------------------------------------"<<std::endl;
          // sleep(0.1);
          loop_rate.sleep();
          fin.close();
          v.clear();
          
          
      }
      catch (...) {
          continue;
      }
      
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("csv_pose_ros");
  auto pub_cv_pose = node->create_publisher<std_msgs::msg::Float32MultiArray>("cv_pose", 10);

  csv_path = argv[1];

  // auto m_pub_x_pose = node->create_publisher<std_msgs::msg::Float32>("x_pose", 10);
  // auto m_pub_y_pose = node->create_publisher<std_msgs::msg::Float32>("y_pose", 10);
  read(pub_cv_pose);

  rclcpp::spin(node);
  return 0;
}