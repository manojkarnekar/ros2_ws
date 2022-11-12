#include <math.h>
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

class Nav2_path
{
public:
	Nav2_path();
    // Nav2_path_fn();
private:
    ros::NodeHandle n;
    ros::Publisher path_pub;
    void Nav2_path_fn();

    geometry_msgs::PoseStamped nav2_pose(float t_x, float t_y, float o_z, float o_w);
    void sendGoal(std::vector<geometry_msgs::PoseStamped> poses_);
    ros::Timer timer = n.createTimer(ros::Duration(1.0 / 50.0), std::bind(&Nav2_path::Nav2_path_fn, this));
};

Nav2_path::Nav2_path()
{
    ROS_INFO("Started nav2_path node");
    path_pub = n.advertise<nav_msgs::Path>("path", 50);
    // Nav2_path_fn();
}

void Nav2_path::Nav2_path_fn()
{
    std::vector<geometry_msgs::PoseStamped> acummulated_poses_;
    for(int i=0; i<30; i++)
    {
        if(i<15)
        {
            acummulated_poses_.push_back(nav2_pose(i, 0.0, 0.0, 0.0));   
        }

        if(i>15)
        {
            acummulated_poses_.push_back(nav2_pose(0.0, i, 0.0, 0.0));
        }
     
    }
    sendGoal(acummulated_poses_);

}

geometry_msgs::PoseStamped Nav2_path::nav2_pose(float t_x, float t_y, float o_z, float o_w)
{   
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = t_x;
    goal_pose.pose.position.y = t_y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = o_z;
    goal_pose.pose.orientation.w = o_w;
    return goal_pose;
}

void Nav2_path::sendGoal(std::vector<geometry_msgs::PoseStamped> poses_)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time();
    path.header.frame_id = "map";
    path.poses = poses_;
    path_pub.publish(path);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"Nav2_path");
	Nav2_path obj;
	ros::spin();
    return 0;
}