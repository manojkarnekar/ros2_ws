#include "control/afs_control.h"

void AfsControl::update()
{

    steer_angle = K_steer * (atan2(bl*dr, dx));
    drive_rn = K_drive * (hypot(bl*dr, dx));

    if(steer_angle>1.45)
    {
        steer_angle = 1.45;
    }

    if(steer_angle<-1.45)
    {
        steer_angle = -1.45;
    }

    std_msgs::msg::Float32 steer_;
    std_msgs::msg::UInt8 steer_vel_;
	std_msgs::msg::UInt16 drive_;
    std_msgs::msg::UInt8 drive_dir;

    steer_.data = steer_angle;
    steer_vel_.data = 70;
	drive_.data = drive_rn;
    drive_dir.data = 0;

    pub_steer->publish(steer_);
    pub_steer_vel->publish(steer_vel_);
    pub_drive->publish(drive_);
    pub_drive_dir->publish(drive_dir);

}

void AfsControl::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ticks_since_target = 0;
	dx = msg->linear.x;
	dy = msg->linear.y;
	dr = msg->angular.z;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AfsControl>());
  rclcpp::shutdown();
  return 0;
}