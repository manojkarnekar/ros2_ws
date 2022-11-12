#include "nav2_goal_pose/nav2_goal_pose.h"

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