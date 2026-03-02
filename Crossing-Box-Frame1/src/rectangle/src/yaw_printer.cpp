#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.orientation, q);          // geometry_msgs ¡ú tf2

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);      // ËÄÔªÊý ¡ú RPY

  ROS_INFO("Current yaw = %.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yaw_printer");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/mavros/local_position/pose", 10, poseCB);
  ros::spin();
  return 0;
}