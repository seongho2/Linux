#include "ros/ros.h"
#include "std_msgs/String.h"

void gugudanCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("구구단 결과:\n%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_gugudan");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("gugudan_result", 10, gugudanCallback);

  ros::spin();

  return 0;
}
