#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_gugudan");
  ros::NodeHandle nh;

  ros::Publisher gugudan_pub = nh.advertise<std_msgs::String>("gugudan_result", 10);
  ros::Rate loop_rate(1); 

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    
    for (int i = 1; i <= 9; ++i)
    {
      for (int j = 1; j <= 9; ++j)
      {
        ss << i << " x " << j << " = " << i*j << "\n";
      }
    }
    
    msg.data = ss.str();

    gugudan_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
