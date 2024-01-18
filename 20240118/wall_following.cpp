#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;
float wall_error_old = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  printf("front_range: [%f]\n", front_sonar);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  left_sonar = msg->range;
  printf("left_range:  [%f]\n", left_sonar);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  right_sonar = msg->range;
  printf("right_range: [%f]\n\n", right_sonar);
}

geometry_msgs::Twist wall_following(float Kp, float Ki, float Kd)
{
  geometry_msgs::Twist cmd_vel;

  float wall_error = left_sonar - right_sonar;
  float wall_error_d = wall_error - wall_error_old;  
  static float wall_error_sum = 0.0;  // Make it static to keep its value between calls
  wall_error_sum += wall_error;

  float steering_control = Kp * wall_error + Kd * wall_error_d + wall_error_sum * Ki;
    
  cmd_vel.linear.x = 0.5;
  cmd_vel.angular.z = steering_control;

  wall_error_old = wall_error;

  return cmd_vel;
}

int main(int argc, char **argv)
{
  int count = 0;
  
  ros::init(argc, argv, "wall_following");
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
  ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
  ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);
  
  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  float Kp = 0.4;
  float Ki = 0.0;
  float Kd = 0.8;

  ros::Rate loop_rate(30.0);  
  
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel = wall_following(Kp, Ki, Kd);

    if (front_sonar <= 1.0)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    sonar_cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
