#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

float yaw_error_old = 0.0;
double roll, pitch, yaw;
double heading_yaw = 45.0;

void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);      
    m.getRPY(roll, pitch, yaw);
    double change_yaw = RAD2DEG(yaw) + 360.0; 
    if (change_yaw >= 360.0) 
    {
        change_yaw -= 360.0;
    }
    printf("%f\n", change_yaw);
}

geometry_msgs::Twist yaw_control(float Kp, float Ki, float Kd)
{
    geometry_msgs::Twist cmd_vel;

    float yaw_error = 0.0;
    float yaw_error_d = 0.0;
    float yaw_error_sum = 0.0;
    float Steering_Angle = 0.0;

    yaw_error = heading_yaw - yaw;
    yaw_error_d = yaw_error - yaw_error_old;
    yaw_error_sum += yaw_error;

    Steering_Angle = Kp * yaw_error + Kd * yaw_error_d + Ki * yaw_error_sum;
    
    cmd_vel.linear.x = 1.0;
    cmd_vel.angular.z = Steering_Angle;

    if (yaw_error >= M_PI)  yaw_error = yaw_error - 2 * M_PI;
    if (yaw_error <= -M_PI)  yaw_error = yaw_error + 2 * M_PI;

    yaw_error_old = yaw_error; 

    return cmd_vel;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_control");

    ros::NodeHandle n;

    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Publisher yaw_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
    ros::Rate loop_rate(30.0);

    float Kp = 0.5;
    float Ki = 0.0;
    float Kd = 0.8;

    int count = 0;
    while (ros::ok())
    {  
        geometry_msgs::Twist cmd_vel = yaw_control(Kp, Ki, Kd);

        if (fabs(heading_yaw - yaw) > 0.01)
        {
            yaw_cmd_vel_pub.publish(cmd_vel);
        }
        else
        {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            yaw_cmd_vel_pub.publish(cmd_vel);
        }  

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
