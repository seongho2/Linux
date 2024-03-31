#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

double roll, pitch, yaw;
double yaw_offset_degree;

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void yaw_offset_degree_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    yaw_offset_degree = msg->data;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "imu_correction");
    ros::NodeHandle n;

    std::string imu_topic = "/imu";
    std::string yaw_offset_degree_topic = "/yaw_offset_degree";
    std::string yaw_d_corrected_topic = "/yaw_d_corrected";

    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~yaw_offset_degree_topic", yaw_offset_degree_topic);
    ros::param::get("~yaw_d_corrected_topic", yaw_d_corrected_topic);

    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1, imu1Callback);
    ros::Subscriber sub_yaw_offset_degree = n.subscribe(yaw_offset_degree_topic, 1, yaw_offset_degree_Callback);
    ros::Publisher pub_yaw_d_corrected = n.advertise<std_msgs::Float32>(yaw_d_corrected_topic, 1);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        ros::spinOnce();

        double yaw_degree = RAD2DEG(yaw);
        double yaw_d_corrected_value = yaw_degree + yaw_offset_degree;

        std_msgs::Float32 yaw_d_corrected;
        yaw_d_corrected.data = yaw_d_corrected_value;

        printf("yaw_d_corrected: %lf\n", yaw_d_corrected_value);

        pub_yaw_d_corrected.publish(yaw_d_corrected);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}

