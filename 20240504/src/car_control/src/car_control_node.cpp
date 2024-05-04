#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define SLAVE_ADDRESS 0x05
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
#define MAX_ANGLE 50	
#define MIN_ANGLE -50
#define MAX_SPEED 255
#define MIN_SPEED -255

unsigned char protocol_data[9] = {'#','C',0,0,0,0,0,0,'*'};

union SteeringUnion
{
    short steering_angle_data;
    char angle_byte[2];
} Steering;

union SpeedUnion
{
    int speed_data;
    char speed_byte[2];
} Car_Speed;

int file_I2C;

int open_I2C(void) 
{
    int file;
    const char *deviceName = "/dev/i2c-0";

    if ((file = open(deviceName, O_RDWR)) < 0) 
    {
        ROS_ERROR("%s에 접근 실패\n", deviceName);
        exit(1);
    }
    ROS_INFO("I2C: 연결됨\n");
    ROS_INFO("I2C: 0x%x에 대한 버스 획득\n", SLAVE_ADDRESS);
    if (ioctl(file, I2C_SLAVE, SLAVE_ADDRESS) < 0) 
    {
        ROS_ERROR("I2C: 버스 접근/스레이브 0x%x에 대한 통신 실패\n", SLAVE_ADDRESS);
        exit(1);
    }

    return file;
}

void close_I2C(int fd)
{
   close(fd);
}

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double linear_data = msg->linear.x ;
    double angular_data = msg->angular.z ;

    if(linear_data >=  MAX_SPEED)
        linear_data = MAX_SPEED;
    if(linear_data <=  MIN_SPEED)
        linear_data = MIN_SPEED;

    Car_Speed.speed_data = (int)linear_data;
    
    if(angular_data >= MAX_ANGLE)  
        angular_data = MAX_ANGLE;
    if(angular_data <= MIN_ANGLE)
        angular_data = MIN_ANGLE;
	  
    Steering.steering_angle_data = (short)angular_data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_control");
    ros::NodeHandle nh;

    file_I2C = open_I2C();
    if(file_I2C < 0)
        return -1;

    ros::Subscriber car_control_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_Callback);
    ros::Rate loop_rate(30.0);  

    while (ros::ok())
    {
        protocol_data[0] = '#';
        protocol_data[1] = 'C';
        protocol_data[2] = Steering.angle_byte[0];
        protocol_data[3] = Steering.angle_byte[1];
        protocol_data[4] = Car_Speed.speed_byte[0];
        protocol_data[5] = Car_Speed.speed_byte[1];
        protocol_data[6] = 0;  
        protocol_data[7] = 0;    
        protocol_data[8] = '*';
        
        write(file_I2C, protocol_data, 9);
        
        ROS_INFO("Steering Angle: %d\n", Steering.steering_angle_data);
        ROS_INFO("Car Speed: %d\n", Car_Speed.speed_data);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
