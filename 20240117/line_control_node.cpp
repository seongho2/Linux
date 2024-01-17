#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define TSL1401CL_SIZE 320
  
#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

float error_old = 0.0;

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
	float THRESHOLD = 0.01;
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 255;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}

void Tsl1401clCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	float THRESHOLD = 0.01;
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

int find_line_center()
{
    int centroid = 0;
    int mass_sum = 0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum > 0) 
    {
        centroid = centroid / mass_sum;
    }
    else 
    {
        centroid = 0;
    }
    
    return centroid;
}


void line_control(geometry_msgs::Twist &cmd_vel)
{
	float Kp = 0.002;
	float Ki = 0.0;
	float Kd = 0.02;

	float error = 0.0;
	float error_d = 0.0;
	
	float Steering_Angle = 0.0;

	float Line_Center = TSL1401CL_SIZE / 2;
	float OFFSET = 0.0;
    error = Line_Center - find_line_center() + OFFSET;
    error_d = error - error_old;
    Steering_Angle = Kp * error + Kd * error_d + Ki * error;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = Steering_Angle;

    error_old = error; 
}

void printThresholdData() {
    printf("Threshold Data: \n");
    for (int i = 0; i < TSL1401CL_SIZE; i++) {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");
}

void printLineCentroid() {
    double centroid = find_line_center();
    printf("Line Centroid: %f\n", centroid);
}

void print_LineControl(geometry_msgs::Twist &cmd_vel) 
{
    printThresholdData();
    printLineCentroid();
    line_control(cmd_vel);
}

int main(int argc, char **argv)
{
    int count = 0;
	
    geometry_msgs::Twist cmd_vel;

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, Tsl1401clCallback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    while (ros::ok()) 
    {
        print_LineControl(cmd_vel);
        
		bool stop = true;

		for (int i = 0; i < TSL1401CL_SIZE; i++)
		{
		if (LineSensor_threshold_Data[i] != 0)
		{
				stop = false;
				break;
			}
		}

		if (stop)
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
		}	


        pub_cmd_vel.publish(cmd_vel);
    
		
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
