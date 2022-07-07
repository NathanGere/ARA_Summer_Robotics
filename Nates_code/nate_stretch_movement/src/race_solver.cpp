#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

ros::Publisher pub;
std::string cmd_vel_output;
std::string laser_scan;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    geometry_msgs::Twist motorizer;
    int size = scan->ranges.size();

    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];

    motorizer.linear.x=3.0;
    if(right <= 0.9 && left <= 0.9)
    {
        motorizer.linear.x=0.5;
        if(right < left && right + 0.3 > left)
        {
            motorizer.angular.z=1.0;
            motorizer.linear.x=1.0;
        }
        if(right < left)
        {
            motorizer.angular.z=0.5;
        }
        if(left < right)
        {
            motorizer.angular.z=-0.5;
        }
    }
    if(center <= 2.0)
    {
        motorizer.linear.x=0.2;
        if(right < left && right + 0.4 > left)
        {
            motorizer.linear.x = 1.0;
        }
        else if(right < left)
        {
            motorizer.angular.z=1.0;
        }
        else if(left < right)
        {
            motorizer.angular.z=-1.0;
        }
    }
    else if(left <= 0.9 && right > left)
    {
        motorizer.angular.z=-0.5;
    }
    else if(right <= 0.9 && left > right)
    {
        motorizer.angular.z=0.5;
    
    }
    else if(right <= 0.9)
    {
        motorizer.angular.z=0.5;
    }
    else if(left <= 0.9)
    {
        motorizer.angular.z=-0.5;
    }
    if(center < 0.5)
    {
        motorizer.linear.x = -2.0;
    }

    pub.publish(motorizer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_solving_node");
    ros::NodeHandle n;

    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, callback);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}