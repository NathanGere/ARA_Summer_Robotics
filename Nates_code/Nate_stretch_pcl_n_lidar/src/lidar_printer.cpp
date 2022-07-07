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
    int i = 0;

    while (i < size)
    {
        if(scan->ranges[i] < 10)
        {
            ROS_INFO("Range at scan->ranges[%d]: %f", i, scan->ranges[i]);
        }
        i++;
    }
    
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_print_node");
    ros::NodeHandle n;

    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, callback);

    ros::Rate loop_rate(20);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}