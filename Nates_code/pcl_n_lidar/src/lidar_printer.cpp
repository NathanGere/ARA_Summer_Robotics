#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

//This program can be used to figure out which indices face each direction on a robot
//Why did the chicken cross the playground? (go to bottom to see the answer)

//global publisher 
ros::Publisher pub;

//setting up param variables
std::string cmd_vel_output;
std::string laser_scan;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //var that equals the total number of indices in the array of laser scans
    int size = scan->ranges.size();

    //var for while loop. first index is at 0
    int i = 0;

    /*
    The following loop with iterate through the entire array of laser scans
    If a scan picks up an object within 10 meters, it will be printed
    */
    while (i < size)
    {
        
        ROS_INFO("Range at scan->ranges[%d]: %f", i, scan->ranges[i]);
    
        i++;
    }
    
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_print_node");
    ros::NodeHandle n;

    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    ros::Subscriber sub = n.subscribe(laser_scan, 1000, callback);

    ros::Rate loop_rate(20);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

//To get to the other slide! 