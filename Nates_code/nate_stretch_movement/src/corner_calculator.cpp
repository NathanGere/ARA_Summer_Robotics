#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//params
std::string laser_scan;
std::string cmd_vel_output;

void corner_calculator(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //arrays to store values
    int indexes_of_corners[size];
    int type_of_corner[size];
    int indexes_of_detected_walls[size];
    float distances_of_detected_walls[size];
    int num_of_corners = 0;

    int i = 0;
    while(i < size) //will loop through scans to store indexes and distances of walls
    {
        if(scan->ranges[i] < 5.01)
        {
            indexes_of_detected_walls[i] = i;
            distances_of_detected_walls[i] = scan->ranges[i];
        }
        else
        {
            indexes_of_detected_walls[i] = 0;
            distances_of_detected_walls[i] = 5.01; //wall is at least 5 meters away
        }
        i++;
    }

    i = 6;
    while(i < size-6) //will loop through new arrays to find corners
    {   
        float five_avg_before = 0.0;
        float sum_before;
        float five_avg_after = 0.0;
        float sum_after;
        if(indexes_of_detected_walls[i-5] != 0 && indexes_of_detected_walls[i-4] != 0 && indexes_of_detected_walls[i-3] != 0 && indexes_of_detected_walls[i-2] != 0 && indexes_of_detected_walls[i-1] != 0)
        {
            sum_before = distances_of_detected_walls[i-5] + distances_of_detected_walls[i-4] + distances_of_detected_walls[i-3] + distances_of_detected_walls[i-2] + distances_of_detected_walls[i-1];
            five_avg_before = sum_before/5;
        }
        if(indexes_of_detected_walls[i+5] != 0 && indexes_of_detected_walls[i+4] != 0 && indexes_of_detected_walls[i+3] != 0 && indexes_of_detected_walls[i+2] != 0 && indexes_of_detected_walls[i+1] != 0)
        {
            sum_after = distances_of_detected_walls[i+5] + distances_of_detected_walls[i+4] + distances_of_detected_walls[i+3] + distances_of_detected_walls[i+2] + distances_of_detected_walls[i+1];
            five_avg_after = sum_after/5;
        }
        if(five_avg_before != 0.0 && five_avg_after != 0.0)
        {
            if(five_avg_before + 0.4 > five_avg_after && five_avg_before - 0.4 < five_avg_after)
            {
               type_of_corner[i] = 0; //not a corner
               indexes_of_corners[i] = 0;
            }
            else
            {
                num_of_corners++;
                type_of_corner[i] = 2;
                indexes_of_corners[i] = i;
            }
        }
        else
        {
            type_of_corner[i] = 1; //wall edge
        }
        i++;
    }

    int p = 1;
    while(p < size-1) //while loop will go through to print all the corners
    {   
        if(type_of_corner[p] == 1)
        {
            ROS_INFO("There is a wall edge at scan->ranges[%d]", p);
        }
        else if(type_of_corner[p] == 2)
        {
            ROS_INFO("There is a corner at scan->ranges[%d]", indexes_of_corners[p]);
        }
        p++;
    }
    ROS_INFO("The number of corners detected is %d", num_of_corners);
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "corner_calculator_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, corner_calculator);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
