#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//params
std::string laser_scan;
std::string cmd_vel_output;

void corner_finder(const sensor_msgs::LaserScan::ConstPtr &scan)
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

    int c = 1;
    while(c < size-1) //will loop through new arrays to find corners
    {   
        if(distances_of_detected_walls[c] != 5.01)
        {
            if(distances_of_detected_walls[c-1] != 0 && distances_of_detected_walls[c+1] != 0)
            {
                if(distances_of_detected_walls[c-1] + 0.01 < distances_of_detected_walls[c] && distances_of_detected_walls[c+1] + 0.01 < distances_of_detected_walls[c])
                {
                    indexes_of_corners[c] = c;
                    type_of_corner[c] = 1; //this will refer to an inside corner
                }
                else if(distances_of_detected_walls[c-1] - 0.01 > distances_of_detected_walls[c] && distances_of_detected_walls[c+1] - 0.01 > distances_of_detected_walls[c])
                {
                    indexes_of_corners[c] = c;
                    type_of_corner[c] = 2; //this will refer to an outside corner
                }
                else
                {
                    indexes_of_corners[c] = 0;
                    type_of_corner[c] = 0;
                }
            }
            else
            {
                indexes_of_corners[c] = 0;
                type_of_corner[c] = 0;
            }
        }
        else
        {
            indexes_of_corners[c] = 0;
            type_of_corner[c] = 0;
        }
        c++;
    }

    int p = 1;
    while(p < size-1) //while loop will go through to print all the corners
    {
        if(indexes_of_corners[p] != 0)
        {
            if(type_of_corner[p] == 1)
            {
                ROS_INFO("There is an inside corner at scan->ranges[%d]", indexes_of_corners[p]);
            }
            else if(type_of_corner[p] == 2)
            {
                ROS_INFO("There is an outside corner at scan->ranges[%d]", indexes_of_corners[p]);
            }
        }
        p++;
    }
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "corner_finder_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, corner_finder);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
