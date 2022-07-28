#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//global variable for tracking if the robot needs to explore
bool lost;
//global variable for causing snake search pattern
int reset;

//params
std::string laser_scan;
std::string cmd_vel_output;


void explorer(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //iteration variable
    int i = 0;
    //tracker for nearby walls
    bool walls_nearby = false;
    //will loop to detect if any walls are within 10 meters on the scan values
    while (i < size)
    {
        if(scan->ranges[i] < 10)
        {
            walls_nearby = true;
        }
        i++;
    }

    //will stop lost method from running again and cause initial setup to occur if walls are nearby
    if(walls_nearby)
    {   
        ROS_INFO("Wall located.");
        lost = false;
    }
    else
    {
        lost = true;
    }

    if(lost)
    {
        if(reset < 50)
        {
            motorizer.linear.x = 1.5;
            motorizer.angular.z = 0.1;
            pub.publish(motorizer);
            ROS_INFO("Exploring . . .");
        }
        else if(reset < 100)
        {
            //publishing forward speed
            motorizer.linear.x = 1.5;
            motorizer.angular.z = -0.3;
            pub.publish(motorizer);
            ROS_INFO("Exploring . . .");
        }
        else
        {
            reset = 0 - 1;
        }
    }

    reset = reset + 1; //to track how many times the cb has been called in a row
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //will assume robot is lost at start
    lost = true;
    //will assume robot has not yet turned at start
    reset = 0;

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, explorer);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
