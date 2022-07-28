#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//params
std::string laser_scan;
std::string cmd_vel_output;

//wall variables
bool wall_on_left;
bool wall_on_right;
bool wall_in_front;
bool crashed;
int index_mem;
float distance_mem;


void wall_avoider(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //geo msg to be published
    geometry_msgs::Twist motorizer;

    //set variables at start
    wall_on_left = false;
    wall_on_right = false;
    wall_in_front = false;

    //will calculate which sides of the robot there are walls on
    int i = 0;
    while(i < 47)
    {
        if(scan->ranges[i] < 0.55) wall_on_left = true;
        i++;
    }

    i = 824;
    while(i < 877)
    {
        if(scan->ranges[i] < 0.55) wall_on_right = true;
        i++;
    }

    i = 1139; //1189 at start
    while(i < 1600) //1550 at start
    {
        if(scan->ranges[i] < 0.55) wall_in_front = true;
        i++;
    }

    i = 1825; //1825 at start
    while(i < 1977)
    {
        if(scan->ranges[i] < 0.55) wall_on_left = true;
        i++;
    }

    //calculations for switch statement
    int l = 0;
    int f = 0;
    int r = 0;
    //setting up calculators
    if(wall_on_left) l = 1;
    if(wall_in_front) f = 2;
    if(wall_on_right) r = 4;
    //var for switch statment
    int wall_locations = l + f + r;

    switch(wall_locations)
    {
        case 0: //no walls
            ROS_INFO("case 0");
            motorizer.linear.x = 2.0;
            motorizer.angular.z = -0.77;
            break;
        case 1: //wall on left
            ROS_INFO("case 1");
            motorizer.linear.x = 2.0;
            motorizer.angular.z = -0.65;
            break;
        case 2: //wall in front
            ROS_INFO("case 2");
            motorizer.angular.z = 0.5;
            break;
        case 3: //wall in front and on left
            ROS_INFO("case 3");
            motorizer.angular.z = 0.5;
            break;
        case 4: //wall on right
            ROS_INFO("case 4");
            motorizer.linear.x = 2.0;
            break;
        case 5: //wall on left and right
            ROS_INFO("case 5");
            motorizer.linear.x = 2.0;
            break;
        case 6: //wall front and right
            ROS_INFO("case 6");
            motorizer.angular.z = 0.5;
            break;
        case 7: //walls front, left, and right
            ROS_INFO("case 7");
            motorizer.angular.z = 0.5;
            break;
    }
    if(scan->ranges[820] < 0.25) 
    {
        ROS_INFO("close on right");
        motorizer.linear.x = 0.3;
        motorizer.angular.z = 1.0;
    }
    if(scan->ranges[1890] < 0.25)
    {   
        ROS_INFO("close on left");
        motorizer.angular.z = -1.0;
        motorizer.linear.x = 0.3;
    } 
    pub.publish(motorizer);
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "wall_avoider_v3_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //variables start as false
    wall_on_left = false;
    wall_on_right = false;
    wall_in_front = false;
    crashed = false;

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, wall_avoider);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}