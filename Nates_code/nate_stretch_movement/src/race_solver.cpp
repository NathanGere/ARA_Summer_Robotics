#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

//this program will have stretch drive in circles and hopefully not crash

//A man drops his scrabble game in the road. What's the word on the street?

//global publisher
ros::Publisher pub;

//setting up parameters
std::string cmd_vel_output;
std::string laser_scan;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //variable motorizer will be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];
    
    //default speed for robot if no conditions below are met
    motorizer.linear.x=3.0;

    /*
    First condition is met if the robot is close to walls on both sides
    It will proceed slightly towards the side on which it has more room, until both sides are about equal
    */
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
    /*
    Second condition is met if there is a wall in front of the robot
    It will angle to whichever side has more room
    If both sides are approxiamately equal, it will prefer the right side
    */
    if(center <= 2.0)
    {
        motorizer.linear.x=0.2;
        if(right < left && right + 0.4 > left)
        {
            motorizer.linear.x = 0.5;
            motorizer.angular.z=-1.0;
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
    /*
    The following else ifs just check for other collisions that could occur and adjust accordingly
    */
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
    /*
    Emergency condition that overides the others and backs the robot up if it hits a wall
    */
    if(center < 0.5)
    {
        motorizer.linear.x = -2.0;
    }

    //movement occurs baby :)
    pub.publish(motorizer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_solving_node");
    ros::NodeHandle n;

    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
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