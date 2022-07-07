#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

//this program will have stretch drive forward until it sees an object
//if the object gets closer, stretch will back up

//I have a step ladder. I never met my real ladder ;(

//global publisher to be used in main and callback
ros::Publisher pub;

//variables for parameters
std::string cmd_vel_output;
std::string laser_scan;
float forward_speed;
float backward_speed;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //variable to be given values to be published
    geometry_msgs::Twist motorizer;

    //creates an integer that is the total number of indices of the lidar array
    int size = scan->ranges.size();

    //scan distance at specific indices for directly in front of and behind the robot
    float front = scan->ranges[1360];
    float back = scan->ranges[300];

    //variables to set up the switch statement
    int f = 0;
    int b = 0;

    //conditions to set up switch statement
    if(front < 1.1)
    {
        f = 1;
    }
    if(back < 1.1)
    {
        b = 2;
    }

    int tracker = f + b;

    switch(tracker)
    {
        //there is nothing behind or in front of the robot
        case 0:
            motorizer.linear.x = forward_speed;
            break;
        
        //there is a wall in front of the robot
        case 1:
            motorizer.linear.x = backward_speed;
            break;
        
        //there is a wall behind the robot
        case 2:
            motorizer.linear.x = forward_speed;
            break;
        
        //there are walls behind and in front of the robot
        case 3:
            motorizer.linear.x = 0.0;
            break;
    }

    pub.publish(motorizer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_avoidance_node");
    ros::NodeHandle n;

    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, callback);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        n.param<float>("forward_speed", forward_speed, 0.5);
        n.param<float>("backward_speed", backward_speed, -0.5);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}