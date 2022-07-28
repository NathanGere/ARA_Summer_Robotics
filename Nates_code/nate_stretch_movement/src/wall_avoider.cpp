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

    //reset variables
    wall_on_left = false;
    wall_on_right = false;
    wall_in_front = false;
    crashed = false;
    
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

    i = 1189;
    while(i < 1550)
    {
        if(scan->ranges[i] < 0.55) wall_in_front = true;
        i++;
    }

    i = 1825;
    while(i < 1977)
    {
        if(scan->ranges[i] < 0.55) wall_on_left = true;
        i++;
    }

    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance
    i = 0;
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
        if((i > 0 && i < 48) && scan->ranges[i] < 0.25)
        {
            crashed = true;
        }
        else if((i > 47 && i < 98) && scan->ranges[i] < 0.3)
        {
            crashed = true;
        }
        else if((i > 47 && i < 583) && scan->ranges[i] < 0.3)
        {
            crashed = true;
        }
        else if((i > 582 && i < 913) && scan->ranges[i] < 0.24)
        {
            crashed = true;
        }
        else if((i > 912 && i < 1803) && scan->ranges[i] < 0.2)
        {
            crashed = true;
        }
        else if((i > 1802 && i < 2000) && scan->ranges[i] < 0.25)
        {
            crashed = true;
        }
        i++;
    }

    if(crashed)
    {   
        int back = 0;
        int right = 0;
        ROS_INFO("Recovering crash . . . ");
        if((1890 <= index_mem && index_mem <= 2000)||(0 <= index_mem && index_mem <= 820))
        {
            back = 1;
        }
        //will give right a value of 2 if it is right
        if(300 <= index_mem && index_mem <= 1361)
        {
            right = 2;
        }

        int total = back + right;
        switch(total)
        {   
            //crash is front left
            case 0:
                motorizer.linear.x = -1.0;
                motorizer.angular.z = -1.0;
                break;
            //crash is back left
            case 1:
                motorizer.linear.x = 1.0;
                motorizer.angular.z = -1.0;
                break;
            //crash is front right
            case 2:
                motorizer.linear.x = -1.0;
                motorizer.angular.z = 1.0;
                break;
            //crash is back right
            case 3:
                motorizer.linear.x = 1.0;
                motorizer.angular.z = 1.0;
                break;
        }

        //move according to conditions for a bit
        ros::Rate recovery_time(10);
        pub.publish(motorizer);
        recovery_time.sleep();

        //stopping robot before it crashes again
        motorizer.linear.x = 0;
        motorizer.angular.z = 0;
        pub.publish(motorizer);
    }
    else
    {
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
                motorizer.linear.x = 0.5;
                motorizer.angular.z = -0.77;
                break;
            case 1: //wall on left
                ROS_INFO("case 1");
                motorizer.linear.x = 0.5;
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
                motorizer.linear.x = 0.5;
                break;
            case 5: //wall on left and right
                ROS_INFO("case 5");
                motorizer.linear.x = 0.5;
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
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "wall_avoider_node");
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