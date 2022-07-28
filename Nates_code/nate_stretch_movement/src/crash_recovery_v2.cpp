#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//global variable for tracking if the robot needs to explore
bool crashed;
int index_mem;
float distance_mem;
//variables to trigger new crash fixing attempts in certain scenarios
int case_5_tracker;
int case_7_tracker;
int case_10_tracker;
int case_11_tracker;
int case_13_tracker;
int case_14_tracker;
int case_15_tracker;

//params
std::string laser_scan;
std::string cmd_vel_output;

void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //loop iteration variable
    int i = 0;
    //tracker for nearby walls
    crashed = false;

    //will remeber locations of crashes to make best possible recovery assuming robot is still upright
    int indices_of_collisions[size];

    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance
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
            indices_of_collisions[i] = i;
        }
        else if((i > 47 && i < 98) && scan->ranges[i] < 0.3)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 47 && i < 583) && scan->ranges[i] < 0.3)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 582 && i < 913) && scan->ranges[i] < 0.24)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 912 && i < 1803) && scan->ranges[i] < 0.19)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else if((i > 1802 && i < 2000) && scan->ranges[i] < 0.25)
        {
            crashed = true;
            indices_of_collisions[i] = i;
        }
        else
        {
            indices_of_collisions[i] = 3000;
        }
        i++;
    }
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //will give back a value of 1 if it is back
    if(crashed)
    {   
        ROS_INFO("Recovering crash . . . ");
        
        //sides of crash
        int BL = 0;
        int BR = 0;
        int FR = 0;
        int FL = 0;

        int c = 0;
        while(c < size)
        {
            if((1890 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1999) || (0 <= indices_of_collisions[c] && indices_of_collisions[c] <= 300)) BL = 1;
            if((300 <= indices_of_collisions[c] && indices_of_collisions[c] <= 820)) BR = 2;
            if((820 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1361)) FR = 4;
            if((1361 <= indices_of_collisions[c] && indices_of_collisions[c] <= 1890)) FL = 8;
            c++;
        }

        int switch_setup = BL + BR + FR + FL;
        switch(switch_setup)
        {
            case 0: //no collisions
                ROS_INFO("case 0: No collisions detected");
                case_5_tracker = 0;
                break;
            case 1: //collison back left
                ROS_INFO("case 1: Collision back left");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 2: //collision back right
                ROS_INFO("case 2: Collision back right");
                motorizer.angular.z = 0.5;
                motorizer.linear.x = 0.5;
                break;
            case 3: //collisions on back left and right
                ROS_INFO("case 3: Collisions back left and right");
                motorizer.linear.x = 1.0;
                break;
            case 4: //collision front right
                ROS_INFO("case 4: Collision front right");
                motorizer.angular.z = -0.5; 
                motorizer.linear.x = -0.5;
                break;
            case 5: //collision front right and back left
                ROS_INFO("case 5: Collision front right and back left");
                if(case_5_tracker < 7)
                {
                    motorizer.angular.z = 0.5;
                    motorizer.linear.x = 0.1;
                    case_5_tracker++;
                }
                else if(case_5_tracker < 15)
                {
                    motorizer.angular.z = -0.5;
                    motorizer.linear.x = -0.1;
                    case_5_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_5_tracker = 0;
                }
                break;
            case 6: //collisions front right and back right
                ROS_INFO("case 6: Collisions back right and front right");
                motorizer.angular.z = 0.5;
                motorizer.linear.x = 0.5;
                break;
            case 7:
                ROS_INFO("case 7: Collisions front right, back right, and back left");
                if(case_7_tracker < 7)
                {
                    motorizer.angular.z = 0.5;
                    motorizer.linear.x = 0.1;
                    case_7_tracker++;
                }
                else if(case_7_tracker < 15)
                {
                    motorizer.angular.z = -0.5;
                    motorizer.linear.x = -0.1;
                    case_7_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_7_tracker = 0;
                }
                break;
            case 8:
                ROS_INFO("case 8: Collision front left");
                motorizer.angular.z = 0.5; 
                motorizer.linear.x = -0.5;
                break;
            case 9:
                ROS_INFO("case 9: Collisions front left and back left");
                motorizer.angular.z = -0.5;
                motorizer.linear.x = 0.5;
                break;
            case 10:
                ROS_INFO("case 10: Collisions front left and back right");
                if(case_10_tracker < 7)
                {
                    motorizer.angular.z = -0.5;
                    motorizer.linear.x = 0.1;
                    case_10_tracker++;
                }
                else if(case_7_tracker < 15)
                {
                    motorizer.angular.z = 0.5;
                    motorizer.linear.x = -0.1;
                    case_10_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_10_tracker = 0;
                }
                break;
            case 11:
                ROS_INFO("case 11: Collisions front left, back left, and back right");
                if(case_11_tracker < 7)
                {
                    motorizer.angular.z = -0.5;
                    motorizer.linear.x = 0.1;
                    case_11_tracker++;
                }
                else if(case_11_tracker < 15)
                {
                    motorizer.angular.z = 0.5;
                    motorizer.linear.x = -0.1;
                    case_11_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_11_tracker = 0;
                }
                break;
            case 12:
                ROS_INFO("case 12: Collisions front left and front right");
                motorizer.linear.x = -0.5;
                break;
            case 13:
                ROS_INFO("case 13: Collisions front left, back left, and front right");
                if(case_13_tracker < 7)
                {
                    motorizer.angular.z = 0.5; 
                    motorizer.linear.x = -0.1;
                    case_13_tracker++;
                }
                else if(case_13_tracker < 15)
                {
                    motorizer.angular.z = -0.5;
                    motorizer.linear.x = 0.1;
                    case_13_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_13_tracker = 0;
                }
                break;
            case 14:
                ROS_INFO("case 14: Collisions front left, front right, and back right");
                if(case_14_tracker < 7)
                {
                    motorizer.angular.z = -0.5; 
                    motorizer.linear.x = -0.1;
                    case_14_tracker++;
                }
                else if(case_14_tracker < 15)
                {
                    motorizer.angular.z = 0.5;
                    motorizer.linear.x = 0.1;
                    case_14_tracker++;
                }
                else
                {   
                    ROS_INFO("This crash may be unrecoverable.");
                    case_14_tracker = 0;
                }
                break;
            case 15:
                ROS_INFO("case 15: Collisions on multiple sides.");
                if(case_15_tracker < 4)
                {
                    motorizer.linear.x = 0.1;
                    case_15_tracker++;
                }
                else if(case_15_tracker < 8)
                {
                    motorizer.linear.x = -0.1;
                    case_15_tracker++;
                }
                else if(case_15_tracker < 20)
                {
                    motorizer.angular.z = -0.5;
                    case_15_tracker++;
                }
                else
                {
                    ROS_INFO("This crash may be unrecoverable.");
                    case_15_tracker = 0;
                }
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
        ROS_INFO("There is no crash detected.");
        case_5_tracker = 0;
        case_7_tracker = 0;
        case_10_tracker = 0;
        case_11_tracker = 0;
        case_13_tracker = 0;
        case_14_tracker = 0;
        case_15_tracker = 0;
    }
}
int main(int argc, char** argv)
{   
    //node setup
    ros::init(argc, argv, "crash_recovery_v2_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //will assume robot is lost at start
    crashed = true;
    case_5_tracker = 0;
    case_7_tracker = 0;
    case_10_tracker = 0;
    case_11_tracker = 0;
    case_13_tracker = 0;
    case_14_tracker = 0;
    case_15_tracker = 0;

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, crash_recovery);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}