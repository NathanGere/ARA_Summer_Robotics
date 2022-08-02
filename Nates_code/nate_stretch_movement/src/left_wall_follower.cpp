#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//global pub and sub
ros::Publisher pub;
ros::Subscriber sub;

//global variables
bool lost; //is the robot lost?
bool crashed; //is the robot crashed?
bool oriented; //is robot facing a wall?
bool close_enough; //is the robot hugging a wall?
bool aligned; //is the robot parallel to a wall?
int index_mem; //index of laser scan of nearest wall
float distance_mem; //distance to nearest wall
//wall variables
bool wall_on_left; 
bool wall_on_right;
bool wall_in_front;
//variables to trigger new crash fixing attempts in certain scenarios
int case_5_tracker;
int case_7_tracker;
int case_10_tracker;
int case_11_tracker;
int case_13_tracker;
int case_14_tracker;
int case_15_tracker;
//var to reset explorer method
int reset;

//params
std::string cmd_vel_output;
std::string laser_scan;

//methods
//decides which method to call
void thinker(const sensor_msgs::LaserScan::ConstPtr &scan);
//will activate if the robot gets lost or in a crash
void explorer();
void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan, int indices_of_collisions[2000], int size);
//will get the robot in position to hug a wall on its right
void get_oriented(const sensor_msgs::LaserScan::ConstPtr &scan);
void get_closer(const sensor_msgs::LaserScan::ConstPtr &scan);
void get_aligned(const sensor_msgs::LaserScan::ConstPtr &scan);
//wall_follower
void left_wall_follower(const sensor_msgs::LaserScan::ConstPtr &scan);

int main(int argc, char** argv)
{
    //setting up node
    ros::init(argc, argv, "left_wall_follower_node");
    ros::NodeHandle n;
    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    //initial conditions for thinker method
    lost = false;
    crashed = false;
    oriented = false;
    close_enough = false;
    aligned = false;
    index_mem = 100;
    distance_mem = 5.0;

    //setting up subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, thinker);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
void thinker(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //loop iteration variable
    int i = 0;
    //tracker for nearby walls
    bool walls_nearby = false;
    crashed = false;
    wall_on_left = false;
    wall_on_right = false;
    wall_in_front = false;
    //will remeber locations of crashes to make best possible recovery assuming robot is still upright
    int indices_of_collisions[size];
    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance and set important conditions
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
        //will occur if no walls are close enough
        if(scan->ranges[i] < 10.0)
        {
            walls_nearby = true;
        }
        //crash measurements
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

    //will calculate which sides of the robot there are walls on
    /*
    i = 0;
    while(i < 47)
    {
        if(scan->ranges[i] < 0.5) wall_on_left = true;
        i++;
    }
    */

    i = 624; //' 624 or 824 to 877, 880 to 890
    while(i < 877)
    {   
        if(scan->ranges[i] < 0.5) wall_on_right = true;
        i++;
    }

    i = 1099;
    while(i < 1640)
    {
        if(scan->ranges[i] < 0.5) wall_in_front = true;
        i++;
    }

    i = 1967; //was 1825
    while(i < 1977)
    {
        if(scan->ranges[i] < 0.5) wall_on_left = true;
        i++;
    }

    //will activate explorer method in next loop of ros::spinOnce()
    if(!walls_nearby)
    {
        lost = true;
    }
    else
    {
        lost = false;
    }
    
    if(lost)
    {
        explorer();
    }
    else if(crashed)
    {
        crash_recovery(scan, indices_of_collisions, size);
    }
    else
    {
        if(!oriented)
        {
            get_oriented(scan);
        }
        else if(!close_enough)
        {
            get_closer(scan);
        }
        else if(!aligned)
        {
            get_aligned(scan);
        }
        else
        {
            left_wall_follower(scan);
        }
    }
}
//will have the robot move around until it finds a wall
void explorer()
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    if(reset < 51)
        {
            motorizer.linear.x = 1.5; //search left and forward
            motorizer.angular.z = 0.1;
            pub.publish(motorizer);
            ROS_INFO("Exploring . . .");
        }
        else if(reset < 101)
        {
            motorizer.linear.x = 1.5; //search right and forward
            motorizer.angular.z = -0.3;
            pub.publish(motorizer);
            ROS_INFO("Exploring . . .");
        }
        else
        {
            reset = 0; //cycle back to previous conditions
        }
        reset++;
}
//will recover robot if it is crashed but still upright
void crash_recovery(const sensor_msgs::LaserScanConstPtr &scan, int indices_of_collisions[2000], int size)
{
    ROS_INFO("Avoiding crash . . . ");

    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;
        
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
            ROS_INFO("case 1: Near collision back left");
            motorizer.angular.z = -0.5;
            motorizer.linear.x = 0.5;
            break;
        case 2: //Near collision back right
            ROS_INFO("case 2: Near collision back right");
            motorizer.angular.z = 0.5;
            motorizer.linear.x = 0.5;
            break;
        case 3: //Near collisions on back left and right
            ROS_INFO("case 3: Near collisions back left and right");
            motorizer.linear.x = 1.0;
            break;
        case 4: //Near collision front right
            ROS_INFO("case 4: Near collision front right");
            motorizer.angular.z = -0.5; 
            motorizer.linear.x = -0.5;
            break;
        case 5: //Near collision front right and back left
            ROS_INFO("case 5: Near collision front right and back left");
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
        case 6: //Near collisions front right and back right
            ROS_INFO("case 6: Near collisions back right and front right");
            motorizer.angular.z = 0.5;
            motorizer.linear.x = 0.5;
            break;
        case 7:
            ROS_INFO("case 7: Near collisions front right, back right, and back left");
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
            ROS_INFO("case 8: Near collision front left");
            motorizer.angular.z = 0.5; 
            motorizer.linear.x = -0.5;
            break;
        case 9:
            ROS_INFO("case 9: Near collisions front left and back left");
            motorizer.angular.z = -0.5;
            motorizer.linear.x = 0.5;
            break;
        case 10:
            ROS_INFO("case 10: Near collisions front left and back right");
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
            ROS_INFO("case 11: Near collisions front left, back left, and back right");
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
            ROS_INFO("case 12: Near collisions front left and front right");
            motorizer.linear.x = -0.5;
            break;
        case 13:
            ROS_INFO("case 13: Near collisions front left, back left, and front right");
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
            ROS_INFO("case 14: Near collisions front left, front right, and back right");
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
            ROS_INFO("case 15: Near collisions on multiple sides.");
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
//will get the robot to face the nearest wall
void get_oriented(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //stopping robot in the case that the lost method was previously called
    motorizer.linear.x = 0.0;
    pub.publish(motorizer);

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1361];
    float left = scan->ranges[1890];

    //iteration variable
    int i = 0;
    //will loop through to find the distance of the closest wall
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
    
        i++;
    }

    //var to track whether or not the front of the robot is facing the nearest wall
    bool facing = false;

    //if the front of the robot is aligned with the closest wall, the facing boolean will be set to true
    if(center > distance_mem - 0.02 && center < distance_mem + 0.02)
    {
        facing = true;
    }

    //if its not facing the wall, the robot will turn to its right
    if(!facing)
    {
        if(index_mem > 300 && index_mem < 1361)
        {
            motorizer.angular.z = -0.5;
            pub.publish(motorizer);
            ROS_INFO("Orienting . . ."); 
        }
        else
        {
            motorizer.angular.z = 0.5;
            pub.publish(motorizer);
            ROS_INFO("Orienting . . ."); 
        }
    }
    //if the robot is facing the wall, the orientation is complete
    else
    {
        oriented = true;
        ROS_INFO("INITIAL ORIENTATION COMPLETE.");
    }
}
//will move the robot towards the nearest wall until it is 0.5 meters away (assuming it is facing the nearest wall)
void get_closer(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //float to store distance in front of the object
    float front = scan->ranges[1361];

    //once the robot is close enough, it will proceed
    if(front > 0.5 && front < 0.6)
    {
        close_enough = true;
        ROS_INFO("PREFERED INITIAL DISTANCE ACHIEVED.");
    } 
    //if there is space, the robot will get closer to the wall
    else if(front > 0.4)
    {
        motorizer.linear.x = 0.5;
        pub.publish(motorizer);
        ROS_INFO("Establishing initial distance . . .");
    }
    //if the robot is too close, it will back up
    else
    {
        motorizer.linear.x = -0.5;
        pub.publish(motorizer);
        ROS_INFO("Establishing initial distance . . .");
    }
}
//will turn the robot to its left until its right side is parallel to the nearest wall
void get_aligned(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //float to store distance in front of the object
    float left = scan->ranges[1890];

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //iteration variable for while loop
    int i = 0;
    //will loop through to find the distance of the closest wall
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
    
        i++;
    }

    //if aligned, the global var will be set to true, and this function will no longer be called
    if(left < distance_mem + 0.02 && left > distance_mem - 0.02)
    {
        aligned = true;
        ROS_INFO("FIRST ALIGNMENT COMPLETE.");
    }
    //will turn the robot to its left if it is not aligned with the nearest wall
    else
    {
        motorizer.angular.z = -0.5;
        pub.publish(motorizer);
        ROS_INFO("Completing first alignment . . .");
    }
}
void left_wall_follower(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //geo msg to be published
    geometry_msgs::Twist motorizer;

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
            motorizer.angular.z = 1.0;
            break;
        case 1: //wall on left
            ROS_INFO("case 1");
            motorizer.linear.x = 0.5;
            break;
        case 2: //wall in front
            ROS_INFO("case 2");
            motorizer.angular.z = 0.5;
        case 3: //wall in front and on left
            ROS_INFO("case 3");
            motorizer.angular.z = -0.5;
            break;
        case 4: //wall on right
            ROS_INFO("case 4");
            motorizer.linear.x = 0.5;
            motorizer.angular.z = -0.65;
            break;
        case 5: //wall on left and right
            ROS_INFO("case 5");
            motorizer.linear.x = 0.5;
            break;
        case 6: //wall front and right
            ROS_INFO("case 6");
            motorizer.angular.z = -0.5;
            break;
        case 7: //walls front, left, and right
            ROS_INFO("case 7");
            motorizer.angular.z = -0.5;
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