#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
//#include <Nate_stretch_pcl_n_lidar/laser_scan.h> //service that calls laser scan values

//global publisher, subscriber, and client
ros::Publisher pub;
ros::Subscriber sub;
//ros::ServiceClient client;

//parameters
std::string cmd_vel_output;
std::string laser_scan;

//global variables
//conditions
bool needs_setup; //tells main if it needs to recalculate conditions
bool aligned; //will track if the robot is aligned with a wall on its right
bool space_to_move; //will track if the robot can move forward
//trackers
int index_mem; //keeps record of index of nearest wall for scan values
float distance_mem; //keeps record of distance to nearest wall
int alignment_tracker; //keeps track of consecutive callings of aligner method

//will recalculate the global conditions and trackers when called
void var_set_up(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];

    //loop iteration variable
    int i = 0;
    index_mem = 100;
    distance_mem = 100.0;
    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance
    while (i < size)
    {
        if(scan->ranges[i] < scan->ranges[index_mem])
        {
            distance_mem = scan->ranges[i];
            index_mem = i;
        }
    
        i++;
    }

    //uses variables from loop to determine alignment
    if(scan->ranges[820] < distance_mem + 0.1 && scan->ranges[820] > distance_mem - 0.1)
    {
        aligned = true;
    }
    else
    {
        aligned = false;
    }

    //uses scan values to determine if there is space for the robot to move forward
    if(scan->ranges[1360] > 1.0)
    {
        space_to_move = true;
    }
    else
    {
        space_to_move = false;
    }

    //reset alignment_tracker
    alignment_tracker = 0;

    //variable setup complete
    needs_setup = false;
    ROS_INFO("Variables have been recalculated.");
}
//will align the robot so that the nearest wall is aligned with its right side
void aligner(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    geometry_msgs::Twist motorizer;
    if(scan->ranges[820] < distance_mem + 0.02 && scan->ranges[820] > distance_mem - 0.02)
    {
        aligned = true; //since the right index is aligned with the nearest wall, this condition will change
        ROS_INFO("Alignment complete");
    }
    else
    {   
        //since the right side is not aligned, the robot will turn to its right
        motorizer.angular.z = -0.5;
        pub.publish(motorizer);
        ROS_INFO("Realigning . . . right = %f, nearest wall = %f", scan->ranges[820], distance_mem);
    }

    //will check forward space after alignment to make sure the robot can proceed to the next step in main
    if(scan->ranges[1360] > 1.0)
    {
        space_to_move = true;
    }
    else
    {
        space_to_move = false;
    }

    //var will keep track of consecutive callings of this method to prevent infinite loops
    alignment_tracker = alignment_tracker + 1;
    if(alignment_tracker > 50)
    {   
        //something must be wrong if it is called 50 times, so the variables will be recalculated
        needs_setup = true;
    }

}
//will turn the robot to its left 45 degrees
void turn_left_move()
{   
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(1.0);

    //publishing turning speed
    motorizer.angular.z = 0.9;
    motorizer.linear.x = 0.5;
    pub.publish(motorizer);
    ROS_INFO("Turning left");

    //pause before stopping
    turn_time.sleep();

    //stopping robot
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //global variables are no longer accurate
    needs_setup = true;
}
void turn_left()
{   
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(1.0);

    //publishing turning speed
    motorizer.angular.z = 0.9;
    pub.publish(motorizer);
    ROS_INFO("Turning left");

    //pause before stopping
    turn_time.sleep();

    //stopping robot
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //global variables are no longer accurate
    needs_setup = true;
}
//will turn the robot to its right 45 degrees
void turn_right_move()
{   
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(0.5);

    //publishing turning speed
    motorizer.angular.z = -0.7;
    motorizer.angular.x = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Turning right");

    //pause before stopping
    turn_time.sleep();

    //stopping robot
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //global variables are no longer accurate
    needs_setup = true;
}
//will move the robot forward
void move_forward(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];

    if(center > 1.0)
    {      
        if(right < distance_mem + 0.1 && right > distance_mem -0.1)
        {
            motorizer.linear.x = 1.0;
            pub.publish(motorizer);
            ROS_INFO("Moving forward");
        }
        else if(right > distance_mem)
        {
            motorizer.linear.x = 1.0;
            motorizer.angular.z = -0.1;
            pub.publish(motorizer);
            ROS_INFO("Moving forward and getting closer to wall");
        }
        else
        {
            turn_right_move();
            needs_setup = true;
        }
    }
    else
    {
        if(left > 1.5 && center > 0.5)
        {
            turn_left_move();
        }
        else if(left > 1.5)
        {
            turn_left();
        }
        needs_setup = true;
    }
    /*
    //length of move time in hz
    ros::Rate turn_time(0.5);

    //publishing turning speed
    motorizer.linear.x = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Moving forward");

    //pause before stopping
    turn_time.sleep();

    //stopping robot
    motorizer.linear.x = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Movement complete");

    //global variables are no longer accurate
    needs_setup = true;
    */
}
//decides which of the above methods to call
void decider(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //check if variables are set up
    if(needs_setup)
    {
        var_set_up(scan);
    }

    //choosing action to take
    if(aligned)
    {   
        if(space_to_move)
        {
            move_forward(scan);
        }
        else
        {
            turn_left_move();
        }
    }
    else
    {
        aligner(scan);
    }
}
int main(int argc, char** argv)
{
    //setting up node
    ros::init(argc, argv, "wall_follower_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    
    //condition will make var_set_up run once the subscriber runs
    needs_setup = true;
    //just to make sure it aligns at the start
    aligned = false;

    //subscribes to laser scan and calls a method which decides which action to take
    sub = n.subscribe<sensor_msgs::LaserScan>(laser_scan, 1000, decider);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}