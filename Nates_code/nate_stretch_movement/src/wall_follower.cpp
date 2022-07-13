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
bool lost; //will be set to true if there are no walls near the robot and it just needs to move around
bool crashed; //will be set to true if specific lidar values are lower than they should be
bool initial_orientation; //tells decider to run orientation methods
bool initial_distance; //will get the robot close to the wall so that it can follow it
bool first_alignment; //will align the robot turning left instead of right
bool needs_setup; //tells decider if it needs to recalculate conditions
bool aligned; //will track if the robot is aligned with a wall on its right
bool space_to_move; //will track if the robot can move forward
//trackers
int index_mem; //keeps record of index of nearest wall for scan values
float distance_mem; //keeps record of distance to nearest wall
int alignment_tracker; //keeps track of consecutive callings of aligner method

//methods
void turn_left_60_and_move();
void turn_left_90_and_move();
void turn_left_180_and_move();

//will have the robot move around until it finds a wall
void explorer(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];

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
        initial_orientation = false;
        initial_distance = false;
        first_alignment = false;
    }
    else
    {
        //since no walls are nearby, the robot will move forward for a longer period to save computational power
        
        //length of turning time in hz
        ros::Rate move_time(0.1);

        //publishing forward speed
        motorizer.linear.x = 1.5;
        pub.publish(motorizer);
        ROS_INFO("Exploring . . .");

        //giving time to move
        move_time.sleep();
    }
}
//will recover robot if it is crashed but still upright
void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //setting up switch statement
    int back = 0;
    int right = 0;

    //will give back a value of 1 if it is back
    if((1890 <= index_mem && index_mem <= 2000)||(0 <= index_mem && index_mem <= 820))
    {
        back = 1;
    }
    //will give right a value of 2 if it is right
    if(300 <= index_mem && index_mem <= 1360)
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
    ros::Rate recovery_time(0.5);
    pub.publish(motorizer);
    recovery_time.sleep();

    //stopping robot before it crashes again
    motorizer.linear.x = 0;
    motorizer.angular.z = 0;
    pub.publish(motorizer);

    //hopefully the robot is no longer stuck, and this will be set to false
    crashed = false;
}
//will orient the robot so that it is facing the nearest wall
void initial_orientation_setup(const sensor_msgs::LaserScan::ConstPtr &scan)
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
    float center = scan->ranges[1360];
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
        motorizer.angular.z = -0.5;
        pub.publish(motorizer);
        ROS_INFO("Orienting . . .");
    }
    //if the robot is facing the wall, the orientation is complete
    else
    {
        initial_orientation = true;
        ROS_INFO("INITIAL ORIENTATION COMPLETE.");
    }
}
//will move the robot towards the nearest wall until it is 0.5 meters away (assuming it is facing the nearest wall)
void initial_distance_setup(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //float to store distance in front of the object
    float front = scan->ranges[1360];

    //if there is space, the robot will get closer to the wall
    if(front > 0.6)
    {
        motorizer.linear.x = 0.5;
        pub.publish(motorizer);
        ROS_INFO("Establishing initial distance . . .");
    }
    //once the robot is close enough, it will proceed
    else
    {
        initial_distance = true;
        ROS_INFO("PREFERED INITIAL DISTANCE ACHIEVED.");
    } 
}
//will turn the robot to its left until its right side is parallel to the nearest wall
void first_alignment_setup(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //float to store distance in front of the object
    float right = scan->ranges[820];

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
    if(right < distance_mem + 0.02 && right > distance_mem - 0.02)
    {
        first_alignment = true;
        ROS_INFO("FIRST ALIGNMENT COMPLETE.");
    }
    //will turn the robot to its left if it is not aligned with the nearest wall
    else
    {
        motorizer.angular.z = 0.5;
        pub.publish(motorizer);
        ROS_INFO("Completing first alignment . . .");
    }
}
//will recalculate the global conditions and trackers when called
void var_set_up(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float back = scan->ranges[300];
    float back_right = scan->ranges[565];
    float right = scan->ranges[820];
    float front_right = scan->ranges[1085];
    float front = scan->ranges[1360];
    float front_left = scan->ranges[1620];
    float left = scan->ranges[1890];
    float back_left = scan->ranges[40];

    //loop iteration variable
    int i = 0;
    index_mem = 100;
    distance_mem = 100.0;
    //tracker for nearby walls
    bool walls_nearby = false;
    //The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance
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
        if(scan->ranges[i] < 0.2)
        {
            crashed = true;
        }
    
        i++;
    }

    //will activate explorer method in next loop of ros::spinOnce()
    if(!walls_nearby)
    {
        lost = true;
    }

    //will activate crash recovery method
    if(back < 0.3 || front < 0.1 || right < 0.17 || left < 0.17)
    {
        crashed = true;
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
    //msg to be published to cmd_vel
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
//will turn the robot left according to scan data
void turn_left(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //everything will need to be reoriented after turning
    needs_setup = true;

    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float back = scan->ranges[300];
    float back_right = scan->ranges[565];
    float right = scan->ranges[820];
    float front_right = scan->ranges[1085];
    float front = scan->ranges[1360];
    float front_left = scan->ranges[1620];
    float left = scan->ranges[1890];
    float back_left = scan->ranges[40];

    //object conditions to track if objects are nearby on the left side
    int fl = 0;
    int l = 0;
    int bl = 0;

    //setting up conditions for switch statement to choose how to turn
    if(back_left < 1.5)
    {
        bl = 1;
    }
    if(left < 1)
    {
        l = 2;
    }
    if(front_left < 1.5)
    {
        fl = 4;
    }

    int total = bl + l + fl;
    switch(total)
    {      
        //there are no objects anywhere on the left
        case 0:
            turn_left_60_and_move();
            break;
        //theres an object on the back left
        case 1:
            turn_left_60_and_move();
            break;
        //theres an object directly left
        case 2:
            turn_left_180_and_move();
            break;
        //thers an object left and back left
        case 3:
            turn_left_180_and_move();
            break;
        //theres an object on the front left
        case 4:
            turn_left_180_and_move();
            break;
        //theres objects front left and back left, but not left
        case 5:
            turn_left_90_and_move();
            break;
        //theres objects left and front left
        case 6:
            turn_left_180_and_move();
            break;
        //theres objects all over the left side
        case 7:
            turn_left_180_and_move();
            break;
    }
}
void turn_left_60_and_move()
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(1.0);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Turning left 60");

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //moving forward
    motorizer.linear.x = 0.7;
    pub.publish(motorizer);
    ROS_INFO("Adjusting");
    turn_time.sleep();
    motorizer.linear.x = 0.0;
    pub.publish(motorizer);
}
void turn_left_90_and_move()
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(1.0);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Turning left 90");

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //moving forward
    motorizer.linear.x = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Adjusting");
    turn_time.sleep();
    motorizer.linear.x = 0.0;
    pub.publish(motorizer);
}
void turn_left_180_and_move()
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(1.0);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Turning left 180");

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);

    //publishing turning speed
    motorizer.angular.z = 1.0;
    pub.publish(motorizer);

    //pause before stopping
    turn_time.sleep();

    //stopping turning
    motorizer.angular.z = 0.0;
    pub.publish(motorizer);
    ROS_INFO("Turning complete");

    //moving forward
    motorizer.linear.x = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Adjusting");
    turn_time.sleep();
    motorizer.linear.x = 0.0;
    pub.publish(motorizer);
}
void turn_left_move()
{   
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(0.5);

    //publishing turning speed
    motorizer.angular.z = 1.0;
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
//will turn the robot to its right 45 degrees
void turn_right_move()
{   
    geometry_msgs::Twist motorizer;

    //length of turning time in hz
    ros::Rate turn_time(0.5);

    //publishing turning speed
    motorizer.angular.z = -1.0;
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
        turn_left(scan);
        needs_setup = true;
    }
}
//decides which of the above methods to call
void decider(const sensor_msgs::LaserScan::ConstPtr &scan)
{   
    //will call lost method if no walls are nearby
    if(lost)
    {
        explorer(scan);
    }
    else
    {
        //will complete intial setup
        if(!initial_orientation)
        {
            initial_orientation_setup(scan);
        }
        else if(!initial_distance)
        {
        initial_distance_setup(scan);
        }
        else if(!first_alignment)
        {
            first_alignment_setup(scan);
        }
        else
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
                    turn_left(scan);
                }
            }
            else
            {
                aligner(scan);
            }
        }
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
    //will be set to true if the robot has no walls in its vicinity
    lost = false;
    //to make sure it runs through initial setup
    initial_orientation = false;
    initial_distance = false;
    first_alignment = false;
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