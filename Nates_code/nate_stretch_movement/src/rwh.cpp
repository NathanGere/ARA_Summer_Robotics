#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

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
bool space_to_move; //is there enough room for the robot to drive forward?

//params
std::string cmd_vel_output;
std::string laser_scan;

//methods
//decides which method to call
void thinker(const sensor_msgs::LaserScan::ConstPtr &scan);
//will activate if the robot gets lost or in a crash
void explorer(const sensor_msgs::LaserScan::ConstPtr &scan);
void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan);
//will get the robot in position to hug a wall on its right
void get_oriented(const sensor_msgs::LaserScan::ConstPtr &scan);
void get_closer(const sensor_msgs::LaserScan::ConstPtr &scan);
void get_aligned(const sensor_msgs::LaserScan::ConstPtr &scan);
//allow robot to move set distances based on odometry
bool driveForwardOdom(double distance);
bool turnOdom(bool clockwise, double radians);
//turning left LOL
void turn_left(const sensor_msgs::LaserScan::ConstPtr &scan);
void turn_left_60_and_move();
void turn_left_90_and_move();
void turn_left_135_and_move();
void turn_left_180_and_move();
//turning right
void turn_right_45_and_move();
void turn_right_90_and_move();
//moving forward
void move_forward();
void move_forward_and_get_closer();

int main(int argc, char** argv)
{
    //setting up node
    ros::init(argc, argv, "rwh_node");
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
/* PLAN

1)
robot checks lost and crash
if lost, drive until no longer lost
if crashed, go opposite way

2)
oriented
close_enough
aligned

3)
if there is distance to move forward, and wall is still correct distance, move forward
if wall disapears, turn right and move -> go back to step 2)
if wall gets slightly too far, move and get closer
if wall gets slightly too close, move and get farther

4)
if there is no space no space to move forward:
check laser scan to make best left turn and move forward slightly
go back to step 2

*/
void thinker(const sensor_msgs::LaserScan::ConstPtr &scan)
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

    if(front > 1.0 && front_left > 0.4 && front_right > 0.4)
    {
        space_to_move = true;
    }

    bool wall_on_right = true;
    bool too_far_from_right = false;
    if(right < distance_mem + 0.1 && right > distance_mem - 0.1)
    {
        wall_on_right = true;
    }
    else if (right > 5.0)
    {
        wall_on_right = false;
    }
    else if (right >= distance_mem + 0.1 && right <= 5.0)
    {
        too_far_from_right = true;
    }
    else
    {
        ROS_INFO("It should not be possible to be closer than the closest wall");
    }

    if(lost)
    {
        explorer(scan);
    }
    else if(crashed)
    {
        crash_recovery(scan);
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
            if(space_to_move)
            {
                if(!wall_on_right)
                {
                    oriented = false;
                    close_enough = false;
                    aligned = false;
                    turn_right_45_and_move();
                }
                else if(too_far_from_right)
                {
                    move_forward_and_get_closer();
                }
                else
                {
                    move_forward();
                }
            }
            else
            {
                if(!wall_on_right)
                {
                    oriented = false;
                    close_enough = false;
                    aligned = false;
                    turn_right_90_and_move();
                }
                else
                {   
                    oriented = false;
                    close_enough = false;
                    aligned = false;
                    turn_left(scan);
                }
            }
        }
    }
}
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
        oriented = false;
        close_enough = false;
        aligned = false;
    }
    else
    {
        //publishing forward speed
        motorizer.linear.x = 1.5;
        pub.publish(motorizer);
        ROS_INFO("Exploring . . .");
    }
}
//will recover robot if it is crashed but still upright
void crash_recovery(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;

    ROS_INFO("Recovering crash . . . ");

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
        close_enough = true;
        ROS_INFO("PREFERED INITIAL DISTANCE ACHIEVED.");
    } 
}
//will turn the robot to its left until its right side is parallel to the nearest wall
void get_aligned(const sensor_msgs::LaserScan::ConstPtr &scan)
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
        aligned = true;
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
bool driveForwardOdom(double distance)
{   
    tf::TransformListener listener;
    //wait for listener to get first message
    listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

    //record transforms
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transfrom from the odometry to base frame
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);

    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    ros::Rate rate(10.0);
    bool done = false;
    while(!done && ros::ok())
    {
        pub.publish(base_cmd);
        rate.sleep();
        //get current transform
        try
        {
            listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            break;
        }
        //see distance traveled
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        double dist_moved = relative_transform.getOrigin().length();
        if(dist_moved > distance) done = true; 
    }
    if (done) return true;
    return true;
}
//method for turning robot a specified amt 
bool turnOdom(bool clockwise, double radians)
{   
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    tf::TransformListener listener;
    //wait for listener to get first message
    listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

    //record transforms
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transfrom from the odometry to base frame
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);

    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = base_cmd.linear.x = 0;
    base_cmd.angular.z = 0.75;

    if(clockwise) base_cmd.angular.z = -base_cmd.angular.z;

    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0, 0, 1);
    if(!clockwise) desired_turn_axis = -desired_turn_axis;

    ros::Rate rate(10.0);
    bool done = false;
    while(!done && ros::ok())
    {
        pub.publish(base_cmd);
        rate.sleep();
        //get current transform
        try
        {
            listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            break;
        }
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
        double angle_turned = relative_transform.getRotation().getAngle();
        if( fabs(angle_turned) < 1.0e-2) continue;
        if( actual_turn_axis.dot( desired_turn_axis ) < 0 ) angle_turned = 2 * M_PI - angle_turned;
        if(angle_turned > radians) done = true;
    }
    if(done) return true;
    return true;
} 
//will turn the robot left according to scan data
void turn_left(const sensor_msgs::LaserScan::ConstPtr &scan)
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
            ROS_INFO("Nothing on my left");
            turn_left_60_and_move();
            break;
        //theres an object on the back left
        case 1:
            ROS_INFO("Object detected back left");
            turn_left_60_and_move();
            break;
        //theres an object directly left
        case 2:
            ROS_INFO("Object detected directly left");
            turn_left_180_and_move();
            break;
        //thers an object left and back left
        case 3:
            ROS_INFO("Object detected directly left and back left");
            turn_left_180_and_move();
            break;
        //theres an object on the front left
        case 4:
            ROS_INFO("Object detected front left");
            turn_left_135_and_move();
            break;
        //theres objects front left and back left, but not left
        case 5:
            ROS_INFO("Object detected front left and back left, but not directly left");
            turn_left_90_and_move();
            break;
        //theres objects left and front left
        case 6:
            ROS_INFO("Object detected front left and directly left");
            turn_left_180_and_move();
            break;
        //theres objects all over the left side
        case 7:
            ROS_INFO("Objects detected everywhere left");
            turn_left_180_and_move();
            break;
    }
}
void turn_left_60_and_move()
{
    bool clockwise = false;
    double radians = 0.8;
    ROS_INFO("Turning left 60");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_60_and_move();
    }
}
void turn_left_90_and_move()
{   
    bool clockwise = false;
    double radians = 1.2;
    ROS_INFO("Turning left 90");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_90_and_move();
    }
}
void turn_left_135_and_move()
{   
    bool clockwise = false;
    double radians = 2.0;
    ROS_INFO("Turning left 135");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_135_and_move();
    }
}
void turn_left_180_and_move()
{   
    bool clockwise = false;
    double radians = 2.8;
    ROS_INFO("Turning left 180");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_180_and_move();
    }
}
void turn_right_45_and_move()
{
    bool clockwise = true;
    double radians = 0.6;
    ROS_INFO("Turning right 45");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_60_and_move();
    }
}
void turn_right_90_and_move()
{
    bool clockwise = true;
    double radians = 1.2;
    ROS_INFO("Turning right 90");
    if(turnOdom(clockwise, radians))
    {
        double distance = 0.5;
        bool left = true;
        if(!driveForwardOdom(distance))
        {
            ROS_ERROR("Failed to move forward.");
        }
    }
    else
    {
        ROS_ERROR("Failed to turn, trying again");
        turn_left_90_and_move();
    }
}
void move_forward()
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;
    motorizer.linear.x = 1.0;
    pub.publish(motorizer);
    ROS_INFO("Moving forward");
}
void move_forward_and_get_closer()
{   
    //msg to be published to cmd_vel
    geometry_msgs::Twist motorizer;
    motorizer.linear.x = 1.0;
    motorizer.angular.z = -0.15;
    pub.publish(motorizer);
    ROS_INFO("Moving forward and getting closer to wall");
}
