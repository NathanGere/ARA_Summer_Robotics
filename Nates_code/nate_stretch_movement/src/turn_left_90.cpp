#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//global pub
ros::Publisher pub;

//params
std::string cmd_vel_output;
std::string laser_scan;

//global var
bool turner;

void turn_left()
{
    geometry_msgs::Twist motorizer;
    if(turner)
    {   
        ros::Rate turn_rate(1.0);
        //publishing turning speed
        motorizer.angular.z = 1.0;
        pub.publish(motorizer);
        ROS_INFO("Turning left 90");
        turn_rate.sleep();
        turner = false;
    }
    else
    {
        ROS_INFO("haha");
    }
}
int main(int argc, char** argv)
{
    //setting up node
    ros::init(argc, argv, "turn_left_90_node");
    ros::NodeHandle n;

    //setting up parameters
    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    //setting up publisher and msg to be published
    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);

    turner = true;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        turn_left();
        loop_rate.sleep();
    }
    
    return 0;
}