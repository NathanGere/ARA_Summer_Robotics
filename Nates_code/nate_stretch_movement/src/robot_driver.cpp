#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
//#include "nate_stretch_movement/robot_driver.h"

class RobotDriver
{
    private:
        ros::NodeHandle n_;
        ros::Publisher cmd_vel_pub;
        tf::TransformListener listener_;
    public:
        //node initialization
        RobotDriver(ros::NodeHandle &n)
        {
            n_ = n;
            //set up publisher
            cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1);
        }
    
    //loop forever while sending drive commands based on keyboard input
    bool driverKeyboard()
    {
        std::cout << "Type a command and then press enter.  "
        "Use 'w' to move forward, 'a' to turn left, "
        "'d' to turn right, and '.' to exit.\n";

        //msg to be published to cmd_vel
        geometry_msgs::Twist base_cmd;

        char cmd[50];
        while(n_.ok())
        {
            std::cin.getline(cmd, 50);
            if(cmd[0] != 'w' && cmd[0] != 'd' && cmd[0] != '.' && cmd[0] != 'a')
            {
                std::cout << "unknown command:" << cmd << "\n";
                continue;
            }

            base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
            //move forward
            if(cmd[0] == 'w')
            {
                base_cmd.linear.x = 0.25;
                base_cmd.angular.z = 0.0;
            }
            //turn left (yaw) and drive forward at the same time
            else if(cmd[0] == 'a')
            {
                base_cmd.angular.z = 0.75;
                base_cmd.linear.x = 0.25;
            }
            //turn right (yaw) and drive forward at the same time
            //quit
            else if(cmd[0] = 'd')
            {
                base_cmd.angular.z = -0.75;
                base_cmd.linear.x = 0.25;
            }
            else if(cmd[0] = '.')
            {   
                base_cmd.angular.z = 0.0;
                base_cmd.linear.x = 0.0;
                break;
            }

            //publish assembled command
            cmd_vel_pub.publish(base_cmd);
        }
        return true;
    }
    bool driveForwardOdom(double distance)
    {
        //wait for listener to get first message
        listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

        //record transforms
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;

        //record the starting transfrom from the odometry to base frame
        listener_.lookupTransform("base_link", "odom", ros::Time(0), start_transform);

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.25;

        ros::Rate rate(10.0);
        bool done = false;
        while(!done && n_.ok())
        {
            cmd_vel_pub.publish(base_cmd);
            rate.sleep();
            //get current transform
            try
            {
                listener_.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
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
        return false;
    }
    bool turnOdom(bool clockwise, double radians)
    {
        while(radians < 0) radians += 2*M_PI;
        while(radians > 2*M_PI) radians -= 2*M_PI;

        //wait for listener to get first message
        listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

        //record transforms
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;

        //record the starting transfrom from the odometry to base frame
        listener_.lookupTransform("base_link", "odom", ros::Time(0), start_transform);

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.y = base_cmd.linear.x = 0;
        base_cmd.angular.z = 0.75;

        if(clockwise) base_cmd.angular.z = -base_cmd.angular.z;

        //the axis we want to be rotating by
        tf::Vector3 desired_turn_axis(0, 0, 1);
        if(!clockwise) desired_turn_axis = -desired_turn_axis;

        ros::Rate rate(10.0);
        bool done = false;
        while(!done && n_.ok())
        {
            cmd_vel_pub.publish(base_cmd);
            rate.sleep();
            //get current transform
            try
            {
                listener_.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
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
        return false;
    } 
};

int main(int argc, char** argv)
{
    //init node
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle n;

    RobotDriver driver(n);
    //driver.driverKeyboard();
    //driver.driveForwardOdom(0.5);
    bool clockwise = true;
    double radians = 1.2;
    driver.turnOdom(clockwise, radians);
}