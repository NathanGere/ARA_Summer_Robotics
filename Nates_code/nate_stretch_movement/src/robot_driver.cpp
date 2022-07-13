#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs_generate_messages_cpp.dir/Twist.h>

class RobotDriver
{
    private:
        ros::NodeHandle n;
        ros::Publisher cmd_vel_pub;
    public:
        //node initialization
        RobotDriver(ros::NodeHanlde &n)
        {
            n_ = n;
            //set up publisher
            cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
        }
    
    //loop forever while sending drive commands based on keyboard input
    bool driveKeyboard()
    {
        std::cout << "Type a command and then press enter.  "
        "Use '+' to move forward, 'l' to turn left, "
        "'r' to turn right, '.' to exit.\n";

        //msg to be published to cmd_vel
        geometry_msgs::Twist base_cmd;

        char cmd[50];
        while(n_.ok())
        {
            std::cin.getline(cmd, 50);
            if(cmd[0] != '+' && cmd[0] != 'r' && cmd[0] != '.')
            {
                std::cout << "unknown command:" << cmd << "\n";
                continue;
            }

            base_cmd.linear.x = base_cmd.linear.y = base.linear.z = 0;
            //move forward
            if()
        }
    }
}