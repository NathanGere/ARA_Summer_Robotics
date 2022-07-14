#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include <ros/ros.h>

class RobotDriver
{   
    private:
    public:
        RobotDriver(ros::NodeHanlde &n);
    bool driverKeyboard();
    bool driveForwardOdom();
    bool turnOdom();
};

#endif