#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

ros::Publisher pub;
std::string cmd_vel_output;
std::string laser_scan;
float forward_speed;
float backward_speed;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    geometry_msgs::Twist motorizer;
    int size = scan->ranges.size();
    float front = scan->ranges[(size-1)/2];
    float back = scan->ranges[0];

    int f = 0;
    int b = 0;

    if(front < 1.1)
    {
        f = 1;
    }
    if(back < 1.1)
    {
        b = 2;
    }

    int tracker = f + b;

    switch(tracker)
    {
        case 0:
            motorizer.linear.x = forward_speed;
        case 1:
            motorizer.linear.x = backward_speed;
            break;
        case 2:
            motorizer.linear.x = forward_speed;
            break;
        case 3:
            motorizer.linear.x = 0.0;
            break;
    }

    pub.publish(motorizer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_avoidance_node");
    ros::NodeHandle n;

    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, callback);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        n.param<float>("forward_speed", forward_speed, 0.5);
        n.param<float>("backward_speed", backward_speed, -0.5);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}