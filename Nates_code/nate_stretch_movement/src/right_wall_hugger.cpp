#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//this code will have a robot drive forward, hugging a wall on its right from a distance of 0.5 meters

//global publisher
ros::Publisher pub;

//global geometry message to be published to cmd_vel
geometry_msgs::Twist motorizer;

//setting up parameters
std::string cmd_vel_output;
std::string laser_scan;

//global subscriber
ros::Subscriber sub;

//global variables for callbacks
float mem;

void wall_finder_cb(const sensor_msgs::LaserScanConstPtr &scan)
{
    //number of indices in laser scan array
    int size = scan->ranges.size();

    //floats will store the distance to nearest object at cherrypicked indices
    float right = scan->ranges[820];
    float center = scan->ranges[1360];
    float left = scan->ranges[1890];
    
    //this loop will attempt to orient the robot so that it is parallel to a wall on its right
    float current = right;
    float mem = 1000;
    ros::Rate pause_rate(0.1);
    ROS_INFO("Finding wall . . .");
    while(current < mem)
    {
        mem = current;
        motorizer.angular.z = -0.5;
        pub.publish(motorizer);
        pause_rate.sleep();
        motorizer.angular.z = 0.0;
        pub.publish(motorizer);
        current = scan->ranges[820];
    }
}
void mover_cb(const sensor_msgs::LaserScanConstPtr &scan)
{
    float distance_keeper = scan->ranges[820];
    float center = scan->ranges[1360];
    if(distance_keeper < mem + 0.1 && distance_keeper > mem - 0.1)
    {
        if(center < 1.0)
        {
            ROS_INFO("\nTurning");
            motorizer.angular.z = 0.5;
            ros::Rate turn_time(2.0);
            pub.publish(motorizer);
            turn_time.sleep();    
        }
        else
        {   
            ROS_INFO("\nMoving Forward");
            motorizer.linear.x = 1.0;
            pub.publish(motorizer);
        }
    }
    else
    {
        ROS_INFO("\nRealigning . . . ");
        motorizer.angular.z = -0.5;
        pub.publish(motorizer);
    }
}
int main(int argc, char** argv)
{
    //setting up node
    ros::init(argc, argv, "right_wall_hugger_node");
    ros::NodeHandle n;

    //for movement on the actual robot, cmd_vel_output: "/stretch/cmd_vel"
    n.param<std::string>("cmd_vel_output", cmd_vel_output, "/stretch_diff_drive_controller/cmd_vel");
    n.param<std::string>("laser_scan", laser_scan, "/scan");

    pub = n.advertise<geometry_msgs::Twist>(cmd_vel_output, 1000);
    sub = n.subscribe(laser_scan, 1000, wall_finder_cb);
    //sub = n.subscribe(laser_scan, 1000, mover_cb);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
#include <ros/ros.h>
#include <Nate_stretch_pcl_n_lidar/laser_scan.h>
#include <sensor_msgs/LaserScan.h>

std::string laser_scan;
ros::ServiceClient client;

void caller_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    Nate_stretch_pcl_n_lidar::laser_scan srv;

    srv.request.header.seq = scan->header.seq;
    srv.request.header.stamp = scan->header.stamp;
    srv.request.header.frame_id = scan->header.frame_id;
    srv.request.angle_min = scan->angle_min;
    srv.request.angle_max = scan->angle_max;
    srv.request.angle_increment = scan->angle_increment;
    srv.request.scan_time = scan->scan_time;
    srv.request.range_min = scan->range_min;
    srv.request.range_max = scan->range_max;
    srv.request.ranges = scan->ranges;
    srv.request.intensities = scan->intensities;

    if(client.call(srv))
    {
        ROS_INFO("Scan values updated");
    }
    else
    {
        ROS_ERROR("Failed to update scan values");
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_client_node");
    ros::NodeHandle n;

    n.param<std::string>("laser_scan", laser_scan, "/scan");

    client = n.serviceClient<Nate_stretch_pcl_n_lidar::laser_scan>("lidar_values");
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, caller_cb);

    ros::spin();

    return 0;
}