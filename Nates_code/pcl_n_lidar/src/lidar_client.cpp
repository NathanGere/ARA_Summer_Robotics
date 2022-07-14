#include <ros/ros.h>
#include <pcl_n_lidar/laser_scan.h>
#include <sensor_msgs/LaserScan.h>

std::string laser_scan;
ros::ServiceClient client;

void caller_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pcl_n_lidar::laser_scan srv;

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

    client = n.serviceClient<pcl_n_lidar::laser_scan>("lidar_values");
    ros::Subscriber sub = n.subscribe(laser_scan, 1000, caller_cb);

    ros::spin();

    return 0;
}