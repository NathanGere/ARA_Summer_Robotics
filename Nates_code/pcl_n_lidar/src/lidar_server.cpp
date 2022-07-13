#include <ros/ros.h>
#include <Nate_stretch_pcl_n_lidar/laser_scan.h>

bool retriever(Nate_stretch_pcl_n_lidar::laser_scan::Request &req, Nate_stretch_pcl_n_lidar::laser_scan::Response &res)
{
    //given the laser scan from "/scan" from the client, it will assign the same values to the service's response
    res.header.seq = req.header.seq;
    res.header.stamp = req.header.stamp;
    res.header.frame_id = req.header.frame_id;
    res.angle_min = req.angle_min;
    res.angle_max = req.angle_max;
    res.angle_increment = req.angle_increment;
    res.scan_time = req.scan_time;
    res.range_min = req.range_min;
    res.range_max = req.range_max;
    res.ranges = req.ranges;
    res.intensities = req.intensities;

    //a print statement so that you know its working
    ROS_INFO("Retrieving values");

    //boolean value for successful call
    return true;
}
int main(int argc, char **argv)
{   
    //node setup
    ros::init(argc, argv, "lidar_server_node");
    ros::NodeHandle n;

    //provides service under "lidar_values" topic
    ros::ServiceServer server = n.advertiseService("lidar_values", retriever);

    //print statement that shows node as activated 
    ROS_INFO("Lidar_server_node: Providing scan values");

    //normal ros stuff
    ros::spin();
    return 0;
}