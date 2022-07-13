#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//code that can be used to filter out values in a point cloud that are not within a specified range

//How many coders does it take to change a light bulb? (scroll down for answer)

//global publisher
ros::Publisher pub;

//parameters
std::string cloud_param;
std::string cloud_output;
std::string filter_field_name;
float filter_limit_min;
float filter_limit_max;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //giving the values to our new cloud
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Uncomment for snowstorm
    /*
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    
    for (auto& point: *cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    */

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (filter_field_name);
    pass.setFilterLimits (filter_limit_min, filter_limit_max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    //creating a topic for our filtered cloud to be published as
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);

    //giving the output some values that were lost in the filtering
    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    //publishing our passthrough cloud
    pub.publish(output_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_passthrough_node");
    ros::NodeHandle n;

    //works in sim, may need a different input cloud_param for the actual robot
    n.param<std::string>("cloud_param", cloud_param, "/camera/depth/color/points");
    n.param<std::string>("cloud_output", cloud_output, "camera_depth_color_points_passthrough");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_param, 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        //params can be altered while running
        n.param<std::string>("filter_field_name", filter_field_name, "y");
        n.param<float>("filter_limit_min", filter_limit_min, -1.0);
        n.param<float>("filter_limit_max", filter_limit_max, 1.0);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 

//one if riley is here. the rest of us would watch
