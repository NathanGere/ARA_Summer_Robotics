#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <iostream>
#include <thread>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>

//this program can be used to downsample a pcl cloud to reduce computation time

//global publisher
ros::Publisher pub;

//parameters
std::string cloud_param;
std::string cloud_output;
float x_filter_limit_min;
float x_filter_limit_max;
float z_filter_limit_min;
float z_filter_limit_max;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //creating clouds for the input and output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for initial cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for final cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_xz_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    //giving the values to our new raw cloud
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    //downsampling the raw cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (raw_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*downsampled_cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (downsampled_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_filter_limit_min, x_filter_limit_max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*ds_x_filtered);

    //filter y
    pass.setInputCloud(ds_x_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(z_filter_limit_min, z_filter_limit_max);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*ds_xz_filtered);

    //creating a topic for our downsampled cloud to be published as
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*ds_xz_filtered, output_msg);

    //giving the output some values that were lost in the downsampling
    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    //publishing our downsample
    pub.publish(output_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_for_nav_node");
    ros::NodeHandle n;

    //works in sim, may need a different input cloud_param for the actual robot
    n.param<std::string>("cloud_param", cloud_param, "/camera/depth/color/points");
    n.param<std::string>("cloud_output", cloud_output, "/pcl_for_nav/points");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_param, 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        //params can be altered while running
        n.param<float>("x_filter_limit_min", x_filter_limit_min, -0.4);
        n.param<float>("x_filter_limit_max", x_filter_limit_max, 0.0);
        n.param<float>("z_filter_limit_min", z_filter_limit_min, 0.0);
        n.param<float>("z_filter_limit_max", z_filter_limit_max, 1.25);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 