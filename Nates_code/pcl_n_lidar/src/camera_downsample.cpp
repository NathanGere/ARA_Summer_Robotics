#include <pcl/filters/voxel_grid.h>
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

//this program can be used to downsample a pcl cloud to reduce computation time
//(scroll to bottom) I invented a new word!

//global publisher
ros::Publisher pub;

//parameters
std::string cloud_param;
std::string cloud_output;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //creating clouds for the input and output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for initial cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //creating variable for final cloud

    //giving the values to our new raw cloud
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    //downsampling the raw cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (raw_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*downsampled_cloud);

    //creating a topic for our downsampled cloud to be published as
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*downsampled_cloud, output_msg);

    //giving the output some values that were lost in the downsampling
    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    //publishing our downsample
    pub.publish(output_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_pcl_downsampler_node");
    ros::NodeHandle n;

    //works in sim, may need a different input cloud_param for the actual robot
    n.param<std::string>("cloud_param", cloud_param, "/camera/depth/color/points");
    n.param<std::string>("cloud_output", cloud_output, "camera_depth_color_points_downsample");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_param, 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 

//the word I invented is plagarism. I also "invented" this joke xD