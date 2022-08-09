#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import Header, String
import numpy as np
   
def callback(cloud_msg):

    '''
    if cloud_msg.is_dense:
        print("It is dense")
    else:
        print("It is not dense")
    '''

    pointCloud = cloud_msg
    print("\n\n\n\n\n\tXYZ POINTS")
    for point in pc2.read_points(pointCloud, field_names=pointCloud.data, skip_nans=True):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]
        print("x: " + str(pt_x) + " y: " + str(pt_y) + " z: " + str(pt_z))
    

def main():
    rospy.init_node("pcl_xyz_node")
    cloud_sub = rospy.Subscriber("/pcl_for_nav/points", PointCloud2, callback, queue_size=1, buff_size=52428800)
    rospy.spin()

if __name__ == '__main__':
    main()

'''

#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import Header, String
import numpy as np

class PCLtoXYZ:

    def __init__(self):
        #initiliaze  ros stuff
        self.cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback, queue_size=1, buff_size=52428800)      

    def callback(ros_point_cloud):

        
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = pc2.read_points(ros_point_cloud, field_names = pointCloud.data, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
            # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)
        
        print(xyz)

if __name__ == '__main__':
    try:
        #setting up node
        rospy.init_node("pcl_xyz_node", anonymous =True)

        converter = PCLtoXYZ()
        converter.callback()

        #setting up params
        #cloud_param = rospy.get_param("/camera/depth/color/points")
        #cloud_ouput = rospy.set_param("camera_depth_color_points_xyz")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
''''''
import rospy
import ros_numpy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2

def cartesian_translator(pointcloud2_array):
    xyz_array = ros_numpy.point_cloud2.get_xyz_points(pointcloud2_array)
    print(xyz_array)

if __name__ == '__main__':
    try:
        #setting up node
        rospy.init_node("pcl_xyz_node", anonymous =True)

        #setting up params
        #cloud_param = rospy.get_param("/camera/depth/color/points")
        #cloud_ouput = rospy.set_param("camera_depth_color_points_xyz")

        sub = rospy.Subscriber("/camera/depth/color/points", pc2, cartesian_translator, None, 1000)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
'''
'''
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
'''
'''
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

      def __init__(self):
            '''#initiliaze  ros stuff 
'''

            self.cloud_sub = rospy.Subscriber("/your_cloud_topic", PointCloud2,self.callback,queue_size=1, buff_size=52428800)      

        def callback(self, ros_point_cloud):
                xyz = np.array([[0,0,0]])
                rgb = np.array([[0,0,0]])
                #self.lock.acquire()
                gen = pc2.read_points(ros_point_cloud, skip_nans=True)
                int_data = list(gen)

                for x in int_data:
                    test = x[3] 
                    # cast float32 to int so that bitwise operations are possible
                    s = struct.pack('>f' ,test)
                    i = struct.unpack('>l',s)[0]
                    # you can get back the float value by the inverse operations
                    pack = ctypes.c_uint32(i).value
                    r = (pack & 0x00FF0000)>> 16
                    g = (pack & 0x0000FF00)>> 8
                    b = (pack & 0x000000FF)
                    # prints r,g,b values in the 0-255 range
                                # x,y,z can be retrieved from the x[0],x[1],x[2]
                    xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
                    rgb = np.append(rgb,[[r,g,b]], axis = 0)
'''