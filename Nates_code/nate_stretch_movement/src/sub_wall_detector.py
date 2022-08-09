#!/usr/bin/env python

import rospy
#import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#import ctypes
#import struct
from std_msgs.msg import Header, String, Float32MultiArray
import numpy as np

def sub_cb(y_points):
    if y_points.data == (3000.0, 3000.0):
        print("There are no objects detected")
    else:
        print("\n\n\tUpdated y_points")
        print(y_points.data)
        #print("The size of the array is " + str(y_points.layout.dim.size))

def main():
    rospy.init_node("sub_wall_detector_node")
    cloud_sub = rospy.Subscriber("/pcl/y_points_of_walls", Float32MultiArray, sub_cb, queue_size=1, buff_size=52428800)
    rospy.spin()

if __name__ == '__main__':
    main()