#!/usr/bin/env python

from calendar import c
from io import StringIO
import rospy
#import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#import ctypes
#import struct
from std_msgs.msg import Header, String, Float32
import numpy as np
   
def sub_cb(y_points):

    print(y_points)

def main():
    rospy.init_node("sub_wall_detector_node")
    cloud_sub = rospy.Subscriber("/pcl/y_points_of_walls", Float32, sub_cb, queue_size=1, buff_size=52428800)
    rospy.spin()

if __name__ == '__main__':
    main()