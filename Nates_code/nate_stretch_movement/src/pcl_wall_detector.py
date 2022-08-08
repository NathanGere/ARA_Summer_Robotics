#!/usr/bin/env python

from calendar import c
import rospy
#import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#import ctypes
#import struct
from std_msgs.msg import Header, String, Float32
import numpy as np
   
def xyz_cb(cloud_msg):

        pointCloud = cloud_msg
        y_points = []
        #print("\n\n\n\n\n\tXYZ POINTS")
        objects_detected = False
        for point in pc2.read_points(pointCloud, field_names=pointCloud.data, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            #print("x: " + str(pt_x) + " y: " + str(pt_y) + " z: " + str(pt_z))
            if -0.4 <= pt_x and 0.0 >= pt_x:
                if pt_z <= 1.25:
                    y_points.append(pt_y)
                    objects_detected = True
                elif pt_z >= 1.3:
                    y_points.append(pt_y)
                    objects_detected = True

        #print("\n")
        #print(y_points)
        
        #tuple_y_pts = convert_to_tuple(y_points)
        pub = rospy.Publisher("/pcl/y_points_of_walls", Float32, queue_size = 10)
        if objects_detected:
            pub.publish(y_points)
        else:
            pub.publish([3000, 3000])
    

def convert_to_tuple(list):
            return tuple(i for i in list)

def main():
    rospy.init_node("pcl_wall_detector_node")
    cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, xyz_cb, queue_size=1, buff_size=52428800)
    rospy.spin()

if __name__ == '__main__':
    main()