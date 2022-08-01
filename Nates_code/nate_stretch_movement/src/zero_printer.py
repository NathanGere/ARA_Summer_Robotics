#!/usr/bin/env python

from numpy import indices
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def launchpoint():
    i = 0
    while i < 2000:
        print("0, ")
        i+=1
if __name__ == '__main__':
    launchpoint()