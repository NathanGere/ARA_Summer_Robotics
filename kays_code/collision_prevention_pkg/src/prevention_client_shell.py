#!/usr/bin/env python

import rospy #always need
import sys #needed for exit
from std_msgs.msg import * #imports all std_msgs, not sure if its important but it works
from geometry_msgs.msg import Twist #needed to move the base
from sensor_msgs.msg import LaserScan #needed for lidar
from collision_prevention_pkg.srv import * #includes service

def driver_callback(data):
    movement = Twist() #initializes the variable movement to allow for Twist commands 
    movement.linear.x = 1
    pub.publish(movement)
    #previous rows are just for testing. Add in your own code
    
    index = 0
    while index < 1999: #is checking all every index for the lidar, acts as a for loop
        if data.ranges[index] < 0.5:
            rospy.wait_for_service('collision_prevention_service') #waits for collision service to respond (not 100% sure if this is needed)
            try:
                prevention_service_call = rospy.ServiceProxy('collision_prevention_service', collision_prevention_service)
                answer = prevention_service_call(index)
                return answer # answer is empty
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e) #if service isn't running this will pop up
                sys.exit(1)

        index+=1

if __name__ == '__main__':
    
    laser_scan_topic = rospy.get_param("laser_scan_param") # default is for gazebo
    base_cmd_vel_topic = rospy.get_param("base_cmd_vel_param")

    rospy.init_node('collision_node',anonymous=True)
    sub = rospy.Subscriber(laser_scan_topic, LaserScan, driver_callback) #subscribes to lidar values
    pub = rospy.Publisher(base_cmd_vel_topic, Twist, queue_size=10) #publishes to stretch base
 
    rospy.spin() #This is needed