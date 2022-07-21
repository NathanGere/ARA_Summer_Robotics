#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *

def some_function():
    rospy.wait_for_service("seeker")

    print(" ")
    print("------------------")
    print("Return to Client")
    print("------------------")
    print(" ")

    try:
        seeker_service_caller = rospy.ServiceProxy("seeker", seeker)

        response = seeker_service_caller

        if bool(response) == True:
            print("Found Human!")
        else: 
            print("No human found! Try searching again!")
            sys.exit(1)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    print(" ")
    print("------------------")
    print("End Client")
    print("------------------")

    print(" ")
    print("------------------------------------------------------------------------------------")


if __name__ == '__main__':
    print(" ")
    print("------------------------------------------------------------------------------------")
    print(" ")
    print("-------------")
    print("Start Client")
    print("-------------")
    print("Move to Server")
    print("-------------")

    rospy.init_node("seeker_client")
    # publish = rospy.Publisher("camera_publisher", Bool, queue_size = 10)
    some_function()
    rospy.spin()