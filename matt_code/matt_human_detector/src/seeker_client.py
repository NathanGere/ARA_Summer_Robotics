#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *
import os

def client_response():
    print("Waiting for response from server\n")
    rospy.wait_for_service("seeker")

    try:

        seeker_service_caller = rospy.ServiceProxy("seeker", seeker)

        response = seeker_service_caller()

        print("Response/{} " .format(response))

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

    print("")
    print("------------------------------------------------------------------------------------")

if __name__ == '__main__':
    print("------------------------------------------------------------------------------------\n")

    print("----------------------------")
    print ("Now in Client File")
    print("----------------------------\n")

    print("Sucessfully made seeker_client node\n")
    rospy.init_node("seeker_client")

    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------\n")

    print("Calling client_response function\n")
    client_response()

    rospy.spin()