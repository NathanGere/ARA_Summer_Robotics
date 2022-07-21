#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *

def client_response():
    print("Waiting for response from server")
    print(" ")
    rospy.wait_for_service("seeker")

    try:

        seeker_service_caller = rospy.ServiceProxy("seeker", seeker)

        response = seeker_service_caller()

        print("Response/{} " .format(response))

        if response == True:
            print("Found person!")
        else: 
            print("No person found! Try searching again!")

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)


    print(" ")
    print("------------------------------------------------------------------------------------")


if __name__ == '__main__':
    print("------------------------------------------------------------------------------------")
    print(" ")
    print("----------------------------")
    print("Starting search for person!")
    print("----------------------------")
    print(" ")

    print("----------------------------")
    print ("Now in Client Flie")
    print("----------------------------")
    print(" ")

    print("Sucessfully made seeker_client node")
    print(" ")
    rospy.init_node("seeker_client")

    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------")
    print(" ")

    print("Calling client_response function")
    print(" ")
    client_response()

    rospy.spin()