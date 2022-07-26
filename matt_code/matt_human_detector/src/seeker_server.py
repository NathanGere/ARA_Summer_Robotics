#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *
# import os 
# os.popen('python3 home/csrobot/stretch_ws/src/depthai-python//examples/ObjectTracker/object_tracker')
# from object_tracker import *

def seek_response(req):

    print("Now recieving response from object_tracker")
    print(" ")
    # label = label_finder()
    label = "chicken"

    print("According to object_tracker file... label is: " + str(label))
    # print(str(label))
    print(" ")

    if str(label) == "person":
        print("Label is person!")
        print(" ")
        req = True
    else:
        print("Keep trying to find person!")
        print(" ")
        req = False

    print("Bool value is: {} " .format(req))
    # print(bool(req))
    print(" ")

    print("Now returning response")
    print(" ")

    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------")
    print(" ")
    
    return seekerResponse(req)



def call_back():

    print("Sucessfully made seeker_server node")
    print(" ")
    rospy.init_node("seeker_server")

    print("----------------------------")
    print("Reentering Service file")
    print("----------------------------")
    print(" ")

    print("Sucessfully initalized service")
    print(" ")
    server = rospy.Service("seeker", seeker, seek_response)

    print("Now calling seek_response function")
    print(" ")

    rospy.spin()

if __name__ == '__main__':
    print("----------------------------")
    print("Now entering in Service file")
    print("----------------------------")
    print(" ")

    print("Calling call_back function")
    print(" ")
    call_back()