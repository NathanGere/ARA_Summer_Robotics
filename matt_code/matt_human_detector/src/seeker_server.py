#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *
import os
import os.path

def seek_response(req):

    label = "cat"

    # ffiillee = os.path.exists('/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/matt_code/matt_human_detector/src/read.txt')
    # print(ffiillee)

    with open("/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/matt_code/matt_human_detector/src/read.txt", "r") as fp:
        label = fp.read()
    
    # if llabel == "person":
        # label = llabel

    # print("According to object_tracker file... llabel is: " + str(llabel))
    print("According to object_tracker file... label is: " + str(label))
    print("")
    # checking if label returned human, if so retuen true otherwise return false
    # print(str(label))

    if str(label) == "person":
        print("Label is person!\n")
        req = True
    else:
        print("Keep trying to find person!\n")
        req = False

    print("Bool value is: {}" .format(req))
    print("")
    # check if the correct boolean value is going to be retuerned the client
    # print(bool(req))

    print("Now returning response\n")


    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------\n")
    
    return seekerResponse(req)


def call_back():

    print("Sucessfully made seeker_server node\n")
    rospy.init_node("seeker_server")

    print("----------------------------")
    print("Reentering Service file")
    print("----------------------------\n")

    print("Sucessfully initalized service\n")
    server = rospy.Service("seeker", seeker, seek_response)

    print("Now calling seek_response function\n")

    rospy.spin()


if __name__ == '__main__':
    print("----------------------------")
    print("Now entering in Service file")
    print("----------------------------\n")

    print("Calling call_back function\n")

    call_back()