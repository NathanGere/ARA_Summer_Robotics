#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import *
from matt_human_detector.srv import *
import time

def checker(label):
    while not rospy.is_shutdown():
        if label == "":
            with open("/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/matt_code/matt_human_detector/txt/read.txt", "r") as fp:
                label=""
                for line in fp: 
                    stripped_line = line.strip()
                    label += stripped_line
            print("Awaiting Response...")
            time.sleep(5)
        else:
            return label

def seek_response(req):

    input = False

    print("Now calling checker function\n")

    label = ""

    label = checker(label)

    with open("/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/matt_code/matt_human_detector/txt/read.txt", "w+") as fp:
        pass

    print("According to object_tracker file... label is: " + str(label))
    print("")

    if str(label) == "person":
        print("Label is person!\n")
        req = True
    else:
        print("Keep trying to find person!\n")
        req = False

    print("Bool value is: {}" .format(req))
    print("")

    print("Now returning response\n")


    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------\n")
    
    return seekerResponse(req)


def call_back():

    print("Sucessfully made seeker_server node\n")
    rospy.init_node("lidar_seeker_server")

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