#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from matt_human_detector.srv import *
# from object_tracker import * 
# import os 
# os.system('python3 home/csrobot/stretch_ws/src/depthai-python/examples/ObjectTracker/object_tracker.py')

def seek_response(req):

    print("Checkpoint 4/5")

    # label = os.popen('python3 home/csrobot/stretch_ws/src/depthai-python//examples/ObjectTracker/object_tracker')
    label = "chicken"
    print("Checkpoint 5/5")

    print(str(label))

    if str(label) == "person":
        print("Label is person!")
        req.output = True
    else:
        print("Keep trying to find person!")
        req.output = False

    return seekerResponse(req.output)

def call_back():
    rospy.init_node("seeker_server")
    print("Checkpoint 1/5")
    server = rospy.Service("seeker", seeker, seek_response)
    print("Checkpoint 2/5")
    rospy.spin()
    print("Checkpoint 3/5")

if __name__ == '__main__':
    print("Start Server")
    print("-------------")
    print(" ")

    call_back()