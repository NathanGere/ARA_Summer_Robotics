#!/usr/bin/env python

from logging import shutdown
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

        return response

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

if __name__ == '__main__':
    m_bool = False
    
    print("------------------------------------------------------------------------------------\n")

    print("----------------------------")
    print ("Now in Client File")
    print("----------------------------\n")

    print("Sucessfully made seeker_client node\n")
    rospy.init_node("pcl_seeker_client")

    print("----------------------------")
    print("Reentering Client file")
    print("----------------------------\n")

    print("Calling client_response function\n")
    

    pub = rospy.Publisher('talker', Bool, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and m_bool == False:
        pub.publish(m_bool)
        m_bool = client_response()
        rate.sleep()

    pub.publish(True)

    print("")
    print("------------------------------------------------------------------------------------")