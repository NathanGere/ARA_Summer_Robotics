#!/usr/bin/env python

import rospy
from std_msgs.msg import * #imports all std_msgs, needed to send string to client interface
from geometry_msgs.msg import Twist #needed to move the base
from collision_prevention_pkg.srv import * #includes service

def execute_escape_collision(req):

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes to stretch base
    client_interface_pub = rospy.Publisher('/client_warning', String, queue_size=10) #publishes a warning to the client interface
    movement = Twist() #initializes the variable movement to allow for Twist commands 

    if req.index >=0:
        movement.linear.x = 0
        movement.angular.z = 0
        pub.publish(movement)

        client_interface_pub.publish('To Avoid Collision, Stretch Was Stopped')

        return collision_prevention_serviceResponse() #returns nothing but is needed

def escape_collision_server():
    rospy.init_node('escaping_collision_server') #initializes sserver node
    service = rospy.Service('collision_prevention_service', collision_prevention_service, execute_escape_collision) #initalizes service
    rospy.spin()

if __name__ == "__main__":
    escape_collision_server()
