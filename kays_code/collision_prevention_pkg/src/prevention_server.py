#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist #needed to move the base
from collision_prevention_pkg.srv import * #includes service

def execute_escape_collision(req):
    print("executing stop") #indicates service was entered, not needed
    pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10) #initializes publisher for base !!for gazebo!!
    
    movement = Twist() #initializes the 

    if req.index >= 0: #here add your own code for what you want it to do depending on the index it is given, this was created for testing
        movement.linear.x = 0
        movement.angular.z = 0
        pub.publish(movement)
    
    print("stopped") #indicates that the service completed its actions, not needed
    return collision_prevention_serviceResponse() #returns nothing but is needed
    

def escape_collision_server():
    rospy.init_node('escaping_collision_server') #initializes sserver node
    service = rospy.Service('collision_prevention_service', collision_prevention_service, execute_escape_collision) #initalizes service
    print("Ready to stop collision when needed") #prints to tell you service is running, not needed
    rospy.spin()

if __name__ == "__main__":
    escape_collision_server()
