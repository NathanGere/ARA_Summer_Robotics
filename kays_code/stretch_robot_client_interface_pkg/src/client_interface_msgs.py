#! /usr/bin/env python

import rospy
from std_msgs.msg import * # imports all std_msgs
from geometry_msgs.msg import Twist # needed to move the base

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

def __init__(self) :
    
#####################################################################################

def subscriber_callback(self, message) :
    if 'base' in message :
        move_base(message)
        
    elif 'arm' in req :
        move_arm(message)
        
    elif 'lift' in message :
        move_lift(message)
        
    elif 'gripper' in message :
        move_gripper(message)
        
#####################################################################################

def move_base(self, message) :
    base_movement = Twist()
    
    if 'forward' in message :
        base_movement.linear.x = 1
        base_movement.angular.z = 0
        
    elif 'backwards' in message :
        base_movement.linear.x = -1
        base_movement.angular.z = 0
        
    elif 'left' in message :
        base_movement.linear.x = 0
        base_movement.angular.z = 1
        
    else 'right' in message :
        base_movement.linear.x = 0
        base_movement.angular.z = -1
    
    base_pub.publish(base_movement)
    
#####################################################################################

def move_arm(self, message) :
    
    if 'extend' in message :
        command = {'joint': 'wrist_extension', 'delta': 0.1)]}
         
    elif 'collapse' in message :
        command = {'joint': 'wrist_extension', 'delta': -0.1]}
        
    
    send_command(self, command)

#####################################################################################

def move_lift(self, message) :
    
    if 'up' in message :
        command = {'joint': 'joint_lift', 'delta': 0.2]}
        
    elif 'down' in message :
        command = {'joint': 'joint_lift', 'delta': -0.2]}
        
    send_command(self, command)
    
#####################################################################################

def move_gripper(self, message) :
    
    if 'open' in message :
        command = {'joint': 'joint_gripper_finger_left', 'delta': 0.1}
    elif 'close' in message :
        command = {'joint': 'joint_gripper_finger_left', 'delta': 0.1}
    
    send_command(self, command)

#####################################################################################

def joint_states_callback(self, joint_state) :
    self.joint_state = joint_state

#####################################################################################

def send_command(self, command) :
    joint_state = self.joint_state
    if (joint_state is not None) and (command is not None):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            
        joint_name = command['joint']
        trajectory_goal.trajectory.joint_names = [joint_name]
        if 'inc' in command:
            inc = command['inc']
            new_value = inc
        elif 'delta' in command:
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
        point.positions = [new_value]
        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        self.trajectory_client.send_goal(trajectory_goal)

#####################################################################################
    
if __name__ == "__main__":
    rospy.init_node('client_interface_msgs_node') #initializes sserver node
    
    command = None
    obtained_joint_state = None
    
    base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes to stretch base
    
    command_sub = rospy.Subscriber('/client_command', String, subscriber_callback) #subscribes to lidar values
    rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
    
    rospy.spin()
