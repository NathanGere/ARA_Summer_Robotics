#! /usr/bin/env python

import rospy
from std_msgs.msg import * # imports all std_msgs
from geometry_msgs.msg import Twist # needed to move the base

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

state = None
#####################################################################################

def joint_states_callback(joint_state_msg) :
    global state
    state = joint_state_msg    

#####################################################################################

def subscriber_callback(given_string) :
    message = given_string.data
    
    if 'base' in message :
        move_base(message)
        
    elif 'arm' in message :
        move_arm(message)
        
    elif 'lift' in message :
        move_lift(message)
        
    elif 'gripper' in message :
        move_gripper(message)

    elif 'wrist' in message :
        move_wrist(message)
        
#####################################################################################

def move_base(message) :
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
        
    elif 'right' in message :
        base_movement.linear.x = 0
        base_movement.angular.z = -1
    
    base_pub.publish(base_movement)
    
#####################################################################################

def move_arm(message) :
    endclient = arm_trajectoryClient

    if 'extend' in message :
        command = {'joint': 'joint_arm_l0', 'delta': 0.1}
         
    elif 'collapse' in message :
        command = {'joint': 'joint_arm_l0', 'delta': -0.1}   
    
    send_command(endclient, command)

#####################################################################################

def move_lift(message) :
    endclient = arm_trajectoryClient
    if 'up' in message :
        command = {'joint': 'joint_lift', 'delta': 0.2}
    elif 'down' in message :
        command = {'joint': 'joint_lift', 'delta': -0.2}
        
    send_command(endclient, command)
    
#####################################################################################

def move_gripper(message) :
    endclient = gripper_trajectoryClient

    if 'open' in message :
        command = {'joint': 'joint_gripper_finger_left', 'delta': 0.1}
    elif 'close' in message :
        command = {'joint': 'joint_gripper_finger_left', 'delta': -0.1}
    
    send_command(endclient, command)

#####################################################################################

def move_wrist(message) :
    endclient = arm_trajectoryClient

    if 'left' in message :
        command = {'joint': 'joint_wrist_yaw', 'delta': 0.2}
    elif 'right' in message :
        command = {'joint': 'joint_wrist_yaw', 'delta': -0.2}

    send_command(endclient, command)

#####################################################################################

def send_command(endclient, command) :

    joint_state = state

    if (joint_state is not None) and (command is not None):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.1)
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
        trajectory_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        endclient.send_goal(trajectory_goal)

#####################################################################################

if __name__ == "__main__":
    rospy.init_node('client_interface_msgs_node') #initializes server node
    
    command = None
    
    base_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes to stretch base
    
    command_sub = rospy.Subscriber('/client_command', String, subscriber_callback) #subscribes to lidar values
    joint_state_sub = rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    arm_trajectoryClient = actionlib.ActionClient('/stretch_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_trajectoryClient = actionlib.ActionClient('/stretch_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.spin()
