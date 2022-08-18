#!/usr/bin/env python

#from __future__ import print_function
import rospy
import math
#from ntpath import join
from std_msgs.msg import String
#from altered_sim_teleop import KeyboardTeleopNode, GetKeyboardCommands
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
import numpy as np
import actionlib
from time import time

#############################################################################################################################################################
class SimHeadUpDown:

    #########################################################################################################################################################
    def __init__(self, current_position = "forward", desired_position = "down"):

        self._current_position = current_position
        self._desired_position = desired_position
        self.trajectory_head_client = actionlib.SimpleActionClient('/stretch_head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.rad_per_deg = math.pi / 180.0
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self._joint_state = JointState()
        self.twist = Twist()
        self.retrieved_joint_state = False
        #server_reached = self.trajectory_head_client.wait_for_server(timeout=rospy.Duration(60.0))
        start_time = time()
        seconds = 4
        while not rospy.is_shutdown():
            current_time = time()
            elapsed_time = current_time - start_time
            if self.retrieved_joint_state:
                self.repo_calculator()
            if elapsed_time > seconds:
                print("finished movement")
                #print(str(elapsed_time))
                break
       
        #deltas = {'rad': self.medium_rad, 'translate': self.medium_translate}
        #self._sim_teleop = KeyboardTeleopNode()
        #self._kb_commands = GetKeyboardCommands(False, False, False, False, False, False)

    #########################################################################################################################################################
    def repo_calculator(self):

        if self._desired_position == "forward":
            self.repo_head_forward()

        elif self._desired_position == "down":
            self.repo_head_down()

        else:
            self.error_msg()
    
    #########################################################################################################################################################
    def joint_states_callback(self, joint_state):
        self._joint_state = joint_state
        self.retrieved_joint_state = True

    #########################################################################################################################################################
    def get_deltas(self):

        deltas = {'rad': self.medium_rad, 'translate': self.medium_translate}
        return deltas

    #########################################################################################################################################################
    def repo_head_forward(self):

        if self._current_position == "forward":
            print("\n\tHead camera already in desired position.")

        elif self._current_position == "down":
            self.raise_up()

        else:
            self.error_msg() #needs to be in down position to be raised to forward straight position

    
    #########################################################################################################################################################
    def repo_head_down(self):

        if self._current_position == "down":
            print("/n/t Head camera already in desired position.")

        elif self._current_position == "forward":
            self.move_down()

        else:
            self.error_msg()

    #########################################################################################################################################################
    def raise_up(self):
        command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        #command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        #self._sim_teleop.send_command(command)
        self.send_command(command)

    #########################################################################################################################################################
    def move_down(self):
        command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        #self._sim_teleop.send_command(command)
        self.send_command(command)

    #########################################################################################################################################################
    def send_command(self, command):

        joint_state = self._joint_state
        #self.trajectory_client_selector(command)
        '''
        if (joint_state is not None) and (command is not None):
            if 'translate_mobile_base' == command['joint'] or 'rotate_mobile_base' == command['joint']:
                self.cmd_vel_pub.publish(self.twist)
                return 0
        '''

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.2)
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

        joint_name = command['joint']

        if joint_name in ['joint_lift', 'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt']:
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
        elif joint_name in ["joint_gripper_finger_left", "wrist_extension"]:
            if joint_name == "joint_gripper_finger_left":
                trajectory_goal.trajectory.joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right']
            else:
                trajectory_goal.trajectory.joint_names = ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
            positions = []
            for j_name in trajectory_goal.trajectory.joint_names:
                joint_index = joint_state.name.index(j_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta/len(trajectory_goal.trajectory.joint_names)
                positions.append(new_value)
            point.positions = positions

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        self.trajectory_head_client.send_goal(trajectory_goal)

    
    #########################################################################################################################################################
    def error_msg(self):
        print("\n\t\t:: HEAD POSITIONS ERROR ::")
        print("\t Either the current position or desired position is incorrect\n")

#############################################################################################################################################################
def middle_man():

    hp = SimHeadUpDown(current_position = "forward", desired_position = "down")

#############################################################################################################################################################

    # MAIN #

#############################################################################################################################################################
if __name__ == '__main__':
    #setting up node
    rospy.init_node("sim_head_down_node", anonymous = True)

    #rospy.get_param("/stretch_head_controller/joints")
    try:

        #hp = SimHeadUpDown(current_position = "down", desired_position = "forward")
        hp = SimHeadUpDown(current_position = "down", desired_position = "forward")
        #middle_man()

    except rospy.ROSInterruptException:
        pass