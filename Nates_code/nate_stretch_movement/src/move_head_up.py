#!/usr/bin/env python

import rospy
from time import time
import stretch_body.robot
from stretch_body.hello_utils import ThreadServiceExit

class HeadUpDown:

    #########################################################################################################################################################
    def __init__(self, current_position, desired_position, robot, displays = True):

        self._current_position = current_position
        self._desired_position = desired_position
        self._robot = robot

        '''
        self.trajectory_head_client = actionlib.SimpleActionClient('/stretch_head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.rad_per_deg = math.pi / 180.0
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self._joint_state = JointState()
        self.twist = Twist()
        self.retrieved_joint_state = False
        start_time = time()
        self._seconds = seconds
        '''
        self._first_print = True
        self._displays = displays

        if self._first_print and self._displays:
            self.initial_print()

        self.repo_calculator()

    #########################################################################################################################################################
    def repo_calculator(self):

        if self._desired_position == "forward":
            self.repo_head_forward()

        elif self._desired_position == "down":
            self.repo_head_down()

        else:
            self.error_msg()
    
    #########################################################################################################################################################
    def update_my_behavior(self, status):
        #update the joint commands based on status data
        pass

    #########################################################################################################################################################
    def repo_head_forward(self):

        if self._current_position == "forward" and self._displays:
            print("\n\tHead camera already in desired position.")

        elif self._current_position == "down":
            self.raise_up()

        else:
            self.error_msg() #needs to be in down position to be raised to forward straight position

    #########################################################################################################################################################
    def repo_head_down(self):

        if self._current_position == "down" and self._displays:
            print("/n/t Head camera already in desired position.")

        elif self._current_position == "forward":
            self.move_down()

        else:
            self.error_msg()

    #########################################################################################################################################################
    def raise_up(self):

        self._robot.head.move_by('head_tilt', 1.57)
        self._robot.push_command()

    #########################################################################################################################################################
    def move_down(self):

        self._robot.head.move_by('head_tilt', -1.57)
        self._robot.push_command()

    #########################################################################################################################################################
    def error_msg(self):
        if self._displays:
            print("\n\t\t:: HEAD POSITIONS ERROR ::")
            print("\t Either the current position or desired position is incorrect\n")

    #########################################################################################################################################################
    def initial_print(self):

        print("\n#########################################################################################################")
        print("\n---------------------------------------------------------------------------------------------------------")
        print("\n\t Returning Head Camera to Home Position")
        print("\n---------------------------------------------------------------------------------------------------------")

        self._first_print = False

#############################################################################################################################################################
def main():

    #setting up stretch body
    robot = stretch_body.robot.Robot()
    robot.startup()
    '''
    if not robot.is_calibrated():
        robot.home() #blocking
    robot.stow()
    '''

    try:

        hd = HeadUpDown("down", "forward", robot)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit, ThreadServiceExit):
        pass

    robot.stop()

if __name__ == '__main__':

    #initializing node
    rospy.init_node("move_head_up_node", anonymous = True)

    main()