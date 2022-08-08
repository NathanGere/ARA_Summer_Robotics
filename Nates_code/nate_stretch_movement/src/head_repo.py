#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from altered_sim_teleop import KeyboardTeleopNode, GetKeyboardCommands

#############################################################################################################################################################
class HeadPositions:

    #########################################################################################################################################################
    def __init__(self, current_position = "forward_straight", desired_position = "forward_down"):
        self._current_position = current_position
        self._desired_position = desired_position
        self._sim_teleop = KeyboardTeleopNode()
        self._kb_commands = GetKeyboardCommands(False, False, False, False, False, False)

    #########################################################################################################################################################
    def repo_calculator(self):

        if self._desired_position == "right_down":
            self.repo_right_down()

        elif self._desired_position == "left_down":
            self.repo_left_down()

        elif self._desired_position == "forward_straight":
            self.repo_forward_straight()

        elif self._desired_position == "forward_down":
            self.repo_forward_down()

        else:
            self.error_msg()
        
    #########################################################################################################################################################
    def repo_forward_straight(self):

        if self._current_position == "forward_straight":
            print("\n\tHead camera already in desired position.")

        elif self._current_position != "forward_down":
            self.raise_up()

        else:
            self.repo_forward_down() #needs to be in forward_down position to be raised to forward straight position
            self.raise_up()
    
    #########################################################################################################################################################
    def repo_forward_down(self):

        if self._current_position == "forward_down":
            print("/n/t Head camera already in desired position.")

        elif self._current_position == "right_down":
            self.move_left()

        elif self._current_position == "left_down":
            self.move_right()

        elif self._current_position == "forward_straight":
            self.move_down()

        else:
            self.error_msg()
        
    #########################################################################################################################################################
    def repo_right_down(self):
        
        if self._current_position == "right_down":
            print("\n\tHead camera already in desired position.")

        elif self._current_position != "forward_down":
            self.move_right()

        else:
            self.repo_forward_down() #needs to be in forward_down position to be moved to right down position
            self.move_right()

    #########################################################################################################################################################
    def repo_left_down(self):

        if self._current_position == "left_down":
            print("\n\tHead camera already in desired position.")

        elif self._current_position != "forward_down":
            self.move_left()

        else:
            self.repo_forward_down() #needs to be in forward_down position to be moved to left down position
            self.move_left()

    #########################################################################################################################################################
    def raise_up(self):
        command = {'joint': 'joint_head_tilt', 'delta': (16.0 * self._kb_commands.get_deltas()['rad'])}
        self._sim_teleop.send_command(command)

    #########################################################################################################################################################
    def move_down(self):
        command = {'joint': 'joint_head_tilt', 'delta': -(16.0 * self._kb_commands.get_deltas()['rad'])}
        
        self._sim_teleop.send_command(command)

    #########################################################################################################################################################
    def move_left(self):
        command = {'joint': 'joint_head_pan', 'delta': (8.0 * self._kb_commands.get_deltas()['rad'])}
        self._sim_teleop.send_command(command)

    #########################################################################################################################################################
    def move_right(self):
        command = {'joint': 'joint_head_pan', 'delta': -(8.0 * self._kb_commands.get_deltas()['rad'])}
        self._sim_teleop.send_command(command)
    
    #########################################################################################################################################################
    def error_msg():
        print("\n\t\t:: HEAD POSITIONS ERROR ::")
        print("\t Either the current position or desired position is incorrect\n")

#############################################################################################################################################################
def middle_man():

    hp = HeadPositions(current_position = "forward_straight", desired_position = "forward_down")

    hp.repo_calculator()

#############################################################################################################################################################

    # MAIN #

#############################################################################################################################################################
if __name__ == '__main__':
    try:
        #setting up node
        rospy.init_node("wall_follower_python_node", anonymous =True)

        middle_man()

    except rospy.ROSInterruptException:
        pass