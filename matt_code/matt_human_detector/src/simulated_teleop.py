#!/usr/bin/env python
from __future__ import print_function

import math
from ntpath import join
import n_keyboard as kb
import argparse as ap

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
import numpy as np
# import hello_helpers.hello_misc as hm
import actionlib


class GetKeyboardCommands:
    # Note : Currently Planning services are not compatible with Stretch Gazebo Version
    def __init__(self, mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on,
                 deliver_object_on):
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on

        self.step_size = 'medium'
        # self.persistent_command_count = 0
        # self.prev_persistent_c = None
        # self.persistent_start_s = time.time()
        # self.max_persistent_delay_s = 1.0
        self.rad_per_deg = math.pi / 180.0
        self.small_deg = 3.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.005  # 0.02
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.big_deg = 12.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.06
        self.mode = 'position'  # 'manipulation' #'navigation'

    def get_deltas(self):
        if self.step_size == 'small':
            deltas = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            deltas = {'rad': self.medium_rad, 'translate': self.medium_translate}
        if self.step_size == 'big':
            deltas = {'rad': self.big_rad, 'translate': self.big_translate}
        return deltas

    def print_commands(self, joint_state, command):
        if command is None:
            return

        joints = joint_state.name

        def in_joints(i):
            return len(list(set(i) & set(joints))) > 0

        print('---------- KEYBOARD TELEOP MENU -----------')
        print('                                           ')
        if in_joints(['joint_head_tilt']):
            print('              i HEAD UP                    ')
        if in_joints(['joint_head_pan']):
            print(' j HEAD LEFT            l HEAD RIGHT       ')
        if in_joints(['joint_head_tilt']):
            print('              , HEAD DOWN                  ')
        print('                                           ')
        print('                                           ')
        print(' 7 BASE ROTATE LEFT     9 BASE ROTATE RIGHT')
        print(' home                   page-up            ')
        print('                                           ')
        print('                                           ')
        if in_joints(['joint_lift']):
            print('              8 LIFT UP                    ')
            print('              up-arrow                     ')
        print(' 4 BASE FORWARD         6 BASE BACK        ')
        print(' left-arrow             right-arrow        ')
        if in_joints(['joint_lift']):
            print('              2 LIFT DOWN                  ')
            print('              down-arrow                   ')
        print('                                           ')
        print('                                           ')
        if in_joints(['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']):
            print('              w ARM OUT                    ')
        if in_joints(['joint_wrist_yaw']):
            print(' a WRIST FORWARD        d WRIST BACK       ')
        if in_joints(['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']):
            print('              x ARM IN                     ')
        if in_joints(['joint_wrist_pitch', 'joint_wrist_roll']):
            print('                                           ')
            print('                                           ')
        if in_joints(['joint_wrist_pitch']):
            print(' c PITCH FORWARD        v PITCH BACK       ')
        if in_joints(['joint_wrist_roll']):
            print(' o ROLL FORWARD         p ROLL BACK        ')
        print('                                           ')
        print('                                           ')
        if in_joints(['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']):
            print('              5 GRIPPER CLOSE              ')
            print('              0 GRIPPER OPEN               ')
        print('                                           ')
        print('  step size:  b BIG, m MEDIUM, s SMALL     ')
        print('                                           ')
        print('              q QUIT                       ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self, node):
        command = None

        c = kb.getch()
        # rospy.loginfo('c =', c)

        ####################################################
        ## MOSTLY MAPPING RELATED CAPABILITIES
        ## (There are non-mapping outliers.)
        ####################################################

        # Sequential performs a fixed number of autonomus mapping iterations
        if (c == '!') and self.mapping_on:
            number_iterations = 4
            for n in range(number_iterations):
                # Trigger a 3D scan with the D435i
                trigger_request = TriggerRequest()
                trigger_result = node.trigger_head_scan_service(trigger_request)
                rospy.loginfo('trigger_result = {0}'.format(trigger_result))

                # Trigger driving the robot to the estimated next best place to scan
                trigger_request = TriggerRequest()
                trigger_result = node.trigger_drive_to_scan_service(trigger_request)
                rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger localizing the robot to a new pose anywhere on the current map
        if ((c == '+') or (c == '=')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_global_localization_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger localizing the robot to a new pose that is near its current pose on the map
        if ((c == '-') or (c == '_')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_local_localization_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger driving the robot to the estimated next best place to perform a 3D scan
        if ((c == '\\') or (c == '|')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_drive_to_scan_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger performing a 3D scan using the D435i
        if (c == ' ') and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_head_scan_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger rotating the mobile base to align with the nearest 3D cliff detected visually
        if ((c == '[') or (c == '{')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_align_with_nearest_cliff_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # DEPRECATED: Trigger extend arm until contact
        if ((c == ']') or (c == '}')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_reach_until_contact_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # DEPRECATED: Trigger lower arm until contact
        if ((c == ':') or (c == ';')) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_lower_until_contact_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        ####################################################
        ## OTHER CAPABILITIES
        ####################################################

        # Trigger Hello World whiteboard writing demo
        if ((c == '`') or (c == '~')) and self.hello_world_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_write_hello_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger open drawer demo with downward hook motion
        if ((c == 'z') or (c == 'Z')) and self.open_drawer_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_open_drawer_down_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger open drawer demo with upward hook motion
        if ((c == '.') or (c == '>')) and self.open_drawer_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_open_drawer_up_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger clean surface demo
        if ((c == '/') or (c == '?')) and self.clean_surface_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_clean_surface_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger grasp object demo
        if ((c == '\'') or (c == '\"')) and self.grasp_object_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_grasp_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Trigger deliver object demo
        if ((c == 'y') or (c == 'Y')) and self.deliver_object_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_deliver_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        ####################################################
        ## BASIC KEYBOARD TELEOPERATION COMMANDS
        ####################################################

        # 8 or up arrow
        if c == '8' or c == '\x1b[A':
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        # 2 or down arrow
        if c == '2' or c == '\x1b[B':
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}
        if self.mode == 'manipulation':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'joint_mobile_base_translation', 'delta': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'joint_mobile_base_translation', 'delta': -self.get_deltas()['translate']}
        elif self.mode == 'position':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'translate_mobile_base', 'inc': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_deltas()['translate']}
            # 1 or end key
            if c == '7' or c == '\x1b[H':
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_deltas()['rad']}
            # 3 or pg down 5~
            if c == '9' or c == '\x1b[5':
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_deltas()['rad']}
        elif self.mode == 'navigation':
            rospy.loginfo('ERROR: Navigation mode is not currently supported.')

        if c == 'w' or c == 'W':
            command = {'joint': 'wrist_extension', 'delta': self.get_deltas()['translate']}
        if c == 'x' or c == 'X':
            command = {'joint': 'wrist_extension', 'delta': -self.get_deltas()['translate']}
        if c == 'd' or c == 'D':
            command = {'joint': 'joint_wrist_yaw', 'delta': -self.get_deltas()['rad']}
        if c == 'a' or c == 'A':
            command = {'joint': 'joint_wrist_yaw', 'delta': self.get_deltas()['rad']}
        if c == 'v' or c == 'V':
            command = {'joint': 'joint_wrist_pitch', 'delta': -self.get_deltas()['rad']}
        if c == 'c' or c == 'C':
            command = {'joint': 'joint_wrist_pitch', 'delta': self.get_deltas()['rad']}
        if c == 'p' or c == 'P':
            command = {'joint': 'joint_wrist_roll', 'delta': -self.get_deltas()['rad']}
        if c == 'o' or c == 'O':
            command = {'joint': 'joint_wrist_roll', 'delta': self.get_deltas()['rad']}
        if c == '5' or c == '\x1b[E' or c == 'g' or c == 'G':
            # grasp
            command = {'joint': 'joint_gripper_finger_left', 'delta': -self.get_deltas()['rad']}
        if c == '0' or c == '\x1b[2' or c == 'r' or c == 'R':
            # release
            command = {'joint': 'joint_gripper_finger_left', 'delta': self.get_deltas()['rad']}
        if c == 'i' or c == 'I':
            command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == ',' or c == '<':
            command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'j' or c == 'J':
            command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == 'l' or c == 'L':
            command = {'joint': 'joint_head_pan', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'b' or c == 'B':
            rospy.loginfo('process_keyboard.py: changing to BIG step size')
            self.step_size = 'big'
        if c == 'm' or c == 'M':
            rospy.loginfo('process_keyboard.py: changing to MEDIUM step size')
            self.step_size = 'medium'
        if c == 's' or c == 'S':
            rospy.loginfo('process_keyboard.py: changing to SMALL step size')
            self.step_size = 'small'
        if c == 'q' or c == 'Q':
            rospy.loginfo('keyboard_teleop exiting...')
            rospy.signal_shutdown('Received quit character (q), so exiting')

        ####################################################

        return command


class KeyboardTeleopNode:

    def __init__(self, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False,
                 grasp_object_on=False, deliver_object_on=False):
        # hm.HelloNode.__init__(self)
        self.keys = GetKeyboardCommands(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on,
                                        deliver_object_on)
        self.rate = 10.0
        self.joint_state = None
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on
        self.twist = Twist()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def send_command(self, command, trajectory_client = actionlib.SimpleActionClient('/stretch_head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)):
        joint_state = self.joint_state
        self.trajectory_client_selector(command)
        if (joint_state is not None) and (command is not None):
            if 'translate_mobile_base' == command['joint'] or 'rotate_mobile_base' == command['joint']:
                self.cmd_vel_pub.publish(self.twist)
                return 0
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
            self.trajectory_client.send_goal(trajectory_goal)

    def trajectory_client_selector(self, command):
        self.trajectory_client = None
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        try:
            joint = command['joint']
            if joint == 'joint_lift' or joint == 'joint_wrist_yaw' or joint == 'wrist_extension':
                self.trajectory_client = self.trajectory_arm_client
            if joint == 'joint_head_pan' or joint == 'joint_head_tilt':
                self.trajectory_client = self.trajectory_head_client
            if joint == 'joint_gripper_finger_right' or joint == 'joint_gripper_finger_left':
                self.trajectory_client = self.trajectory_gripper_client
            if joint == 'translate_mobile_base' or joint == 'rotate_mobile_base':
                if joint == 'translate_mobile_base':
                    if 'inc' in command:
                        self.twist.linear.x = command['inc']
                    else:
                        self.twist.linear.x = command['delta']
                else:
                    if 'inc' in command:
                        self.twist.angular.z = command['inc']
                    else:
                        self.twist.angular.z = command['delta']
        except TypeError:
            0

    def main(self):
        rospy.init_node('keyboard_teleop_gazebo')
        # hm.HelloNode.main(self, 'keyboard_teleop', 'keyboard_teleop', wait_for_first_pointcloud=False)

        self.trajectory_gripper_client = actionlib.SimpleActionClient(
            '/stretch_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.trajectory_gripper_client.wait_for_server(timeout=rospy.Duration(60.0))

        self.trajectory_head_client = actionlib.SimpleActionClient('/stretch_head_controller/follow_joint_trajectory',
                                                                   FollowJointTrajectoryAction)
        server_reached = self.trajectory_head_client.wait_for_server(timeout=rospy.Duration(60.0))

        self.trajectory_arm_client = actionlib.SimpleActionClient('/stretch_arm_controller/follow_joint_trajectory',
                                                                  FollowJointTrajectoryAction)
        server_reached = self.trajectory_arm_client.wait_for_server(timeout=rospy.Duration(60.0))

        self.cmd_vel_pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)

        if self.mapping_on:
            rospy.loginfo('Node ' + self.node_name + ' waiting to connect to /funmap/trigger_head_scan.')

            rospy.wait_for_service('/funmap/trigger_head_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_head_scan.')
            self.trigger_head_scan_service = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_drive_to_scan.')
            self.trigger_drive_to_scan_service = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_global_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_global_localization.')
            self.trigger_global_localization_service = rospy.ServiceProxy('/funmap/trigger_global_localization',
                                                                          Trigger)

            rospy.wait_for_service('/funmap/trigger_local_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_local_localization.')
            self.trigger_local_localization_service = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')
            self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy(
                '/funmap/trigger_align_with_nearest_cliff', Trigger)

            rospy.wait_for_service('/funmap/trigger_reach_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
            self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact',
                                                                          Trigger)

            rospy.wait_for_service('/funmap/trigger_lower_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
            self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact',
                                                                          Trigger)

        if self.hello_world_on:
            rospy.wait_for_service('/hello_world/trigger_write_hello')
            rospy.loginfo('Node ' + self.node_name + ' connected to /hello_world/trigger_write_hello.')
            self.trigger_write_hello_service = rospy.ServiceProxy('/hello_world/trigger_write_hello', Trigger)

        if self.open_drawer_on:
            rospy.wait_for_service('/open_drawer/trigger_open_drawer_down')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_down.')
            self.trigger_open_drawer_down_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_down', Trigger)

            rospy.wait_for_service('/open_drawer/trigger_open_drawer_up')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_up.')
            self.trigger_open_drawer_up_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_up', Trigger)

        if self.clean_surface_on:
            rospy.wait_for_service('/clean_surface/trigger_clean_surface')
            rospy.loginfo('Node ' + self.node_name + ' connected to /clean_surface/trigger_clean_surface.')
            self.trigger_clean_surface_service = rospy.ServiceProxy('/clean_surface/trigger_clean_surface', Trigger)

        if self.grasp_object_on:
            rospy.wait_for_service('/grasp_object/trigger_grasp_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /grasp_object/trigger_grasp_object.')
            self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

        if self.deliver_object_on:
            rospy.wait_for_service('/deliver_object/trigger_deliver_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /deliver_object/trigger_deliver_object.')
            self.trigger_deliver_object_service = rospy.ServiceProxy('/deliver_object/trigger_deliver_object', Trigger)

        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        rate = rospy.Rate(self.rate)

        command = 1  # set equal to not None, so menu is printed out on first loop
        while not rospy.is_shutdown():
            if self.joint_state is not None:
                self.keys.print_commands(self.joint_state, command)
                command = self.keys.get_command(self)
                self.send_command(command)
            rate.sleep()


if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Keyboard teleoperation for stretch.')
        parser.add_argument('--mapping_on', action='store_true',
                            help='Turn on mapping control. For example, the space bar will trigger a head scan. This requires that the mapping node be run (funmap).')
        parser.add_argument('--hello_world_on', action='store_true',
                            help='Enable Hello World writing trigger, which requires connection to the appropriate hello_world service.')
        parser.add_argument('--open_drawer_on', action='store_true',
                            help='Enable Open Drawer trigger, which requires connection to the appropriate open_drawer service.')
        parser.add_argument('--clean_surface_on', action='store_true',
                            help='Enable Clean Surface trigger, which requires connection to the appropriate clean_surface service.')
        parser.add_argument('--grasp_object_on', action='store_true',
                            help='Enable Grasp Object trigger, which requires connection to the appropriate grasp_object service.')
        parser.add_argument('--deliver_object_on', action='store_true',
                            help='Enable Deliver Object trigger, which requires connection to the appropriate deliver_object service.')

        args, unknown = parser.parse_known_args()
        mapping_on = args.mapping_on
        hello_world_on = args.hello_world_on
        open_drawer_on = args.open_drawer_on
        clean_surface_on = args.clean_surface_on
        grasp_object_on = args.grasp_object_on
        deliver_object_on = args.deliver_object_on

        node = KeyboardTeleopNode(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on,
                                  deliver_object_on)
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
