#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import time
import os
import stretch_body.robot
from stretch_body.hello_utils import ThreadServiceExit

#############################################################################################################################################################
class StretchWallFollower:
    
    #########################################################################################################################################################
    def __init__(self, twist_pub = rospy.Publisher("/stretch//cmd_vel", Twist, queue_size=10), displays = True, escape_loop_on = False, 
                                                forward_speed = 0.15, max_turning_speed = -0.3, min_turning_speed_right = -0.15, min_turning_speed_left = 0.15):
        self._pub = twist_pub #wall follower publisher
        self._displays = displays
        self._escape_loop_on = escape_loop_on
        self.motor_cmd = Twist() #motor command to be published in multiple methods
        #class attribute for intial print statements
        self.first_print = True
        self.first_orient = True
        self.first_distance = True
        self.first_align = True
        #class attributes for initial setup of robot's orientation and position in relation to the nearest wall
        self.oriented = False
        self.close_enough = False
        self.aligned = False
        #class attributes for explorer method
        self.reset = 0
        #class attributes for crash recovery method
        self.case_5_tracker = 0
        self.case_7_tracker = 0
        self.case_10_tracker = 0
        self.case_11_tracker = 0
        self.case_13_tracker = 0
        self.case_14_tracker = 0
        self.case_15_tracker = 0
        #class attributes for whether the values have been updated
        self.scan_updated = False
        self.cloud_updated = False
        #class attributes for parameters
        self._forward_speed = forward_speed
        self._max_turning_speed = max_turning_speed
        self._min_turning_speed_right = min_turning_speed_right
        self._min_turning_speed_left = min_turning_speed_left
        #class attribute for type of movement last made
        self.last_movement_made = "none"
        self.current_movement = "none"
        self.num_of_consecutive_right_turns = 0
        self.num_of_consecutive_left_turns = 0

        #retrieving values for wall follower calculations
        self.cloud_sub = rospy.Subscriber("/stretch_pcl_for_nav/points", PointCloud2, self.cloud_cb, queue_size = 1, buff_size = 52428800)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self._scan = LaserScan()
        self._cloud = PointCloud2()

        #actually calling methods in the class
        while not rospy.is_shutdown():
            if self.scan_updated and self.cloud_updated:
                self.setup_wall_follower()

    #########################################################################################################################################################
    def setup_wall_follower(self):

        #stopping robot just in case
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        y_points, x_points, num_of_y_points = self.cloud_calculations()

        self.follow_walls_please(y_points, x_points, num_of_y_points)

    #########################################################################################################################################################
    def scan_cb(self, scan):
        self._scan = scan
        self.scan_updated = True

    #########################################################################################################################################################
    def cloud_cb(self, cloud):
        self._cloud = cloud
        self.cloud_updated = True

    #########################################################################################################################################################
    def cloud_calculations(self):
        
        pointCloud = self._cloud
        #setting up storage of areas where objects are detected
        y_points = []
        x_points = []
        num_of_y_points = 0
        #print("\n\n\n\n\n\tXYZ POINTS")
        #self.pcl_objects_detected = False
        
        for point in pc2.read_points(pointCloud, field_names=pointCloud.data, skip_nans=True):

            if len(point) == 3:
                pt_x = float(point[0])
                pt_y = float(point[1])
                pt_z = float(point[2])
                #print("x: " + str(pt_x) + " y: " + str(pt_y) + " z: " + str(pt_z))

                hypotenuse = np.sqrt((pt_x * pt_x) + (pt_y * pt_y))
                if hypotenuse < 0.4:
                    if pt_z <= 1.25:
                        y_points.append(pt_y)
                        x_points.append(pt_x)
                        num_of_y_points+=1
                        #self.pcl_objects_detected = True

                    elif pt_z >= 1.3:

                        y_points.append(pt_y)
                        x_points.append(pt_x)
                        num_of_y_points+=1
                        #self.pcl_objects_detected = True
        
        #print("Number of y_pts: " + str(num_of_y_points))
        return y_points, x_points, num_of_y_points

    #########################################################################################################################################################
    def follow_walls_please(self, y_points, x_points, num_of_y_points):

        if self.first_print:
            self.initial_print()
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #variable for size of laser self._scan
        size = len(self._scan.ranges)

        #conditions to trigger explorer and crash recovery methods
        walls_nearby = False
        crashed = False

        #trackers for nearby walls
        wall_on_left = False
        wall_on_right = False
        wall_on_front_right = False
        wall_on_front_left = False

        #condtion for turning left or right
        right_turn = False

        #condition for moving straight forward
        forward_cone = True

        #locations of and distances to nearest wall
        index_mem = 0
        distance_mem = 0.0

        #will remeber locations of crashes to make best possible recovery assuming robot is still upright
        indices_of_collisions = [3000] * 2000

        #calculating num of consecutive turns
        if self._escape_loop_on:
            self.consecutive_turns_tracker()

        #calculating variable values
        distance_mem, index_mem, walls_nearby, crashed, indices_of_collisions = self.lost_and_crashed_calculations(size, distance_mem, index_mem, walls_nearby, 
                                                                                                                crashed, indices_of_collisions, num_of_y_points)
        
        #calculating conditions based on laserscan / point cloud
        wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone = self.wall_conditions_calculations(y_points, x_points, 
                                                num_of_y_points, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)

        #will activate explorer method if lost
        if not walls_nearby:
        
            lost = True
        
        else:
        
            lost = False
        
        if lost:
        
            self.explorer()
        
        elif crashed:

            if self._displays:

                self.crash_recovery_with_displays(size, indices_of_collisions)

            else:

                self.crash_recovery(size, indices_of_collisions)
        
        else:
        
            if not self.oriented:
            
                self.get_oriented(size, index_mem, distance_mem)
            
            elif not self.close_enough:
            
                self.get_closer(forward_cone)
            
            elif not self.aligned:
            
                self.get_aligned(size, index_mem, distance_mem)

            elif self._escape_loop_on and (self.num_of_consecutive_left_turns > 7 or self.num_of_consecutive_right_turns > 7):
                
                self.escape_circle(wall_on_right, wall_on_front_right, wall_on_front_left)

            elif self._displays:

                self.right_wall_follower_with_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)

            else:

                self.right_wall_follower_without_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)
    
    #########################################################################################################################################################
    def lost_and_crashed_calculations(self, size, distance_mem, index_mem, walls_nearby, crashed, indices_of_collisions, num_of_y_points):

        #loop iteration variable
        i = 0
        
        #The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance and set important conditions
        while not rospy.is_shutdown() and i < size:

            if self._scan.ranges[i] < self._scan.ranges[index_mem]:
                distance_mem = self._scan.ranges[i]
                index_mem = i

            #will occur if no walls are close enough
            if self._scan.ranges[i] < 10.0 or num_of_y_points != 0:
                walls_nearby = True

            #crash measurements
            if i > 0 and i < 48 and self._scan.ranges[i] < 0.25:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 47 and i < 98 and self._scan.ranges[i] < 0.3:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 47 and i < 583 and self._scan.ranges[i] < 0.3:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 582 and i < 913 and self._scan.ranges[i] < 0.24:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 912 and i < 1803 and self._scan.ranges[i] < 0.19:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 1802 and i < 2000 and self._scan.ranges[i] < 0.25:
            
                crashed = True
                indices_of_collisions[i] = i
        
            i+=1

        return distance_mem, index_mem, walls_nearby, crashed, indices_of_collisions

    #########################################################################################################################################################
    def wall_conditions_calculations(self, y_points, x_points, num_of_y_points, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, 
                                                                                                                                right_turn, forward_cone):

        #elimating edge cases for making turns
        if self._scan.ranges[1361] >= 0.8 or self._scan.ranges[1200] >= 1.5 or self._scan.ranges[1020] >= 1.5:
            right_turn = True

        #will calculate which sides of the robot there are walls on
        i = 821
        while not rospy.is_shutdown() and i < 1900:

            #will determine wall locations
            if self._scan.ranges[i] < 0.4:

                #if statements for walls
                if i <= 1021:
                    wall_on_right = True
                elif i <= 1361:
                    wall_on_front_right = True
                elif i <= 1600:
                    wall_on_front_left = True
                elif i <= 1800:
                    wall_on_left = True

                #if statement for forward space
                if i >= 1161 and i <= 1561:
                    forward_cone = False
            
            i+=1

        i = 0
        #condition to determine if there is anything detected by the pointcloud
        if num_of_y_points != 0:

            #print("walls detected by pcl")
            
            #loop to figure out locations of pcl objects detected
            while not rospy.is_shutdown() and i < num_of_y_points:

                if y_points[i] > 0.0:
                    
                    if -(x_points[i]) == y_points[i]:
                        wall_on_left = True
                        wall_on_front_left = True
                    elif -(x_points[i]) < y_points[i]:
                        wall_on_left = True
                    elif -(x_points[i]) > y_points[i]:
                        wall_on_front_left = True

                    if y_points[i] < 0.2:
                        forward_cone = False

                elif y_points[i] < 0.0:

                    if x_points[i] == y_points[i]:
                        wall_on_right = True
                        wall_on_front_right = True
                    elif x_points[i] > y_points[i]:
                        wall_on_right = True
                    elif x_points[i] < y_points[i]:
                        wall_on_front_right = True

                    if y_points[i] > -0.2:
                        forward_cone = False

                else:
                    wall_on_front_right = True
                    wall_on_front_left = True
                    forward_cone = False
                
                i+=1

        return wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone

    #########################################################################################################################################################
    def explorer(self):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        if self.reset < 51:
            self.motor_cmd.linear.x = 1.5 #search left and forward
            self.motor_cmd.angular.z = 0.1
            self._pub.publish(self.motor_cmd)
            if self._displays:
                print("Exploring . . .")
        
        elif self.reset < 101:
            self.motor_cmd.linear.x = 1.5 #search right and forward
            self.motor_cmd.angular.z = -0.3
            self._pub.publish(self.motor_cmd)
            if self._displays:
                print("Exploring . . .")
        
        else:
            self.reset = 0 #cycle back to previous conditions
        
        self.current_movement = "exploring"
        self.reset+=1
    
    #########################################################################################################################################################
    def crash_recovery_with_displays(self, size, indices_of_collisions):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        print("Avoiding crash . . . ")

        #sides of crash
        BL = 0
        BR = 0
        FR = 0
        FL = 0

        c = 0
        while not rospy.is_shutdown() and c < size:
        
            if 1890 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1999 or 0 <= indices_of_collisions[c] and indices_of_collisions[c] <= 300:
                BL = 1
            if 300 <= indices_of_collisions[c] and indices_of_collisions[c] <= 820:
                BR = 2
            if 820 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1361:
                FR = 4
            if 1361 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1890:
                FL = 8
            c+=1
        
        type_of_crash = BL + BR + FR + FL
        
        if type_of_crash == 0: #no Near collisions
            print("case 0: No collisions detected")
            self.case_5_tracker = 0
            self.case_7_tracker = 0
            self.case_10_tracker = 0
            self.case_11_tracker = 0
            self.case_13_tracker = 0
            self.case_14_tracker = 0
            self.case_15_tracker = 0
            print("\n\t     ")
            print("\t  o  ")
            print("\t     ")
            
        if type_of_crash == 1: #collison back left
            print("case 1: Near collision back left")
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            print("\n\t     ")
            print("\t  o  ")
            print("\tX    ")
            
        elif type_of_crash == 2: #Near collision back right
            print("case 2: Near collision back right")
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            print("\n\t     ")
            print("\t  o  ")
            print("\t    X")
            
        elif type_of_crash == 3: #Near collisions on back left and right
            print("case 3: Near collisions back left and right")
            self.motor_cmd.linear.x = 1.0
            print("\n\t     ")
            print("\t  o  ")
            print("\tX X X")
            
        elif type_of_crash == 4: #Near collision front right
            print("case 4: Near collision front right")
            self.motor_cmd.angular.z = -0.5 
            self.motor_cmd.linear.x = -0.5
            print("\n\t    X")
            print("\t  o  ")
            print("\t     ")
            
        elif type_of_crash == 5: #Near collision front right and back left
            print("case 5: Near collision front right and back left")
            print("\n\t    X")
            print("\t  o  ")
            print("\tX    ")

            if self.case_5_tracker < 7: 
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_5_tracker+=1
            
            elif self.case_5_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                self.case_5_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_5_tracker = 0
            
        elif type_of_crash == 6: #Near collisions front right and back right
            print("case 6: Near collisions back right and front right")
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            print("\n\t    X")
            print("\t  o X")
            print("\t    X")
            
        elif type_of_crash == 7:
            print("case 7: Near collisions front right, back right, and back left")
            print("\n\t    X")
            print("\t  o X")
            print("\tX X X")
            if self.case_7_tracker < 7:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_7_tracker+=1
            
            elif self.case_7_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                self.case_7_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_7_tracker = 0
                
        elif type_of_crash == 8:
            print("case 8: Near collision front left")
            self.motor_cmd.angular.z = 0.5 
            self.motor_cmd.linear.x = -0.5
            print("\n\tX    ")
            print("\t  o  ")
            print("\t     ")
            
        elif type_of_crash == 9:
            print("case 9: Near collisions front left and back left")
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            print("\n\tX    ")
            print("\tX o  ")
            print("\tX    ")
            
        elif type_of_crash == 10:
            print("case 10: Near collisions front left and back right")
            print("\n\tX    ")
            print("\t  o  ")
            print("\t    X")
            if self.case_10_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_10_tracker+=1
            
            elif self.case_10_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1            
                self.case_10_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_10_tracker = 0
            
        elif type_of_crash == 11:
            print("case 11: Near collisions front left, back left, and back right")
            print("\n\tX    ")
            print("\tX o  ")
            print("\tX X X")
            if self.case_11_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_11_tracker+=1
            
            elif self.case_11_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1
                self.case_11_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_11_tracker = 0
            
        elif type_of_crash == 12:
            print("case 12: Near collisions front left and front right")
            self.motor_cmd.linear.x = -0.5
            print("\n\tX X X")
            print("\t  o  ")
            print("\t     ")
            
        elif type_of_crash == 13:
            print("case 13: Near collisions front left, back left, and front right")
            print("\n\tX X X")
            print("\tX o  ")
            print("\tX    ")
            if self.case_13_tracker < 7:
                self.motor_cmd.angular.z = 0.5 
                self.motor_cmd.linear.x = -0.1
                self.case_13_tracker+=1
            
            elif self.case_13_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_13_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_13_tracker = 0
            
        elif type_of_crash == 14:
            print("case 14: Near collisions front left, front right, and back right")
            print("\n\tX X X")
            print("\t  o X")
            print("\t    X")
            if self.case_14_tracker < 7:
                self.motor_cmd.angular.z = -0.5 
                self.motor_cmd.linear.x = -0.1
                self.case_14_tracker+=1
            
            elif self.case_14_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_14_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_14_tracker = 0
            
        elif type_of_crash == 15:
            print("case 15: Near collisions on multiple sides.")
            print("\n\tX X X")
            print("\tX o X")
            print("\tX X X")

            if self.case_15_tracker < 4:
                self.motor_cmd.linear.x = 0.1
                self.case_15_tracker+=1
            
            elif self.case_15_tracker < 8:
                self.motor_cmd.linear.x = -0.1
                self.case_15_tracker+=1
            
            elif self.case_15_tracker < 20:
            
                self.motor_cmd.angular.z = -0.5
                self.case_15_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                self.case_15_tracker = 0
                
        #move according to conditions for a bit
        recovery_time = rospy.Rate(20)
        self._pub.publish(self.motor_cmd)
        recovery_time.sleep()

        self.current_movement = "avoiding crash"

        #stopping robot before it crashes again
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0
        self._pub.publish(self.motor_cmd)

    #########################################################################################################################################################
    def crash_recovery(self, size, indices_of_collisions):

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #sides of crash
        BL = 0
        BR = 0
        FR = 0
        FL = 0

        c = 0
        while not rospy.is_shutdown() and c < size:
        
            if 1890 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1999 or 0 <= indices_of_collisions[c] and indices_of_collisions[c] <= 300:
                BL = 1
            if 300 <= indices_of_collisions[c] and indices_of_collisions[c] <= 820:
                BR = 2
            if 820 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1361:
                FR = 4
            if 1361 <= indices_of_collisions[c] and indices_of_collisions[c] <= 1890:
                FL = 8
            c+=1
        
        type_of_crash = BL + BR + FR + FL
        
        if type_of_crash == 0: #no Near collisions
            self.case_5_tracker = 0
            self.case_7_tracker = 0
            self.case_10_tracker = 0
            self.case_11_tracker = 0
            self.case_13_tracker = 0
            self.case_14_tracker = 0
            self.case_15_tracker = 0
            
        if type_of_crash == 1: #collison back left
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 2: #Near collision back right
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 3: #Near collisions on back left and right
            self.motor_cmd.linear.x = 1.0
            
        if type_of_crash == 4: #Near collision front right
            self.motor_cmd.angular.z = -0.5 
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 5: #Near collision front right and back left

            if self.case_5_tracker < 7: 
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_5_tracker+=1
            
            elif self.case_5_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                self.case_5_tracker+=1
            
            else:
                self.case_5_tracker = 0
            
        if type_of_crash == 6: #Near collisions front right and back right
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 7:

            if self.case_7_tracker < 7:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_7_tracker+=1
            
            elif self.case_7_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                self.case_7_tracker+=1
            
            else:
                self.case_7_tracker = 0
                
        if type_of_crash == 8:
            self.motor_cmd.angular.z = 0.5 
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 9:
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 10:

            if self.case_10_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_10_tracker+=1
            
            elif self.case_10_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1            
                self.case_10_tracker+=1
            
            else:
                self.case_10_tracker = 0
            
        if type_of_crash == 11:

            if self.case_11_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_11_tracker+=1
            
            elif self.case_11_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1
                self.case_11_tracker+=1
            
            else:
                self.case_11_tracker = 0
            
        if type_of_crash == 12:
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 13:

            if self.case_13_tracker < 7:
                self.motor_cmd.angular.z = 0.5 
                self.motor_cmd.linear.x = -0.1
                self.case_13_tracker+=1
            
            elif self.case_13_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                self.case_13_tracker+=1
            
            else:
                self.case_13_tracker = 0
            
        if type_of_crash == 14:
            if self.case_14_tracker < 7:
                self.motor_cmd.angular.z = -0.5 
                self.motor_cmd.linear.x = -0.1
                self.case_14_tracker+=1
            
            elif self.case_14_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                self.case_14_tracker+=1
            
            else:
                self.case_14_tracker = 0
            
        if type_of_crash == 15:

            if self.case_15_tracker < 4:
                self.motor_cmd.linear.x = 0.1
                self.case_15_tracker+=1
            
            elif self.case_15_tracker < 8:
                self.motor_cmd.linear.x = -0.1
                self.case_15_tracker+=1
            
            elif self.case_15_tracker < 20:
            
                self.motor_cmd.angular.z = -0.5
                self.case_15_tracker+=1
            
            else:
                self.case_15_tracker = 0
                
        #move according to conditions for a bit
        recovery_time = rospy.Rate(10)
        self._pub.publish(self.motor_cmd)
        recovery_time.sleep()

        self.current_movement = "avoiding crash"

        #stopping robot before it crashes again
        self.motor_cmd.linear.x = 0
        self.motor_cmd.angular.z = 0
        self._pub.publish(self.motor_cmd)
    
    #########################################################################################################################################################
    def get_oriented(self, size, index_mem, distance_mem):

        if self._displays and self.first_orient:
            print("\n#########################################################################################################")
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Orienting Robot to Face Nearest Wall")
            print("\n---------------------------------------------------------------------------------------------------------")

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #floats will store the distance to nearest object at cherrypicked indices
        right = self._scan.ranges[820]
        center = self._scan.ranges[1361]
        left = self._scan.ranges[1890]

        #iteration variable
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < size:
        
            if self._scan.ranges[i] < self._scan.ranges[index_mem]:
            
                distance_mem = self._scan.ranges[i]
                index_mem = i
            
            i+=1
        
        #var to track whether or not the front of the robot is facing the nearest wall
        facing = False

        #if the front of the robot is aligned with the closest wall, the facing boolean will be set to true
        if center > distance_mem - 0.01 and center < distance_mem + 0.01:
        
            facing = True
        
        #if its not facing the wall, the robot will turn to its right
        if not facing:
        
            if index_mem > 300 and index_mem < 1361:
            
                self.motor_cmd.angular.z = -0.5
                self._pub.publish(self.motor_cmd)
            
            else:
            
                self.motor_cmd.angular.z = 0.5
                self._pub.publish(self.motor_cmd)
            
        #if the robot is facing the wall, the orientation is complete
        else:
            self.oriented = True
            if self._displays:
                print("INITIAL ORIENTATION COMPLETE.")

        self.current_movement = "orienting"
        self.first_orient = False
    
    #########################################################################################################################################################
    def get_closer(self, forward_cone):

        if self._displays and self.first_distance:
            print("\n#########################################################################################################")
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Establishing the Preferred Initial Distance of the Robot to the Wall")
            print("\n---------------------------------------------------------------------------------------------------------")

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #float to store distance in front of the object
        front = self._scan.ranges[1361]

        #once the robot is close enough, it will proceed
        if front > 0.4 and front < 0.5:

            self.close_enough = True
            if self._displays:
                print("PREFERED INITIAL DISTANCE ACHIEVED.")
        
        #if there is space, the robot will get closer to the wall
        elif front > 0.5 and forward_cone:
        
            self.motor_cmd.linear.x = 0.3
            self._pub.publish(self.motor_cmd)
        
        #if the robot is too close, it will back up
        else:
            if self._scan.ranges[300] > 0.5:
                self.motor_cmd.linear.x = -0.3
                self._pub.publish(self.motor_cmd)
            else:
                self.close_enough = True
                if self._displays:
                    print("Space is tight. This is best distance possible")

        self.current_movement = "establishing initial distance"
        self.first_distance = False

    #########################################################################################################################################################
    def get_aligned(self, size, index_mem, distance_mem):
        
        if self._displays and self.first_align:
            print("\n#########################################################################################################")
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Aligning Robot to be Parallel with the Wall")
            print("\n---------------------------------------------------------------------------------------------------------")

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #float to store distance in front of the object
        right = self._scan.ranges[820]

        #iteration variable for while loop
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < size:
        
            if self._scan.ranges[i] < self._scan.ranges[index_mem]:
            
                distance_mem = self._scan.ranges[i]
                index_mem = i
            
            i+=1
        
        #if aligned, the global var will be set to true, and this function will no longer be called
        if right < distance_mem + 0.02 and right > distance_mem - 0.02:
            
            self.aligned = True
            if self._displays:
                print("FIRST ALIGNMENT COMPLETE.")
        
        #will turn the robot to its left if it is not aligned with the nearest wall
        else:
        
            self.motor_cmd.angular.z = 0.5
            self._pub.publish(self.motor_cmd)

        self.current_movement = "aligning"
        self.first_align = False
        
    #########################################################################################################################################################
    def right_wall_follower_with_displays(self, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #calculations for switch statement
        r = 0
        fr = 0
        fl = 0
        l = 0
        #setting up calculators
        if wall_on_right:
            r = 1
        if wall_on_front_right:
            fr = 2
        if wall_on_front_left:
            fl = 4
        if wall_on_left:
            l = 8
        #var for condtions
        wall_locations =  l + fl + fr + r
        
        if wall_locations == 0: #no walls
            print("\ncase 0")
            self.motor_cmd.linear.x = self._forward_speed
            self.motor_cmd.angular.z = self._max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t| \   |   / |")
            print("\t|  \  |  /  |")
            print("\t|   \ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 1: #wall on right
            print("\ncase 1")
            self.motor_cmd.linear.x = self._forward_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t| \   |   /#|")
            print("\t|  \  |  /##|")
            print("\t|   \ | /###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 2: #wall on front right
            print("\ncase 2")
            '''
            if right_turn:
                self.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\    |####/|")
                print("\t| \   |###/ |")
                print("\t|  \  |##/  |")
                print("\t|   \ |#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

            else:
            '''
            self.motor_cmd.angular.z = self._min_turning_speed_left
            print("\t_____________")
            print("\t|\    |####/|")
            print("\t| \   |###/ |")
            print("\t|  \  |##/  |")
            print("\t|   \ |#/   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 3: #wall on right and front right
            print("\ncase 3")
            self.motor_cmd.angular.z = self._min_turning_speed_left
            print("\t_____________")
            print("\t|\    |####/|")
            print("\t| \   |###/#|")
            print("\t|  \  |##/##|")
            print("\t|   \ |#/###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 4: #wall on front left
            print("\ncase 4")
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t| \###|   / |")
                print("\t|  \##|  /  |")
                print("\t|   \#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t| \###|   / |")
                print("\t|  \##|  /  |")
                print("\t|   \#| /   |")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        #CALCULATIONS REQ
        elif wall_locations == 5: #wall on front left and right
            print("\ncase 5")
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t| \##|||  /#|")
                print("\t|  \#||| /##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t| \###|   /#|")
                print("\t|  \##|  /##|")
                print("\t|   \#| /###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        #CALCULATIONS REQ?
        elif wall_locations == 6: #wall on front left and front right
            print("\ncase 6")
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/ |")
                print("\t|  \##|##/  |")
                print("\t|   \#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/ |")
                print("\t|  \##|##/  |")
                print("\t|   \#|#/   |")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 7: #wall on right, front right, and front left
            print("\ncase 7")
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t| \##|||##/#|")
                print("\t|  \#|||#/##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/#|")
                print("\t|  \##|##/##|")
                print("\t|   \#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 8: #wall on left
            print("\ncase 8")
            self.motor_cmd.linear.x = self._forward_speed
            self.motor_cmd.angular.z = self._max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   / |")
            print("\t|##\  |  /  |")
            print("\t|###\ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 9: #wall on right and left
            print("\ncase 9")
            self.motor_cmd.linear.x = self._forward_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   /#|")
            print("\t|##\  |  /##|")
            print("\t|###\ | /###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")
                

        elif wall_locations == 10: #wall on front right and left
            print("\ncase 10")
            self.motor_cmd.angular.z = self._min_turning_speed_right
            print("\t_____________")
            print("\t|\    |####/|")
            print("\t|#\   |###/ |")
            print("\t|##\  |##/  |")
            print("\t|###\ |#/   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        #CALCULATIONS REQ
        elif wall_locations == 11: #wall on right, front right, and left
            print("\ncase 11")
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed
                print("\t_____________")
                print("\t|\   |||###/|")
                print("\t|#\  |||##/#|")
                print("\t|##\ |||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\    |####/|")
                print("\t|#\   |###/#|")
                print("\t|##\  |##/##|")
                print("\t|###\ |#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 12: #wall on left and front left
            print("\ncase 12")
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t|#\###|   / |")
                print("\t|##\##|  /  |")
                print("\t|###\#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t|#\###|   / |")
                print("\t|##\##|  /  |")
                print("\t|###\#| /   |")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        #CALCULATIONS REQ
        elif wall_locations == 13: #wall on left, front left, and right
            print("\ncase 13")
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t|#\##|||  /#|")
                print("\t|##\#||| /##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t|#\###|   /#|")
                print("\t|##\##|  /##|")
                print("\t|###\#| /###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 14: #wall on left, front left, and front right
            print("\ncase 14")
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/ |")
                print("\t|##\##|##/  |")
                print("\t|###\#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/ |")
                print("\t|##\##|##/  |")
                print("\t|###\#|#/   |")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        #CALCULATIONS REQ
        elif wall_locations == 15: #wall on right, front right, front left, and left
            print("\ncase 15")
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t|#\##|||##/#|")
                print("\t|##\#|||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/#|")
                print("\t|##\##|##/##|")
                print("\t|###\#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
        
        #calculations for current movement type
        if self.motor_cmd.linear.x == self._forward_speed:
            
            if self.motor_cmd.angular.z == self._max_turning_speed or self.motor_cmd.angular.z == self._min_turning_speed_right:
                self.current_movement = "moving forward and turning right"
            
            elif self.motor_cmd.angular.z == self._min_turning_speed_left:
                self.current_movement = "moving forward and turning left"
            
            else:
                self.current_movement = "moving forward"

        elif self.motor_cmd.angular.z == self._max_turning_speed or self.motor_cmd.angular.z == self._min_turning_speed_right:
            self.current_movement = "turning right"

        else:
            self.current_movement = "turning left"


        self._pub.publish(self.motor_cmd)

    ##########################################################################################################################################################
    def right_wall_follower_without_displays(self, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #calculations for switch statement
        r = 0
        fr = 0
        fl = 0
        l = 0
        #setting up calculators
        if wall_on_right:
            r = 1
        if wall_on_front_right:
            fr = 2
        if wall_on_front_left:
            fl = 4
        if wall_on_left:
            l = 8
        #var for condtions
        wall_locations =  l + fl + fr + r
        
        if wall_locations == 0: #no walls
            self.motor_cmd.linear.x = self._forward_speed
            self.motor_cmd.angular.z = self._max_turning_speed

        elif wall_locations == 1: #wall on right
            self.motor_cmd.linear.x = self._forward_speed

        elif wall_locations == 2: #wall on front right
            self.motor_cmd.angular.z = self._min_turning_speed_left

        elif wall_locations == 3: #wall on right and front right
            self.motor_cmd.angular.z = self._min_turning_speed_left

        elif wall_locations == 4: #wall on front left
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 5: #wall on front left and right
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_right

        #CALCULATIONS REQ?
        elif wall_locations == 6: #wall on front left and front right
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        elif wall_locations == 7: #wall on right, front right, and front left
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        elif wall_locations == 8: #wall on left
            self.motor_cmd.linear.x = self._forward_speed
            self.motor_cmd.angular.z = self._max_turning_speed

        elif wall_locations == 9: #wall on right and left
            self.motor_cmd.linear.x = self._forward_speed             

        elif wall_locations == 10: #wall on front right and left
            self.motor_cmd.angular.z = self._min_turning_speed_right

        #CALCULATIONS REQ
        elif wall_locations == 11: #wall on right, front right, and left
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        elif wall_locations == 12: #wall on left and front left
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 13: #wall on left, front left, and right
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_right

        elif wall_locations == 14: #wall on left, front left, and front right
            if right_turn:
                self.motor_cmd.angular.z = self._min_turning_speed_right

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 15: #wall on right, front right, front left, and left
            if forward_cone:
                self.motor_cmd.linear.x = self._forward_speed

            else:
                self.motor_cmd.angular.z = self._min_turning_speed_left

        #calculations for current movement type
        if self.motor_cmd.linear.x == self._forward_speed:
            
            if self.motor_cmd.angular.z == self._max_turning_speed or self.motor_cmd.angular.z == self._min_turning_speed_right:
                self.current_movement = "moving forward and turning right"
            
            elif self.motor_cmd.angular.z == self._min_turning_speed_left:
                self.current_movement = "moving forward and turning left"
            
            else:
                self.current_movement = "moving forward"

        elif self.motor_cmd.angular.z == self._max_turning_speed or self.motor_cmd.angular.z == self._min_turning_speed_right:
            self.current_movement = "turning right"

        else:
            self.current_movement = "turning left"
        
        self._pub.publish(self.motor_cmd)

    ##########################################################################################################################################################
    def consecutive_turns_tracker(self):
        '''
            last_movement values can be moved_forward, turned_left, avoided_crash, explored or turned_right
            current_movement values can be the same

            num_of_consecutive_right_turns will be incremented by 1 in the scenario that the current move is right turn and the last move was forward
            vice versa for consecutive left_turns

            num_of_consecutive_right turns will be reset to 0 if the current move is left turn and vice versa
        '''
        '''
        if self.last_movement_made == "none" and self._displays:

            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Setting up the Consecutive Turns Tracker")
            print("\n---------------------------------------------------------------------------------------------------------")
        '''

        if self.last_movement_made == "exploring" or self.current_movement == "exploring":
            self.num_of_consecutive_right_turns = 0
            self.num_of_consecutive_left_turns = 0

        elif self.last_movement_made == "moving forward":

            if self.current_movement == "turning right" or self.current_movement == "moving forward and turning right":
                self.num_of_consecutive_left_turns = 0
                self.num_of_consecutive_right_turns+=1

            elif self.current_movement == "turning left" or self.current_movement == "moving forward and turning left":
                self.num_of_consecutive_right_turns = 0
                self.num_of_consecutive_left_turns+=1
        
        elif self.last_movement_made == "turning left" or self.current_movement == "moving forward and turning left":

            if self.current_movement == "turning right":
                self.num_of_consecutive_left_turns = 0
                self.num_of_consecutive_right_turns+=1

        elif self.last_movement_made == "turning right" or self.current_movement == "moving forward and turning right":

            if self.current_movement == "turning left":
                self.num_of_consecutive_right_turns = 0
                self.num_of_consecutive_left_turns+=1

        self.last_movement_made = self.current_movement

    ##########################################################################################################################################################
    def escape_circle(self, wall_on_right, wall_on_front_right, wall_on_front_left):

        #resetting motor cmds
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        if self.last_movement_made != "escaping loop" and self._displays:
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Initializing Escape Loop Procedure")
            print("\n---------------------------------------------------------------------------------------------------------")

        if wall_on_right:

            #escape object stuck on
            self.motor_cmd.angular.z = self._min_turning_speed_left

        elif not wall_on_front_left and not wall_on_front_right:

            #move towards a new wall hopefully
            self.motor_cmd.linear.x = self._forward_speed

        else:

            #reset conditions to set up with new wall
            self.num_of_consecutive_left_turns = 0
            self.num_of_consecutive_right_turns = 0
            self.oriented = False
            self.close_enough = False
            self.aligned = False

        self.current_movement = "escaping loop"
        self._pub.publish(self.motor_cmd)

    ##########################################################################################################################################################
    def initial_print(self):

        if self._displays:

            print("\n#########################################################################################################")
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Initializing Wall Follower Program")
            print("\n---------------------------------------------------------------------------------------------------------")
            rospy.sleep(0.5)
            print("\n Active ROS Nodes:")
            print("\n\t Node: PCLforNavNode")
            print("\t . . . downsampling and filtering point cloud for navigation purposes")
            #rospy.sleep(0.5)
            print("\n\t Node: PointCloudWallFollower")
            print("\t . . . subscribing to point cloud and laser scan")
            #rospy.sleep(0.5)
            print("\t . . . setting up wall follower functions")
            rospy.sleep(0.5)
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Firing up Wall Follower Functions")
            print("\n---------------------------------------------------------------------------------------------------------")
            rospy.sleep(0.5)
            print("\n Wall Follower")
            print("\n\t starting intial orientation program . . .")
            #rospy.sleep(0.5)
            print("\n\t starting wall recognition algorithm . . . ")
            #rospy.sleep(0.5)
            print("\n\t checking surroundings . . . ")
            #rospy.sleep(0.5)
            print("\n\t calculating variables . . . ")
            rospy.sleep(0.5)
            print("\n---------------------------------------------------------------------------------------------------------")
            print("\n\t Running Wall Follower")
            print("\n---------------------------------------------------------------------------------------------------------")
            rospy.sleep(0.5)
        
        self.first_print = False

#############################################################################################################################################################
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
        rospy.sleep(0.1)

    #########################################################################################################################################################
    def move_down(self):

        self._robot.head.move_by('head_tilt', -1.57)
        self._robot.push_command()
        rospy.sleep(0.1)

    #########################################################################################################################################################
    def error_msg(self):
        if self._displays:
            print("\n\t\t:: HEAD POSITIONS ERROR ::")
            print("\t Either the current position or desired position is incorrect\n")

    #########################################################################################################################################################
    def initial_print(self):

        print("\n#########################################################################################################")
        print("\n---------------------------------------------------------------------------------------------------------")
        print("\n\t Repositioning Head Camera")
        print("\n---------------------------------------------------------------------------------------------------------")

        self._first_print = False

##############################################################################################################################################################
def fix_head_position():
    os.system(path)

##############################################################################################################################################################

    # MAIN #

##############################################################################################################################################################
def main(path):

    #setting up stretch body
    robot = stretch_body.robot.Robot()
    robot.startup()
    if not robot.is_calibrated():
        robot.home() #blocking
    robot.stow()

    try:

        hd = HeadUpDown("forward", "down", robot)

        wf = StretchWallFollower()
        
        rospy.on_shutdown(fix_head_position)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit, ThreadServiceExit):
        #hu = SimHeadUpDown("down", "forward", 15)
        os.system(path)
        pass

    robot.stop()

if __name__ == '__main__':

    #initializing node
    rospy.init_node("stretch_wall_follower_node", anonymous = True)

    path = '/home/maru/stretch_ws/src/ARA_Summer_Robotics/Nates_code/nate_stretch_movement/src/move_head_up.sh'

    main(path)