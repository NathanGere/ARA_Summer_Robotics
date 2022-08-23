#!/usr/bin/env python

import rospy
#import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
#import ctypes
#import struct
from std_msgs.msg import Header, String, Float32MultiArray
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#############################################################################################################################################################
class PCLWallFollower:
    
    #########################################################################################################################################################
    def __init__(self, pub = rospy.Publisher("/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=10)):
        self._pub = pub #wall follower publisher
        self.motor_cmd = Twist() #motor command to be published in multiple methods
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
        #class attribute for if pointcloud detected objects
        #self.pcl_objects_detected = False

    #########################################################################################################################################################
    def setup_wall_follower(self, scan, displays = True):

        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0
        self._pub.publish(self.motor_cmd)

        ############################
        def middle_man_3(cloud_msg):
            
            #calling subscriber callback but with additional argument of the laser scan
            self.cloud_cb(cloud_msg, scan, displays)

        cloud_sub = rospy.Subscriber("/pcl_for_nav/points", PointCloud2, middle_man_3, queue_size = 1, buff_size = 52428800)

    #########################################################################################################################################################
    def cloud_cb(self, cloud_msg, scan, displays):
        
        pointCloud = cloud_msg
        #setting up storage of areas where objects are detected
        y_points = []
        x_points = []
        num_of_y_points = 0
        #print("\n\n\n\n\n\tXYZ POINTS")
        #self.pcl_objects_detected = False
        
        for point in pc2.read_points(pointCloud, field_names=pointCloud.data, skip_nans=True):

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
        
        '''
        if not self.pcl_objects_detected:
            num_of_y_points = 0
            y_points = [3000.0, 3000.0]
        '''
        
        #print("Number of y_pts: " + str(num_of_y_points))
        self.follow_walls_please(scan, y_points, x_points, num_of_y_points, displays)

    #########################################################################################################################################################
    def follow_walls_please(self, scan, y_points, x_points, num_of_y_points, displays = True):

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #variable for size of laser scan
        size = len(scan.ranges)
        
        #loop iteration variable
        i = 0

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

        #The following loop with iterate through the entire array of laser scans to find the index of the nearest wall and the wall's distance and set important conditions
        while not rospy.is_shutdown() and i < size:

            if scan.ranges[i] < scan.ranges[index_mem]:
                distance_mem = scan.ranges[i]
                index_mem = i

            #will occur if no walls are close enough
            if scan.ranges[i] < 10.0 or num_of_y_points != 0:
                walls_nearby = True

            #crash measurements
            if i > 0 and i < 48 and scan.ranges[i] < 0.25:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 47 and i < 98 and scan.ranges[i] < 0.3:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 47 and i < 583 and scan.ranges[i] < 0.3:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 582 and i < 913 and scan.ranges[i] < 0.24:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 912 and i < 1803 and scan.ranges[i] < 0.19:
                crashed = True
                indices_of_collisions[i] = i

            elif i > 1802 and i < 2000 and scan.ranges[i] < 0.25:
            
                crashed = True
                indices_of_collisions[i] = i
        
            i+=1
        
        #elimating edge cases for making turns
        if scan.ranges[1361] >= 0.8 or scan.ranges[1200] >= 1.5:
            right_turn = True

        #will calculate which sides of the robot there are walls on
        i = 821
        while not rospy.is_shutdown() and i < 1900:

            #will determine wall locations
            if scan.ranges[i] < 0.4:

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

            print("walls detected by pcl")
            
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

        #will activate explorer method in next loop of ros::spinOnce()
        if not walls_nearby:
        
            lost = True
        
        else:
        
            lost = False
        
        if lost:
        
            self.explorer(displays)
        
        elif crashed:

            if displays:

                self.crash_recovery_with_displays(size, indices_of_collisions)

            else:

                self.crash_recovery(size, indices_of_collisions)
        
        else:
        
            if not self.oriented:
            
                self.get_oriented(scan, size, index_mem, distance_mem, displays)
            
            elif not self.close_enough:
            
                self.get_closer(scan, displays)
            
            elif not self.aligned:
            
                self.get_aligned(scan, size, index_mem, distance_mem, displays)

            elif displays:

                self.right_wall_follower_with_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)

            else:

                self.right_wall_follower_without_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)
    
    #########################################################################################################################################################
    def explorer(self, displays):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        if self.reset < 51:
            self.motor_cmd.linear.x = 1.5 #search left and forward
            self.motor_cmd.angular.z = 0.1
            self._pub.publish(self.motor_cmd)
            if displays:
                print("Exploring . . .")
        
        elif self.reset < 101:
            self.motor_cmd.linear.x = 1.5 #search right and forward
            self.motor_cmd.angular.z = -0.3
            self._pub.publish(self.motor_cmd)
            if displays:
                print("Exploring . . .")
        
        else:
            self.reset = 0 #cycle back to previous conditions
        
        self.reset+=1
    
    ########################################################################################################################################################
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

        #stopping robot before it crashes again
        self.motor_cmd.linear.x = 0
        self.motor_cmd.angular.z = 0
        self._pub.publish(self.motor_cmd)
    
    #########################################################################################################################################################
    def get_oriented(self, scan, size, index_mem, distance_mem, displays):

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0
        self._pub.publish(self.motor_cmd)

        #floats will store the distance to nearest object at cherrypicked indices
        right = scan.ranges[820]
        center = scan.ranges[1361]
        left = scan.ranges[1890]

        #iteration variable
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < size:
        
            if scan.ranges[i] < scan.ranges[index_mem]:
            
                distance_mem = scan.ranges[i]
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
                if displays:
                    print("Orienting . . .") 
            
            else:
            
                self.motor_cmd.angular.z = 0.5
                self._pub.publish(self.motor_cmd)
                if displays:
                    print("Orienting . . .") 
            
        #if the robot is facing the wall, the orientation is complete
        else:
            self.oriented = True
            if displays:
                print("INITIAL ORIENTATION COMPLETE.")
    
    #########################################################################################################################################################
    def get_closer(self, scan, displays):

        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #float to store distance in front of the object
        front = scan.ranges[1361]

        #once the robot is close enough, it will proceed
        if front > 0.4 and front < 0.5:

            self.close_enough = True
            if displays:
                print("PREFERED INITIAL DISTANCE ACHIEVED.")
        
        #if there is space, the robot will get closer to the wall
        elif front > 0.5:
        
            self.motor_cmd.linear.x = 0.3
            self._pub.publish(self.motor_cmd)
            if displays:
                print("Establishing initial distance . . .")
        
        #if the robot is too close, it will back up
        else:
            if scan.ranges[300] > 0.5:
                self.motor_cmd.linear.x = -0.3
                self._pub.publish(self.motor_cmd)
                if displays:
                    print("Establishing initial distance . . .")
            else:
                self.close_enough = True
                if displays:
                    print("Space is tight. This is as far as we can get")

    #########################################################################################################################################################
    def get_aligned(self, scan, size, index_mem, distance_mem, displays):
        
        #resetting motor_cmd
        self.motor_cmd.linear.x = 0.0
        self.motor_cmd.angular.z = 0.0

        #float to store distance in front of the object
        right = scan.ranges[820]

        #iteration variable for while loop
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < size:
        
            if scan.ranges[i] < scan.ranges[index_mem]:
            
                distance_mem = scan.ranges[i]
                index_mem = i
            
            i+=1
        
        #if aligned, the global var will be set to true, and this function will no longer be called
        if right < distance_mem + 0.02 and right > distance_mem - 0.02:
            
            self.aligned = True
            if displays:
                print("FIRST ALIGNMENT COMPLETE.")
        
        #will turn the robot to its left if it is not aligned with the nearest wall
        else:
        
            self.motor_cmd.angular.z = 0.5
            self._pub.publish(self.motor_cmd)
            if displays:
                print("Completing first alignment . . .")
        
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
            self.motor_cmd.linear.x = forward_speed
            self.motor_cmd.angular.z = max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t| \   |   / |")
            print("\t|  \  |  /  |")
            print("\t|   \ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 1: #wall on right
            print("\ncase 1")
            self.motor_cmd.linear.x = forward_speed
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
            self.motor_cmd.angular.z = min_turning_speed_left
            print("\t_____________")
            print("\t|\    |####/|")
            print("\t| \   |###/ |")
            print("\t|  \  |##/  |")
            print("\t|   \ |#/   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 3: #wall on right and front right
            print("\ncase 3")
            self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t| \###|   / |")
                print("\t|  \##|  /  |")
                print("\t|   \#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t| \##|||  /#|")
                print("\t|  \#||| /##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_right
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
                self.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/ |")
                print("\t|  \##|##/  |")
                print("\t|   \#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t| \##|||##/#|")
                print("\t|  \#|||#/##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/#|")
                print("\t|  \##|##/##|")
                print("\t|   \#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 8: #wall on left
            print("\ncase 8")
            self.motor_cmd.linear.x = forward_speed
            self.motor_cmd.angular.z = max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   / |")
            print("\t|##\  |  /  |")
            print("\t|###\ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 9: #wall on right and left
            print("\ncase 9")
            self.motor_cmd.linear.x = forward_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   /#|")
            print("\t|##\  |  /##|")
            print("\t|###\ | /###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")
                

        elif wall_locations == 10: #wall on front right and left
            print("\ncase 10")
            self.motor_cmd.angular.z = min_turning_speed_right
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
                self.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\   |||###/|")
                print("\t|#\  |||##/#|")
                print("\t|##\ |||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t|#\###|   / |")
                print("\t|##\##|  /  |")
                print("\t|###\#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t|#\##|||  /#|")
                print("\t|##\#||| /##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_right
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
                self.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/ |")
                print("\t|##\##|##/  |")
                print("\t|###\#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
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
                self.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t|#\##|||##/#|")
                print("\t|##\#|||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                self.motor_cmd.angular.z = min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/#|")
                print("\t|##\##|##/##|")
                print("\t|###\#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
        
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
            self.motor_cmd.linear.x = forward_speed
            self.motor_cmd.angular.z = max_turning_speed

        elif wall_locations == 1: #wall on right
            self.motor_cmd.linear.x = forward_speed

        elif wall_locations == 2: #wall on front right
            self.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 3: #wall on right and front right
            self.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 4: #wall on front left
            if right_turn:
                self.motor_cmd.angular.z = min_turning_speed_right

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 5: #wall on front left and right
            if forward_cone:
                self.motor_cmd.linear.x = forward_speed

            else:
                self.motor_cmd.angular.z = min_turning_speed_right

        #CALCULATIONS REQ?
        elif wall_locations == 6: #wall on front left and front right
            if right_turn:
                self.motor_cmd.angular.z = min_turning_speed_right

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 7: #wall on right, front right, and front left
            if forward_cone:
                self.motor_cmd.linear.x = forward_speed

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 8: #wall on left
            self.motor_cmd.linear.x = forward_speed
            self.motor_cmd.angular.z = max_turning_speed

        elif wall_locations == 9: #wall on right and left
            self.motor_cmd.linear.x = forward_speed             

        elif wall_locations == 10: #wall on front right and left
            self.motor_cmd.angular.z = min_turning_speed_right

        #CALCULATIONS REQ
        elif wall_locations == 11: #wall on right, front right, and left
            if forward_cone:
                self.motor_cmd.linear.x = forward_speed

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 12: #wall on left and front left
            if right_turn:
                self.motor_cmd.angular.z = min_turning_speed_right

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 13: #wall on left, front left, and right
            if forward_cone:
                self.motor_cmd.linear.x = forward_speed

            else:
                self.motor_cmd.angular.z = min_turning_speed_right

        elif wall_locations == 14: #wall on left, front left, and front right
            if right_turn:
                self.motor_cmd.angular.z = min_turning_speed_right

            else:
                self.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 15: #wall on right, front right, front left, and left
            if forward_cone:
                self.motor_cmd.linear.x = forward_speed

            else:
                self.motor_cmd.angular.z = min_turning_speed_left
        
        self._pub.publish(self.motor_cmd)
    
#############################################################################################################################################################
def middle_man():

    wf = PCLWallFollower()

    def middle_man_2(scan):

        wf.setup_wall_follower(scan)

    #setting up subscriber
    sub = rospy.Subscriber("/scan", LaserScan, middle_man_2)
    
#############################################################################################################################################################

    # MAIN #

#############################################################################################################################################################
if __name__ == '__main__':
    try:
        #initializing node
        rospy.init_node("pcl_wall_follower_node", anonymous = True)

        #setting up params
        forward_speed = rospy.get_param("/forward_speed", 0.3)
        max_turning_speed = rospy.get_param("/max_turning_speed", -0.6)
        min_turning_speed_right = rospy.get_param("/min_turning_speed_right", -0.3)
        min_turning_speed_left = rospy.get_param("/min_turning_speed_left", 0.3)

        #launchpoint for wall follower program
        middle_man()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass