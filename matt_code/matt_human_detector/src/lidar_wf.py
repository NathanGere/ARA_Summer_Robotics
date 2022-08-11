#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#############################################################################################################################################################
class WallFollower:
    
    #########################################################################################################################################################
    def __init__(WFC, pub = rospy.Publisher("/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=10)):
        WFC._pub = pub #wall follower publisher
        WFC.motor_cmd = Twist() #motor command to be published in multiple methods
        #local variables for initial setup of robot's orientation and position in relation to the nearest wall
        WFC.oriented = False
        WFC.close_enough = False
        WFC.aligned = False
        #local variable for explorer method
        WFC.reset = 0
        #local variables for crash recovery method
        WFC.case_5_tracker = 0
        WFC.case_7_tracker = 0
        WFC.case_10_tracker = 0
        WFC.case_11_tracker = 0
        WFC.case_13_tracker = 0
        WFC.case_14_tracker = 0
        WFC.case_15_tracker = 0
        
        #tracker for if human located
        WFC.bool_sub = rospy.Subscriber("talker", Bool, WFC.bool_cb)
        WFC._m_bool = False

    #########################################################################################################################################################
    def follow_walls_please(WFC, scan, wf, displays = True):

        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            if scan.ranges[i] < 10.0:
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

        #will activate explorer method in next loop of ros::spinOnce()
        if not walls_nearby:
        
            lost = True
        
        else:
        
            lost = False
        
        if WFC._m_bool:
            WFC._pub.publish(WFC.motor_cmd)

        elif lost:
        
            wf.explorer(displays)
        
        elif crashed:

            if displays:

                wf.crash_recovery_with_displays(size, indices_of_collisions)

            else:

                wf.crash_recovery(size, indices_of_collisions)
        
        else:
        
            if not WFC.oriented:
            
                wf.get_oriented(scan, size, index_mem, distance_mem, displays)
            
            elif not WFC.close_enough:
            
                wf.get_closer(scan, displays)
            
            elif not WFC.aligned:
            
                wf.get_aligned(scan, size, index_mem, distance_mem, displays)

            elif displays:

                wf.right_wall_follower_with_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)

            else:

                wf.right_wall_follower_without_displays(wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone)
    
    #########################################################################################################################################################
    def explorer(WFC, displays):
        
        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

        if WFC.reset < 51:
            WFC.motor_cmd.linear.x = 1.5 #search left and forward
            WFC.motor_cmd.angular.z = 0.1
            WFC._pub.publish(WFC.motor_cmd)
            if displays:
                print("Exploring . . .")
        
        elif WFC.reset < 101:
            WFC.motor_cmd.linear.x = 1.5 #search right and forward
            WFC.motor_cmd.angular.z = -0.3
            WFC._pub.publish(WFC.motor_cmd)
            if displays:
                print("Exploring . . .")
        
        else:
            WFC.reset = 0 #cycle back to previous conditions
        
        WFC.reset+=1
    
    ########################################################################################################################################################
    def crash_recovery_with_displays(WFC, size, indices_of_collisions):
        
        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            WFC.case_5_tracker = 0
            WFC.case_7_tracker = 0
            WFC.case_10_tracker = 0
            WFC.case_11_tracker = 0
            WFC.case_13_tracker = 0
            WFC.case_14_tracker = 0
            WFC.case_15_tracker = 0
            print("\n\t     ")
            print("\t  o  ")
            print("\t     ")
            
        if type_of_crash == 1: #collison back left
            print("case 1: Near collision back left")
            WFC.motor_cmd.angular.z = -0.5
            WFC.motor_cmd.linear.x = 0.5
            print("\n\t     ")
            print("\t  o  ")
            print("\tX    ")
            
        if type_of_crash == 2: #Near collision back right
            print("case 2: Near collision back right")
            WFC.motor_cmd.angular.z = 0.5
            WFC.motor_cmd.linear.x = 0.5
            print("\n\t     ")
            print("\t  o  ")
            print("\t    X")
            
        if type_of_crash == 3: #Near collisions on back left and right
            print("case 3: Near collisions back left and right")
            WFC.motor_cmd.linear.x = 1.0
            print("\n\t     ")
            print("\t  o  ")
            print("\tX X X")
            
        if type_of_crash == 4: #Near collision front right
            print("case 4: Near collision front right")
            WFC.motor_cmd.angular.z = -0.5 
            WFC.motor_cmd.linear.x = -0.5
            print("\n\t    X")
            print("\t  o  ")
            print("\t     ")
            
        if type_of_crash == 5: #Near collision front right and back left
            print("case 5: Near collision front right and back left")
            print("\n\t    X")
            print("\t  o  ")
            print("\tX    ")

            if WFC.case_5_tracker < 7: 
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_5_tracker+=1
            
            elif WFC.case_5_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_5_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_5_tracker = 0
            
        if type_of_crash == 6: #Near collisions front right and back right
            print("case 6: Near collisions back right and front right")
            WFC.motor_cmd.angular.z = 0.5
            WFC.motor_cmd.linear.x = 0.5
            print("\n\t    X")
            print("\t  o X")
            print("\t    X")
            
        if type_of_crash == 7:
            print("case 7: Near collisions front right, back right, and back left")
            print("\n\t    X")
            print("\t  o X")
            print("\tX X X")
            if WFC.case_7_tracker < 7:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_7_tracker+=1
            
            elif WFC.case_7_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_7_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_7_tracker = 0
                
        if type_of_crash == 8:
            print("case 8: Near collision front left")
            WFC.motor_cmd.angular.z = 0.5 
            WFC.motor_cmd.linear.x = -0.5
            print("\n\tX    ")
            print("\t  o  ")
            print("\t     ")
            
        if type_of_crash == 9:
            print("case 9: Near collisions front left and back left")
            WFC.motor_cmd.angular.z = -0.5
            WFC.motor_cmd.linear.x = 0.5
            print("\n\tX    ")
            print("\tX o  ")
            print("\tX    ")
            
        if type_of_crash == 10:
            print("case 10: Near collisions front left and back right")
            print("\n\tX    ")
            print("\t  o  ")
            print("\t    X")
            if WFC.case_10_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_10_tracker+=1
            
            elif WFC.case_10_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = -0.1            
                WFC.case_10_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_10_tracker = 0
            
        if type_of_crash == 11:
            print("case 11: Near collisions front left, back left, and back right")
            print("\n\tX    ")
            print("\tX o  ")
            print("\tX X X")
            if WFC.case_11_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_11_tracker+=1
            
            elif WFC.case_11_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_11_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_11_tracker = 0
            
        if type_of_crash == 12:
            print("case 12: Near collisions front left and front right")
            WFC.motor_cmd.linear.x = -0.5
            print("\n\tX X X")
            print("\t  o  ")
            print("\t     ")
            
        if type_of_crash == 13:
            print("case 13: Near collisions front left, back left, and front right")
            print("\n\tX X X")
            print("\tX o  ")
            print("\tX    ")
            if WFC.case_13_tracker < 7:
                WFC.motor_cmd.angular.z = 0.5 
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_13_tracker+=1
            
            elif WFC.case_13_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_13_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_13_tracker = 0
            
        if type_of_crash == 14:
            print("case 14: Near collisions front left, front right, and back right")
            print("\n\tX X X")
            print("\t  o X")
            print("\t    X")
            if WFC.case_14_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5 
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_14_tracker+=1
            
            elif WFC.case_14_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_14_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_14_tracker = 0
            
        if type_of_crash == 15:
            print("case 15: Near collisions on multiple sides.")
            print("\n\tX X X")
            print("\tX o X")
            print("\tX X X")

            if WFC.case_15_tracker < 4:
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_15_tracker+=1
            
            elif WFC.case_15_tracker < 8:
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_15_tracker+=1
            
            elif WFC.case_15_tracker < 20:
            
                WFC.motor_cmd.angular.z = -0.5
                WFC.case_15_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                WFC.case_15_tracker = 0
                
        #move according to conditions for a bit
        recovery_time = rospy.Rate(10)
        WFC._pub.publish(WFC.motor_cmd)
        recovery_time.sleep()

        #stopping robot before it crashes again
        WFC.motor_cmd.linear.x = 0
        WFC.motor_cmd.angular.z = 0
        WFC._pub.publish(WFC.motor_cmd)

    #########################################################################################################################################################
    def crash_recovery(WFC, size, indices_of_collisions):

        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            WFC.case_5_tracker = 0
            WFC.case_7_tracker = 0
            WFC.case_10_tracker = 0
            WFC.case_11_tracker = 0
            WFC.case_13_tracker = 0
            WFC.case_14_tracker = 0
            WFC.case_15_tracker = 0
            
        if type_of_crash == 1: #collison back left
            WFC.motor_cmd.angular.z = -0.5
            WFC.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 2: #Near collision back right
            WFC.motor_cmd.angular.z = 0.5
            WFC.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 3: #Near collisions on back left and right
            WFC.motor_cmd.linear.x = 1.0
            
        if type_of_crash == 4: #Near collision front right
            WFC.motor_cmd.angular.z = -0.5 
            WFC.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 5: #Near collision front right and back left

            if WFC.case_5_tracker < 7: 
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_5_tracker+=1
            
            elif WFC.case_5_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_5_tracker+=1
            
            else:
                WFC.case_5_tracker = 0
            
        if type_of_crash == 6: #Near collisions front right and back right
            WFC.motor_cmd.angular.z = 0.5
            WFC.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 7:

            if WFC.case_7_tracker < 7:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_7_tracker+=1
            
            elif WFC.case_7_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_7_tracker+=1
            
            else:
                WFC.case_7_tracker = 0
                
        if type_of_crash == 8:
            WFC.motor_cmd.angular.z = 0.5 
            WFC.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 9:
            WFC.motor_cmd.angular.z = -0.5
            WFC.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 10:

            if WFC.case_10_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_10_tracker+=1
            
            elif WFC.case_10_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = -0.1            
                WFC.case_10_tracker+=1
            
            else:
                WFC.case_10_tracker = 0
            
        if type_of_crash == 11:

            if WFC.case_11_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_11_tracker+=1
            
            elif WFC.case_11_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_11_tracker+=1
            
            else:
                WFC.case_11_tracker = 0
            
        if type_of_crash == 12:
            WFC.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 13:

            if WFC.case_13_tracker < 7:
                WFC.motor_cmd.angular.z = 0.5 
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_13_tracker+=1
            
            elif WFC.case_13_tracker < 15:
                WFC.motor_cmd.angular.z = -0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_13_tracker+=1
            
            else:
                WFC.case_13_tracker = 0
            
        if type_of_crash == 14:
            if WFC.case_14_tracker < 7:
                WFC.motor_cmd.angular.z = -0.5 
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_14_tracker+=1
            
            elif WFC.case_14_tracker < 15:
                WFC.motor_cmd.angular.z = 0.5
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_14_tracker+=1
            
            else:
                WFC.case_14_tracker = 0
            
        if type_of_crash == 15:

            if WFC.case_15_tracker < 4:
                WFC.motor_cmd.linear.x = 0.1
                WFC.case_15_tracker+=1
            
            elif WFC.case_15_tracker < 8:
                WFC.motor_cmd.linear.x = -0.1
                WFC.case_15_tracker+=1
            
            elif WFC.case_15_tracker < 20:
            
                WFC.motor_cmd.angular.z = -0.5
                WFC.case_15_tracker+=1
            
            else:
                WFC.case_15_tracker = 0
                
        #move according to conditions for a bit
        recovery_time = rospy.Rate(10)
        WFC._pub.publish(WFC.motor_cmd)
        recovery_time.sleep()

        #stopping robot before it crashes again
        WFC.motor_cmd.linear.x = 0
        WFC.motor_cmd.angular.z = 0
        WFC._pub.publish(WFC.motor_cmd)
    
    #########################################################################################################################################################
    def get_oriented(WFC, scan, size, index_mem, distance_mem, displays):

        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0
        WFC._pub.publish(WFC.motor_cmd)

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
            
                WFC.motor_cmd.angular.z = -0.5
                WFC._pub.publish(WFC.motor_cmd)
                if displays:
                    print("Orienting . . .")
            
            else:
            
                WFC.motor_cmd.angular.z = 0.5
                WFC._pub.publish(WFC.motor_cmd)
                if displays:
                    print("Orienting . . .")
            
        #if the robot is facing the wall, the orientation is complete
        else:
            WFC.oriented = True
            if displays:
                print("INITIAL ORIENTATION COMPLETE.")
    
    #########################################################################################################################################################
    def get_closer(WFC, scan, displays):

        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

        #float to store distance in front of the object
        front = scan.ranges[1361]

        #once the robot is close enough, it will proceed
        if front > 0.4 and front < 0.5:

            WFC.close_enough = True
            if displays:
                print("PREFERED INITIAL DISTANCE ACHIEVED.")
        
        #if there is space, the robot will get closer to the wall
        elif front > 0.5:
        
            WFC.motor_cmd.linear.x = 0.3
            WFC._pub.publish(WFC.motor_cmd)
            if displays:
                print("Establishing initial distance . . .")
        
        #if the robot is too close, it will back up
        else:
            if scan.ranges[300] > 0.5:
                WFC.motor_cmd.linear.x = -0.3
                WFC._pub.publish(WFC.motor_cmd)
                if displays:
                    print("Establishing initial distance . . .")
            else:
                WFC.close_enough = True
                if displays:
                    print("Space is tight. This is as far as we can get")

    #########################################################################################################################################################
    def get_aligned(WFC, scan, size, index_mem, distance_mem, displays):
        
        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            
            WFC.aligned = True
            if displays:
                print("FIRST ALIGNMENT COMPLETE.")
        
        #will turn the robot to its left if it is not aligned with the nearest wall
        else:
            WFC.motor_cmd.angular.z = 0.5
            WFC._pub.publish(WFC.motor_cmd)
            if displays:
                print("Completing first alignment . . .")
        
    #########################################################################################################################################################
    def right_wall_follower_with_displays(WFC, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone):
        
        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            WFC.motor_cmd.linear.x = forward_speed
            WFC.motor_cmd.angular.z = max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t| \   |   / |")
            print("\t|  \  |  /  |")
            print("\t|   \ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 1: #wall on right
            print("\ncase 1")
            WFC.motor_cmd.linear.x = forward_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t| \   |   /#|")
            print("\t|  \  |  /##|")
            print("\t|   \ | /###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 2: #wall on front right
            print("\ncase 2")
            
            # if right_turn:
                # WFC.motor_cmd.angular.z = min_turning_speed_right
                # print("\t_____________")
                # print("\t|\    |####/|")
                # print("\t| \   |###/ |")
                # print("\t|  \  |##/  |")
                # print("\t|   \ |#/RRR|")
                # print("\t----|^^^|----")
                # print("\t    |___|    ")

            # else:
            WFC.motor_cmd.angular.z = min_turning_speed_left
            print("\t_____________")
            print("\t|\    |####/|")
            print("\t| \   |###/ |")
            print("\t|  \  |##/  |")
            print("\t|   \ |#/   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 3: #wall on right and front right
            print("\ncase 3")
            WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t| \###|   / |")
                print("\t|  \##|  /  |")
                print("\t|   \#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t| \##|||  /#|")
                print("\t|  \#||| /##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_right
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
                WFC.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/ |")
                print("\t|  \##|##/  |")
                print("\t|   \#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t| \##|||##/#|")
                print("\t|  \#|||#/##|")
                print("\t|   \|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t| \###|###/#|")
                print("\t|  \##|##/##|")
                print("\t|   \#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")

        elif wall_locations == 8: #wall on left
            print("\ncase 8")
            WFC.motor_cmd.linear.x = forward_speed
            WFC.motor_cmd.angular.z = max_turning_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   / |")
            print("\t|##\  |  /  |")
            print("\t|###\ | /   |")
            print("\t----|^^^|----")
            print("\t    |___|    ")

        elif wall_locations == 9: #wall on right and left
            print("\ncase 9")
            WFC.motor_cmd.linear.x = forward_speed
            print("\t_____________")
            print("\t|\    |    /|")
            print("\t|#\   |   /#|")
            print("\t|##\  |  /##|")
            print("\t|###\ | /###|")
            print("\t----|^^^|----")
            print("\t    |___|    ")
                

        elif wall_locations == 10: #wall on front right and left
            print("\ncase 10")
            WFC.motor_cmd.angular.z = min_turning_speed_right
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
                WFC.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\   |||###/|")
                print("\t|#\  |||##/#|")
                print("\t|##\ |||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|    /|")
                print("\t|#\###|   / |")
                print("\t|##\##|  /  |")
                print("\t|###\#| /RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||   /|")
                print("\t|#\##|||  /#|")
                print("\t|##\#||| /##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_right
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
                WFC.motor_cmd.angular.z = min_turning_speed_right
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/ |")
                print("\t|##\##|##/  |")
                print("\t|###\#|#/RRR|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
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
                WFC.motor_cmd.linear.x = forward_speed
                print("\t_____________")
                print("\t|\###|||###/|")
                print("\t|#\##|||##/#|")
                print("\t|##\#|||#/##|")
                print("\t|###\|||/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
                print("\t_____________")
                print("\t|\####|####/|")
                print("\t|#\###|###/#|")
                print("\t|##\##|##/##|")
                print("\t|###\#|#/###|")
                print("\t----|^^^|----")
                print("\t    |___|    ")
        
        WFC._pub.publish(WFC.motor_cmd)

    ##########################################################################################################################################################
    def right_wall_follower_without_displays(WFC, wall_on_right, wall_on_front_right, wall_on_front_left, wall_on_left, right_turn, forward_cone):
        
        #resetting motor_cmd
        WFC.motor_cmd.linear.x = 0.0
        WFC.motor_cmd.angular.z = 0.0

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
            WFC.motor_cmd.linear.x = forward_speed
            WFC.motor_cmd.angular.z = max_turning_speed

        elif wall_locations == 1: #wall on right
            WFC.motor_cmd.linear.x = forward_speed

        elif wall_locations == 2: #wall on front right
            WFC.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 3: #wall on right and front right
            WFC.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 4: #wall on front left
            if right_turn:
                WFC.motor_cmd.angular.z = min_turning_speed_right

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 5: #wall on front left and right
            if forward_cone:
                WFC.motor_cmd.linear.x = forward_speed

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_right

        #CALCULATIONS REQ?
        elif wall_locations == 6: #wall on front left and front right
            if right_turn:
                WFC.motor_cmd.angular.z = min_turning_speed_right

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 7: #wall on right, front right, and front left
            if forward_cone:
                WFC.motor_cmd.linear.x = forward_speed

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 8: #wall on left
            WFC.motor_cmd.linear.x = forward_speed
            WFC.motor_cmd.angular.z = max_turning_speed

        elif wall_locations == 9: #wall on right and left
            WFC.motor_cmd.linear.x = forward_speed             

        elif wall_locations == 10: #wall on front right and left
            WFC.motor_cmd.angular.z = min_turning_speed_right

        #CALCULATIONS REQ
        elif wall_locations == 11: #wall on right, front right, and left
            if forward_cone:
                WFC.motor_cmd.linear.x = forward_speed

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        elif wall_locations == 12: #wall on left and front left
            if right_turn:
                WFC.motor_cmd.angular.z = min_turning_speed_right

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 13: #wall on left, front left, and right
            if forward_cone:
                WFC.motor_cmd.linear.x = forward_speed

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_right

        elif wall_locations == 14: #wall on left, front left, and front right
            if right_turn:
                WFC.motor_cmd.angular.z = min_turning_speed_right

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left

        #CALCULATIONS REQ
        elif wall_locations == 15: #wall on right, front right, front left, and left
            if forward_cone:
                WFC.motor_cmd.linear.x = forward_speed

            else:
                WFC.motor_cmd.angular.z = min_turning_speed_left
        
        WFC._pub.publish(WFC.motor_cmd)

    ##########################################################################################################################################################
    def bool_cb(WFC, m_bool):
        WFC._m_bool = m_bool

#############################################################################################################################################################
def middle_man():

    wf = WallFollower()

    def middle_man_2(scan):

        wf.follow_walls_please(scan, wf, False)

    #setting up subscriber
    sub = rospy.Subscriber("/scan", LaserScan, middle_man_2)
    
#############################################################################################################################################################

    # MAIN #

#############################################################################################################################################################
if __name__ == '__main__':
    try:
        #setting up node
        rospy.init_node("wall_follower_python_node", anonymous =True)

        #setting up params
        forward_speed = rospy.get_param("/forward_speed", 0.3)
        max_turning_speed = rospy.get_param("/max_turning_speed", -0.6)
        min_turning_speed_right = rospy.get_param("/min_turning_speed_right", -0.3)
        min_turning_speed_left = rospy.get_param("/min_turning_speed_left", 0.3)

        middle_man()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass