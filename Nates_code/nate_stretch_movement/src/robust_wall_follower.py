#!/usr/bin/env python

from numpy import indices
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#global variables
oriented = False #is robot facing a wall?
close_enough = False #is the robot hugging a wall?
aligned = False #is the robot parallel to a wall?

#variables to trigger new crash fixing attempts in certain scenarios
case_5_tracker = 0
case_7_tracker = 0
case_10_tracker = 0
case_13_tracker = 0
case_14_tracker = 0
case_15_tracker = 0
#var to reset explorer method
reset = 0
#change mode
mode = 0

#params
#std::string cmd_vel_output
#std::string laser_scan

class WallFollower:

    def __init__(self, pub, scan, size, wall_on_left, wall_on_right, wall_in_front, indices_of_collisions, index_mem, distance_mem):
        self.wf_pub = pub
        self.wf_scan = scan
        self.motor_cmd = Twist()
        self.wf_size = size
        self.wf_wall_on_left = wall_on_left
        self.wf_wall_on_right = wall_on_right
        self.wf_wall_in_front = wall_in_front
        self.wf_indices_of_collisions = indices_of_collisions
        self.wf_index_mem = index_mem
        self.wf_distance_mem = distance_mem
        #self._index_mem = index_mem
        #self._distance_mem = distance_mem

    #METHOD TO HAVE ROBOT FIND WALLS
    def explorer(self):

        global reset

        if reset < 51:
            self.motor_cmd.linear.x = 1.5 #search left and forward
            self.motor_cmd.angular.z = 0.1
            self.wf_pub.publish(self.motor_cmd)
            print("Exploring . . .")
        
        elif reset < 101:
            self.motor_cmd.linear.x = 1.5 #search right and forward
            self.motor_cmd.angular.z = -0.3
            self.wf_pub.publish(self.motor_cmd)
            print("Exploring . . .")
        
        else:
            reset = 0 #cycle back to previous conditions
        
        reset+=1

    #METHOD TO GET ROBOT OUT OF STICKY SITUATIONS
    def crash_recovery(self):

        print("Avoiding crash . . . ")
        
        #setting up globals
        global case_5_tracker
        global case_7_tracker
        global case_10_tracker
        global case_13_tracker
        global case_14_tracker
        global case_15_tracker

        #sides of crash
        BL = 0
        BR = 0
        FR = 0
        FL = 0
        
        global crashed

        c = 0
        while not rospy.is_shutdown and c < self.wf_size:
        
            if 1890 <= self.wf_indices_of_collisions[c] and self.wf_indices_of_collisions[c] <= 1999 or 0 <= self.wf_indices_of_collisions[c] and self.wf_indices_of_collisions[c] <= 300:
                BL = 1
            if 300 <= self.wf_indices_of_collisions[c] and self.wf_indices_of_collisions[c] <= 820:
                BR = 2
            if 820 <= self.wf_indices_of_collisions[c] and self.wf_indices_of_collisions[c] <= 1361:
                FR = 4
            if 1361 <= self.wf_indices_of_collisions[c] and self.wf_indices_of_collisions[c] <= 1890:
                FL = 8
            c+=1
        
        type_of_crash = BL + BR + FR + FL
        
        if type_of_crash == 0: #no Near collisions
            print("case 0: No collisions detected")
            case_5_tracker = 0
            crashed = False
            
        if type_of_crash == 1: #collison back left
            print("case 1: Near collision back left")
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 2: #Near collision back right
            print("case 2: Near collision back right")
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 3: #Near collisions on back left and right
            print("case 3: Near collisions back left and right")
            self.motor_cmd.linear.x = 1.0
            
        if type_of_crash == 4: #Near collision front right
            print("case 4: Near collision front right")
            self.motor_cmd.angular.z = -0.5 
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 5: #Near collision front right and back left
            print("case 5: Near collision front right and back left")

            if case_5_tracker < 7: 
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                case_5_tracker+=1
            
            elif case_5_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                case_5_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_5_tracker = 0
            
        if type_of_crash == 6: #Near collisions front right and back right
            print("case 6: Near collisions back right and front right")
            self.motor_cmd.angular.z = 0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 7:
            print("case 7: Near collisions front right, back right, and back left")

            if case_7_tracker < 7:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                case_7_tracker+=1
            
            elif case_7_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = -0.1
                case_7_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_7_tracker = 0
                
        if type_of_crash == 8:
            print("case 8: Near collision front left")
            self.motor_cmd.angular.z = 0.5 
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 9:
            print("case 9: Near collisions front left and back left")
            self.motor_cmd.angular.z = -0.5
            self.motor_cmd.linear.x = 0.5
            
        if type_of_crash == 10:
            print("case 10: Near collisions front left and back right")

            if case_10_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                case_10_tracker+=1
            
            elif case_7_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1
                case_10_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_10_tracker = 0
            
        if type_of_crash == 11:
            print("case 11: Near collisions front left, back left, and back right")

            if case_11_tracker < 7:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                case_11_tracker+=1
            
            elif case_11_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = -0.1
                case_11_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_11_tracker = 0
            
        if type_of_crash == 12:
            print("case 12: Near collisions front left and front right")
            self.motor_cmd.linear.x = -0.5
            
        if type_of_crash == 13:
            print("case 13: Near collisions front left, back left, and front right")

            if case_13_tracker < 7:
                self.motor_cmd.angular.z = 0.5 
                self.motor_cmd.linear.x = -0.1
                case_13_tracker+=1
            
            elif case_13_tracker < 15:
                self.motor_cmd.angular.z = -0.5
                self.motor_cmd.linear.x = 0.1
                case_13_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_13_tracker = 0
            
        if type_of_crash == 14:
            print("case 14: Near collisions front left, front right, and back right")
            if case_14_tracker < 7:
                self.motor_cmd.angular.z = -0.5 
                self.motor_cmd.linear.x = -0.1
                case_14_tracker+=1
            
            elif case_14_tracker < 15:
                self.motor_cmd.angular.z = 0.5
                self.motor_cmd.linear.x = 0.1
                case_14_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_14_tracker = 0
            
        if type_of_crash == 15:
            print("case 15: Near collisions on multiple sides.")

            if case_15_tracker < 4:
                self.motor_cmd.linear.x = 0.1
                case_15_tracker+=1
            
            elif case_15_tracker < 8:
                self.motor_cmd.linear.x = -0.1
                case_15_tracker+=1
            
            elif case_15_tracker < 20:
            
                self.motor_cmd.angular.z = -0.5
                case_15_tracker+=1
            
            else:
                print("This crash may be unrecoverable.")
                case_15_tracker = 0
                
        #move according to conditions for a bit
        recovery_time = rospy.Rate(10)
        self.wf_pub.publish(self.motor_cmd)
        recovery_time.sleep()

        #stopping robot before it crashes again
        self.motor_cmd.linear.x = 0
        self.motor_cmd.angular.z = 0
        self.wf_pub.publish(self.motor_cmd)

    #METHOD TO HAVE ROBOT FACE NEAREST WALL
    def get_oriented(self):

        #stopping robot in the case that the lost method was previously called
        self.motor_cmd.linear.x = 0.0
        self.wf_pub.publish(self.motor_cmd)

        #floats will store the distance to nearest object at cherrypicked indices
        right = self.wf_scan.ranges[820]
        center = self.wf_scan.ranges[1361]
        left = self.wf_scan.ranges[1890]

        #iteration variable
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < self.wf_size:
        
            if self.wf_scan.ranges[i] < self.wf_scan.ranges[self.wf_index_mem]:
            
                wf_distance_mem = self.wf_scan.ranges[i]
                wf_index_mem = i
            
            i+=1
        
        #var to track whether or not the front of the robot is facing the nearest wall
        facing = False

        #if the front of the robot is aligned with the closest wall, the facing boolean will be set to true
        if center > self.wf_distance_mem - 0.02 and center < self.wf_distance_mem + 0.02:
        
            facing = True
        
        #if its not facing the wall, the robot will turn to its right
        if not facing:
        
            if self.wf_index_mem > 300 and self.wf_index_mem < 1361:
            
                self.motor_cmd.angular.z = -0.5
                self.wf_pub.publish(self.motor_cmd)
                print("Orienting . . .") 
            
            else:
            
                self.motor_cmd.angular.z = 0.5
                self.wf_pub.publish(self.motor_cmd)
                print("Orienting . . .") 
            
        #if the robot is facing the wall, the orientation is complete
        else:
            global oriented
            oriented = True
            print("INITIAL ORIENTATION COMPLETE.")
        

    #WILL MOVE ROBOT SO IT IS 0.5 METERS FROM WALL IT IS FACING
    def get_closer(self):

        #float to store distance in front of the object
        front = self.wf_scan.ranges[1361]

        #once the robot is close enough, it will proceed
        if front > 0.5 and front < 0.6:

            global close_enough
            close_enough = True
            print("PREFERED INITIAL DISTANCE ACHIEVED.")
        
        #if there is space, the robot will get closer to the wall
        elif front > 0.4:
        
            self.motor_cmd.linear.x = 0.5
            self.wf_pub.publish(self.motor_cmd)
            print("Establishing initial distance . . .")
        
        #if the robot is too close, it will back up
        else:
        
            self.motor_cmd.linear.x = -0.5
            self.wf_pub.publish(self.motor_cmd)
            print("Establishing initial distance . . .")
        
    #METHOD TO GET ROBOTS RIGHT SIDE PARALLEL WITH A WALL
    def get_aligned(self):

        #float to store distance in front of the object
        right = self.wf_scan.ranges[820]

        #iteration variable for while loop
        i = 0
        #will loop through to find the distance of the closest wall
        while not rospy.is_shutdown() and i < self.wf_size:
        
            if self.wf_scan.ranges[i] < self.wf_scan.ranges[self.wf_index_mem]:
            
                self.wf_distance_mem = self.wf_scan.ranges[i]
                self.wf_index_mem = i
            
            i+=1
        
        #if aligned, the global var will be set to true, and this function will no longer be called
        if right < self.wf_distance_mem + 0.02 and right > self.wf_distance_mem - 0.02:
            
            global aligned
            aligned = True
            print("FIRST ALIGNMENT COMPLETE.")
        
        #will turn the robot to its left if it is not aligned with the nearest wall
        else:
        
            self.motor_cmd.angular.z = 0.5
            self.wf_pub.publish(self.motor_cmd)
            print("Completing first alignment . . .")
        

    #METHOD TO FOLLOW WALLS ON LEFT
    def left_wall_follower(self):

        #calculations for switch statement
        l = 0
        f = 0
        r = 0
        #setting up calculators
        if self.wf_wall_on_left:
            l = 1
        if self.wf_wall_in_front: 
            f = 2
        if self.wf_wall_on_right: 
            r = 4
        #var for switch statment
        wall_locations = l + f + r

        if wall_locations == 0: #no walls
            print("lwf: case 0")
            self.motor_cmd.linear.x = 0.5
            self.motor_cmd.angular.z = 1.0
        
        if wall_locations == 1: #wall on left
            print("lwf: case 1")
            self.motor_cmd.linear.x = 0.5

        if wall_locations == 2: #wall in front
            print("lwf: case 2")
            self.motor_cmd.angular.z = -0.5
        
        if wall_locations == 3: #wall in front and on left
            print("lwf: case 3")
            self.motor_cmd.angular.z = -0.5
        
        if wall_locations == 4: #wall on right
            print("lwf: case 4")
            self.motor_cmd.linear.x = 0.5
            self.motor_cmd.angular.z = -0.65
        
        if wall_locations == 5: #wall on left and right
            print("lwf: case 5")
            self.motor_cmd.linear.x = 0.5

        if wall_locations == 6: #wall front and right
            print("lwf: case 6")
            self.motor_cmd.angular.z = -0.5
        
        if wall_locations == 7: #walls front, left, and right
            print("lwf: case 7")
            self.motor_cmd.angular.z = -0.5
        
        if self.wf_scan.ranges[820] < 0.25:
        
            print("lwf: close on right")
            self.motor_cmd.linear.x = 0.3
            self.motor_cmd.angular.z = 1.0
        
        if self.wf_scan.ranges[1890] < 0.25:
        
            print("lwf: close on left")
            self.motor_cmd.angular.z = -1.0
            self.motor_cmd.linear.x = 0.3
        
        self.wf_pub.publish(self.motor_cmd)

    #METHOD TO FOLLOW WALLS ON THE RIGHT
    def right_wall_follower(self):

        #calculations for switch statement
        l = 0
        f = 0
        r = 0
        #setting up calculators
        if self.wf_wall_on_left:
            l = 1
        if self.wf_wall_in_front:
            f = 2
        if self.wf_wall_on_right:
            r = 4
        #var for condtions
        wall_locations = l + f + r
        
        if wall_locations == 0: #no walls
            print("rwf: case 0")
            self.motor_cmd.linear.x = 0.5
            self.motor_cmd.angular.z = -1.0
            
        if wall_locations == 1: #wall on left
            print("rwf: case 1")
            self.motor_cmd.linear.x = 0.5
            self.motor_cmd.angular.z = -0.65
            
        if wall_locations == 2: #wall in front
            print("rwf: case 2")
            self.motor_cmd.angular.z = 0.5
        if wall_locations == 3: #wall in front and on left
            print("rwf: case 3")
            self.motor_cmd.angular.z = 0.5
            
        if wall_locations == 4: #wall on right
            print("rwf: case 4")
            self.motor_cmd.linear.x = 0.5
            
        if wall_locations == 5: #wall on left and right
            print("rwf: case 5")
            self.motor_cmd.linear.x = 0.5
            
        if wall_locations == 6: #wall front and right
            print("rwf: case 6")
            self.motor_cmd.angular.z = 0.5
            
        if wall_locations == 7: #walls front, left, and right
            print("rwf: case 7")
            self.motor_cmd.angular.z = 0.5
                
        
        if self.wf_scan.ranges[820] < 0.25: 
        
            print("close on right")
            self.motor_cmd.linear.x = 0.3
            self.motor_cmd.angular.z = 1.0
        
        if self.wf_scan.ranges[1890] < 0.25:
        
            print("close on left")
            self.motor_cmd.angular.z = -1.0
            self.motor_cmd.linear.x = 0.3
        
        self.wf_pub.publish(self.motor_cmd)

#METHOD TO DECIDE WHICH METHOD TO CALL
def thinker(scan):

    #setting up publisher
    pub = rospy.Publisher("/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=10)

    #number of indices in laser scan array
    size = len(scan.ranges)

    #loop iteration variable
    i = 0
    #tracker for nearby walls
    global mode
    global oriented
    global close_enough
    global aligned
    walls_nearby = False
    crashed = False
    wall_on_left = False
    wall_on_right = False
    wall_in_front = False
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
    
    #will calculate which sides of the robot there are walls on
    i = 0
    while not rospy.is_shutdown() and i < 47:
    
        if scan.ranges[i] < 0.5:
            wall_on_left = True

        i+=1
    
    i = 880 #' 624 or 824 to 877
    while not rospy.is_shutdown() and i < 890:
       
        if scan.ranges[i] < 0.5:
            wall_on_right = True
        i+=1

    i = 1099
    while not rospy.is_shutdown() and i < 1639:
    
        if scan.ranges[i] < 0.5:
            wall_in_front = True

        i+=1

    i = 1639
    while not rospy.is_shutdown() and i < 1640:

        if scan.ranges[i] < 0.5:
            wall_in_front = True
        i+=1

    i = 1825
    while not rospy.is_shutdown() and i < 1977:
    
        if scan.ranges[i] < 0.5:
            wall_on_left = True 
        i+=1
    
    #setting up instance of wall follower class
    wf = WallFollower(pub, scan, size, wall_on_left, wall_on_right, wall_in_front, indices_of_collisions, index_mem, distance_mem)

    #will activate explorer method in next loop of ros::spinOnce()
    if not walls_nearby:
    
        lost = True
    
    else:
    
        lost = False
    
    if lost:
    
        wf.explorer()
    
    elif crashed:
    
        wf.crash_recovery()
    
    else:
    
        if not oriented:
        
            wf.get_oriented()
        
        elif not close_enough:
        
            wf.get_closer()
        
        elif not aligned:
        
            wf.get_aligned()
        
        elif mode < 600:
        
            wf.right_wall_follower()
            mode+=1
        
        else:
        
            wf.left_wall_follower()
            mode+=1
            if mode > 1200:
                mode = 0
            
#BASICALLY ALSO MAIN
def launchpoint():
    try:
        #setting up node
        rospy.init_node("robust_wall_follower_python_node", anonymous =True)

        #setting up subscriber
        sub = rospy.Subscriber("/scan", LaserScan, thinker)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#MAIN
if __name__ == '__main__':
    launchpoint()