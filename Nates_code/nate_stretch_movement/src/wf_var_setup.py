#!/usr/bin/env python

from numpy import indices
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WFVarSetup:
    
    def __init__(self, scan):
        self._scan = scan
        self._size = len(scan.ranges)
    
    def pub(self):
        pub = rospy.Publisher("/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=10)
        return pub
    
    def laser_scan(self):
        return self._scan
    
    def length(self):
        size = len(self._scan.ranges)
        return size
    
    def left_wall(self):
        i = 0
        wall_on_left = False

        while not rospy.is_shutdown() and i < 47:
        
            if self._scan.ranges[i] < 0.5:
                wall_on_left = True

            i+=1

        i = 1825
        while not rospy.is_shutdown() and i < 1977:
        
            if self._scan.ranges[i] < 0.5:
                wall_on_left = True 
            i+=1

        return wall_on_left

    def right_wall(self):
        i = 880 #' 624 or 824 to 877
        wall_on_right = False
        while not rospy.is_shutdown() and i < 890:
        
            if self._scan.ranges[i] < 0.5:
                wall_on_right = True
            i+=1
        return wall_on_right

    def front_wall(self):
        i = 880 #' 624 or 824 to 877
        wall_in_front = False
        while not rospy.is_shutdown() and i < 1640:
        
            if self._scan.ranges[i] < 0.5:
                wall_in_front = True

            i+=1
        return wall_in_front

    def collisions(self):

        indices_of_collisions = [3000] * self._size

        i = 0

        while not rospy.is_shutdown() and i < self._size:

            #crash measurements
            if i >= 0 and i < 48 and self._scan.ranges[i] < 0.25:
                indices_of_collisions[i] = i

            elif i > 47 and i < 98 and self._scan.ranges[i] < 0.3:
                indices_of_collisions[i] = i

            elif i > 47 and i < 583 and self._scan.ranges[i] < 0.3:
                indices_of_collisions[i] = i

            elif i > 582 and i < 913 and self._scan.ranges[i] < 0.24:
                indices_of_collisions[i] = i

            elif i > 912 and i < 1803 and self._scan.ranges[i] < 0.19:
                indices_of_collisions[i] = i

            elif i > 1802 and i < 2000 and self._scan.ranges[i] < 0.25:
                indices_of_collisions[i] = i
            else:
                indices_of_collisions[i] = 3000
        
            i+=1

        #print(indices_of_collisions)
        
        return indices_of_collisions

    def nearest_wall_index(self):
        i = 0
        index_mem = 0
        while not rospy.is_shutdown() and i < self._size:

            if self._scan.ranges[i] < self._scan.ranges[index_mem]:
                index_mem = i
            i+=1
        
        return index_mem

    def nearest_wall_distance(self):

        i = 0
        index_mem = 0
        distance_mem = 0.0
        while not rospy.is_shutdown() and i < self._size:

            if self._scan.ranges[i] < self._scan.ranges[index_mem]:
                distance_mem = self._scan.ranges[i]
                index_mem = i
            i+=1
    
        return distance_mem

#SUBSCRIBER CALLBACK
def calling_setup(scan):
    wfvs = WFVarSetup(scan)
    print("The size of the array of laser values is " + str(wfvs.length()))
    if wfvs.left_wall():
        print("There is a wall on the left")
    else:
        print("There is no wall on the left")
    if wfvs.front_wall():
        print("There is a wall in front")
    else:
        print("There is no wall in front")
    if wfvs.right_wall():
        print("There is a wall on the right")
    else:
        print("There is no wall on the right")
    locations_of_collisions = wfvs.collisions()
    #print(locations_of_collisions)
    i = 0
    while not rospy.is_shutdown() and i < 2000:
        if locations_of_collisions[i] != 3000:
            print("There is a collision at scan.ranges[" + str(i) + "]")
        i+=1
    print("The index of the nearest wall is " + str(wfvs.nearest_wall_index()))
    print("The distance to the nearest wall is " + str(wfvs.nearest_wall_distance()))
    
#BASICALLY ALSO MAIN
def jumpstart():
    try:
        #setting up node
        rospy.init_node("WF_Var_Setup_Node", anonymous =True)

        #setting up subscriber
        sub = rospy.Subscriber("/scan", LaserScan, calling_setup)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#MAIN
if __name__ == '__main__':
    jumpstart()