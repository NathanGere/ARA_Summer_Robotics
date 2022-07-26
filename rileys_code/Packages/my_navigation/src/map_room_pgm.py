#!/usr/bin/env python
import rospy
from numpy import angle, linspace, inf, tanh
from math import sin, pi
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


# Params
get_map_service_name = rospy.get_param("~get_map_service_name", "static_map")       # Topic for GetMap service
twist_topic = rospy.get_param("~twist_topic", "/cmd_vel")                           # Topic for the Command velocity
laser_topic = rospy.get_param("laser_topic", "/scan")                               # Topic for the LaserScan
obstacle_avoid_distance = rospy.get_param("~obstacle_avoid_distance", 1.0)          # Distance at which obstacle avoidance kicks in (m)
straight_ahead_angle = rospy.get_param("~straight_ahead_angle", 0)                  # LaserScan angle that corresponds to straight ahead (deg)
laser_refresh_rate = rospy.get_param("~laser_refresh_rate", 5)                      # Refresh rate of the LaserScan (sec)
explore_time = rospy.get_param("~explore_time", 10)                                 # Time to explore before checking the map (sec)
speed = rospy.get_param("~speed", 0.5)                                              # Linear speed of the base (m/sec)
turn = rospy.get_param("~turn", 0.3)                                                # Angular speed of the base (rad/sec)



class movement:
    def __init__(self):
        self.laser = LaserScan()
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

    def laser_update(self, msg):
        self.laser = msg
    
    def explore(self):
            rate = rospy.Rate(laser_refresh_rate)
            self.twist.linear.x = speed
            twist_pub.publish(self.twist)

            for i in range(int(explore_time * laser_refresh_rate)):

                angle_of_turn = 0
                # Obstacle avoid if needed
                if self.laser.range_min <= obstacle_avoid_distance:
                    if self.laser.angle_increment is 0.0 :
                        print("error imcrement is 0 ")
                        return
                    number_of_scans = (self.laser.angle_max - self.laser.angle_min) / self.laser.angle_increment
                    # Find the angle of the closest object
                    for i in range(number_of_scans):

                        if self.laser.ranges[i] is self.laser.range_min:
                            angle_of_closest_object = self.laser.angle_min + ( i * self.laser.angle_increment)
                    # Turn away from that object 
                    if angle_of_closest_object > straight_ahead_angle:
                        # Object is to the left so turn right
                        angle_of_turn = -90 + (angle_of_closest_object - straight_ahead_angle)
                    elif angle_of_closest_object < straight_ahead_angle:
                        # Object is to the right so turn left
                        angle_of_turn = 90 - (angle_of_closest_object - straight_ahead_angle)
                    else:
                        # Object is straight ahead (default turn angle will be right)
                        angle_of_turn = 90
                    self.twist.angular.z = angle_of_turn / 90
            twist_pub.publish(self.twist)
            rate.sleep

            # Stop
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            twist_pub.publish(self.twist)

# Global variables 
motion = movement()
map = gmap()
laser_sub = rospy.Subscriber(laser_topic, LaserScan, motion.laser_update)
twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=1)


if __name__ == "__main__":
     # ROS Node
    rospy.init_node("slam_map_room")
    motion.explore()





