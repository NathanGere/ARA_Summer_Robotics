#!/usr/bin/env python
import rospy
from time import time, sleep
from map_enclosed import map
# from map_explorer import movement


# Params
get_map_service_name = rospy.get_param("~get_map_service_name", "static_map")       # Topic for GetMap service
#twist_topic = rospy.get_param("~twist_topic", "/cmd_vel")                           # Topic for the Command velocity
#laser_topic = rospy.get_param("laser_topic", "/scan")                               # Topic for the LaserScan
#obstacle_avoid_distance = rospy.get_param("~obstacle_avoid_distance", 1.0)          # Distance at which obstacle avoidance kicks in (m)
#straight_ahead_angle = rospy.get_param("~straight_ahead_angle", 0)                  # LaserScan angle that corresponds to straight ahead (deg)
#laser_refresh_rate = rospy.get_param("~laser_refresh_rate", 5)                      # Refresh rate of the LaserScan (sec)
#explore_time = rospy.get_param("~explore_time", 10)                                 # Time to explore before checking the map (sec)
#time_before_intervention = rospy.get_param("~intervention_time", 5)                 # Time before asking if the user wants to continue mapping or cut losses and format (minutes)
#speed = rospy.get_param("~speed", 0.5)                                              # Linear speed of the base (m/sec)
#turn = rospy.get_param("~turn", 0.3)                                                # Angular speed of the base (rad/sec)


# Temp variable because the param doesnt exist yet
time_before_intervention = 121


# This file is the main file that will do the enitre autonomous mapping based off map_enclosed and map_explore
# map_explore has not been made yet, and map_enclosed might need a function to convert possible wall gaps into xy coords
if __name__ == "__main__":
     # ROS Node
     #rospy.init_node("slam_map_room")
     #map = map(get_map_service_name)
     #movement = movement()

     # Dealing with time
     start_time = int(time())
     time_before_intervention_sec = time_before_intervention * 60   # convert to seconds


     #map_enclosed = map.is_enclosed_from_scratch()
     map_enclosed = False
     while map_enclosed == False:
          #movement.explore(explore_time)
          #map_enclosed = map.is_enclosed_from_scratch()
          current_time = int(time())
          time_left = start_time + time_before_intervention_sec - current_time
          if time_left <= 0:
               # Time to ask for intervention
               print("It has been {} minutes and I havent finished.".format(time_before_intervention))
               print("You can see the current version of the map in RViz.")
               print("If you would like to continue scanning, respond with (y)es.")
               print("If you are satisfied with the map and would like it formatted, respond with (n)o.")
               user_request = str(raw_input("Continue Scanning?   (y/n): "))
               if user_request.lower() == 'n':
                    #map.find_wall_gaps()
                    #print("Formatting Map")
                    #map.cut_losses_and_format_map()
                    # Publishing the map so it can be saved by map_saver
                    #print("Publishing Closed Map.")
                    #map.publish_closed_map()
                    #print('Enclosed Map Written as "make a text variable for the enclosed maps file name"')
                    break
               # Reset the intervention timer by making start_time the current time
               start_time = int(time())
          # 10 second countdown
          elif time_left <= 10:
               print(int(time_left))
               sleep(1)
          # Hour mark
          elif time_left % 3600 == 0:
               hours_left = int(time_left/3600)
               if hours_left == 1:
                    print("{} hour left...".format(hours_left))
               else:
                    print("{} hours left...".format(hours_left))
               sleep(1)
          # Minute mark if under an hour
          elif time_left % 60 == 0 and time_left < 3600:
               minutes_left = int(time_left/60)
               if minutes_left == 1:
                    print("{} minute left...".format(minutes_left))
               else:
                    print("{} minutes left...".format(minutes_left))
               sleep(1)
          
               

     print("Completed.")




## current goal is having minute updates and having a variable for the final map file name


### Important info for tomorrow!!!!! : origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw. 
##### ^^^^^^ found at http://wiki.ros.org/map_server
