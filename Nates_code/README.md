# Nate's Code
## Overview
The objectice Nathan was trying to reach was trying get the robot to make an acurate map of a room with... this was completed with assistance from a Gazebo world

## Running Demo (Outdated)
```bash
#Terminal 1:
roslaunch Nate_mapping_and_naviagtion autonomous_mapping_gazebo.launch
```
This will open Rviz showing a border and a laser scan of the world you are loading into. Simultaneously Gazebo will open and provide you with the world that you want to navagate.

## WALL FOLLOWER

```bash
#Terminal 1:
roslaunch stretch_gazebo gazebo.launch world:="/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/Gazebo_worlds/DuckieTown_world.world"
```
This will just open the DuckieTown world for the robot to navigate in.

```bash
#Terminal 2:
rosrun nate_stretch_movement wall_follower.py
```
This launches my wall following file if you would like to see it work before using it.

To incorporate my wall follower into one of your files:
```python
  #bin/whatever the hell goes up here for python
  from wall_follower import WallFollower

  #THIS IS METHOD IS PRETTY CRUCIAL TO LAUNCHING THE WALL FOLLOWER
  def middle_man():

      wf = WallFollower() #WallFollower takes your publisher as a param, so if you are not publishing to "/stretch_diff_drive_controller/cmd_vel", you will need to give it your publisher

      def middle_man_2(scan):

          wf.follow_walls_please(scan, wf) #this method can be given False as the last parameter if you do not want my code to print anything

      #setting up subscriber, since the wall follower needs laser scans
      sub = rospy.Subscriber("/scan", LaserScan, middle_man_2)

  if __name__ == '__main__':
      try:
          #setting up node
          rospy.init_node("wall_follower_python_node", anonymous =True)

          #THESE PARAMS ARE NECESSARY FOR THE WALL FOLLOWER, although you can change the values if you are feeling risky
          forward_speed = rospy.get_param("/forward_speed", 0.3)
          max_turning_speed = rospy.get_param("/max_turning_speed", -0.5)
          min_turning_speed_right = rospy.get_param("/min_turning_speed_right", -0.3)
          min_turning_speed_left = rospy.get_param("/min_turning_speed_left", 0.3)

          middle_man() #function that actually calls the wallfollower

          rospy.spin() #change to spinOnce in a while loop if you want to change parameters while running

      except rospy.ROSInterruptException:
          pass
```

