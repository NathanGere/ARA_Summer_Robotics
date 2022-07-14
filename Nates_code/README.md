# Nate's Code
## Overview
The objectice Nathan was trying to reach was trying get the robot to make an acurate map of a room with... this was completed with assistance from a Gazebo world

## Running Demo
```bash
#Terminal 1:
roslaunch Nate_mapping_and_naviagtion autonomous_mapping_gazebo.launch
```
This will open Rviz showing a border and a laser scan of the world you are loading into. Simultaneously Gazebo will open and provide you with the world that you want to navagate.

## RIGHT WALL HUGGING STUFF FOR COWORKERS
```bash
#Terminal 1:
cd /home/csrobot/stretch_ws/src/ARA_Summer_Robotics
```
You will need to checkout my branch  to get my updated files, but first, save your work on main:
```bash
#Terminal 1:
git add your_folder/.
git commit -m "saving files before swapping branches"
git checkout feature/nate/right_wall_hugger
git fetch
```
Try not to swap back to main (git checkout main) when you are done. Now we should be ready to begin.
```bash
#Terminal 1:
roslaunch stretch_gazebo gazebo.launch world:="/home/csrobot/stretch_ws/src/ARA_Summer_Robotics/Gazebo_worlds/DuckieTown_world.world"
```
This will just open the DuckieTown world for the robot to navigate in.

```bash
#Terminal 2:
roslaunch nate_stretch_movement wall_follower.launch
```
This launches my wall following file (which does not work LOL :( ). This program has issues when it comes to turning left.
Alternatively, try the command below, which does the same thing, but crashes without turning:

```bash
#Terminal 2:
roslaunch nate_stretch_movement rwh.launch
```

## USEFUL METHODS IN MY FILES

While my overall program does not work, parts of it does.
In visual studios please find rwh.cpp:
Nates_code -> nate_stretch_movement -> src -> rwh.cpp

You may have noticed that the robot does actually get lined up with the wall correctly at the start of my code.
The methods responsible for this are:

```bash
void get_oriented(const sensor_msgs::LaserScan &scan);
void get_closer(const sensor_msgs::LaserScan &scan);
void get_aligned(const sensor_msgs::LaserScan &scan);
```

I also found some useful methods online for moving the robot by specified amounts. They work by constantly comparing the robot's current odometry to its starting position with transforms in order to calculate how far it has traveled. For use find:

```bash
bool turnOdom(bool clockwise, double radians);
bool driveForwardOdom(double distance);
```
For example, turnOdom can be used like follows:
```bash
bool clockwise = false; //for turning left
double radians = 1.2; //for turning 90 degrees. It should be 1.57, but the robot keeps moving a little after turning, so just subtract accordingly

if(turnOdom(clockwise, radians)) ROS_INFO("Turning was successful");
```
## FOR PYTHON LOSERS

If you take a look at src in nate_stretch_movement, you will notice robot_driver.cpp and python_robot_driver.py

Python_robot_driver.py only contains a teleop, but you should be able to see how it is similar to robot_driver.cpp (just without turnOdom and driveForwardOdom). If you want these methods in python (they are pretty useful, up to you), they should not be too hard to convert. Good luck nerds.

