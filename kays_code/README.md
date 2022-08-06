# Kay's Code
## Overview
The objective of these codes is to be used as a client interface to allow for easier communication with Stretch Robot. Additionally there is a collision prevention package which is compatable with the Stretch Robot.

## stretch_robot_client_interface_pkg
The purpose of this package is to contain all the needed files for the client interface.

This package also includes a launch file which launches a client interface application of the created collision prevention package as well as a server on a rosbridge on port 2022.

## collision_prevention_pkg
This package is currently able avoid obsticals using the lidar of the stretch robot. 

There is a python shell file for a client and a server that can be adjusted based on the application.

Using the current server the teleop twist python file provided by hello_robot was adjusted to help with collision prevention.

All default topics are for stretch robot in gazebo!
