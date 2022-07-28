# Matthew's Code
## Overview
The objective Matthew was trying to reach was being able to have the robot seek a human model within the house. If there was no human model detected (the user), it would keep seeking throughout the house until the human model is found.

## Equipment
Matthew used the OAK-D Lite camera and their code (which you can find here... https://github.com/luxonis).

## Installation
There is a few things you need to downlaod to make their code compatable with your computer (The steps to install their code can be found on this website... https://docs.luxonis.com/projects/api/en/latest/install/#).

## Troubleshooting
If there is bugs or errors you don't understand you can reference this website to help out with the most common ones (Most of the things you will have trouble with will most liekly be answered here... https://docs.luxonis.com/en/latest/pages/troubleshooting/)

## Running Demo
```bash
#Terminal 1:
python3 object_tracker.py
# Note, you want to have the entire file path already set up to launch
# ~/catkin_ws/src/depthai-python/examples/ObjectTracker$ python3 object_tracker.py
```
## Note
I pushed the entire deptai-python directory, if you pull it without the proper installations, you will get wonky errors. I recommend going to the Installation sections and following instructions to download the entire software for yourself. Then grapping the custom made files and adding them to your newly installed depthai-python directory. 

Copy this line into your depthai-python/examples/CMakeLists.txt under the # Add examples -> ## ObjectTracker

```bash
add_python_example(seeker_return ObjectTracker/seeker_return.py)
```

In your object_tracker.py script, add any code that doesn't resemble yours you just downlaoded, and as well as my seeker_return.py