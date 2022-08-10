#!/bin/bash

var1="cat"
var2="person"
test_char1="y"
test_char2="Y"
test_char3="n"
test_char4="N"

if [ "$var1" != "$var2" ];
then
    # Move to this directory
    cd src/ARA_Summer_Robotics/matt_code/matt_human_detector/launch/

    # Prompt user to launch specific launch file
    echo Do you want to launch the pcl seeker y/n?
    read userInput
    

    if [[ -z "$userInput" ]];
    then
        echo Empty!
        exit 0
    fi

    if [[ "$userInput" == [a-zA-Z] ]];
        echo Is Letter!
    then
        if [[ "$userInput" == "$test_char1" || "$test_char2" ]];
        then
            echo Launching P.C.L. Wall Follower!
            roslaunch matt_human_detector pcl_wf.launch
        fi
        if [[ "$userInput" == "$test_char3" || "$test_char4" ]];
        then
            echo Launching Lidar Wall Follower!
            roslaunch matt_human_detector lidar_wf.launch
        fi
        if [[ "$userInput" != "$test_char1" || "$test_char2" || "$test_char3" || "$test_char4" ]];
        then
            echo Invalid input!
        fi
    else
        echo Invalid input!
    fi

    # Move to this directory
    cd ../../
    cd depthai-python/examples/ObjectTracker/

    # start the camera to look for human
    python3 object_tracker.py   

    # Check bool.txt
    filename='bool.txt'
    n=1

    while read line;
    do
        # reading each line
        echo "Line No. $n : $line"
        n=$((n+1))
        done < $filename

    # If person is returned from bool.txt
    if grep -q person "$filename";
    then
    
        echo "">| bool.txt
        # Move to this directory
        cd ../../../
        cd matt_human_detector/txt/

        echo ------------------------------------------------------------------------------------
        echo In bash, passing argument... person ...to read.txt
    
        # Write to read.txt that person was found
        echo "person">| read.txt
        echo Passing person is sucessful!

        echo ------------------------------------------------------------------------------------

    fi
fi