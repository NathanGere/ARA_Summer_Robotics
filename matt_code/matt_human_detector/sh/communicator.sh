#!/bin/bash

var1="cat"
var2="person"

if [ "$var1" != "$var2" ]; then
    # Move to this directory
    cd src/ARA_Summer_Robotics/matt_code/depthai-python/examples/ObjectTracker/

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