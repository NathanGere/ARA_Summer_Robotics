#!/bin/bash

cd src/ARA_Summer_Robotics/matt_code/depthai-python/examples/ObjectTracker/

filename='bool.txt'
n=1
while read line; do
# reading each line
echo "Line No. $n : $line"
n=$((n+1))
done < $filename

# cat latest_saver.txt
# print person!!!!!!!!!!!!!!!!!!!!
if grep -q person "$filename"; then

    echo found human
    cd ../../../
    cd matt_human_detector/src/

    # fp=$1
    # if [ -f "$fp" ]; then
        # echo "File exists"
    # else
        # echo "File does not exist"
    # fi 
    
    # bash /file/path/to/file.sh file.txt

    echo "person">> read.txt
    cat p_saver.txt

    # STR="person"
    # python seeker_client.py
    # python seeker_server.py "${STR}"
    # roslaunch matt_human_detector human_detector.launch $STR
fi