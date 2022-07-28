#!/usr/bin/env python3

import time

def person_response_returner():
    print("----------------------------")
    print ("Now in seeker_return File")
    print("----------------------------\n")
    print("-----------------------------------------------------------")
    print("Searching label.txt to find what object_tracker has given!")
    print("-----------------------------------------------------------\n")
    file = open("label.txt", 'r')
    human_detector = file.read()
    # human_detector = "person"

    print("--------------------------------------------------------------")
    print("From object_tracker... label responds with: " + human_detector)
    print("--------------------------------------------------------------\n")

    if human_detector == "person":
        print("---------------")
        print("Human found!")
        print("---------------\n")

    file.close()

    print("--------------------------------------------------------")
    print("Rewriting label.txt to empty to avoid miscalculations!")
    print("--------------------------------------------------------\n")

    with open("label.txt", "w+") as filep:
        space = " "
        filep.write(space)

    human_bool = False

    if human_detector == "person":
        human_bool = True
        print("---------------------------------------------------------------------------------------------------")
        print("Human_bool returns true! All moving functions will be terminated and human will recieve delivery!")
        print("---------------------------------------------------------------------------------------------------\n")
    else:
        human_bool = False
        print("----------------------------")
        print("No human! Keep searching!")
        print("----------------------------\n")

    file_pointer = open("label.txt", "r")
    checker = file_pointer.read()
    print("--------------")
    print("Checker is: " + checker)
    print("--------------\n")
    file_pointer.close()    
    print("--------------------------")
    print("Now terminating program!")
    print("--------------------------\n")
    print("------------------------------------------------------------------------------------")
    time.sleep(10)