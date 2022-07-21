#!/usr/bin/env python

import rospy #always need
import sys #needed for exit


human = True

def map_terminator():
    print(" ")
    print("-----------------------------")
    print("Enter '~' to cancel mapping node")
    letter = raw_input("Enter Your Letter ")

    if letter == '~':
        print("Killing Mapper Node")
        print("-----------------------------")
        print(" ")
        return True
    else:
        print("Invalid input, try again!")
        print("-----------------------------")
        print(" ")
        map_terminator()

def mover(human):
    print("-----------------------------")
    if human == True:
        print("Success! Human found!")
        print("Terminating all other programs!")
        print("-----------------------------")
        print(" ")
    else:
        print("Error! No Human!")
        print("Moving Rooms!")
        print("-----------------------------")
        print(" ")

if __name__ == "__main__":

    # If user enters letters, it will terminate the corresponding node
    # result = map_terminator()

    #Look for human
    human = human_detector()

    #If human is detected, return true and terminate all programs except this one and display sucess
    mover(human)

    #Executes code forever until user enters Ctrl + C
    # rospy.spin()