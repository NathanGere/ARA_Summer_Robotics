#!/usr/bin/env python3

import time
import sys

class object_receiver:
################################################################
    def main_object(self):

        # Set up local variables
        human_detector = "empty"
        human_bool = False
        empty = "empty"
        checker = "Checkmate"

        print("------------------------------------------------------------------------------------\n")
        print("Searching label.txt to find what object_tracker has given!\n")

        # Gets value from label.txt, and gives it to human_detector
        human_detector = object_receiver.person_return(self)

        print("From object_tracker... label responds with: " + human_detector)
        print("")

        # Check-up to make sure that human_detector has value of "person"
        if human_detector == "person":
            print("Human found!\n")

        print("Rewriting label.txt to empty to avoid miscalculations!\n")

        # Re-write label.txt to make sure there is no false positives
        label1 = "label.txt"
        object_receiver.rewrite_file(self, empty, label1)

        # After three true conditions, give this return
        human_bool = object_receiver.boolean_return(self, human_detector)
        if human_bool == True:
            with open("bool.txt", "w+") as fp:
                fp.write("person") 
        print("------------------------------------------------------------------------------------")

        # return human_bool
        sys.exit(1)
################################################################
    def person_return(self):

        with open("label.txt", "r") as fp:
            human_detector = fp.read()

            return human_detector
################################################################
    def rewrite_file(self, empty, name):

        with open(name, "w+") as fp:
            fp.write(empty) 
################################################################
    def boolean_return(self, human_detector):

            if human_detector == "person":
                human_bool = True
                print("human_bool returns true!\n")
            else:
                human_bool = False
                print("No human! Keep searching!\n")

            return human_bool
################################################################
    def rewrite_file(self, empty, name):

        with open(name, "w+") as fp:
            fp.write(empty) 
################################################################