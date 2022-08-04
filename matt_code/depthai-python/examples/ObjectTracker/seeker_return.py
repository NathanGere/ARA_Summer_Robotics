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

        print("------------------------------------------------------------------------------------")
        print("Searching label.txt to find what object_tracker has given!\n")

        # Gets value from label.txt, and gives it to human_detector
        human_detector = object_receiver.person_return(self)

        print("From object_tracker... label responds with: " + human_detector)
        print("")

        #Allows for easy access to last return from object_tracker.py
        print("Saving current return to saver.txt\n")
        object_receiver.last_return(self, human_detector)

        # Check-up to make sure that human_detector has value of "person"
        if human_detector == "person":
            print("Human found!\n")

        print("Rewriting label.txt to empty to avoid miscalculations!\n\n")

        # Re-write label.txt to make sure there is no false positives
        label1 = "label.txt"
        object_receiver.rewrite_file(self, empty, label1)

        # After three true conditions, give this return
        i = 0
        while i < 3:
            human_bool = object_receiver.boolean_return(self, human_detector)

            label2 = "latest_saver.txt"
            object_receiver.rewrite_file(self, empty, label2)

            temp = human_detector

            with open("latest_saver.txt", "w+") as fp:
                fp.write(temp) 

            i+=1

        if human_bool == False:
            time.sleep(5)
        
        print("------------------------------------------------------------------------------------")

        # return human_bool
        sys.exit(1)
################################################################
    def person_return(self):

        with open("label.txt", "r") as fp:
            human_detector = fp.read()

            return human_detector
################################################################
    def last_return(self, human_detector):

        with open("latest_saver.txt", "w+") as fp:
            fp.write(human_detector) 

        with open("latest_saver.txt", "r") as fp:
            save = fp.read() 
            print("Last save is: " + save)
            print("")
################################################################
    def rewrite_file(self, empty, name):

        with open(name, "w+") as fp:
            fp.write(empty) 
################################################################
    def boolean_return(self, human_detector):

        # Increment through here three times, to avoid false "person"
        with open("counter.txt", "r") as fp:
            i = fp.read()
            print("We are on increment #" + i)

        if int(i) < 3:
            with open("counter.txt", "w+") as fp:
                j = int(i) + 1
                fp.write('{}'.format(j))
            
            with open("label.txt", "r") as fp:
                label = fp.read()
                print("\tLabel from label.txt is: " + label)

            with open("latest_saver.txt", "r") as fp:
                latest_saver = fp.read()
                print("\tLatest_saver from latest_saver.txt is: " + latest_saver)

            if label == latest_saver:
                print("\nLabel and lastest_saver are the same!\n")
                human_bool = False
        time.sleep(5)
        
        if int(i) == 3:

            with open("label.txt", "r") as fp:
                label = fp.read()
                print("\tLabel from label.txt is: " + label)

            with open("latest_saver.txt", "r") as fp:
                latest_saver = fp.read()
                print("\tLatest_saver from latest_saver.txt is: " + latest_saver)

            if label == latest_saver:
                print("\nLabel and lastest_saver are the same!\n")
                human_bool = False

            print("\n")

            if human_detector == "person":
                # time.sleep(3)
                human_bool = True
                print("human_bool returns true!\n")
                with open("bool.txt", "w+") as fp:
                    fp.write("person") 
            else:
                human_bool = False
                # time.sleep(3)
                print("No human! Keep searching!\n")
            
            with open("counter.txt", "w+") as fp:
                k = 1
                fp.write('{}'.format(k))

            return human_bool
################################################################

    def rewrite_file(self, empty, name):

        with open(name, "w+") as fp:
            fp.write(empty) 

################################################################