#!/usr/bin/env python3

def person_response_returner():
    print("----------------------------")
    print ("Now in seeker_return File")
    print("----------------------------\n")
    print("Searching label.txt to find what object_tracker has given!\n")
    file = open("label.txt", 'r')
    human_detector = file.read()
    # human_detector = "person"

    print("From object_tracker... label responds with: " + human_detector)

    # print("\n")
    if human_detector == "person":
        print("Human found!\n")
    else:
        print("No human found!\n")
    file.close()

    with open("label.txt", "w+") as filep:
        space = " "
        filep.write(space)

    file_pointer = open("label.txt", "r")
    checker = file_pointer.read()
    print("Checker is: " + checker)
    file_pointer.close()    

    print("Now terminating program!\n")
    print("------------------------------------------------------------------------------------")
    exit()