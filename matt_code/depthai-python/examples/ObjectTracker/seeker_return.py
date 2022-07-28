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

    if human_detector == "person":
        print("Human found!\n")
    elif human_detector == "background":
        print("No human found!\n")
        print("Found background\n")
    elif human_detector == "aeroplane":
        print("No human found!\n")
        print("Found aeroplane\n")
    elif human_detector == "bicycle":
        print("No human found!\n")
        print("Found bicycle\n")
    elif human_detector == "bird":
        print("No human found!\n")
        print("Found bird\n")
    elif human_detector == "boat":
        print("No human found!\n")
        print("Found boat\n")
    elif human_detector == "bottle":
        print("No human found!\n")
        print("Found bottle\n")
    elif human_detector == "bus":
        print("No human found!\n")
        print("Found bus\n")
    elif human_detector == "car":
        print("No human found!\n")
        print("Found car\n")
    elif human_detector == "cat":
        print("No human found!\n")
        print("Found cat\n")
    elif human_detector == "chair":
        print("No human found!\n")
        print("Found chair\n")
    elif human_detector == "cow":
        print("No human found!\n")
        print("Found cow\n")
    elif human_detector == "diningtable":
        print("No human found!\n")
        print("Found diningtable\n")
    elif human_detector == "dog":
        print("No human found!\n")
        print("Found dog\n")
    elif human_detector == "horse":
        print("No human found!\n")
        print("Found horse\n")
    elif human_detector == "motorbike":
        print("No human found!\n")
        print("Found motorbike\n")
    elif human_detector == "pottedplant":
        print("No human found!\n")
        print("Found pottedplant\n")
    elif human_detector == "sheep":
        print("No human found!\n")
        print("Found sheep\n")
    elif human_detector == "sofa":
        print("No human found!\n")
        print("Found sofa\n")
    elif human_detector == "train":
        print("No human found!\n")
        print("Found train\n")
    elif human_detector == "tvmoniter":
        print("No human found!\n")
        print("Found tvmoniter\n")
    else:
        print("No human found")
        print("Found nothing :(")
    file.close()

    print("Rewriting label.txt to empty to avoid miscalculations!\n")

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
