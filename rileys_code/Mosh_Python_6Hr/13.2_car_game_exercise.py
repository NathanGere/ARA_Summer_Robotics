# Status Variable
is_started = False

# While they havent quit...
while True:
    # Get new input
    command = input("> ").lower()  # Put input to lowercase for simplicity

    # Do specific actions depending on input
    if command == "start":
        if is_started:
            print("Car is already started.")
        else:
            print("Car started... Ready to go!!")
            is_started = True
    elif command == "stop":
        if not is_started:
            print("Car is already stopped")
        else:
            print("Car stopped.")
            is_started = False
    elif command == "help":
        # The message below is tabbed weird because the tabs would be included in the message
        # The current tab syntax shown is correct
        help_msg = """   
start   :  starts the car
stop    :  stops the car
help    :  gives command info
quit    :  quits program 
        """
        print(help_msg)
    elif command == "quit":
        break
    else:
        print("I don't understand that command.")
        print('Try typing "help" for help')
    