secret_number = 9
guess_count = 0
guess_limit = 3
while guess_count < guess_limit:
    guess = int(input("Guess my number (1 - 10): "))
    guess_count += 1
    if guess == secret_number:
        print("Congrats You Win!!!")
        break                               # Breaking will skip the else statement in the while loop
else:                                       # If the condition in the while loop is no longer true, this executes
    print("You Lost :(")
