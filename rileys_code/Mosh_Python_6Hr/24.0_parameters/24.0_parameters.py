# Function definition, follows the format def function_name(parameters go here):
# Parameters are varables that can be passed to the function for use
# For example, when a name is passed, it is used to make the greeting personal
# With Parameters, if nothing is provided, the program will error

#####Terminology#####
# Parameters are the placeholders for information, arguments are the values provided
# Example: name is the parameter, "John" and "Bob" are the arguments
def personal_greet_user(name):
    # These next 2 lines are run when the function is called
    print(f"Hi there {name}!")
    print("Welcome aboard")


# Multiple Parameters are allowed, as long as they are seperated by commas
def full_name_greet(first_name, last_name):
    print(f"Hello {first_name} {last_name}!!")

# Print start and finish, and call the functions in between
print("Start")

# Passing "John" as an argument
personal_greet_user("John")

# Passing "Bob" as an argument
name = "Bob"
personal_greet_user(name)

# Using multiple parameters
first_name = "Wallace"
last_name = "Zarthomelue"
full_name_greet(first_name, last_name)

print("Finish")