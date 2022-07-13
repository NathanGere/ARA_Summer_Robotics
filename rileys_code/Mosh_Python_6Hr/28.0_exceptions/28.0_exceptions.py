# This line below will work as long as there is an integer
#age = int(input('Age: '))


# try except will try the code in the try section
# If an error pops up that matches the except,
# the except will run instead of crashing


try:
    # Try this code
    age = int(input("Age: "))
    income = 20000
    risk = income / age
    print(age)
# If we get an error of type ValueError
except ValueError:
    # Execute this code
    print("Invalid Value")

# Multiple Exceptions can be used for a single "try"
# This one catches ZeroDivisionError 
except ZeroDivisionError:
    print("Age Cannot be Zero")
