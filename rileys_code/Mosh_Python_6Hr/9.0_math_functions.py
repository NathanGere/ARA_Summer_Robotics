import math 
# Rounds up regardless of decimal
print(math.ceil(2.9))  

# Rounds down regardless of decimal                 
print(math.floor(2.9))                  

# Returns a float with magnitude 10 but the sign of -5
print(math.copysign(10, -5))     

# Returns the absolute value of -5
print(math.fabs(-5))  

# Returns integer factorial of 5 (gives an error if input is negative or not an integer)
print(math.factorial(5))

# Returns true if the value is infinite, false otherwise
print(math.isinf(55555555))

# There are many other math functions, they can be found here:
# https://docs.python.org/3/library/math.html

# Importing math isnt needed for these:
print(round(2.9))           # Rounds a Decimal
print(abs(-2.9))            # Absolute Value


