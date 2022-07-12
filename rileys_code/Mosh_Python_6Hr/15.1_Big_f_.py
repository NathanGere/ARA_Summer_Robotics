numbers = [5,2,5,2,2] #Given values for drawing an "F"

# For each number in the list
for index in numbers:
    # Make a fresh output string
    output = ''
    # For the value of that number
    for x in range(index):
        # Add an "X" to the output 
        output += 'X'
    # Print the output
    print(output)