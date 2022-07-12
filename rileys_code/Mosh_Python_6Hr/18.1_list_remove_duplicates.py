# List of numbers with duplicates
list = [1,2,3,4,5,6,7,8,9,6,7,8,9]

# Print the unaltered list for comparison
print(list)

# For each item in the list
for item in list:
    # If there is more than 1 occurance
    if list.count(item) > 1:
        # Remove the first occurance
        list.remove(item)

# Print the new list for comparison
print(list)