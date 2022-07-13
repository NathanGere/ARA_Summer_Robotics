# Find the largest number in this list
list_of_nums = [243,567,345,79,345,234,879,345]
# Use the first value as the initial max value
max = list_of_nums[0]

# For each item in the list
for number in list_of_nums:
    # If the number is larger than the max
    if number > max:
        # Make it the new max
        max = number
# Inform the user
print(f"List: {list_of_nums}")
print(f"Max: {max}")
