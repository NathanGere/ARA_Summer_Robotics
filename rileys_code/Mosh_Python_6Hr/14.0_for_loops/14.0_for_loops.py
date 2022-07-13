 # This will make item a character in the string and move to the next character for each iteration
print("Python:")
for item in "Python":
    print(item)

# This will make item go through a list and assign itself to the next list member after each iteration
print("\nNames List:")
for item in ["Mosh", 'John', "Sarah", "Elsa", "Alexander"]:
    print(item)

# Same thing as above but with numbers
print("\nNumbers List:")
for item in [1, 2, 3, 4, 5]:
    print(item)

# Use range function instead of a large list of numbers
# This will iterate from zero to one before the number inputted into range
# Ex: range(10) will yeild 0-9
range_num_part_1 = 10
print(f"\nrange({range_num_part_1}):")
for item in range(range_num_part_1):
    print(item)

# This will iterate from the first number to one before the second number inputted into range
# Ex: range(5, 10) will yeild 5-9
range_num_part_2_1 = 5
range_num_part_2_2 = 10
print(f"\nrange({range_num_part_2_1}, {range_num_part_2_2}):")
for item in range(range_num_part_2_1,  range_num_part_2_2):
    print(item)

# This will iterate from the first number to one before the second number inputted into range, 
# only including numbers that alternate by the third number
# Ex: range(5, 20, 2) will yeild 5-9, every 2 numbers
# 5, 7, 9, 11, 13, 15, 17, 19
range_num_part_3_1 = 5
range_num_part_3_2 = 20
range_num_part_3_3 = 2
print(f"\nrange({range_num_part_3_1}, {range_num_part_3_2}, {range_num_part_3_3}):")
for item in range(range_num_part_3_1,  range_num_part_3_2, range_num_part_3_3):
    print(item)
