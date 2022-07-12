names = ["John", "Bob", "Sarah", "Mary", "Demetrious"]

# This will print ['John', 'Bob', 'Sarah', 'Mary', 'Demetrious']
print(names)

# Printing by index is possible
print("\nForward:")
print(names[0])
print(names[1])
print(names[2])
print(names[3])
print(names[4])

# Negative indexes move from the back
print("\nBackwards:")
print(names[-1])
print(names[-2])
print(names[-3])
print(names[-4])
print(names[-5])

# Colons go through a range
print("\nColons:")
# This does the 3rd and 4th item (first index, index to stop before)
# This will not edit the original list
print(names[2:4]) 

# Indexes can be accessed and edited originally
names[0] = "Jon"
print("\nEdit names[0]:")
print(names)