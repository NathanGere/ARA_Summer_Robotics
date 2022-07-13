# List of Numbers
numbers = [5, 2, 1, 7, 4]
print(f"List: {numbers}")

####################
### LIST METHODS ###
####################


# Append to the list
print("\nAppending 20")
numbers.append(20)
print(f"List: {numbers}")

# Inserts to the list at an index (index, value)
print("\nInserting 10 at index 0")
numbers.insert(0, 10)
print(f"List: {numbers}")

# Removes a number from the list the first time it appears
print("\nRemoving 10 5 from the list")
numbers.remove(5)
print(f"List: {numbers}")

# Pops the last value out of the list
print("\nPopping the last value from the list")
numbers.pop()
print(f"List: {numbers}")

# Return the index of a value at its first occurance
# CAREFUL!!!! If its not in the list then this will error out
print(f"\nIndex of 7: {numbers.index(7)}")

# Ckecks to see if a value is in the list
x = 50
y = 7
print(f"Is {x} in numbers?:  {x in numbers}")
print(f"Is {y} in numbers?:  {y in numbers}")

# Count the occurances of a value in the list
print("\nAppending two 5's")
numbers.append(5)
numbers.append(5)
print(f"List: {numbers}")
print(f"There are ({numbers.count(5)}) 5's in the list.")

# Sorts the list
print("\nSorting the whole list")
numbers.sort()
print(f"List: {numbers}")

# Reverse the list
print("\nReversing the whole list")
numbers.reverse()
print(f"List: {numbers}")

# Shallow Copy the list
print("\nCopying the whole list")
numbers_2 = numbers.copy()
print(f"List 2: {numbers_2}")

# Clears all the Values from the list
print("\nClearing the whole list")
numbers.clear()
print(f"List: {numbers}")
