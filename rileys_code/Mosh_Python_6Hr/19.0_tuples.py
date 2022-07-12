# This is a list because it uses brackets
list = [1,2,3] 

# This is a tuple because it uses parenthesis
tuple = (1, 2, 3, 1, 2)

# Indexing a tuple works the same as a list
print(f"tuple[0] = {tuple[0]}.")

# Tuples cannot be changed, trying to change a tuple by index returns an error

# This will return the amount of times a value is inside a tuple
value = 1
print(f"tuple.count({value}) = {tuple.count(value)}.")

# This will return the index of the first occurance of a value in the tuple
value = 1
print(f"tuple.index({value}) = {tuple.index(value)}.")


