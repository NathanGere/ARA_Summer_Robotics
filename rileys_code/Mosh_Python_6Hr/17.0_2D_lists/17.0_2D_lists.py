# This is what a 2D list looks like in python
matrix = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
]

# This prints the matrix
print(f"Matrix: {matrix}")
# This gets the first list
print(f"matrix[0]: {matrix[0]}")

# This gets the second item of the second list
print(f"matrix[1][1]: {matrix[1][1]}")

# Editing at an index
matrix[1][1] = 42
print(f"EDITED matrix[1][1]: {matrix[1][1]}")

# Reverting to original value for future examples
matrix[1][1] = 5

# Printing the whole matrix with nested loops
print("Matrix:  (Done by Nested Loops)")
for row in matrix:
    for item in row:
        print(item)

