# Tuple to work with
tuple = (1, 2, 3)

# Inefficient way to operate with values in a tuple
inefficient_multiplication = tuple[0] * tuple[1] * tuple[2]
print(f"Inefficient Multipication: {inefficient_multiplication}")

# More efficient way to work with tuple values
x = tuple[0]
y = tuple[1]
z = tuple[2]

efficient_multiplication = x * y * z
print(f"Efficient Multipication: {efficient_multiplication}")

# Using unpacking to get tuple values the most efficient way
x, y, z = tuple

unpacking_multiplication = x * y * z
print(f"Unpacking Multipication: {unpacking_multiplication}")

# Unpacking works with lists too
list = [1,2,3]
x, y, z = list

list_unpacking_multiplication = x * y * z
print(f"List Unpacking Multipication: {list_unpacking_multiplication}")
