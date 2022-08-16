# This is just a file made for testing reshape and 2D arrays
import numpy

list = []
for i in range(72):
    list.append(i)

print("1D list:")
print(list)

new_list = numpy.reshape(list,(12,6))
print("Shaped List:")
print(new_list)