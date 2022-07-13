# Class naming convention is CamelCase example: EmailClient
class Point:
    def move(self):
        print("Move")
    
    def draw(self):
        print("Draw")


# Objects/Instances are instances of a class
point1 = Point()
# Functions can be called like this
point1.draw()
point1.move()
# Sub-Variables can be made like this
point1.x = 10
point1.y = 20
print(f"point1 x&y: {point1.x}, {point1.y}.")
# Many variables can be made from the same class
point2 = Point()
# point2 has no x or y components until they are created and initialized below
point2.x = 1
point2.y = 2
print(f"point2 x&y: {point2.x}, {point2.y}.")