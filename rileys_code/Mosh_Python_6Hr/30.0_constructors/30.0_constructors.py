# Class naming convention is CamelCase example: EmailClient
class Point:
    # A constructor is called whenever a point object is created
    # This one takes x and y as parameters
    def __init__(self, x, y):
        # self refers to the object that called this function
        # that is why every function takes "self" as a parameter
        self.x = x
        self.y = y

    def move(self):
        print("Move")
    
    def draw(self):
        print("Draw")

# Making a variable Point (This is where the constructor would be called)
# Nothing is supposed to be passed for self
point = Point(10, 20)
print(f"point x&y: {point.x}, {point.y}.")
# Attributes in the object can still be edited this way
point.x = 4
point.y = 6
print(f"point x&y: {point.x}, {point.y}.")