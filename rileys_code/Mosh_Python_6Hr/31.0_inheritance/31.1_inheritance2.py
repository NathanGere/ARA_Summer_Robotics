# Check out 31.0_inheritance.py if you haven't yet
# This is a continuation of that file

# To fix the problem or repetition, we can use a parent class like this
class Mammal:
    def walk(self):
        print("Walk")


# And include the parent class in the children classes like this
class Dog(Mammal):
    # "pass" does nothing but it tells the compiler to ignore that this is an empty class
    # Normally, and empty class causes an error
    pass


# Same with the cat class
class Cat(Mammal):
    # Adding a method here makes it exclusive to Cat.
    # Other mammals cannot use it
    def meow(self):
        print("Meow")


# Using these classes
dog1 = Dog()
cat1 = Cat()

# Using the methods
print("Dog walk: ")
dog1.walk()
print("Cat walk: ")
cat1.walk()
print("Cat meow: ")
cat1.meow()
