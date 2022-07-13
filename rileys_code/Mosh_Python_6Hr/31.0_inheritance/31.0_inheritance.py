# Lets say we have a Dog class like this
class Dog:
    def walk(self):
        print("walk")


# And a Cat class like this, we want the cat to also have a walk method
class Cat:
    # We could completely retype and copy the Dog's walk function
    # But that is repetitive and takes up a lot of unneeded space
    # Remember DRY, Dont Repeat Yourself!!!
    def walk(self):
        print("walk")
    # Instead, use inheritance. Check out 31.1_inheritance2.py