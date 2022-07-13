# Create a class "Person" with a name attribute and a "talk()" method
class Person:
    # Constructor to set the name attribute
    def __init__(self, name):
        self.name = name

    # Talk method to say hello and refrence self.name
    def talk(self):
        print("\nTalk has been called")
        print(f"Hello I'm {self.name}!!")
        print("Nice to meet you.")

# Setting a Person object and testing it
john = Person("John")
john.talk()

bob = Person("Bob")
bob.talk()