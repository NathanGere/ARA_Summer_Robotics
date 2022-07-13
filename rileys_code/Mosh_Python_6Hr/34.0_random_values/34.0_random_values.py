import random

# Generates a random value between 0-1
random.random()
# Print 3 of them
for i in range(3):
    print(random.random())


# Generates a random value between 10-20 (including 10 and 20)
random.randint(10, 20)
# Print 3 of them
for i in range(10):
    print(random.randint(10,20))

# Randomly picking from a list
members = ["David", "Marchand", "Luther", "Cornelius"]
leader = random.choice(members)
print(leader)