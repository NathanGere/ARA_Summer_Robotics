# Dictionaries can be used to store key:value pairs
# Each key must have a unique name. Ex: there cannot be two "age" keys
customer = {
    "name"          : "John Smith",
    "age"           : 30,
    "is_verified"   : True
} 

# Accessing key:value pairs
print(f'customer["name"] = {customer["name"]}.')
print(f'customer["age"] = {customer["age"]}.')
print(f'customer["is_verified"] = {customer["is_verified"]}.')

# Normally asking for a key that does not exist will return an error
# But using customer.get will return "None" if the key does not exist
print(f'customer.get("birthday")= {customer.get("birthday")}.')

# We can also set a default value if the key does not exist by using the second parameter
print(f'customer.get("birthday", "January 1, 2000")= {customer.get("birthday", "January 1, 2000")}.')

# Brackets can be used to access key:value pairs and change values
print("Changing values in the dictionary")
customer['name'] = "Bob Burger"
customer['age'] = 42
customer['is_verified'] = False
# Key:Value pairs can be added like this
print("Adding a birthday key:value")
customer['birthday'] = "January 2, 2020"
# You can also print the whole dictionary like this
print(customer)
