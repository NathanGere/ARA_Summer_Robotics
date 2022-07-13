
def full_name_greet(first_name, last_name):
    print(f"Hello {first_name} {last_name}!!")


# Using multiple parameters
first_name = "Wallace"
last_name = "Zarthomelue"

# These are positional arguments, their position matters
full_name_greet(first_name, last_name)
# Changing the order changes the output
full_name_greet(last_name, first_name)

# Keyword arguments dont require an order because we specify what argument matches which parameter
# Usually positional arguments are used, but especially with numerical arguments, keyword arguments improve readibility
# General practice is to use positional arguments before keyword arguments
# Example: function(positional_argument, keyword_argument= this)
full_name_greet(last_name= last_name, first_name= first_name)

