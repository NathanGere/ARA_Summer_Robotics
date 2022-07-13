# Get input
message = input(">")

# Message split will split a string by the character given
# In this case, the string will be split up by spaces
# split returns a list of split strings
word_list = message.split(" ")

# For each word in the list, add 1 to the count
count = 0
for i in word_list:
    count += 1

# Print the count
print(f"I found {count} words.")