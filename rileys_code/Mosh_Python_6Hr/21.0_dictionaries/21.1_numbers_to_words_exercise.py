# Dicitonary relating numbers to their words
number_words = {
    "1": "one",
    "2": "two",
    "3": "three",
    "4": "four",
    "5": "five",
    "6": "six",
    "7": "seven",
    "8": "eight",
    "9": "nine"
}

# Setting up input and output strings
input_string = input("Phone: ")
output = ""

# For each character in the input
for character in input_string:
    # Add the word value followed by a space
    # (The exclamation point is the default if a number isnt inputted)
    output += number_words.get(character, "!") + " "

# Print the output
print(output)