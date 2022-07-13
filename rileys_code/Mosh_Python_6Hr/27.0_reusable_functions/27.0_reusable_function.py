# Turning the previous "22.0_word_counter.py" into a function

def word_count(message):


    # Message split will split a string by the character given
    # In this case, the string will be split up by spaces
    # split returns a list of split strings
    word_list = message.split(" ")

    # For each word in the list, add 1 to the count
    count = 0
    for i in word_list:
        count += 1

    # Return the count
    return count

# Examples for the function
message1 = "There was a man"
message2 = "He was wearing a hat"
message3 = "He was also wearing a hoodie"
message4 = "He was cool"

print(f'word_count("{message1}")= {word_count(message1)}.')
print(f'word_count("{message2}")= {word_count(message2)}.')
print(f'word_count("{message3}")= {word_count(message3)}.')
print(f'word_count("{message4}")= {word_count(message4)}.')
