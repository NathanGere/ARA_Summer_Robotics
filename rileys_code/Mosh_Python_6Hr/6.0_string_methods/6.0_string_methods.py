course = 'Python for Beginners'
print(len(course))          # len gives the lenght  of the parameter (general purpose, not just strings)
print(course.upper())       # converts text to uppercase (returns new string, doesnt alter original)
print(course.lower())       # converts text to lowercase (returns new string, doesnt alter original)
print(course.title())       # converts text to titlecase (returns new string, doesnt alter original)
print(course.find('P'))     # returns index in which a character is found in the string (first one found)
print(course.find('Beginners'))                  # can be used for strings too
print(course.replace('Beginners', 'Newbies'))    # finds first param and replaces it with the second
print(course.replace('e', 'w'))                  # does it for every occurence
print('Python' in course)                        # returns bool to see if its inside course
print('python' in course)                        # returns bool to see if its inside course





