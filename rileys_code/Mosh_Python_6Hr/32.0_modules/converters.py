# Lets say we have a Module, or a file with many functions, like this one
# We want to use these functions in another file, but copy and pasting these functions is inefficient
# Remember DRY, Don't Repeat Yourself
# Check out 32.0_app.py
def lbs_to_kg(weight):
    return weight * 0.45

def kg_to_lbs(weight):
    return weight / 0.45