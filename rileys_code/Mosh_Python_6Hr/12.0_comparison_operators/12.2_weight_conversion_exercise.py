# Get weight to convert
weight = input("Enter weight for conversion: ")
# Find out which weight type was given
choice = input("Did you just enter (L)bs or (K)gs?: ")
# Convert depending on what input was given
if choice == 'k' or choice == 'K':
    # Convert to Lbs
    converted_weight = int(weight) / 0.45
    print(f"Weight in Lbs is {converted_weight} Lbs.")
elif choice == 'l' or choice == 'L':
    # Convert to Kgs
    converted_weight = int(weight) * 0.45
    print(f"Weight in Kgs is {converted_weight} Kgs.")
else:
    print("Error, incorrect input was received")