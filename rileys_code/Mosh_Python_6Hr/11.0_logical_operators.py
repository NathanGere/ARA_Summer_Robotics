# Play with these boolean values and see how it changes output
has_high_income = True
has_good_credit = True
has_criminal_record = False

# The and operator only returns true of both conditions are true
if has_high_income and has_good_credit:
    print("You are eligible for a loan with an 'and' statement")
# the or operator returns true if either or both conditions are true
if has_high_income or has_good_credit:
    print("You are eligible for the loan with an 'or' statement")
# the not operator swaps the boolean value of its condition
if has_good_credit and not has_criminal_record:
    print("You have good credit and no criminal record")