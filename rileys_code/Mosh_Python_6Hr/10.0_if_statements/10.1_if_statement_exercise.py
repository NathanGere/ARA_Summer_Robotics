home_price = 1000000
has_good_credit = False         # play with this value
down_payment = 0

if has_good_credit:
    down_payment = home_price * 0.1
else:
    down_payment = home_price * 0.2

msg = f"You're Down Payment is: ${int(down_payment)}"
print(msg)