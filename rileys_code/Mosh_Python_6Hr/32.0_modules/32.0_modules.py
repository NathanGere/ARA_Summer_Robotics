# We can import the entire converters module using "import" followed by the file name (.py not included)
import converters
# This gives us an object of converters with the methods of the functions in that file
print(converters.kg_to_lbs(170))


# Or we can import specific functions from the file like this
# from (file name minus the ".py") import (function name)
from converters import kg_to_lbs
# This allows us to call the function as if it were a part of this file
print(kg_to_lbs(170))

