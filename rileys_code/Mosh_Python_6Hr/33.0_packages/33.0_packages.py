# Including modules only works if they are in the same directory as the file calling upon them (like in the last folder)
# It is better practice to have a package (or directory/folder) which holds all the modules and can be called from anywhere

# For this example, the "ecommerce" folder is the example package

# Importing works by the syntax "import (package name).(module name)"
import ecommerce.shipping
# Calling the functions works like this "(package name).(module name).(function name)(parameters)"
ecommerce.shipping.calculate_shipping()

# A slight improvement is to use "from" to make function calls shorter
from ecommerce import shipping
# This function call is smaller
shipping.calculate_shipping()

# This next way is much more common as it avoids long function calls
from ecommerce.shipping import calculate_shipping
# Shortest function call
calculate_shipping()

# If you want to import multiple functions or multiple packages, it would look like this:
# Multiple functions: "from (package name).(module name) import (function 1 name), (function 2 name), (function 3 name)"
# Multiple modules: "from (package name)import (module 1 name), (module 2 name), (module 3 name)"