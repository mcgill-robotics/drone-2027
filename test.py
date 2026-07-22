import sys, os
import math


def calculate_stuff(x, y=5):
    unused_var = 100
    a_very_long_list_that_is_not_formatted_properly = [
        "apple",
        "banana",
        "cherry",
        "date",
        "elderberry",
        "fig",
        "grape",
    ]

    if x == y:
        print("x equals y")
    else:
        result = x * y + 10
        return result


calculate_stuff(2, 5)
