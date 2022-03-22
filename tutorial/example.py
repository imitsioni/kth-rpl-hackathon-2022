import numpy as np
import matplotlib.pyplot as plt


def diff_of_squares(a: int, b: int) -> float:
    """This function calculates the difference of squares of two integers.

    Args:
        a (int): Value to be squared and the minuend of the subtraction
        b (int): Value to be squared and the subtrahend of the subtraction

    Returns:
        float: The difference of squares
    """
    # Complete the following line with the difference of squares
    diff = a^2 - b^2

    return float(diff)


a = [2, 3, 4, 5, 6, 10010]
b = [7, 9, 5, 13, 11]
diffs = []
# Calculate the differences of squares of the integers in lists a and b and
#  plot them

plt.figure()
plt.plot(diffs)
plt.show()
