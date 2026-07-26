import math


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2)


def predicted_minimum(first, second):
    if not first or not second:
        return float("inf")
    count = min(len(first), len(second))
    return min(distance(first[i], second[i]) for i in range(count))


def predicted_vertical_minimum(first, second):
    if not first or not second:
        return float("inf")
    count = min(len(first), len(second))
    return min(abs(first[i][2] - second[i][2]) for i in range(count))


def separation_clear(position1, position2, prediction1, prediction2,
                     minimum_3d, minimum_vertical, enforce_vertical):
    current = distance(position1, position2)
    predicted = predicted_minimum(prediction1, prediction2)
    vertical = abs(position1[2] - position2[2])
    predicted_vertical = predicted_vertical_minimum(prediction1, prediction2)
    okay = current >= minimum_3d and predicted >= minimum_3d
    if enforce_vertical:
        okay = (
            okay
            and vertical >= minimum_vertical
            and predicted_vertical >= minimum_vertical)
    return okay, current, predicted
