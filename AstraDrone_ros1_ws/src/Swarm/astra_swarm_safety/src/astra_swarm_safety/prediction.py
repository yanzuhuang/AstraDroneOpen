import math
from itertools import combinations


def pairwise_ids(uav_ids):
    """Return every unique vehicle pair in deterministic order."""
    normalized = [int(uid) for uid in uav_ids]
    if not normalized or normalized != sorted(set(normalized)):
        raise ValueError("uav_ids must be a non-empty sorted unique list")
    return list(combinations(normalized, 2))


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


def ellipsoid_distance(a, b):
    """EGO-Swarm's peer metric: horizontal axes 1, vertical axis 2."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        ((a[2] - b[2]) / 2.0) ** 2)


def predicted_ellipsoid_minimum(first, second):
    if not first or not second:
        return float("inf")
    count = min(len(first), len(second))
    return min(ellipsoid_distance(first[i], second[i])
               for i in range(count))


def separation_clear(position1, position2, prediction1, prediction2,
                     minimum_3d, swarm_clearance):
    current = distance(position1, position2)
    predicted = predicted_minimum(prediction1, prediction2)
    ellipsoid_limit = 2.0 * float(swarm_clearance)
    current_ellipsoid = ellipsoid_distance(position1, position2)
    predicted_ellipsoid = predicted_ellipsoid_minimum(
        prediction1, prediction2)
    okay = (
        current >= minimum_3d
        and predicted >= minimum_3d
        and current_ellipsoid >= ellipsoid_limit
        and predicted_ellipsoid >= ellipsoid_limit)
    return okay, current, predicted
