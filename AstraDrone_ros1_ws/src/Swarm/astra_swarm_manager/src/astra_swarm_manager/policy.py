"""Pure coordination decisions kept separate from ROS transport."""

import math


AIRBORNE_STATES = {
    "TAKEOFF", "HOVER_READY", "TRACK_EGO", "HOLD", "RETURN_HOME",
    "HOME_HOVER", "LANDING",
}


def distance3(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def distance_to_vertical_segment(position, base, height):
    """Shortest 3-D distance to the peer's planned vertical takeoff segment."""
    top_z = float(base[2]) + float(height)
    nearest_z = min(max(float(position[2]), float(base[2])), top_z)
    return distance3(position, (base[0], base[1], nearest_z))


def uav2_takeoff_allowed(elapsed, delay, uav1_healthy, uav1_position,
                        uav2_home, protection_radius, predictions_clear,
                        takeoff_height=4.0, minimum_3d_separation=3.0):
    """Delay is necessary but never sufficient for UAV2 takeoff."""
    if elapsed < delay or not uav1_healthy:
        return False
    outside_horizontal = math.hypot(
        float(uav1_position[0]) - float(uav2_home[0]),
        float(uav1_position[1]) - float(uav2_home[1])) > protection_radius
    corridor_clear = distance_to_vertical_segment(
        uav1_position, uav2_home, takeoff_height) >= minimum_3d_separation
    return corridor_clear and (outside_horizontal or predictions_clear)


def transition_permissions(uav1_phase, uav2_phase, uav2_height,
                           uav2_final_height, height_tolerance, safety_clear):
    """UAV2 descends first; UAV1 is released only after stable 24 m proof."""
    uav2_waiting = uav2_phase == "WAIT_TRANSITION_PERMISSION"
    uav2_confirmed = (
        abs(uav2_height - uav2_final_height) <= height_tolerance
        and uav2_phase not in {"WAIT_TRANSITION_PERMISSION", "LAYER_TRANSITION"}
    )
    return (
        bool(safety_clear and uav2_confirmed
             and uav1_phase == "WAIT_TRANSITION_PERMISSION"),
        bool(safety_clear and uav2_waiting),
        uav2_confirmed,
    )


def landing_permissions(uav1_waiting, uav2_waiting, zones_overlap,
                        active_owner):
    """Return (uav1, uav2, owner); an active owner is sticky until cleared."""
    if not zones_overlap:
        return uav1_waiting, uav2_waiting, 0
    if active_owner == 1:
        return uav1_waiting, False, 1
    if active_owner == 2:
        return False, uav2_waiting, 2
    if uav1_waiting:
        return True, False, 1
    if uav2_waiting:
        return False, True, 2
    return False, False, 0
