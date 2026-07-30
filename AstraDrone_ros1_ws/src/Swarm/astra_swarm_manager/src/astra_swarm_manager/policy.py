"""Pure coordination decisions kept separate from ROS transport."""

import math
from itertools import combinations


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
    """UAV2 descends first; UAV1 waits for proof on UAV2's final layer."""
    uav2_waiting = uav2_phase == "WAIT_TRANSITION_PERMISSION"
    uav2_confirmed = (
        abs(uav2_height - uav2_final_height) <= height_tolerance
        and uav2_phase in {"EVALUATING", "NAVIGATING"}
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


def fixed_layers_clear(heights, minimum_vertical_separation):
    """Validate fixed mission layers against the configured safety contract."""
    values = [float(height) for height in heights]
    if not values or not all(math.isfinite(value) for value in values):
        return False
    return all(
        abs(first - second) >= float(minimum_vertical_separation)
        for first, second in combinations(values, 2))


def scheduled_takeoff_allowed(vehicle_index, elapsed, interval, healthy,
                              px4_ready, safety_clear,
                              configuration_safe):
    """Apply one coordinator clock to all vehicles.

    Vehicle index is zero based.  The first vehicle may establish initial
    clearance without a pre-existing global safety latch; all later vehicles
    require the live safety monitor.
    """
    if vehicle_index < 0 or elapsed < 0.0:
        return False
    if not healthy or not px4_ready or not configuration_safe:
        return False
    if elapsed < max(0.0, float(interval)) * vehicle_index:
        return False
    return vehicle_index == 0 or safety_clear


def serialized_landing_permissions(waiting_ids, active_owner):
    """Grant at most one landing permission for an arbitrary fleet."""
    waiting = sorted(set(int(uid) for uid in waiting_ids))
    if active_owner in waiting:
        return {uid: uid == active_owner for uid in waiting}, active_owner
    if waiting:
        owner = waiting[0]
        return {uid: uid == owner for uid in waiting}, owner
    return {}, 0
