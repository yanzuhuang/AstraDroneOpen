"""Pure fail-closed swarm coordination geometry and arbitration."""

import math
from itertools import combinations


def distance3(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def ellipsoid_distance(a, b):
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + ((float(a[2]) - float(b[2])) / 2.0) ** 2)


def distance_to_vertical_segment(position, base, height):
    top_z = float(base[2]) + float(height)
    nearest_z = min(max(float(position[2]), float(base[2])), top_z)
    return distance3(position, (base[0], base[1], nearest_z))


def mission_geometry_clear(heights, phase_degrees, radius,
                           minimum_3d, swarm_clearance):
    """Validate fixed layers at their configured angular phases.

    Vertical spacing alone is deliberately not a takeoff criterion.  The same
    EGO ellipsoid used by the optimizer must be clear at every configured pair.
    """
    heights = [float(value) for value in heights]
    phases = [float(value) for value in phase_degrees]
    if (not heights or len(heights) != len(phases)
            or not all(math.isfinite(v) for v in heights + phases)
            or radius <= 0.0 or minimum_3d <= 0.0
            or swarm_clearance <= 0.0):
        return False
    points = [
        (radius * math.cos(math.radians(angle)),
         radius * math.sin(math.radians(angle)), height)
        for height, angle in zip(heights, phases)
    ]
    return all(
        distance3(points[i], points[j]) >= minimum_3d
        and ellipsoid_distance(points[i], points[j])
        >= 2.0 * swarm_clearance
        for i, j in combinations(range(len(points)), 2))


def corridor_clear(peer_points, corridor_base, corridor_height,
                   minimum_3d, swarm_clearance):
    """Require every current/future peer sample to clear a takeoff corridor."""
    if not peer_points:
        return False
    bottom = tuple(float(v) for v in corridor_base)
    top = (bottom[0], bottom[1], bottom[2] + float(corridor_height))
    for point in peer_points:
        nearest_z = min(max(float(point[2]), bottom[2]), top[2])
        nearest = (bottom[0], bottom[1], nearest_z)
        if (distance3(point, nearest) < minimum_3d
                or ellipsoid_distance(point, nearest)
                < 2.0 * swarm_clearance):
            return False
    return True


def scheduled_takeoff_allowed(vehicle_index, elapsed, interval, healthy,
                              px4_ready, safety_clear,
                              configuration_safe, corridor_is_clear=True):
    if vehicle_index < 0 or elapsed < 0.0:
        return False
    if not (healthy and px4_ready and safety_clear
            and configuration_safe and corridor_is_clear):
        return False
    return elapsed >= max(0.0, float(interval)) * vehicle_index


def entry_candidate_allowed(healthy, mission_phase, flight_state):
    """Only an armed-flight hover may receive a new ENTRY corridor lease.

    A bridge that has fail-closed into LANDING/DONE must not be revived by a
    later latched mission-phase update.
    """
    return bool(
        healthy
        and mission_phase == "WAIT_ENTRY_PERMISSION"
        and flight_state == "HOVER_READY")


def eligible_entry_ids(uav_ids, states, health):
    """Return only received, healthy hover states waiting for ENTRY.

    Coordination starts before the first state frame can arrive, so a missing
    state is a normal fail-closed condition rather than an indexing error.
    """
    return [
        uid for uid in uav_ids
        if uid in states and entry_candidate_allowed(
            bool(health.get(uid, False)),
            states[uid].mission_phase,
            states[uid].flight_state)]


def serialized_permissions(waiting_ids, active_owner):
    """Grant one sticky corridor owner in deterministic order."""
    waiting = sorted(set(int(uid) for uid in waiting_ids))
    if active_owner:
        return {uid: uid == active_owner for uid in waiting}, active_owner
    if not waiting:
        return {}, 0
    owner = waiting[0]
    return {uid: uid == owner for uid in waiting}, owner


def advance_entry_owner(entry_owner, exit_owner, owner_healthy,
                        orbit_established, returning):
    """Advance sticky corridor ownership without opening an unknown corridor."""
    entry_owner = int(entry_owner)
    exit_owner = int(exit_owner)
    if not entry_owner or not owner_healthy:
        return entry_owner, exit_owner
    if orbit_established:
        return 0, exit_owner
    if returning and not exit_owner:
        return 0, entry_owner
    return entry_owner, exit_owner


def entry_owner_orbit_established(mission_phase, already_released,
                                   active_orbit_phases):
    """Disambiguate a pre-entry HOLD from a supervised in-orbit HOLD."""
    phase = str(mission_phase)
    return (
        phase in set(active_orbit_phases)
        and (phase != "HOLDING" or bool(already_released)))


def angular_separation_degrees(first, second, center):
    """Return the unsigned shortest XY angle between two tower positions."""
    first_angle = math.degrees(math.atan2(
        float(first[1]) - float(center[1]),
        float(first[0]) - float(center[0])))
    second_angle = math.degrees(math.atan2(
        float(second[1]) - float(center[1]),
        float(second[0]) - float(center[0])))
    return abs((first_angle - second_angle + 180.0) % 360.0 - 180.0)


def orbit_release_allowed(waiting_position, orbiting_positions, center,
                          desired_separation_degrees,
                          phase_tolerance_degrees):
    """Release a gate hover only when it completes the requested XY phase."""
    desired = float(desired_separation_degrees)
    tolerance = float(phase_tolerance_degrees)
    if (not all(math.isfinite(float(v)) for v in waiting_position[:2])
            or not all(math.isfinite(float(v)) for v in center[:2])
            or not math.isfinite(desired) or not math.isfinite(tolerance)
            or desired <= 0.0 or desired >= 180.0 or tolerance < 0.0):
        return False
    if not orbiting_positions:
        return True
    return all(
        abs(angular_separation_degrees(
            waiting_position, position, center) - desired) <= tolerance
        for position in orbiting_positions)


def orbit_phase_hold_ids(positions, center, desired_separation_degrees,
                         phase_tolerance_degrees, previously_held=None):
    """Return CCW trailing vehicles that must pause to preserve ring phase.

    Each vehicle is compared with the next vehicle ahead in the configured
    counter-clockwise orbit direction.  A new hold engages at the lower edge
    of the phase band.  An existing hold releases halfway back into the band
    so position noise cannot chatter HOLD/RESUME.
    """
    desired = float(desired_separation_degrees)
    tolerance = float(phase_tolerance_degrees)
    held = set(int(uid) for uid in (previously_held or set()))
    if (not all(math.isfinite(float(v)) for v in center[:2])
            or not math.isfinite(desired) or not math.isfinite(tolerance)
            or desired <= 0.0 or desired >= 180.0
            or tolerance < 0.0 or tolerance >= desired):
        return set()

    angles = []
    for uid, position in positions.items():
        if (len(position) < 2
                or not all(math.isfinite(float(v)) for v in position[:2])):
            return set()
        angle = math.degrees(math.atan2(
            float(position[1]) - float(center[1]),
            float(position[0]) - float(center[0]))) % 360.0
        angles.append((angle, int(uid)))
    if len(angles) < 2:
        return set()

    angles.sort()
    engage_gap = desired - tolerance
    release_gap = desired - 0.5 * tolerance
    result = set()
    for index, (angle, uid) in enumerate(angles):
        leader_angle = angles[(index + 1) % len(angles)][0]
        forward_gap = (leader_angle - angle) % 360.0
        threshold = release_gap if uid in held else engage_gap
        if forward_gap < threshold:
            result.add(uid)
    return result


def rotate_xy_about_center(position, center, angle_radians, radius=None):
    """Project a tower-relative XY point along the configured orbit."""
    dx = float(position[0]) - float(center[0])
    dy = float(position[1]) - float(center[1])
    current_radius = math.hypot(dx, dy)
    if current_radius <= 0.0:
        return None
    projected_radius = current_radius if radius is None else float(radius)
    angle = math.atan2(dy, dx) + float(angle_radians)
    return (
        float(center[0]) + projected_radius * math.cos(angle),
        float(center[1]) + projected_radius * math.sin(angle))


def serialized_landing_permissions(waiting_ids, active_owner):
    waiting = sorted(set(int(uid) for uid in waiting_ids))
    if active_owner in waiting:
        return {uid: uid == active_owner for uid in waiting}, active_owner
    if not waiting:
        return {}, 0
    owner = waiting[0]
    return {uid: uid == owner for uid in waiting}, owner


# Compatibility helpers retained for the established dual-mission tests.
def fixed_layers_clear(heights, minimum_vertical_separation):
    values = [float(height) for height in heights]
    if not values or not all(math.isfinite(value) for value in values):
        return False
    return all(
        abs(first - second) >= float(minimum_vertical_separation)
        for first, second in combinations(values, 2))


def transition_permissions(uav1_phase, uav2_phase, uav2_height,
                           uav2_final_height, height_tolerance, safety_clear):
    uav2_waiting = uav2_phase == "WAIT_TRANSITION_PERMISSION"
    uav2_confirmed = (
        abs(uav2_height - uav2_final_height) <= height_tolerance
        and uav2_phase in {"EVALUATING", "NAVIGATING"})
    return (
        bool(safety_clear and uav2_confirmed
             and uav1_phase == "WAIT_TRANSITION_PERMISSION"),
        bool(safety_clear and uav2_waiting),
        uav2_confirmed)


def landing_permissions(uav1_waiting, uav2_waiting, zones_overlap,
                        active_owner):
    if not zones_overlap:
        return uav1_waiting, uav2_waiting, 0
    grants, owner = serialized_permissions(
        ([1] if uav1_waiting else []) + ([2] if uav2_waiting else []),
        active_owner)
    return grants.get(1, False), grants.get(2, False), owner


def uav2_takeoff_allowed(elapsed, delay, uav1_healthy, uav1_position,
                        uav2_home, protection_radius, predictions_clear,
                        takeoff_height=4.0, minimum_3d_separation=3.0):
    outside = math.hypot(
        float(uav1_position[0]) - float(uav2_home[0]),
        float(uav1_position[1]) - float(uav2_home[1])) > protection_radius
    clear = distance_to_vertical_segment(
        uav1_position, uav2_home, takeoff_height) >= minimum_3d_separation
    return (elapsed >= delay and uav1_healthy and clear
            and (outside or predictions_clear))
