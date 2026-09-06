"""Pure fail-closed swarm coordination geometry and arbitration."""

import math
from itertools import combinations, product


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


def _sample_polyline(points, sample_count=48):
    values = [tuple(float(value) for value in point) for point in points]
    if len(values) < 2:
        return values
    lengths = [distance3(values[index - 1], values[index])
               for index in range(1, len(values))]
    total = sum(lengths)
    if total <= 1.0e-9:
        return [values[0]] * (sample_count + 1)
    sampled = []
    for step in range(sample_count + 1):
        target = total * float(step) / float(sample_count)
        traversed = 0.0
        for index, length in enumerate(lengths, start=1):
            if target <= traversed + length or index == len(lengths):
                ratio = (target - traversed) / max(length, 1.0e-9)
                start, finish = values[index - 1], values[index]
                sampled.append(tuple(
                    start[axis] + ratio * (finish[axis] - start[axis])
                    for axis in range(3)))
                break
            traversed += length
    return sampled


def _segments_intersect_xy(first_start, first_finish,
                           second_start, second_finish):
    def orientation(a, b, c):
        return ((float(b[0]) - float(a[0]))
                * (float(c[1]) - float(a[1]))
                - (float(b[1]) - float(a[1]))
                * (float(c[0]) - float(a[0])))

    first_a = orientation(first_start, first_finish, second_start)
    first_b = orientation(first_start, first_finish, second_finish)
    second_a = orientation(second_start, second_finish, first_start)
    second_b = orientation(second_start, second_finish, first_finish)
    epsilon = 1.0e-9
    def on_segment(a, b, c):
        return (min(float(a[0]), float(b[0])) - epsilon
                <= float(c[0]) <= max(float(a[0]), float(b[0])) + epsilon
                and min(float(a[1]), float(b[1])) - epsilon
                <= float(c[1]) <= max(float(a[1]), float(b[1])) + epsilon)

    # Collinear/touching paths are conservatively treated as crossing only
    # when the relevant point actually lies on the finite segment.
    if abs(first_a) <= epsilon and on_segment(
            first_start, first_finish, second_start):
        return True
    if abs(first_b) <= epsilon and on_segment(
            first_start, first_finish, second_finish):
        return True
    if abs(second_a) <= epsilon and on_segment(
            second_start, second_finish, first_start):
        return True
    if abs(second_b) <= epsilon and on_segment(
            second_start, second_finish, first_finish):
        return True
    return ((first_a > 0.0) != (first_b > 0.0)
            and (second_a > 0.0) != (second_b > 0.0))


def _polylines_cross_xy(first, second):
    return any(
        _segments_intersect_xy(first[first_index - 1], first[first_index],
                               second[second_index - 1],
                               second[second_index])
        for first_index in range(1, len(first))
        for second_index in range(1, len(second)))


def _polyline_length(points):
    return sum(distance3(points[index - 1], points[index])
               for index in range(1, len(points)))


def _entry_candidate_minimum_tier(item, nominal_angle, center, nominal_radius,
                                  radius_step, epsilon=1.0e-6):
    """Return the first hard fallback tier in which a candidate may compete."""
    radius = float(item.get(
        "orbit_staging_radius",
        math.hypot(float(item["staging"][0]) - float(center[0]),
                   float(item["staging"][1]) - float(center[1]))))
    radius_tier = int(round((radius - nominal_radius) / radius_step))
    expected_radius = nominal_radius + radius_step * radius_tier
    if (radius_tier not in (0, 1, 2) or abs(radius - expected_radius) > 1.0e-3):
        return None, radius_tier, radius
    angle_deviation = abs((float(item["angle_deg"])
                           - float(nominal_angle) + 180.0) % 360.0 - 180.0)
    if radius_tier == 0:
        return (0 if angle_deviation <= epsilon else 1), radius_tier, radius
    return radius_tier + 1, radius_tier, radius


def joint_entry_corridor_selection(
        candidates_by_uid, ordered_uav_ids, nominal_angles, center, starts,
        direction, minimum_gap_degrees=20.0, maximum_gap_degrees=30.0,
        hard_clearance=0.5, minimum_3d=3.0, swarm_clearance=1.5,
        angular_window_degrees=12.0, nominal_orbit_radius=12.5,
        orbit_radius_step=2.0):
    """Select one latched full ENTRY corridor per vehicle, fail closed.

    The selector exhausts one hard fallback tier before considering the next:
    nominal 12.5 m, adjusted-angle 12.5 m, up to 14.5 m, then up to 16.5 m.
    Candidate dictionaries contain the endpoint/path/EGO-local verdicts and
    their common-world full paths.  This joint layer preserves those checks and
    additionally proves role order, non-crossing geometry and time-aligned
    inter-vehicle clearance.
    """
    ids = [int(uid) for uid in ordered_uav_ids]
    if (not ids or len(ids) != len(set(ids)) or direction not in (-1, 1)
            or any(uid not in candidates_by_uid or not candidates_by_uid[uid]
                   for uid in ids)
            or any(uid not in nominal_angles or uid not in starts
                   for uid in ids)
            or not (0.0 < minimum_gap_degrees <= maximum_gap_degrees < 180.0)
            or hard_clearance <= 0.0 or minimum_3d <= 0.0
            or swarm_clearance <= 0.0 or nominal_orbit_radius <= 0.0
            or orbit_radius_step <= 0.0):
        return {}, "INVALID_OR_MISSING_CANDIDATES", {}

    prepared = {uid: [] for uid in ids}
    candidate_outcomes = {str(uid): {} for uid in ids}
    for uid in ids:
        for item in candidates_by_uid[uid]:
            minimum_tier, radius_tier, staging_radius = (
                _entry_candidate_minimum_tier(
                    item, nominal_angles[uid], center, nominal_orbit_radius,
                    orbit_radius_step))
            candidate = dict(item)
            candidate["minimum_joint_tier"] = minimum_tier
            candidate["radius_tier"] = radius_tier
            candidate["orbit_staging_radius"] = staging_radius
            prepared[uid].append(candidate)
            candidate_outcomes[str(uid)][candidate["id"]] = {
                "minimum_joint_tier": minimum_tier,
                "radius_tier": radius_tier,
                "orbit_staging_radius": staging_radius,
                "angle_deg": float(candidate["angle_deg"]),
                "endpoint_clearance": float(candidate.get(
                    "endpoint_clearance", candidate.get("clearance", -1.0))),
                "path_clearance": float(candidate.get("clearance", -1.0)),
                "endpoint_valid": bool(candidate.get("endpoint_valid", True)),
                "path_valid": bool(candidate.get("path_valid", True)),
                "ego_status": candidate.get("ego_status", "NOT_PRECHECKED"),
                "evaluated_combinations": 0,
                "local_hard_rejected": 0,
                "role_rejected": 0,
                "crossing_rejected": 0,
                "predicted_distance_rejected": 0,
                "minimum_predicted_distance": None,
                "valid_combinations": 0,
                "best_valid_rank": None,
                "selected": False,
                "final_rejection_reason": "NOT_EVALUATED",
            }

    diagnostics = {
        "tier_contract": {
            "0": "ALL_NOMINAL_12_5",
            "1": "ALL_12_5_WITH_SECTOR_ANGLE_ADJUSTMENT",
            "2": "ALLOW_UP_TO_14_5",
            "3": "ALLOW_UP_TO_16_5",
        },
        "tiers": {},
        "candidate_outcomes": candidate_outcomes,
    }
    best = None
    best_rank = None
    selected_tier = None
    selected_metrics = None
    for tier in range(4):
        tier_candidates = {
            uid: [item for item in prepared[uid]
                  if item["minimum_joint_tier"] is not None
                  and item["minimum_joint_tier"] <= tier]
            for uid in ids}
        tier_diagnostics = {
            "candidate_counts": {
                str(uid): len(tier_candidates[uid]) for uid in ids},
            "evaluated": 0,
            "local_hard_rejected": 0,
            "role_rejected": 0,
            "crossing_rejected": 0,
            "conflict_rejected": 0,
            "valid_combinations": 0,
            "result": "NO_CANDIDATES_FOR_TIER",
        }
        diagnostics["tiers"][str(tier)] = tier_diagnostics
        if any(not tier_candidates[uid] for uid in ids):
            continue

        tier_best = None
        tier_best_rank = None
        tier_best_metrics = None
        for combination in product(*(tier_candidates[uid] for uid in ids)):
            tier_diagnostics["evaluated"] += 1
            selected = dict(zip(ids, combination))
            outcomes = [candidate_outcomes[str(uid)][selected[uid]["id"]]
                        for uid in ids]
            for outcome in outcomes:
                outcome["evaluated_combinations"] += 1

            local_hard_invalid = any(
                not bool(item.get("endpoint_valid", True))
                or not bool(item.get("path_valid", True))
                or not bool(item.get("ego_candidate_valid", True))
                or float(item.get("clearance", -1.0)) + 1.0e-9
                < hard_clearance for item in combination)
            if local_hard_invalid:
                tier_diagnostics["local_hard_rejected"] += 1
                for outcome in outcomes:
                    outcome["local_hard_rejected"] += 1
                continue
            if any(abs((float(selected[uid]["angle_deg"])
                        - float(nominal_angles[uid]) + 180.0)
                       % 360.0 - 180.0)
                   > angular_window_degrees + 1.0e-9 for uid in ids):
                tier_diagnostics["role_rejected"] += 1
                for outcome in outcomes:
                    outcome["role_rejected"] += 1
                continue
            gaps = []
            role_valid = True
            for leader, follower in zip(ids[:-1], ids[1:]):
                gap = normalize_degrees(direction * (
                    float(selected[leader]["angle_deg"])
                    - float(selected[follower]["angle_deg"])))
                gaps.append(gap)
                if not (minimum_gap_degrees <= gap <= maximum_gap_degrees):
                    role_valid = False
                    break
            if not role_valid:
                tier_diagnostics["role_rejected"] += 1
                for outcome in outcomes:
                    outcome["role_rejected"] += 1
                continue
            paths = {
                uid: ([tuple(point) for point in selected[uid].get("path", [])]
                      or [tuple(starts[uid]), tuple(selected[uid]["pre"]),
                          tuple(selected[uid]["entry"]),
                          tuple(selected[uid]["staging"])])
                for uid in ids}
            if any(_polylines_cross_xy(paths[first], paths[second])
                   for first, second in combinations(ids, 2)):
                tier_diagnostics["crossing_rejected"] += 1
                for outcome in outcomes:
                    outcome["crossing_rejected"] += 1
                continue
            # A committed predecessor may already be flying its selected
            # corridor.  Keep its latched corridor for crossing checks, while
            # using its current-pose/trajectory projection for time-aligned
            # separation against the joining role.
            predicted_paths = {
                uid: [tuple(point) for point in selected[uid].get(
                    "predicted_path", paths[uid])]
                for uid in ids}
            samples = {
                uid: _sample_polyline(predicted_paths[uid]) for uid in ids}
            minimum_pair_distance = float("inf")
            conflict = False
            for first, second in combinations(ids, 2):
                for first_point, second_point in zip(
                        samples[first], samples[second]):
                    separation = distance3(first_point, second_point)
                    minimum_pair_distance = min(
                        minimum_pair_distance, separation)
                    if (separation + 1.0e-9 < minimum_3d
                            or ellipsoid_distance(first_point, second_point)
                            + 1.0e-9 < 2.0 * swarm_clearance):
                        conflict = True
            for outcome in outcomes:
                previous_minimum = outcome["minimum_predicted_distance"]
                if (previous_minimum is None
                        or minimum_pair_distance < previous_minimum):
                    outcome["minimum_predicted_distance"] = (
                        minimum_pair_distance)
            if conflict:
                tier_diagnostics["conflict_rejected"] += 1
                for outcome in outcomes:
                    outcome["predicted_distance_rejected"] += 1
                continue

            minimum_clearance = min(float(item["clearance"])
                                    for item in combination)
            nominal_gaps = [
                normalize_degrees(direction * (
                    float(nominal_angles[leader])
                    - float(nominal_angles[follower])))
                for leader, follower in zip(ids[:-1], ids[1:])]
            role_gap_deviation = sum(
                abs((gap - nominal_gap + 180.0) % 360.0 - 180.0)
                for gap, nominal_gap in zip(gaps, nominal_gaps))
            angle_deviation = sum(abs((float(selected[uid]["angle_deg"])
                                       - float(nominal_angles[uid]) + 180.0)
                                      % 360.0 - 180.0) for uid in ids)
            ingress_length = sum(float(item.get(
                "ingress_length", _polyline_length(paths[uid])))
                for uid, item in zip(ids, combination))
            identifiers = tuple(selected[uid]["id"] for uid in ids)
            # Prefix arbitration must prefer the configured relative role
            # geometry before minimizing each candidate's absolute angle.
            # Otherwise a locally attractive middle-role point can consume
            # the only 20--30 degree slot available to the trailing role.
            rank = (-minimum_clearance, role_gap_deviation,
                    angle_deviation, ingress_length, identifiers)
            rank_values = {
                "minimum_clearance": minimum_clearance,
                "role_gap_deviation": role_gap_deviation,
                "angle_deviation": angle_deviation,
                "ingress_length": ingress_length,
                "candidate_ids": list(identifiers),
                "gaps": gaps,
                "minimum_pair_distance": minimum_pair_distance,
            }
            tier_diagnostics["valid_combinations"] += 1
            for outcome in outcomes:
                outcome["valid_combinations"] += 1
                current = outcome["best_valid_rank"]
                if (current is None
                        or (-rank_values["minimum_clearance"],
                            rank_values["role_gap_deviation"],
                            rank_values["angle_deviation"],
                            rank_values["ingress_length"],
                            tuple(rank_values["candidate_ids"]))
                        < (-current["minimum_clearance"],
                           current["role_gap_deviation"],
                           current["angle_deviation"],
                           current["ingress_length"],
                           tuple(current["candidate_ids"]))):
                    outcome["best_valid_rank"] = rank_values
            if tier_best_rank is None or rank < tier_best_rank:
                tier_best = selected
                tier_best_rank = rank
                tier_best_metrics = {
                    "gaps": gaps,
                    "minimum_clearance": minimum_clearance,
                    "minimum_pair_distance": minimum_pair_distance,
                    "role_gap_deviation": role_gap_deviation,
                    "angle_deviation": angle_deviation,
                    "ingress_length": ingress_length,
                    "rank": rank_values,
                }

        if tier_best is None:
            tier_diagnostics["result"] = "NO_JOINT_SAFE_COMBINATION"
            continue
        tier_diagnostics["result"] = "SELECTED"
        tier_diagnostics["selected_rank"] = tier_best_metrics["rank"]
        best = tier_best
        best_rank = tier_best_rank
        selected_metrics = tier_best_metrics
        selected_tier = tier
        break

    if best is None:
        for outcomes in candidate_outcomes.values():
            for outcome in outcomes.values():
                outcome["final_rejection_reason"] = (
                    "UNSUPPORTED_RADIUS_TIER"
                    if outcome["minimum_joint_tier"] is None
                    else "NO_HARD_CONSTRAINT_VALID_COMBINATION")
        diagnostics["selected_tier"] = None
        diagnostics["result"] = "NO_JOINT_SAFE_COMBINATION_ALL_TIERS"
        return {}, "NO_JOINT_SAFE_COMBINATION_ALL_TIERS", diagnostics

    selected_ids = {uid: best[uid]["id"] for uid in ids}
    for uid in ids:
        for candidate_id, outcome in candidate_outcomes[str(uid)].items():
            if candidate_id == selected_ids[uid]:
                outcome["selected"] = True
                outcome["final_rejection_reason"] = "SELECTED"
            elif (outcome["minimum_joint_tier"] is None
                  or outcome["minimum_joint_tier"] > selected_tier):
                outcome["final_rejection_reason"] = (
                    "LOWER_TIER_SUCCEEDED_BEFORE_CANDIDATE_ELIGIBLE")
            elif outcome["valid_combinations"] > 0:
                outcome["final_rejection_reason"] = "VALID_BUT_RANKED_LOWER"
            else:
                evaluated = outcome["evaluated_combinations"]
                local = outcome["local_hard_rejected"]
                role = outcome["role_rejected"]
                crossing = outcome["crossing_rejected"]
                predicted = outcome["predicted_distance_rejected"]
                if not outcome["endpoint_valid"]:
                    rejection = "ENDPOINT_INVALID"
                elif not outcome["path_valid"]:
                    rejection = "PATH_INVALID"
                elif outcome["ego_status"] == "PLANNER_UNREACHABLE":
                    rejection = "EGO_PLANNER_UNREACHABLE"
                elif evaluated > 0 and local == evaluated:
                    rejection = "LOCAL_CLEARANCE_OR_EGO_REJECTED"
                elif evaluated > 0 and local + role == evaluated:
                    rejection = "ROLE_ORDER_OR_SPACING_REJECTED"
                elif (evaluated > 0
                      and local + role + crossing == evaluated):
                    rejection = "PATH_CROSSING_REJECTED"
                elif (evaluated > 0
                      and local + role + crossing + predicted == evaluated):
                    rejection = "PREDICTED_DISTANCE_REJECTED"
                else:
                    rejection = "NO_VALID_COMBINATION_MULTIPLE_REASONS"
                outcome["final_rejection_reason"] = rejection
    diagnostics.update(selected_metrics)
    diagnostics["selected_tier"] = selected_tier
    diagnostics["selected_rank"] = selected_metrics["rank"]
    diagnostics["result"] = "OK_TIER_{}".format(selected_tier)
    return best, diagnostics["result"], diagnostics


def incremental_entry_corridor_selection(
        candidates_by_uid, committed_selection, ordered_uav_ids,
        nominal_angles, center, starts, direction,
        minimum_gap_degrees=20.0, maximum_gap_degrees=30.0,
        hard_clearance=0.5, minimum_3d=3.0, swarm_clearance=1.5,
        angular_window_degrees=12.0, nominal_orbit_radius=12.5,
        orbit_radius_step=2.0, committed_prediction_paths=None,
        globally_clear=True):
    """Latch one ENTRY role at a time while preserving the committed prefix.

    The existing joint selector remains the sole geometry/safety evaluator.
    Already committed roles are supplied as singleton candidate sets, so a
    later role can never replace an earlier corridor. Their current predicted
    trajectories augment (but do not replace) the committed-corridor crossing
    checks. A missing or rejected later role retains prior grants; only a
    global safety failure revokes every permission.
    """
    order = [int(uid) for uid in ordered_uav_ids]
    committed = dict(committed_selection or {})
    grants = {uid: False for uid in order}
    if not order or len(order) != len(set(order)):
        return committed, grants, "INVALID_ROLE_ORDER", {}
    committed_ids = set(committed)
    expected_prefix = set(order[:len(committed_ids)])
    if (committed_ids != expected_prefix
            or any(uid not in nominal_angles or uid not in starts
                   for uid in order[:min(len(order), len(committed_ids) + 1)])):
        return committed, grants, "INVALID_COMMITTED_ROLE_PREFIX", {}
    if not globally_clear:
        return committed, grants, "GLOBAL_SAFETY_INHIBIT", {}
    for uid in committed_ids:
        grants[uid] = True
    if len(committed_ids) == len(order):
        return committed, grants, "ALL_ROLES_COMMITTED", {
            "committed_role_order": list(order)}

    next_uid = order[len(committed_ids)]
    next_candidates = list(candidates_by_uid.get(next_uid, []))
    if not next_candidates:
        return committed, grants, "WAITING_FOR_UAV{}_CANDIDATES".format(
            next_uid), {"next_role": next_uid,
                        "committed_role_order": order[:len(committed_ids)]}

    prediction_paths = committed_prediction_paths or {}
    if any(uid not in prediction_paths or not prediction_paths[uid]
           for uid in committed_ids):
        return committed, grants, "COMMITTED_TRAJECTORY_MISSING_OR_STALE", {
            "next_role": next_uid,
            "committed_role_order": order[:len(committed_ids)]}

    prefix = order[:len(committed_ids) + 1]
    prefix_candidates = {}
    for uid in prefix[:-1]:
        fixed = dict(committed[uid])
        fixed["predicted_path"] = list(prediction_paths[uid])
        prefix_candidates[uid] = [fixed]
    prefix_candidates[next_uid] = next_candidates
    selected, reason, diagnostics = joint_entry_corridor_selection(
        prefix_candidates, prefix, nominal_angles, center, starts, direction,
        minimum_gap_degrees, maximum_gap_degrees, hard_clearance,
        minimum_3d, swarm_clearance, angular_window_degrees,
        nominal_orbit_radius, orbit_radius_step)
    diagnostics["next_role"] = next_uid
    diagnostics["committed_role_order"] = prefix[:-1]
    if not selected:
        return committed, grants, reason, diagnostics

    updated = dict(committed)
    updated[next_uid] = selected[next_uid]
    grants[next_uid] = True
    diagnostics["committed_role_order"] = prefix
    return updated, grants, "UAV{}_{}".format(next_uid, reason), diagnostics


def reconcile_entry_corridor_commitments(
        committed_selection, commit_state, ordered_uav_ids,
        current_generations, current_candidate_ids,
        current_locked_candidate_ids=None, safely_entered_ids=None,
        released_ids=None):
    """Rollback a stale committed ENTRY role and every downstream role.

    A commitment remains sticky only while its mission is still using the
    generation and candidate that the manager selected.  Roles that have
    already safely entered (or have been released) are immutable
    predecessors.  The first stale, not-yet-entered role becomes the next
    incremental arbitration owner; every later selection is withdrawn.
    """
    order = [int(uid) for uid in ordered_uav_ids]
    selection = dict(committed_selection or {})
    states = {
        int(uid): dict(value) for uid, value in (commit_state or {}).items()}
    locked = current_locked_candidate_ids or {}
    entered = set() if safely_entered_ids is None else set(
        int(uid) for uid in safely_entered_ids)
    released = set() if released_ids is None else set(
        int(uid) for uid in released_ids)
    if (not order or len(order) != len(set(order))
            or set(selection) != set(order[:len(selection)])):
        return selection, states, [], "INVALID_COMMITTED_ROLE_PREFIX"

    for uid in order:
        state = states.setdefault(uid, {
            "selected_candidate_id": "",
            "candidate_generation": None,
            "committed": False,
            "entered": False,
            "released": False,
            "mission_lock_observed": False,
        })
        state["entered"] = bool(state.get("entered", False) or uid in entered)
        state["released"] = bool(
            state.get("released", False) or uid in released)
        if uid not in selection:
            continue
        selected_id = selection[uid].get("id", "")
        state["selected_candidate_id"] = selected_id
        state["committed"] = True
        observed_lock = str(locked.get(uid, ""))
        if observed_lock == selected_id:
            state["mission_lock_observed"] = True

    rollback_index = None
    rollback_reason = "COMMITMENTS_CURRENT"
    for index, uid in enumerate(order):
        if uid not in selection:
            break
        state = states[uid]
        if state["entered"] or state["released"]:
            continue
        if uid not in current_generations:
            continue
        current_generation = int(current_generations[uid])
        committed_generation = state.get("candidate_generation")
        selected_id = state.get("selected_candidate_id", "")
        candidate_ids = set(current_candidate_ids.get(uid, set()))
        observed_lock = str(locked.get(uid, ""))
        stale_reason = ""
        if committed_generation is None:
            stale_reason = "COMMITTED_GENERATION_MISSING"
        elif current_generation != int(committed_generation):
            stale_reason = "CANDIDATE_GENERATION_CHANGED"
        elif selected_id not in candidate_ids:
            stale_reason = "LOCKED_CANDIDATE_DISAPPEARED"
        elif observed_lock and observed_lock != selected_id:
            stale_reason = "MISSION_LOCK_CHANGED"
        elif (state.get("mission_lock_observed", False)
              and not observed_lock):
            stale_reason = "MISSION_LOCK_CLEARED"
        if stale_reason:
            rollback_index = index
            rollback_reason = "UAV{}_{}".format(uid, stale_reason)
            break

    if rollback_index is None:
        return selection, states, [], rollback_reason

    revoked = []
    for uid in order[rollback_index:]:
        if uid not in selection:
            continue
        revoked.append(uid)
        selection.pop(uid, None)
        state = states[uid]
        state["selected_candidate_id"] = ""
        state["candidate_generation"] = None
        state["committed"] = False
        state["mission_lock_observed"] = False
    return selection, states, revoked, rollback_reason


def task_start_barrier_ready(uav_ids, states, health, globally_clear):
    """Release the formal mission only after every vehicle is hover-ready.

    Takeoff permission and task-start permission are intentionally separate.
    This makes a zero takeoff interval a real simultaneous launch while still
    preventing a faster vehicle from leaving its hover point before its peers.
    """
    if not globally_clear or not uav_ids:
        return False
    return all(
        uid in states
        and bool(health.get(uid, False))
        and states[uid].flight_state == "HOVER_READY"
        and states[uid].mission_phase == "WAIT_INPUTS"
        for uid in uav_ids)


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


def normalize_degrees(angle):
    """Normalize an angle to the common [0, 360) contract."""
    value = float(angle) % 360.0
    return 0.0 if abs(value - 360.0) <= 1.0e-9 else value


def directed_phase_gap_degrees(follower, leader, center, direction):
    """Return the forward orbit angle from follower to leader.

    ``direction`` is +1 for counter-clockwise and -1 for clockwise.  The
    result is always in [0, 360), so wraparound never changes formation role.
    """
    if direction not in (-1, 1):
        return float("nan")
    follower_angle = math.degrees(math.atan2(
        float(follower[1]) - float(center[1]),
        float(follower[0]) - float(center[0])))
    leader_angle = math.degrees(math.atan2(
        float(leader[1]) - float(center[1]),
        float(leader[0]) - float(center[0])))
    return normalize_degrees(direction * (leader_angle - follower_angle))


def formation_phase_decision(positions, center, ordered_uav_ids, direction,
                             normal_min_degrees, normal_max_degrees,
                             warning_min_degrees, emergency_degrees,
                             leader_wait_degrees, previously_held=None):
    """Evaluate role-bound phase gaps without permitting overtaking.

    The input order is leader, middle, trailing.  A following UAV is held at
    the emergency boundary.  A predecessor is also held when its follower has
    fallen beyond ``leader_wait_degrees``; this lets the formation close by
    forward motion only and never commands a leader to fly backwards.
    """
    ids = [int(uid) for uid in ordered_uav_ids]
    held_before = set(int(uid) for uid in (previously_held or set()))
    if (len(ids) < 2 or len(ids) != len(set(ids)) or direction not in (-1, 1)):
        return set(), {}, {}, "INVALID_ROLE_ORDER"
    thresholds = [normal_min_degrees, normal_max_degrees,
                  warning_min_degrees, emergency_degrees,
                  leader_wait_degrees]
    if (not all(math.isfinite(float(value)) for value in thresholds)
            or not (0.0 < emergency_degrees <= warning_min_degrees
                    <= normal_min_degrees < normal_max_degrees
                    < leader_wait_degrees < 180.0)):
        return set(), {}, {}, "INVALID_PHASE_THRESHOLDS"
    if any(uid not in positions for uid in ids):
        return set(), {}, {}, "MISSING_ROLE_POSITION"

    holds = set()
    # Only a held follower (too close to its predecessor) propagates HOLD to
    # vehicles farther back in the fixed role chain.  A predecessor stopped by
    # LEADER_WAIT must leave its follower free to advance and close the gap;
    # propagating that HOLD would freeze both sides of the error permanently.
    downstream_hold_sources = set()
    gaps = {}
    bands = {}
    for leader_id, follower_id in zip(ids[:-1], ids[1:]):
        gap = directed_phase_gap_degrees(
            positions[follower_id], positions[leader_id], center, direction)
        if not math.isfinite(gap):
            return set(), {}, {}, "INVALID_ROLE_POSITION"
        key = "{}-{}".format(leader_id, follower_id)
        gaps[key] = gap
        # A gap over 180 degrees means the named follower has crossed the
        # leader in the configured direction.  Holding both adjacent vehicles
        # is the only role-preserving fail-closed action.
        if gap > 180.0:
            holds.update((leader_id, follower_id))
            downstream_hold_sources.add(leader_id)
            bands[key] = "OVERTAKE_OR_REVERSE"
        elif gap < emergency_degrees:
            holds.add(follower_id)
            downstream_hold_sources.add(follower_id)
            bands[key] = "EMERGENCY"
        elif gap < warning_min_degrees:
            bands[key] = "WARNING_SLOW"
        elif gap < normal_min_degrees:
            bands[key] = "RECOVERY"
        elif gap <= normal_max_degrees:
            bands[key] = "NORMAL"
        elif gap < leader_wait_degrees:
            bands[key] = "FOLLOWER_BEHIND"
        else:
            holds.add(leader_id)
            bands[key] = "LEADER_WAIT"

    # Release hysteresis: an emergency follower remains held until it is back
    # inside the normal lower bound.  A waiting predecessor remains held until
    # its gap is no larger than the configured normal maximum.
    for leader_id, follower_id in zip(ids[:-1], ids[1:]):
        key = "{}-{}".format(leader_id, follower_id)
        gap = gaps[key]
        if follower_id in held_before and gap < normal_min_degrees:
            holds.add(follower_id)
            downstream_hold_sources.add(follower_id)
        if leader_id in held_before and gap > normal_max_degrees:
            holds.add(leader_id)

    # An emergency follower HOLD propagates down the physical chain so a still
    # farther follower cannot overtake it.  LEADER_WAIT is intentionally not a
    # propagation source: its follower is the vehicle that must keep moving.
    for index, uid in enumerate(ids[:-1]):
        if uid in downstream_hold_sources:
            holds.update(ids[index + 1:])
    return holds, gaps, bands, "OK"


def formation_speed_scale_targets(ordered_uav_ids, gaps, held_uav_ids,
                                  emergency_degrees, normal_min_degrees,
                                  minimum_warning_scale):
    """Compute forward-only role-bound speed scales for the orbit adapter."""
    ids = [int(uid) for uid in ordered_uav_ids]
    held = set(int(uid) for uid in held_uav_ids)
    emergency = float(emergency_degrees)
    normal_min = float(normal_min_degrees)
    minimum_scale = float(minimum_warning_scale)
    if (len(ids) < 2 or len(ids) != len(set(ids))
            or not all(math.isfinite(value) for value in (
                emergency, normal_min, minimum_scale))
            or not (0.0 < emergency < normal_min < 180.0)
            or not (0.0 < minimum_scale <= 1.0)):
        return {}, "INVALID_SPEED_SCALE_CONFIG"

    scales = {uid: 1.0 for uid in ids}
    for leader_id, follower_id in zip(ids[:-1], ids[1:]):
        key = "{}-{}".format(leader_id, follower_id)
        if key not in gaps or not math.isfinite(float(gaps[key])):
            return {}, "MISSING_PHASE_GAP"
        gap = float(gaps[key])
        if gap < emergency:
            scales[follower_id] = 0.0
        elif gap < normal_min:
            fraction = (gap - emergency) / (normal_min - emergency)
            warning_scale = minimum_scale + fraction * (1.0 - minimum_scale)
            scales[follower_id] = min(scales[follower_id], warning_scale)

    for uid in held:
        if uid in scales:
            scales[uid] = 0.0
    return scales, "OK"


def slew_speed_scale(current, target, dt_seconds, rise_rate, fall_rate):
    """Limit recovery acceleration while allowing faster safety slowdown."""
    values = [current, target, dt_seconds, rise_rate, fall_rate]
    if (not all(math.isfinite(float(value)) for value in values)
            or dt_seconds < 0.0 or rise_rate <= 0.0 or fall_rate <= 0.0):
        return 0.0
    current = min(1.0, max(0.0, float(current)))
    target = min(1.0, max(0.0, float(target)))
    limit = (rise_rate if target > current else fall_rate) * dt_seconds
    if target > current:
        return min(target, current + limit)
    return max(target, current - limit)


def role_chain_hold_ids(ordered_uav_ids, source_uav_id):
    """Return only the vehicles physically behind a held formation role."""
    ids = [int(uid) for uid in ordered_uav_ids]
    source = int(source_uav_id)
    if (not ids or len(ids) != len(set(ids)) or source not in ids):
        return set()
    return set(ids[ids.index(source) + 1:])


def contiguous_role_segments(ordered_uav_ids, active_uav_ids):
    """Keep active formation links without bridging over an absent role."""
    order = [int(uid) for uid in ordered_uav_ids]
    active = set(int(uid) for uid in active_uav_ids)
    if not order or len(order) != len(set(order)) or not active <= set(order):
        return []
    segments = []
    current = []
    for uid in order:
        if uid in active:
            current.append(uid)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def entry_ready_barrier(uav_ids, states, health, mission_heights,
                        height_tolerance, maximum_speed, globally_clear):
    """Evaluate every independent ENTRY hover at its configured height."""
    if not globally_clear or not uav_ids:
        return False, {}
    heights = [float(value) for value in mission_heights]
    if len(heights) != len(uav_ids):
        return False, {uid: "HEIGHT_CONFIG_MISMATCH" for uid in uav_ids}
    reasons = {}
    for index, uid in enumerate(uav_ids):
        if uid not in states:
            reasons[uid] = "STATE_MISSING"
            continue
        state = states[uid]
        speed = math.sqrt(
            float(state.velocity.x) ** 2 + float(state.velocity.y) ** 2
            + float(state.velocity.z) ** 2)
        if not bool(health.get(uid, False)):
            reasons[uid] = "HEALTH_OR_TRAJECTORY_INVALID"
        elif state.mission_phase not in {
                "ENTRY_READY", "WAIT_ORBIT_PERMISSION"}:
            reasons[uid] = "NOT_AT_ENTRY_GATE:{}".format(state.mission_phase)
        elif abs(float(state.current_height) - heights[index]) > float(
                height_tolerance):
            reasons[uid] = "HEIGHT_NOT_READY"
        elif not math.isfinite(speed) or speed > float(maximum_speed):
            reasons[uid] = "SPEED_NOT_SETTLED"
        # A mission-supervised position latch is represented by bridge HOLD.
        # ENTRY_READY makes that latch unambiguous; HOLD_SAFE and all fault
        # states remain fail-closed.
        elif state.flight_state in {"HOLD_SAFE", "ERROR", "FAILSAFE",
                                   "SAFETY_INHIBIT"}:
            reasons[uid] = "FLIGHT_STATE_{}".format(state.flight_state)
        else:
            reasons[uid] = "READY"
    return all(reasons.get(uid) == "READY" for uid in uav_ids), reasons


def derive_layer_target_heights(uav_ids, initial_heights, layer_offsets,
                                layer_index):
    """Derive one layer's per-UAV targets without vehicle-specific rules."""
    ids = [int(uid) for uid in uav_ids]
    heights = [float(value) for value in initial_heights]
    offsets = [float(value) for value in layer_offsets]
    if (not ids or len(ids) != len(set(ids)) or len(heights) != len(ids)
            or not offsets or layer_index < 0 or layer_index >= len(offsets)
            or not all(math.isfinite(value) for value in heights + offsets)
            or abs(offsets[0]) > 1.0e-9
            or any(offsets[index] >= offsets[index - 1]
                   for index in range(1, len(offsets)))):
        return {}
    return {
        uid: heights[index] + offsets[layer_index]
        for index, uid in enumerate(ids)
    }


def layer_transition_speed_config_valid(multi_layer_enabled,
                                        transition_speed, orbit_speed):
    """Validate transition speed only when layer transitions are active."""
    transition = float(transition_speed)
    orbit = float(orbit_speed)
    if not math.isfinite(transition) or not math.isfinite(orbit):
        return False
    if transition <= 0.0 or orbit <= 0.0:
        return False
    return (not bool(multi_layer_enabled)) or transition <= orbit


def downstream_handoff_ready(uid, role_order, layers, released_layers):
    """A role remains available until its immediate follower joins this layer."""
    index = role_order.index(uid)
    return (index + 1 == len(role_order)
            or released_layers.get(role_order[index + 1], -1) >= layers[uid])


def async_role_transition_permissions(
        role_order, states, health, initial_heights, offsets, layers,
        confirmed_release_layers, height_tolerance, maximum_speed,
        globally_clear,
        owner=0, started=False):
    """One sticky owner; completion is witnessed, never inferred from height alone.

    Per-role layers count completed transitions. No round-wide reset or barrier
    is used; a predecessor may orbit or remain safely latched at target-layer
    staging while its follower is still on an older layer.  A follower becomes
    eligible as soon as its immediate predecessor has physically completed the
    target-layer transition; target-layer orbit release is intentionally not a
    second transition barrier.
    """
    layers = dict(layers)
    grants = {uid: False for uid in role_order}
    reasons = {uid: "WAITING_FOR_ORBIT_COMPLETION" for uid in role_order}
    if (not role_order or len(set(role_order)) != len(role_order)
            or set(layers) != set(role_order)
            or set(initial_heights) != set(role_order)
            or owner not in set(role_order) | {0}
            or not offsets or height_tolerance <= 0 or maximum_speed < 0
            or any(not math.isfinite(float(v)) for v in
                   list(initial_heights.values()) + list(offsets)
                   + [height_tolerance, maximum_speed])
            or any(not isinstance(v, int) or v < 0 or v >= len(offsets)
                   for v in layers.values())):
        return grants, owner, started, layers, {uid: "INVALID_CONFIG" for uid in role_order}
    if (not globally_clear or any(uid not in states or not health.get(uid, False)
                                for uid in role_order)):
        return grants, owner, started, layers, {uid: "STALE_OR_SAFETY_INVALID" for uid in role_order}

    if any(states[uid].mission_phase == "LAYER_TRANSITION" and uid != owner
           for uid in role_order):
        return grants, owner, started, layers, {uid: "UNOWNED_TRANSITION" for uid in role_order}

    def settled(uid, layer):
        state = states[uid]
        values = [state.current_height, state.velocity.x,
                  state.velocity.y, state.velocity.z]
        return (all(math.isfinite(float(v)) for v in values)
                and abs(values[0] - initial_heights[uid] - offsets[layer]) <= height_tolerance
                and math.sqrt(sum(v*v for v in values[1:])) <= maximum_speed)

    if owner:
        phase = states[owner].mission_phase
        started = started or phase == "LAYER_TRANSITION"
        target = layers[owner] + 1
        if (started and target < len(offsets)
                and phase == "ORBIT_STAGING_READY" and settled(owner, target)):
            layers[owner] = target
            reasons[owner] = "TRANSITION_COMPLETE"
            # Return this tick without granting another owner: the caller first
            # resets only this role's orbit release and exposes the new layer.
            return grants, 0, False, layers, reasons
        if phase in {"WAIT_TRANSITION_PERMISSION", "LAYER_TRANSITION", "HOLDING"}:
            grants[owner] = True
            reasons[owner] = "OWNER_ACTIVE"
        else:
            reasons[owner] = "OWNER_AWAITING_STABLE_TARGET"
        return grants, owner, started, layers, reasons

    for index, uid in enumerate(role_order):
        target = layers[uid] + 1
        if target >= len(offsets):
            reasons[uid] = "FINAL_LAYER"
            continue
        if states[uid].mission_phase != "WAIT_TRANSITION_PERMISSION":
            continue
        if index and layers[role_order[index - 1]] < target:
            reasons[uid] = "WAITING_FOR_PREDECESSOR"
            continue
        if (confirmed_release_layers.get(uid, -1) != layers[uid]
                or not settled(uid, layers[uid])):
            reasons[uid] = "SOURCE_NOT_READY"
            continue
        grants[uid] = True
        reasons[uid] = "OWNER_GRANTED"
        return grants, uid, False, layers, reasons
    return grants, 0, False, layers, reasons


def serialized_transition_permissions(
        ordered_uav_ids, states, health, target_heights, height_tolerance,
        maximum_speed, globally_clear, active_owner=0, started_ids=None,
        completed_ids=None, enabled=True):
    """Grant one sticky layer-transition owner in the configured role order.

    A transition is complete only after its owner has entered LAYER_TRANSITION,
    returned to an active orbit phase, reached its configured target
    height, and settled below ``maximum_speed``. Missing/stale/unsafe inputs
    revoke every grant without transferring the sticky owner.
    """
    order = [int(uid) for uid in ordered_uav_ids]
    started = set() if started_ids is None else set(started_ids)
    completed = set() if completed_ids is None else set(completed_ids)
    grants = {uid: False for uid in order}
    reasons = {uid: "WAITING_FOR_PREDECESSOR" for uid in order}
    owner = int(active_owner)
    if not enabled:
        return grants, 0, set(), set(), {
            uid: "MULTI_LAYER_DISABLED" for uid in order}
    if (not order or len(order) != len(set(order))
            or owner not in set(order) | {0}
            or set(target_heights) != set(order)
            or not math.isfinite(float(height_tolerance))
            or not math.isfinite(float(maximum_speed))
            or float(height_tolerance) <= 0.0
            or float(maximum_speed) < 0.0):
        return grants, owner, started, completed, {
            uid: "INVALID_TRANSITION_CONFIG" for uid in order}
    if (not globally_clear
            or any(uid not in states or not bool(health.get(uid, False))
                   for uid in order)):
        return grants, owner, started, completed, {
            uid: "STALE_OR_SAFETY_INVALID" for uid in order}

    def settled_at_target(uid):
        state = states[uid]
        velocity = state.velocity
        speed = math.sqrt(
            float(velocity.x) ** 2 + float(velocity.y) ** 2
            + float(velocity.z) ** 2)
        return bool(
            math.isfinite(speed)
            and abs(float(state.current_height)
                    - float(target_heights[uid])) <= float(height_tolerance)
            and speed <= float(maximum_speed))

    completion_phases = {
        "ORBIT_STAGING_READY", "EVALUATING", "TARGET_LOCKED", "NAVIGATING"}
    allowed_owner_phases = {
        "WAIT_TRANSITION_PERMISSION", "LAYER_TRANSITION", "HOLDING"}
    failure_phases = {
        "WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE", "NORMAL_RETURN",
        "RETURN_EGRESS", "HOME_OVERHEAD_TRANSIT", "SEGMENTED_HOME_DESCENT",
        "RETURN_HOME", "FAILURE_LANDING", "DONE", "ERROR",
        "FAILSAFE", "SAFETY_INHIBIT"}

    if owner:
        owner_phase = states[owner].mission_phase
        if owner_phase == "LAYER_TRANSITION":
            started.add(owner)
        if (owner in started and owner_phase in completion_phases
                and settled_at_target(owner)):
            completed.add(owner)
            reasons[owner] = "TRANSITION_COMPLETE"
            owner = 0
        elif owner_phase in failure_phases:
            reasons[owner] = "OWNER_PHASE_{}".format(owner_phase)
            return grants, owner, started, completed, reasons

    if not owner:
        for index, uid in enumerate(order):
            if uid in completed:
                reasons[uid] = "TRANSITION_COMPLETE"
                continue
            if any(predecessor not in completed for predecessor in order[:index]):
                reasons[uid] = "WAITING_FOR_PREDECESSOR"
                break
            phase = states[uid].mission_phase
            if phase == "LAYER_TRANSITION":
                reasons[uid] = "UNOWNED_TRANSITION_FAIL_CLOSED"
                return grants, 0, started, completed, reasons
            if phase == "WAIT_TRANSITION_PERMISSION":
                owner = uid
                reasons[uid] = "OWNER_GRANTED"
            else:
                reasons[uid] = "WAITING_FOR_LAYER_COMPLETION"
            break

    for uid in order:
        if uid in completed:
            reasons[uid] = "TRANSITION_COMPLETE"
    if owner:
        phase = states[owner].mission_phase
        if phase in allowed_owner_phases:
            grants[owner] = True
            reasons[owner] = "OWNER_ACTIVE"
        elif owner in started and phase in completion_phases:
            reasons[owner] = "OWNER_AWAITING_STABLE_TARGET"
        else:
            reasons[owner] = "OWNER_PHASE_{}_FAIL_CLOSED".format(phase)
    return grants, owner, started, completed, reasons


def orbit_staging_ready_barrier(uav_ids, states, health, mission_heights,
                                height_tolerance, maximum_speed,
                                globally_clear):
    """Require every UAV to be latched at its own first orbit point."""
    if not globally_clear or not uav_ids:
        return False, {}
    heights = [float(value) for value in mission_heights]
    if len(heights) != len(uav_ids):
        return False, {uid: "HEIGHT_CONFIG_MISMATCH" for uid in uav_ids}
    reasons = {}
    for index, uid in enumerate(uav_ids):
        state = states.get(uid)
        if state is None:
            reasons[uid] = "STATE_MISSING"
            continue
        speed = math.sqrt(
            float(state.velocity.x) ** 2 + float(state.velocity.y) ** 2
            + float(state.velocity.z) ** 2)
        if not bool(health.get(uid, False)):
            reasons[uid] = "HEALTH_OR_TRAJECTORY_INVALID"
        elif state.mission_phase != "ORBIT_STAGING_READY":
            reasons[uid] = "NOT_AT_ORBIT_STAGING:{}".format(
                state.mission_phase)
        elif abs(float(state.current_height) - heights[index]) > float(
                height_tolerance):
            reasons[uid] = "HEIGHT_NOT_READY"
        elif not math.isfinite(speed) or speed > float(maximum_speed):
            reasons[uid] = "SPEED_NOT_SETTLED"
        elif state.flight_state in {"HOLD_SAFE", "ERROR", "FAILSAFE",
                                   "SAFETY_INHIBIT"}:
            reasons[uid] = "FLIGHT_STATE_{}".format(state.flight_state)
        else:
            reasons[uid] = "READY"
    return all(reasons.get(uid) == "READY" for uid in uav_ids), reasons


def directed_tangential_speed(position, velocity, center, direction):
    """Return signed speed along the configured tower orbit direction."""
    if direction not in (-1, 1):
        return float("nan")
    dx = float(position[0]) - float(center[0])
    dy = float(position[1]) - float(center[1])
    radius = math.hypot(dx, dy)
    if (not math.isfinite(radius) or radius <= 1.0e-9
            or not all(math.isfinite(float(v)) for v in velocity[:2])):
        return float("nan")
    tangent_x = direction * (-dy / radius)
    tangent_y = direction * (dx / radius)
    return float(velocity[0]) * tangent_x + float(velocity[1]) * tangent_y


def predicted_pair_clear(first_position, second_position,
                         first_prediction, second_prediction,
                         minimum_3d, swarm_clearance):
    """Apply the same 3-D and EGO ellipsoid limits to a release pair."""
    if (not first_prediction or not second_prediction
            or minimum_3d <= 0.0 or swarm_clearance <= 0.0):
        return False
    count = min(len(first_prediction), len(second_prediction))
    ellipsoid_limit = 2.0 * float(swarm_clearance)
    pairs = [(first_position, second_position)] + [
        (first_prediction[index], second_prediction[index])
        for index in range(count)]
    return all(
        distance3(first, second) >= float(minimum_3d)
        and ellipsoid_distance(first, second) >= ellipsoid_limit
        for first, second in pairs)


def sequential_orbit_release_allowed(follower_position, leader_position,
                                     leader_velocity, center, direction,
                                     phase_min_degrees, phase_max_degrees,
                                     minimum_forward_speed,
                                     leader_trajectory_fresh,
                                     follower_ready, predicted_clear,
                                     globally_clear):
    """Evaluate one UAV3->UAV2 or UAV2->UAV1 release gate."""
    phase = directed_phase_gap_degrees(
        follower_position, leader_position, center, direction)
    speed = directed_tangential_speed(
        leader_position, leader_velocity, center, direction)
    conditions = {
        "phase_in_window": bool(
            math.isfinite(phase)
            and float(phase_min_degrees) <= phase <= float(phase_max_degrees)),
        "leader_forward_stable": bool(
            math.isfinite(speed) and speed >= float(minimum_forward_speed)),
        "leader_trajectory_fresh": bool(leader_trajectory_fresh),
        "follower_ready": bool(follower_ready),
        "predicted_clear": bool(predicted_clear),
        "globally_clear": bool(globally_clear),
    }
    return all(conditions.values()), phase, speed, conditions


def target_layer_late_release_allowed(phase, phase_max_degrees, conditions):
    """Release a safe target-layer follower that arrived after the window.

    The ordinary 65--70 degree gate remains authoritative for the initial
    orbit. During an asynchronous layer transition, waiting another full
    release window after the predecessor is already safely ahead only
    increases the gap. All non-phase gates remain mandatory, and a phase at
    or beyond 180 degrees is rejected to preserve fixed role order.
    """
    required = (
        "leader_forward_stable", "leader_trajectory_fresh",
        "follower_ready", "predicted_clear", "globally_clear")
    if not isinstance(conditions, dict):
        return False
    try:
        phase = float(phase)
        phase_max_degrees = float(phase_max_degrees)
    except (TypeError, ValueError):
        return False
    return bool(
        math.isfinite(phase) and math.isfinite(phase_max_degrees)
        and 0.0 < phase_max_degrees < phase < 180.0
        and all(bool(conditions.get(name, False)) for name in required))


def completed_predecessor_handoff_allowed(
        follower_position, leader_position, predecessor_orbit_complete,
        predecessor_in_return_phase, leader_trajectory_fresh,
        follower_ready, predicted_clear, globally_clear,
        minimum_3d, swarm_clearance):
    """Fail-closed release fallback after the predecessor completed orbit.

    This is distinct from the normal 65--70 degree moving-window gate.  It is
    available only after completion has been observed and retains current and
    predicted 3-D/EGO-ellipsoid separation plus freshness/global safety.
    Fixed-role ordering remains the caller's responsibility.
    """
    leader = tuple(float(value) for value in leader_position)
    follower = tuple(float(value) for value in follower_position)
    finite_positions = bool(
        len(leader) >= 3 and len(follower) >= 3
        and all(math.isfinite(value) for value in leader[:3] + follower[:3]))
    current_3d_clear = bool(
        finite_positions
        and float(minimum_3d) > 0.0
        and distance3(leader[:3], follower[:3]) >= float(minimum_3d))
    current_ellipsoid_clear = bool(
        finite_positions
        and float(swarm_clearance) > 0.0
        and ellipsoid_distance(leader[:3], follower[:3])
        >= 2.0 * float(swarm_clearance))
    conditions = {
        "predecessor_orbit_complete": bool(predecessor_orbit_complete),
        "predecessor_in_return_phase": bool(predecessor_in_return_phase),
        "leader_trajectory_fresh": bool(leader_trajectory_fresh),
        "follower_ready": bool(follower_ready),
        "current_3d_clear": current_3d_clear,
        "current_ellipsoid_clear": current_ellipsoid_clear,
        "predicted_clear": bool(predicted_clear),
        "globally_clear": bool(globally_clear),
    }
    return all(conditions.values()), conditions


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
