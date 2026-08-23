"""Math-only contract for training raw-Mid360 truth registration."""

import numpy as np


def quaternion_rotation_matrix_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be finite xyzw")
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def register_sensor_points_world(
    points_sensor, world_position, world_orientation_xyzw,
    body_sensor_translation,
):
    points = np.asarray(points_sensor, dtype=np.float64)
    position = np.asarray(world_position, dtype=np.float64)
    translation = np.asarray(body_sensor_translation, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_sensor must have shape [N,3]")
    if position.shape != (3,) or translation.shape != (3,):
        raise ValueError("position/translation must have shape [3]")
    if not (
        np.all(np.isfinite(points))
        and np.all(np.isfinite(position))
        and np.all(np.isfinite(translation))
    ):
        raise ValueError("registration input must be finite")
    rotation = quaternion_rotation_matrix_xyzw(world_orientation_xyzw)
    return (points + translation).dot(rotation.T) + position


def segment_corridor_statistics(points_world, start, goal, flight_z, half_height, radius):
    points = np.asarray(points_world, dtype=np.float64)
    start_xy = np.asarray(start[:2], dtype=np.float64)
    goal_xy = np.asarray(goal[:2], dtype=np.float64)
    direction = goal_xy - start_xy
    denominator = float(np.dot(direction, direction))
    if denominator <= 1.0e-12 or half_height <= 0.0 or radius <= 0.0:
        raise ValueError("invalid route corridor")
    flight = points[np.abs(points[:, 2] - float(flight_z)) <= float(half_height)]
    if flight.size == 0:
        return {"flight_band_points": 0, "corridor_points": 0, "minimum_distance": None}
    relative = flight[:, :2] - start_xy
    parameter = np.clip(relative.dot(direction) / denominator, 0.0, 1.0)
    closest = start_xy + parameter[:, None] * direction
    distances = np.linalg.norm(flight[:, :2] - closest, axis=1)
    return {
        "flight_band_points": int(flight.shape[0]),
        "corridor_points": int(np.count_nonzero(distances <= float(radius))),
        "minimum_distance": float(np.min(distances)),
    }
