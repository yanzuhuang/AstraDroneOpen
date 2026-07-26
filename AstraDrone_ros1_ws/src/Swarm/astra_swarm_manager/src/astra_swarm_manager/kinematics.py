"""Small, ROS-independent kinematic estimator for swarm prediction."""

import math


class KinematicEstimator:
    def __init__(self, velocity_alpha=0.25, acceleration_alpha=0.15,
                 maximum_speed=3.0, maximum_acceleration=2.0):
        self.velocity_alpha = float(velocity_alpha)
        self.acceleration_alpha = float(acceleration_alpha)
        self.maximum_speed = float(maximum_speed)
        self.maximum_acceleration = float(maximum_acceleration)
        self.last_position = None
        self.last_stamp = None
        self.velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]

    @staticmethod
    def limit_vector(vector, maximum):
        norm = math.sqrt(sum(value * value for value in vector))
        if norm <= maximum or norm <= 1e-9:
            return list(vector)
        scale = maximum / norm
        return [value * scale for value in vector]

    def update(self, position, stamp):
        position = tuple(float(value) for value in position)
        stamp = float(stamp)
        if self.last_position is not None and self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if 0.02 <= dt <= 1.0:
                raw_velocity = self.limit_vector([
                    (position[i] - self.last_position[i]) / dt
                    for i in range(3)
                ], self.maximum_speed)
                previous_velocity = list(self.velocity)
                self.velocity = [
                    ((1.0 - self.velocity_alpha) * self.velocity[i]
                     + self.velocity_alpha * raw_velocity[i])
                    for i in range(3)
                ]
                raw_acceleration = self.limit_vector([
                    (self.velocity[i] - previous_velocity[i]) / dt
                    for i in range(3)
                ], self.maximum_acceleration)
                self.acceleration = [
                    ((1.0 - self.acceleration_alpha) * self.acceleration[i]
                     + self.acceleration_alpha * raw_acceleration[i])
                    for i in range(3)
                ]
        self.last_position = position
        self.last_stamp = stamp
        return list(self.velocity), list(self.acceleration)
