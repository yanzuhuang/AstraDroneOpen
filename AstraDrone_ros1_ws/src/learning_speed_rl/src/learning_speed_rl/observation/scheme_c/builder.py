"""Pure builder joining one timestamped lidar packet to trajectory and state."""

import numpy as np

from .sampling import TrajectorySampler
from .types import FutureTrajectoryFeature, ObservationC, SystemStateFeature


class ObservationCBuilder:
    def __init__(
        self, sampler: TrajectorySampler, world_frame_id: str,
        body_frame_id: str, state_source_type: str = "fast_lio_state_estimate",
        state_contract_version: str = "astradrone_planning_odometry_v1.0",
        state_velocity_source: str = "timestamped_state_position_difference_world",
    ):
        if not all((world_frame_id, body_frame_id, state_source_type,
                    state_contract_version, state_velocity_source)):
            raise ValueError("Observation C world/body frames must be nonempty")
        self.sampler = sampler
        self.world_frame_id = world_frame_id.lstrip("/")
        self.body_frame_id = body_frame_id.lstrip("/")
        self.state_source_type = str(state_source_type)
        self.state_contract_version = str(state_contract_version)
        self.state_velocity_source = str(state_velocity_source)

    def build(self, lidar, trajectory, kinematic_state, previous_v_max):
        if trajectory.frame_id.lstrip("/") != self.world_frame_id:
            raise ValueError("trajectory frame does not match configured world frame")
        if lidar.frame_id.lstrip("/") != self.body_frame_id:
            raise ValueError("lidar feature frame does not match configured body frame")
        if not trajectory.is_active_at(lidar.stamp_sec):
            raise ValueError("trajectory timestamp mismatch")

        positions_world, offsets, sampling_metadata = self.sampler.sample(
            trajectory, lidar.stamp_sec
        )
        pose = kinematic_state.pose
        positions_body = pose.world_to_body(positions_world)
        desired_world = trajectory.evaluate_stamp(lidar.stamp_sec)
        tracking_error_body = (desired_world - pose.translation) @ pose.rotation
        actual_velocity_body = kinematic_state.velocity_world @ pose.rotation

        future = FutureTrajectoryFeature(
            positions_body=positions_body,
            sample_offsets=offsets,
            sampling_mode=self.sampler.config.sampling_mode,
            sample_spacing=self.sampler.config.sample_spacing,
            max_distance=self.sampler.config.max_distance,
            trajectory_id=trajectory.trajectory_id,
            trajectory_start_time_sec=trajectory.start_time_sec,
            source_frame_id=trajectory.frame_id,
            metadata=sampling_metadata,
        )
        system_state = SystemStateFeature(
            actual_velocity_body=actual_velocity_body,
            tracking_error_body=tracking_error_body,
            previous_v_max=previous_v_max,
        )
        return ObservationC(
            stamp_sec=lidar.stamp_sec,
            frame_id=lidar.frame_id,
            lidar_surrogate=lidar,
            future_trajectory=future,
            system_state=system_state,
            metadata={
                "pose_lookup": getattr(kinematic_state, "lookup_mode", "matched"),
                "trajectory_source": "traj_utils/Bspline",
                "state_source_type": self.state_source_type,
                "state_contract_version": self.state_contract_version,
                "tracking_error_definition": "desired_bspline_position_minus_actual_state_position",
                "actual_velocity_source": self.state_velocity_source,
                "previous_v_max_source": "ego_applied_v_max",
            },
        )
