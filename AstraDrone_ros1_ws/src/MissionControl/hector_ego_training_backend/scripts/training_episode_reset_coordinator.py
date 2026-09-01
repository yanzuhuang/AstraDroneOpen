#!/usr/bin/env python3
"""Formal training-only Hover -> ENTRY_GATE -> teleport-reset coordinator."""

import json
import math
import os
from pathlib import Path
import resource
import statistics
import threading
import time

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from astra_custom_msgs.msg import PlannerStatus
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty as EmptyMessage, Header, String
from std_srvs.srv import Empty, Trigger
from traj_utils.msg import Bspline

from learning_speed_rl.msg import (
    LidarSurrogateStamped,
    ObservationC,
    SpeedActionStamped,
    SpeedAppliedStamped,
    SpeedRequestStamped,
)
from learning_speed_rl.training import (
    BalancedForestMapScheduler,
    actual_speed_mps_from_body_velocity,
    lidar_clutter_metrics,
    reward_from_config,
    reward_input_from_signals,
    tracking_error_m_from_body_error,
)

from hector_ego_training_backend.episode_reset_contract import (
    EpisodeIdentityLedger,
    RandomResetConfig,
    RandomResetSampler,
    ResetSamplingError,
    StaticObstacleXY,
    action_matches,
    classify_observation_staleness,
    episode_count_stop_due,
    fixed_terminal_hold_matches,
    observation_matches,
    route_acceptance_required,
    sac_closure_matches,
    sac_closure_reset_allowed,
    trajectory_matches,
    validate_reset_candidate,
)


VERSION = "astradrone_training_episode_reset_v1.0"
PLANNER_FAILURES = {
    PlannerStatus.GOAL_IN_OCCUPANCY,
    PlannerStatus.CURRENT_POSITION_IN_OCCUPANCY,
    PlannerStatus.NO_FEASIBLE_TRAJECTORY,
    PlannerStatus.REPLAN_FAILED,
    PlannerStatus.EMERGENCY_STOP_TIMEOUT,
    PlannerStatus.TRAJECTORY_EXPIRED,
    PlannerStatus.MAP_STALE,
}


def _norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def _roll_pitch(quaternion):
    sinr = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cosr = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return ordered[low]
    weight = index - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _statistics(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "count": len(finite),
        "min": min(finite) if finite else None,
        "median": statistics.median(finite) if finite else None,
        "p95": _percentile(finite, 0.95),
        "max": max(finite) if finite else None,
    }


def _stamp_sec(message):
    if message is None or not hasattr(message, "header"):
        return None
    return float(message.header.stamp.to_sec())


def _bspline_timing(message, now_sim):
    if message is None:
        return {
            "trajectory_id": None,
            "start_stamp_sec": None,
            "end_stamp_sec": None,
            "duration_sec": None,
            "remaining_lifetime_sec": None,
        }
    degree = int(message.order)
    knots = list(message.knots)
    start = float(message.start_time.to_sec())
    end = None
    if degree > 0 and len(knots) > 2 * degree:
        end = start + float(knots[len(knots) - 1 - degree]) - float(
            knots[degree]
        )
    return {
        "trajectory_id": int(message.traj_id),
        "start_stamp_sec": start,
        "end_stamp_sec": end,
        "duration_sec": None if end is None else end - start,
        "remaining_lifetime_sec": None if end is None else end - float(now_sim),
    }


class TrainingEpisodeResetCoordinator:
    def __init__(self):
        self._lock = threading.RLock()
        self._output_dir = str(rospy.get_param("~output_dir", "")).strip()
        self._qualification_only = bool(rospy.get_param("~qualification_only", True))
        self._episode_count = int(rospy.get_param("~episode_count", 5))
        self._runner_mode = str(
            rospy.get_param("~runner_mode", "qualification")
        ).strip().lower()
        self._require_acceptance = bool(
            rospy.get_param("~require_automated_acceptance", False)
        )
        self._fixed_v_max = float(rospy.get_param("~fixed_v_max", 0.30))
        self._reward_runtime_audit_enabled = bool(
            rospy.get_param("~reward_runtime_audit_enabled", False)
        )
        self._reward = (
            reward_from_config({"reward": rospy.get_param("~reward")})
            if self._reward_runtime_audit_enabled else None
        )
        self._action_owner = str(
            rospy.get_param("~action_owner", "fixed")
        ).strip().lower()
        self._sac_closure_timeout = float(
            rospy.get_param("~sac_closure_timeout_wall", 30.0)
        )
        if not self._qualification_only:
            raise ValueError("qualification_only must remain true")
        if not self._output_dir:
            raise ValueError("~output_dir is required")
        if self._runner_mode not in (
            "qualification", "training", "evaluation", "map_switch_preflight"
        ):
            raise ValueError("runner_mode is invalid")
        if self._episode_count <= 0:
            raise ValueError("episode_count must be positive")
        if self._action_owner not in ("fixed", "sac"):
            raise ValueError("action_owner must be fixed or sac")
        if self._sac_closure_timeout <= 0.0:
            raise ValueError("sac_closure_timeout_wall must be positive")
        if self._require_acceptance and self._runner_mode != "qualification":
            raise ValueError("automated acceptance is qualification-only")
        if self._require_acceptance and self._episode_count < 20:
            raise ValueError("automated acceptance requires at least 20 episodes")
        os.makedirs(self._output_dir, exist_ok=True)

        self._max_episode_time = float(rospy.get_param("~max_episode_time", 15.0))
        self._startup_timeout = float(rospy.get_param("~startup_timeout_wall", 30.0))
        self._trajectory_timeout = float(rospy.get_param("~trajectory_timeout_wall", 8.0))
        self._observation_timeout = float(rospy.get_param("~observation_timeout_wall", 8.0))
        self._reset_timeout = float(rospy.get_param("~reset_timeout_wall", 12.0))
        self._reset_target_timeout = float(
            rospy.get_param("~reset_target_timeout_wall", 2.0)
        )
        self._hold_timeout = float(rospy.get_param("~terminal_hold_timeout_wall", 5.0))
        self._entry = tuple(float(v) for v in rospy.get_param("~entry_goal", [2.0, 0.0, 1.5]))
        self._hover = tuple(float(v) for v in rospy.get_param("~hover_pose", [0.0, 0.0, 1.5, 0.0]))
        self._nominal_hover = self._hover
        self._goal_position_tolerance = float(rospy.get_param("~goal_position_tolerance", 0.25))
        self._goal_speed_tolerance = float(rospy.get_param("~goal_speed_tolerance", 0.20))
        self._goal_sustain_time = float(rospy.get_param("~goal_sustain_time", 0.30))
        self._observation_freshness = float(rospy.get_param("~observation_freshness", 0.35))
        self._invalid_observation_grace = float(rospy.get_param("~invalid_observation_grace", 0.50))
        self._truth_freshness = float(rospy.get_param("~truth_freshness", 0.20))
        self._lidar_freshness = float(rospy.get_param("~lidar_freshness", 0.30))
        self._reset_position_tolerance = float(rospy.get_param("~reset_position_tolerance", 0.25))
        self._reset_speed_tolerance = float(rospy.get_param("~reset_speed_tolerance", 0.35))
        self._reset_roll_pitch_tolerance = float(rospy.get_param("~reset_roll_pitch_tolerance", 0.30))
        self._collision_min_z = float(rospy.get_param("~collision_min_z", 0.40))
        self._publish_synthetic_ego_cloud = bool(
            rospy.get_param("~publish_synthetic_ego_cloud", True)
        )
        self._require_registered_ego_cloud = bool(
            rospy.get_param("~require_registered_ego_cloud", False)
        )
        self._truth_cloud_clear_service = str(
            rospy.get_param("~truth_cloud_clear_service", "")
        ).strip()
        self._require_forest_map_ready = bool(
            rospy.get_param("~require_forest_map_ready", False)
        )
        self._forest_map_switch_timeout = float(
            rospy.get_param("~forest_map_switch_timeout_wall", 45.0)
        )
        self._controllers = list(rospy.get_param("~controller_names", ["controller/pose", "controller/twist"]))
        if len(self._entry) != 3 or len(self._hover) != 4:
            raise ValueError("entry_goal/hover_pose dimensions are invalid")
        random_enabled = bool(rospy.get_param("~random_start/enabled", False))
        obstacle_values = rospy.get_param("~random_start/static_obstacles", [])
        obstacles = tuple(
            StaticObstacleXY(
                name=str(value["name"]),
                x=float(value["x"]),
                y=float(value["y"]),
                radius_at_hover_z=float(value["radius_at_hover_z"]),
            )
            for value in obstacle_values
        )
        if random_enabled:
            center_x = float(rospy.get_param("~random_start/center_x"))
            center_y = float(rospy.get_param("~random_start/center_y"))
            reset_z = float(rospy.get_param("~random_start/z"))
            reset_yaw = float(rospy.get_param("~random_start/yaw"))
            if max(
                abs(center_x - self._nominal_hover[0]),
                abs(center_y - self._nominal_hover[1]),
                abs(reset_z - self._nominal_hover[2]),
                abs(reset_yaw - self._nominal_hover[3]),
            ) > 1.0e-12:
                raise ValueError(
                    "random reset center/z/yaw must match nominal hover_pose"
                )
        else:
            center_x, center_y, reset_z, reset_yaw = self._nominal_hover
        self._random_reset_config = RandomResetConfig(
            enabled=random_enabled,
            center_x=center_x,
            center_y=center_y,
            x_min_offset=float(
                rospy.get_param("~random_start/x_min_offset", 0.0)
                if random_enabled else 0.0
            ),
            x_max_offset=float(
                rospy.get_param("~random_start/x_max_offset", 0.0)
                if random_enabled else 0.0
            ),
            y_min_offset=float(
                rospy.get_param("~random_start/y_min_offset", 0.0)
                if random_enabled else 0.0
            ),
            y_max_offset=float(
                rospy.get_param("~random_start/y_max_offset", 0.0)
                if random_enabled else 0.0
            ),
            z=reset_z,
            yaw=reset_yaw,
            seed=int(rospy.get_param("~random_start/seed", 1001)),
            max_sampling_attempts=int(
                rospy.get_param("~random_start/max_sampling_attempts", 1)
            ),
            uav_collision_radius_xy=float(
                rospy.get_param("~random_start/uav_collision_radius_xy", 0.0)
            ),
            ego_obstacles_inflation=float(
                rospy.get_param("~random_start/ego_obstacles_inflation", 0.0)
            ),
            additional_static_clearance=float(
                rospy.get_param("~random_start/additional_static_clearance", 0.0)
            ),
            static_obstacles=obstacles,
        )
        self._reset_sampler = RandomResetSampler(self._random_reset_config)
        initial_candidate = self._random_reset_config.nominal
        self._current_episode_start = {
            "initial_spawn_nominal": True,
            "nominal_hover": list(self._nominal_hover),
            "sampled_reset_x": initial_candidate.x,
            "sampled_reset_y": initial_candidate.y,
            "sampled_reset_z": initial_candidate.z,
            "sampled_yaw": initial_candidate.yaw,
            "reset_random_seed": self._random_reset_config.seed,
            "sample_index": 0,
            "attempt_count": 0,
            "candidate_validation_result": validate_reset_candidate(
                initial_candidate, self._random_reset_config
            ),
        }

        self._ledger = EpisodeIdentityLedger()
        self._binding = self._ledger.current
        self._state = "BOOTSTRAP"
        self._terminal_latched = False
        self._terminal_outcome = ""
        self._terminal_reason = ""
        self._reset_barrier = 0.0
        self._callback_sequence = 0
        self._previous_trajectory_id = 0
        self._accepted_trajectory_id = 0
        self._accepted_trajectory_start = 0.0
        self._accepted_trajectory_start_secs = 0
        self._accepted_trajectory_start_nsecs = 0
        self._goal_sequence = 0
        self._goal_stamp = 0.0
        self._odom = None
        self._odom_receipt_wall = 0.0
        self._odom_sequence = 0
        self._lidar = None
        self._lidar_receipt_wall = 0.0
        self._lidar_sequence = 0
        self._v2 = None
        self._v2_receipt_wall = 0.0
        self._v2_sequence = 0
        self._observation = None
        self._observation_receipt_wall = 0.0
        self._observation_sequence = 0
        self._last_valid_observation_wall = 0.0
        self._trajectory = None
        self._backend = {}
        self._planner = None
        self._request = None
        self._action = None
        self._applied = None
        self._sac_closure = None
        self._sac_closure_keys = set()
        self._duplicate_sac_closure_count = 0
        self._position_command = None
        self._v2_diagnostics = {}
        self._c_diagnostics = {}
        self._active_episode = False
        self._goal_ready_since_sim = None
        self._fixed_terminal_hold_active = False
        self._episode_observation_total = 0
        self._episode_observation_valid = 0
        self._episode_positions = []
        self._episode_tracking_errors = []
        self._episode_actual_speeds = []
        self._episode_reward_steps = []
        self._reward_runtime_step_count = 0
        self._reward_runtime_invalid_count = 0
        self._reward_runtime_audit_failure = ""

        self._events = []
        self._episodes = []
        self._resets = []
        self._old_generation_valid_contamination = 0
        self._stale_cloud_state_count = 0
        self._trajectory_generation_mismatch = 0
        self._controller_failure_count = 0
        self._planner_failure_count = 0
        self._collision_count = 0
        self._surrogate_obstacle_counts = []
        self._surrogate_free_counts = []
        self._surrogate_unknown_counts = []
        self._surrogate_all_zero_count = 0
        self._surrogate_all_unknown_count = 0
        self._tracking_errors = []
        self._actual_speeds = []
        self._registered_cloud_state = {}
        self._forest_map_state = {}
        self._forest_map_switches = []
        self._forest_preflight_records = []
        self._stale_snapshots = []
        self._active_stale_snapshot_id = None
        self._stale_gap_active = False
        self._registered_corridor_counts = []
        self._registered_corridor_minimum_distances = []
        self._event_file = open(
            os.path.join(self._output_dir, "qualification_events.jsonl"),
            "w", encoding="utf-8",
        )
        self._stale_snapshot_file = open(
            os.path.join(self._output_dir, "stale_observation_snapshots.jsonl"),
            "w", encoding="utf-8", buffering=1,
        )
        self._reward_step_file = (
            open(
                os.path.join(self._output_dir, "reward_runtime_steps.jsonl"),
                "w", encoding="utf-8",
            )
            if self._reward_runtime_audit_enabled else None
        )

        self._identity_pub = rospy.Publisher(
            "training/episode_identity", String, queue_size=1, latch=True
        )
        self._goal_pub = rospy.Publisher("planning/goal", PoseStamped, queue_size=1)
        self._reset_hover_pub = rospy.Publisher(
            "training/reset_hover", PoseStamped, queue_size=1, latch=True
        )
        self._cancel_pub = rospy.Publisher("planning/cancel", EmptyMessage, queue_size=1)
        self._forest_map_request_pub = rospy.Publisher(
            "/uav1/learning_speed/forest_map_request", String,
            queue_size=1, latch=True,
        )
        self._request_pub = rospy.Publisher(
            "learning_speed/mock_v_max", SpeedRequestStamped, queue_size=10
        )
        self._stale_snapshot_pub = rospy.Publisher(
            "learning_speed/stale_observation_snapshot",
            String, queue_size=10, latch=True,
        )
        self._cloud_pub = None
        if self._publish_synthetic_ego_cloud:
            self._cloud_pub = rospy.Publisher("qualification/cloud", PointCloud2, queue_size=1)

        rospy.Subscriber("Odometry", Odometry, self._odom_callback, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber("livox/lidar", PointCloud2, self._lidar_callback, queue_size=20)
        rospy.Subscriber("learning_speed/observation_v2/stamped", LidarSurrogateStamped, self._v2_callback, queue_size=20)
        rospy.Subscriber("learning_speed/observation_c", ObservationC, self._observation_callback, queue_size=50)
        rospy.Subscriber("planning/bspline", Bspline, self._trajectory_callback, queue_size=20)
        rospy.Subscriber("planning/pos_cmd", PositionCommand, self._position_command_callback, queue_size=50)
        rospy.Subscriber("planner/status", PlannerStatus, self._planner_callback, queue_size=20)
        rospy.Subscriber("position_command_to_hector/backend_state", String, self._backend_callback, queue_size=10)
        rospy.Subscriber("learning_speed/mock_v_max", SpeedRequestStamped, self._request_callback, queue_size=20)
        rospy.Subscriber("learning_speed/action_stamped", SpeedActionStamped, self._action_callback, queue_size=20)
        rospy.Subscriber("learning_speed/applied_v_max_stamped", SpeedAppliedStamped, self._applied_callback, queue_size=20)
        rospy.Subscriber(
            "learning_speed/sac_episode_closed", String,
            self._sac_closure_callback, queue_size=10,
        )
        rospy.Subscriber(
            "/uav1/learning_speed/forest_map_state", String,
            self._forest_map_state_callback, queue_size=10,
        )
        rospy.Subscriber("learning_speed/observation_v2/diagnostics", DiagnosticArray, self._v2_diagnostic_callback, queue_size=10)
        rospy.Subscriber("learning_speed/observation_c/diagnostics", DiagnosticArray, self._c_diagnostic_callback, queue_size=10)
        if self._require_registered_ego_cloud:
            rospy.Subscriber(
                "training/cloud_registration/state", String,
                self._registered_cloud_state_callback, queue_size=10,
            )

        self._switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self._adapter_hold = rospy.ServiceProxy("position_command_to_hector/hold", Trigger)
        self._adapter_prepare = rospy.ServiceProxy("position_command_to_hector/prepare_reset", Trigger)
        self._adapter_resume = rospy.ServiceProxy("position_command_to_hector/resume_reset_hover", Trigger)
        self._adapter_engage = rospy.ServiceProxy("position_command_to_hector/engage", Trigger)
        self._adapter_activate = rospy.ServiceProxy("position_command_to_hector/activate_trajectory", Trigger)
        self._clear_c = rospy.ServiceProxy("observation_c/clear_temporal_history", Trigger)
        self._clear_v2 = rospy.ServiceProxy("observation_v2/clear_temporal_history", Trigger)
        self._clear_environment_map = rospy.ServiceProxy(
            "planning/clear_environment_map", Trigger
        )
        self._clear_truth_cloud = (
            rospy.ServiceProxy(self._truth_cloud_clear_service, Trigger)
            if self._truth_cloud_clear_service else None
        )

        self._cloud_timer = (
            rospy.Timer(rospy.Duration(0.1), self._cloud_timer_callback)
            if self._publish_synthetic_ego_cloud else None
        )
        self._overall_wall_start = time.monotonic()
        self._overall_sim_start = rospy.Time.now().to_sec()
        self._publish_identity()

    def _next_sequence_locked(self):
        self._callback_sequence += 1
        return self._callback_sequence

    def _event(self, name, **fields):
        event = {
            "event": name,
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "sim_time": rospy.Time.now().to_sec(),
            "episode_id": self._binding.episode_id,
            "episode_key": self._binding.episode_key,
            "reset_generation": self._binding.reset_generation,
            "coordinator_state": self._state,
        }
        event.update(fields)
        with self._lock:
            self._events.append(event)
            self._event_file.write(json.dumps(event, sort_keys=True) + "\n")
            self._event_file.flush()
        rospy.logwarn("[TRAINING EPISODE] %s %s", name, fields)

    def _publish_identity(self):
        payload = {
            "version": VERSION,
            "episode_id": self._binding.episode_id,
            "episode_key": self._binding.episode_key,
            "reset_generation": self._binding.reset_generation,
            "state": self._state,
            "terminal_latched": self._terminal_latched,
            "terminal_outcome": self._terminal_outcome,
            "terminal_reason": self._terminal_reason,
            "action_request_id": self._binding.episode_id,
            "start_reset": dict(self._current_episode_start),
            "trajectory_id": self._accepted_trajectory_id,
            "trajectory_start_time": self._accepted_trajectory_start,
            "trajectory_start_time_secs": self._accepted_trajectory_start_secs,
            "trajectory_start_time_nsecs": self._accepted_trajectory_start_nsecs,
            "reset_barrier_stamp": self._reset_barrier,
        }
        self._identity_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def _diagnostic_values(message):
        if not message.status:
            return {}
        return {item.key: item.value for item in message.status[0].values}

    def _odom_callback(self, message):
        with self._lock:
            self._odom_sequence = self._next_sequence_locked()
            self._odom = message
            self._odom_receipt_wall = time.monotonic()
            if self._active_episode:
                position = message.pose.pose.position
                self._episode_positions.append(
                    (position.x, position.y, position.z)
                )

    def _lidar_callback(self, message):
        with self._lock:
            self._lidar_sequence = self._next_sequence_locked()
            self._lidar = message
            self._lidar_receipt_wall = time.monotonic()

    def _v2_callback(self, message):
        semantic = (
            list(message.semantic)
            if not isinstance(message.semantic, (bytes, bytearray))
            else list(bytearray(message.semantic))
        )
        with self._lock:
            self._v2_sequence = self._next_sequence_locked()
            self._v2 = message
            self._v2_receipt_wall = time.monotonic()
            if self._binding.reset_generation > 0 and (
                int(message.temporal_generation) != self._binding.reset_generation
                or message.header.stamp.to_sec() <= self._reset_barrier + 1.0e-9
                or message.pose_source_stamp.to_sec() <= self._reset_barrier + 1.0e-9
            ):
                self._stale_cloud_state_count += 1
            if self._active_episode and message.valid:
                obstacle = semantic.count(2)
                free = semantic.count(1)
                unknown = semantic.count(0)
                self._surrogate_obstacle_counts.append(obstacle)
                self._surrogate_free_counts.append(free)
                self._surrogate_unknown_counts.append(unknown)
                if message.lidar_surrogate and all(
                    abs(float(value)) <= 1.0e-12
                    for value in message.lidar_surrogate
                ):
                    self._surrogate_all_zero_count += 1
                if unknown == len(semantic) and semantic:
                    self._surrogate_all_unknown_count += 1

    def _observation_callback(self, message):
        receipt = time.monotonic()
        with self._lock:
            self._observation_sequence = self._next_sequence_locked()
            self._observation = message
            self._observation_receipt_wall = receipt
            self._stale_gap_active = False
            if message.valid:
                self._last_valid_observation_wall = receipt
            if self._active_episode:
                self._episode_observation_total += 1
                if message.valid:
                    self._episode_observation_valid += 1
                    velocity = message.actual_velocity_body
                    speed = _norm3(velocity.x, velocity.y, velocity.z)
                    tracking = float(message.tracking_error_norm)
                    self._actual_speeds.append(speed)
                    self._tracking_errors.append(tracking)
                    self._episode_actual_speeds.append(speed)
                    self._episode_tracking_errors.append(tracking)
                    if self._reward_runtime_audit_enabled:
                        try:
                            self._record_reward_runtime_step_locked(message)
                        except (TypeError, ValueError) as error:
                            self._reward_runtime_invalid_count += 1
                            self._reward_runtime_audit_failure = str(error)
            # Between clear_temporal_history and ledger.advance_after_reset(),
            # current-generation+1 packets may validly arrive while no Episode
            # or replay owner is active. They are next-generation warm-up, not
            # old-generation contamination. Keep the strict gate everywhere
            # outside that coordinator-owned RESETTING window.
            if self._state != "RESETTING" and message.valid and (
                int(message.temporal_generation) != self._binding.reset_generation
                or int(message.lidar_temporal_generation) != self._binding.reset_generation
                or message.header.stamp.to_sec() <= self._reset_barrier + 1.0e-9
            ):
                self._old_generation_valid_contamination += 1
            if (
                message.valid
                and self._accepted_trajectory_id > 0
                and int(message.trajectory_id) < self._accepted_trajectory_id
            ):
                self._trajectory_generation_mismatch += 1

    def _record_reward_runtime_step_locked(self, message):
        """Audit one valid Observation C sample without creating SAC Replay."""

        if self._reward is None or not message.valid:
            return
        applied_message = None if self._applied is None else self._applied[0]
        if applied_message is None:
            raise ValueError("canonical applied v_max is unavailable")
        if not action_matches(
            self._binding,
            applied_message.episode_id,
            applied_message.step_index,
            applied_message.request_id,
        ):
            raise ValueError("canonical applied v_max identity mismatch")
        clutter = lidar_clutter_metrics(
            message.lidar_surrogate, message.lidar_semantic
        )
        actual_speed = actual_speed_mps_from_body_velocity(
            (
                message.actual_velocity_body.x,
                message.actual_velocity_body.y,
                message.actual_velocity_body.z,
            )
        )
        tracking_error = tracking_error_m_from_body_error(
            (
                message.tracking_error_body.x,
                message.tracking_error_body.y,
                message.tracking_error_body.z,
            )
        )
        reward_input = reward_input_from_signals(
            nearest_obstacle_distance_m=clutter[
                "nearest_obstacle_distance_m"
            ],
            known_obstacle_bin_fraction=clutter[
                "known_obstacle_bin_fraction"
            ],
            unknown_bin_count=clutter["unknown_bin_count"],
            lidar_bin_count=len(message.lidar_surrogate),
            applied_v_max_mps=applied_message.applied_v_max,
            previous_applied_v_max_mps=message.previous_v_max,
            actual_speed_mps=actual_speed,
            tracking_error_m=tracking_error,
            dangerous_terminal=False,
            terminated=False,
            observation_valid=True,
            same_episode=True,
            truncated=False,
        )
        evaluation = self._reward.evaluate(reward_input)
        if not evaluation.reward_valid:
            raise ValueError(
                "reward rejected runtime step: {}".format(
                    evaluation.invalid_reason
                )
            )
        planner = self._planner
        backend = dict(self._backend)
        record = {
            "schema_version": "learning_speed_reward_runtime_step_v1.0",
            "qualification_only": True,
            "formal_sac_transition": False,
            "episode_id": self._binding.episode_id,
            "episode_key": self._binding.episode_key,
            "reset_generation": self._binding.reset_generation,
            "step_index": len(self._episode_reward_steps),
            "observation_stamp": message.header.stamp.to_sec(),
            "observation_trajectory_id": int(message.trajectory_id),
            "actual_speed_mps": actual_speed,
            "applied_v_max_mps": float(applied_message.applied_v_max),
            "previous_applied_v_max_mps": float(message.previous_v_max),
            "tracking_error_norm_m": tracking_error,
            "nearest_obstacle_distance_m": clutter[
                "nearest_obstacle_distance_m"
            ],
            "known_obstacle_bin_fraction": clutter[
                "known_obstacle_bin_fraction"
            ],
            "known_obstacle_bin_count": clutter[
                "known_obstacle_bin_count"
            ],
            "unknown_bin_count": clutter["unknown_bin_count"],
            "lidar_bin_count": len(message.lidar_surrogate),
            "reward_input": {
                "dangerous_terminal": False,
                "terminated": False,
                "truncated": False,
            },
            "terminal_type": "NON_TERMINAL",
            "terminal_reason": "",
            "planner": {
                "state": "" if planner is None else planner.planner_state,
                "failure_reason": (
                    "" if planner is None else planner.failure_reason
                ),
                "current_position_in_collision": bool(
                    planner is not None
                    and planner.current_position_in_collision
                ),
                "emergency_stop_active": bool(
                    planner is not None and planner.emergency_stop_active
                ),
            },
            "collision": bool(
                planner is not None and planner.current_position_in_collision
            ),
            "controller": {
                "mode": str(backend.get("mode", "missing")),
                "ready": bool(backend.get("ready", False)),
                "controllers_running": bool(
                    backend.get("controllers_running", False)
                ),
            },
        }
        self._attach_reward_evaluation(record, evaluation)
        self._episode_reward_steps.append(record)

    @staticmethod
    def _attach_reward_evaluation(record, evaluation):
        context = evaluation.complexity_context
        record.update(
            reward=evaluation.to_record(),
            reward_branch=(None if context is None else context.label),
            branch_weights=(
                None
                if context is None
                else {
                    "safe": context.branch_weights[0],
                    "middle": context.branch_weights[1],
                    "dangerous": context.branch_weights[2],
                }
            ),
            phi_1=evaluation.phi_1,
            phi_2=evaluation.phi_2,
            r_speed=evaluation.reward_speed,
            r_smoothing=evaluation.reward_smoothing,
            r_error=evaluation.reward_error,
            r_danger=evaluation.reward_danger,
            total_reward=evaluation.reward_total,
        )

    def _finalize_reward_runtime_steps(self, outcome, reason):
        if not self._reward_runtime_audit_enabled:
            return 0
        with self._lock:
            records = list(self._episode_reward_steps)
            planner = self._planner
        if not records:
            raise RuntimeError("reward runtime audit captured no valid steps")
        dangerous_terminal = bool(
            reason == "collision"
            or (planner is not None and planner.emergency_stop_active)
        )
        terminal_record = records[-1]
        raw = terminal_record
        truncated = outcome == "TRUNCATED"
        reward_input = reward_input_from_signals(
            nearest_obstacle_distance_m=raw[
                "nearest_obstacle_distance_m"
            ],
            known_obstacle_bin_fraction=raw[
                "known_obstacle_bin_fraction"
            ],
            unknown_bin_count=raw["unknown_bin_count"],
            lidar_bin_count=raw["lidar_bin_count"],
            applied_v_max_mps=raw["applied_v_max_mps"],
            previous_applied_v_max_mps=raw[
                "previous_applied_v_max_mps"
            ],
            actual_speed_mps=raw["actual_speed_mps"],
            tracking_error_m=raw["tracking_error_norm_m"],
            dangerous_terminal=dangerous_terminal,
            terminated=not truncated,
            observation_valid=True,
            same_episode=True,
            truncated=truncated,
        )
        evaluation = self._reward.evaluate(reward_input)
        terminal_record["reward_input"] = {
            "dangerous_terminal": dangerous_terminal,
            "terminated": not truncated,
            "truncated": truncated,
        }
        terminal_record["terminal_type"] = outcome
        terminal_record["terminal_reason"] = reason
        self._attach_reward_evaluation(terminal_record, evaluation)
        for record in records:
            self._reward_step_file.write(
                json.dumps(record, sort_keys=True) + "\n"
            )
        self._reward_step_file.flush()
        self._reward_runtime_step_count += len(records)
        return len(records)

    def _trajectory_callback(self, message):
        with self._lock:
            sequence = self._next_sequence_locked()
            self._trajectory = (message, sequence, time.monotonic())

    def _position_command_callback(self, message):
        with self._lock:
            sequence = self._next_sequence_locked()
            self._position_command = (message, sequence, time.monotonic())

    def _planner_callback(self, message):
        with self._lock:
            self._next_sequence_locked()
            self._planner = message

    def _backend_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._next_sequence_locked()
            self._backend = payload

    def _request_callback(self, message):
        with self._lock:
            sequence = self._next_sequence_locked()
            self._request = (message, sequence, time.monotonic())

    def _action_callback(self, message):
        with self._lock:
            sequence = self._next_sequence_locked()
            self._action = (message, sequence, time.monotonic())

    def _applied_callback(self, message):
        with self._lock:
            sequence = self._next_sequence_locked()
            self._applied = (message, sequence, time.monotonic())

    def _sac_closure_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        key = (
            str(payload.get("episode_id", "")),
            int(payload.get("reset_generation", -1)),
        )
        with self._lock:
            if key in self._sac_closure_keys:
                self._duplicate_sac_closure_count += 1
                return
            self._sac_closure_keys.add(key)
            self._sac_closure = payload

    def _forest_map_state_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._forest_map_state = payload

    def _v2_diagnostic_callback(self, message):
        with self._lock:
            self._v2_diagnostics = self._diagnostic_values(message)

    def _c_diagnostic_callback(self, message):
        with self._lock:
            self._c_diagnostics = self._diagnostic_values(message)

    def _registered_cloud_state_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._registered_cloud_state = payload
            if payload.get("ready", False):
                count = payload.get("corridor_points")
                distance = payload.get("corridor_minimum_distance")
                if count is not None:
                    self._registered_corridor_counts.append(int(count))
                if distance is not None and math.isfinite(float(distance)):
                    self._registered_corridor_minimum_distances.append(
                        float(distance)
                    )

    def _cloud_timer_callback(self, _event):
        if self._cloud_pub is None:
            return
        stamp = rospy.Time.now()
        if stamp.is_zero():
            return
        header = Header(stamp=stamp, frame_id="world")
        points = ((-4.0, -4.0, 0.25), (-4.0, 4.0, 0.25), (4.0, -4.0, 0.25), (4.0, 4.0, 0.25))
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _wait_services(self):
        names = (
            "/controller_manager/switch_controller", "/gazebo/set_model_state",
            "/gazebo/pause_physics", "/gazebo/unpause_physics",
            "/uav1/position_command_to_hector/hold",
            "/uav1/position_command_to_hector/prepare_reset",
            "/uav1/position_command_to_hector/resume_reset_hover",
            "/uav1/position_command_to_hector/engage",
            "/uav1/position_command_to_hector/activate_trajectory",
            "/uav1/observation_c/clear_temporal_history",
            "/uav1/observation_v2/clear_temporal_history",
        )
        required = list(names)
        required.append("/uav1/planning/clear_environment_map")
        if self._truth_cloud_clear_service:
            required.append(self._truth_cloud_clear_service)
        for name in required:
            rospy.wait_for_service(name, timeout=self._startup_timeout)

    def _wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        self._event("WAIT_TIMEOUT", description=description)
        return False

    def _actual_state_locked(self):
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        v = self._odom.twist.twist.linear
        roll, pitch = _roll_pitch(self._odom.pose.pose.orientation)
        return (p.x, p.y, p.z), (v.x, v.y, v.z), roll, pitch, self._odom.header.stamp.to_sec()

    @staticmethod
    def _stamped_speed_snapshot(item, now_monotonic, kind):
        if item is None:
            return None
        message, sequence, receipt_monotonic = item
        result = {
            "kind": kind,
            "callback_sequence": int(sequence),
            "header_stamp_sec": _stamp_sec(message),
            "receipt_monotonic_sec": float(receipt_monotonic),
            "receipt_age_wall_sec": max(
                0.0, float(now_monotonic) - float(receipt_monotonic)
            ),
            "episode_id": str(message.episode_id),
            "step_index": int(message.step_index),
            "request_id": int(message.request_id),
            "version": str(message.version),
        }
        if hasattr(message, "requested_v_max"):
            result["requested_v_max"] = float(message.requested_v_max)
        if hasattr(message, "filtered_v_max"):
            result["filtered_v_max"] = float(message.filtered_v_max)
        if hasattr(message, "applied_v_max"):
            result["applied_v_max"] = float(message.applied_v_max)
        return result

    def _capture_stale_snapshot_locked(
        self, now_sim, now_monotonic, now_wall_epoch, start_sim
    ):
        observation = self._observation
        lidar = self._lidar
        v2 = self._v2
        trajectory_item = self._trajectory
        trajectory = None if trajectory_item is None else trajectory_item[0]
        trajectory_sequence = (
            None if trajectory_item is None else int(trajectory_item[1])
        )
        trajectory_receipt = (
            None if trajectory_item is None else float(trajectory_item[2])
        )
        usage = resource.getrusage(resource.RUSAGE_SELF)
        try:
            load_average = list(os.getloadavg())
        except OSError:
            load_average = None
        observation_receipt_age = (
            None
            if self._observation_receipt_wall <= 0.0
            else max(
                0.0,
                float(now_monotonic) - self._observation_receipt_wall,
            )
        )
        snapshot_id = "{}:g{}:stale:{}".format(
            self._binding.episode_key,
            self._binding.reset_generation,
            len(self._stale_snapshots) + 1,
        )
        snapshot = {
            "schema_version": "astradrone_stale_observation_snapshot_v1.0",
            "event": "STALE_CANDIDATE",
            "snapshot_id": snapshot_id,
            "episode_id": int(self._binding.episode_id),
            "episode_key": str(self._binding.episode_key),
            "reset_generation": int(self._binding.reset_generation),
            "coordinator_state": str(self._state),
            "terminal_reason_candidate": "invalid_observation:stale",
            "stale_threshold_wall_sec": float(self._observation_freshness),
            "candidate_sim_time_sec": float(now_sim),
            "candidate_wall_time_sec": float(now_wall_epoch),
            "candidate_monotonic_time_sec": float(now_monotonic),
            "episode_elapsed_sim_sec": float(now_sim) - float(start_sim),
            "observation": {
                "present": observation is not None,
                "valid": None if observation is None else bool(observation.valid),
                "diagnostics": (
                    [] if observation is None else list(observation.diagnostics)
                ),
                "header_stamp_sec": _stamp_sec(observation),
                "lookup_receipt_stamp_sec": (
                    None
                    if observation is None
                    else float(observation.lookup_receipt_time.to_sec())
                ),
                "coordinator_receipt_monotonic_sec": (
                    None
                    if self._observation_receipt_wall <= 0.0
                    else float(self._observation_receipt_wall)
                ),
                "coordinator_receipt_age_wall_sec": observation_receipt_age,
                "source_age_sim_sec": (
                    None
                    if observation is None
                    else float(now_sim) - float(observation.header.stamp.to_sec())
                ),
                "source_to_receipt_latency_sec": (
                    None
                    if observation is None
                    else float(observation.source_to_receipt_latency_sec)
                ),
                "temporal_generation": (
                    None
                    if observation is None
                    else int(observation.temporal_generation)
                ),
                "lidar_temporal_generation": (
                    None
                    if observation is None
                    else int(observation.lidar_temporal_generation)
                ),
                "lidar_pose_source_stamp_sec": (
                    None
                    if observation is None
                    else float(observation.lidar_pose_source_stamp.to_sec())
                ),
                "selected_trajectory_id": (
                    None
                    if observation is None
                    else int(observation.selected_trajectory_id)
                ),
                "selected_trajectory_missing": (
                    None
                    if observation is None
                    else bool(observation.selected_trajectory_missing)
                ),
                "selected_trajectory_start_stamp_sec": (
                    None
                    if observation is None
                    else float(observation.selected_trajectory_start_stamp.to_sec())
                ),
                "selected_trajectory_end_stamp_sec": (
                    None
                    if observation is None
                    else float(observation.selected_trajectory_end_stamp.to_sec())
                ),
                "latest_trajectory_id_at_lookup": (
                    None
                    if observation is None
                    else int(observation.latest_trajectory_id_at_lookup)
                ),
                "latest_trajectory_start_stamp_sec": (
                    None
                    if observation is None
                    else float(observation.latest_trajectory_start_stamp.to_sec())
                ),
                "callback_sequence": int(self._observation_sequence),
            },
            "raw_lidar": {
                "present": lidar is not None,
                "header_stamp_sec": _stamp_sec(lidar),
                "source_age_sim_sec": (
                    None
                    if lidar is None
                    else float(now_sim) - float(lidar.header.stamp.to_sec())
                ),
                "coordinator_receipt_monotonic_sec": (
                    None
                    if self._lidar_receipt_wall <= 0.0
                    else float(self._lidar_receipt_wall)
                ),
                "receipt_age_wall_sec": (
                    None
                    if self._lidar_receipt_wall <= 0.0
                    else max(
                        0.0,
                        float(now_monotonic) - self._lidar_receipt_wall,
                    )
                ),
                "callback_sequence": int(self._lidar_sequence),
            },
            "observation_v2": {
                "present": v2 is not None,
                "valid": None if v2 is None else bool(v2.valid),
                "diagnostics": [] if v2 is None else list(v2.diagnostics),
                "header_stamp_sec": _stamp_sec(v2),
                "pose_source_stamp_sec": (
                    None if v2 is None else float(v2.pose_source_stamp.to_sec())
                ),
                "temporal_generation": (
                    None if v2 is None else int(v2.temporal_generation)
                ),
                "build_duration_ms": (
                    None if v2 is None else float(v2.build_duration_ms)
                ),
                "coordinator_receipt_monotonic_sec": (
                    None
                    if self._v2_receipt_wall <= 0.0
                    else float(self._v2_receipt_wall)
                ),
                "receipt_age_wall_sec": (
                    None
                    if self._v2_receipt_wall <= 0.0
                    else max(
                        0.0,
                        float(now_monotonic) - self._v2_receipt_wall,
                    )
                ),
                "callback_sequence": int(self._v2_sequence),
            },
            "current_bspline": {
                **_bspline_timing(trajectory, now_sim),
                "callback_sequence": trajectory_sequence,
                "receipt_monotonic_sec": trajectory_receipt,
                "receipt_age_wall_sec": (
                    None
                    if trajectory_receipt is None
                    else max(0.0, float(now_monotonic) - trajectory_receipt)
                ),
            },
            "latest_request": self._stamped_speed_snapshot(
                self._request, now_monotonic, "request"
            ),
            "latest_action": self._stamped_speed_snapshot(
                self._action, now_monotonic, "action"
            ),
            "latest_applied": self._stamped_speed_snapshot(
                self._applied, now_monotonic, "applied"
            ),
            "callback_evidence": {
                "global_callback_sequence": int(self._callback_sequence),
                "odom_sequence": int(self._odom_sequence),
                "raw_lidar_sequence": int(self._lidar_sequence),
                "observation_v2_sequence": int(self._v2_sequence),
                "observation_c_sequence": int(self._observation_sequence),
                "callbacks_since_last_observation_c": int(
                    self._callback_sequence - self._observation_sequence
                ),
                "subscriber_queue_sizes": {
                    "raw_lidar": 20,
                    "observation_v2": 20,
                    "observation_c": 50,
                    "trajectory": 20,
                    "request_action_applied": 20,
                },
                "queue_depth_directly_available": False,
                "observation_c_diagnostics": dict(self._c_diagnostics),
                "observation_v2_diagnostics": dict(self._v2_diagnostics),
            },
            "process_load": {
                "load_average_1_5_15": load_average,
                "coordinator_user_cpu_sec": float(usage.ru_utime),
                "coordinator_system_cpu_sec": float(usage.ru_stime),
                "coordinator_max_rss_mib": float(usage.ru_maxrss) / 1024.0,
            },
        }
        self._stale_snapshots.append(snapshot)
        self._active_stale_snapshot_id = snapshot_id
        self._stale_gap_active = True
        self._stale_snapshot_file.write(json.dumps(snapshot, sort_keys=True) + "\n")
        self._stale_snapshot_file.flush()
        return snapshot

    def _record_stale_terminal_latch(self, snapshot_id, terminal_reason):
        with self._lock:
            record = {
                "schema_version": "astradrone_stale_observation_snapshot_v1.0",
                "event": "STALE_TERMINAL_LATCH",
                "snapshot_id": str(snapshot_id),
                "episode_id": int(self._binding.episode_id),
                "episode_key": str(self._binding.episode_key),
                "reset_generation": int(self._binding.reset_generation),
                "terminal_reason": str(terminal_reason),
                "terminal_sim_time_sec": float(rospy.Time.now().to_sec()),
                "terminal_wall_time_sec": float(time.time()),
                "terminal_monotonic_time_sec": float(time.monotonic()),
                "callback_sequence": int(self._callback_sequence),
            }
            self._stale_snapshot_file.write(json.dumps(record, sort_keys=True) + "\n")
            self._stale_snapshot_file.flush()
        self._stale_snapshot_pub.publish(
            String(data=json.dumps(record, sort_keys=True))
        )
        return record

    def _classify_observation_staleness_locked(self, now_monotonic):
        observation_age = (
            math.inf
            if self._observation_receipt_wall <= 0.0
            else max(0.0, now_monotonic - self._observation_receipt_wall)
        )
        lidar_age = (
            math.inf
            if self._lidar_receipt_wall <= 0.0
            else max(0.0, now_monotonic - self._lidar_receipt_wall)
        )
        v2_age = (
            math.inf
            if self._v2_receipt_wall <= 0.0
            else max(0.0, now_monotonic - self._v2_receipt_wall)
        )
        return classify_observation_staleness(
            observation_present=self._observation is not None,
            observation_receipt_age_sec=observation_age,
            raw_lidar_present=self._lidar is not None,
            raw_lidar_receipt_age_sec=lidar_age,
            observation_v2_present=self._v2 is not None,
            observation_v2_valid=(
                self._v2 is not None and bool(self._v2.valid)
            ),
            observation_v2_receipt_age_sec=v2_age,
            observation_freshness_sec=self._observation_freshness,
            raw_lidar_freshness_sec=self._lidar_freshness,
        )

    def _commit_stale_terminal_if_current(self):
        """Atomically revalidate freshness and linearize a stale terminal."""

        with self._lock:
            now_monotonic = time.monotonic()
            reason = self._classify_observation_staleness_locked(now_monotonic)
            if reason is None:
                record = {
                    "schema_version": "astradrone_stale_observation_snapshot_v1.0",
                    "event": "STALE_CANDIDATE_RECOVERED",
                    "snapshot_id": str(
                        self._active_stale_snapshot_id
                        or "missing_stale_candidate"
                    ),
                    "episode_id": int(self._binding.episode_id),
                    "episode_key": str(self._binding.episode_key),
                    "reset_generation": int(self._binding.reset_generation),
                    "recovery_monotonic_time_sec": float(now_monotonic),
                    "observation_callback_sequence": int(
                        self._observation_sequence
                    ),
                    "global_callback_sequence": int(self._callback_sequence),
                }
                self._stale_snapshot_file.write(
                    json.dumps(record, sort_keys=True) + "\n"
                )
                self._stale_snapshot_file.flush()
                self._stale_gap_active = False
                self._active_stale_snapshot_id = None
                return None
            # The observation callback uses the same lock.  Once these fields
            # are set, a later callback is causally after the terminal latch.
            self._terminal_latched = True
            self._terminal_outcome = "FAILURE"
            self._terminal_reason = reason
            return "FAILURE", reason

    def _physical_ready(self):
        now = time.monotonic()
        with self._lock:
            state = self._actual_state_locked()
            backend = dict(self._backend)
            lidar = self._lidar
            lidar_wall = self._lidar_receipt_wall
            odom_wall = self._odom_receipt_wall
            registered = dict(self._registered_cloud_state)
        registered_ready = True
        if self._require_registered_ego_cloud:
            registered_ready = bool(
                registered.get("ready", False)
                and int(registered.get("generation", -1))
                == self._binding.reset_generation
                and float(registered.get("source_stamp", 0.0))
                > self._reset_barrier + 1.0e-9
                and float(registered.get("source_age", math.inf))
                <= self._lidar_freshness
            )
        return bool(
            state is not None and lidar is not None
            and now - odom_wall <= self._truth_freshness
            and now - lidar_wall <= self._lidar_freshness
            and backend.get("ready", False)
            and backend.get("controllers_running", False)
            and backend.get("coordinator_managed_goals", False)
            and not backend.get("trajectory_gate_armed", True)
            and registered_ready
        )

    def _forest_ready(self):
        if not self._require_forest_map_ready:
            return True
        with self._lock:
            state = dict(self._forest_map_state)
        return bool(
            state.get("status") == "ready"
            and state.get("map_ready", False)
            and state.get("passed", False)
            and int(state.get("loaded_obstacle_count", -1)) == 18
            and int(state.get("residual_obstacle_count", -1)) == 0
            and state.get("old_obstacles_absent_verified", False)
        )

    def _v2_ready(self):
        with self._lock:
            message = self._v2
        return bool(
            message is not None and message.valid
            and int(message.temporal_generation) == self._binding.reset_generation
            and int(message.history_frames) >= 5
            and message.header.stamp.to_sec() > self._reset_barrier + 1.0e-9
            and message.pose_source_stamp.to_sec() > self._reset_barrier + 1.0e-9
        )

    def _publish_fixed_action(self):
        message = SpeedRequestStamped()
        message.header.stamp = rospy.Time.now()
        message.version = "learning_speed_request_v1.0"
        message.episode_id = self._binding.episode_key
        message.step_index = 0
        message.request_id = self._binding.episode_id
        message.requested_v_max = self._fixed_v_max
        with self._lock:
            self._action = None
            self._applied = None
        self._request_pub.publish(message)
        self._event("FIXED_ACTION_REQUEST", request_id=message.request_id, v_max=self._fixed_v_max)

        def acknowledged():
            with self._lock:
                action = None if self._action is None else self._action[0]
                applied = None if self._applied is None else self._applied[0]
            return bool(
                action is not None and applied is not None
                and action_matches(self._binding, action.episode_id, action.step_index, action.request_id)
                and action_matches(self._binding, applied.episode_id, applied.step_index, applied.request_id)
                and abs(action.filtered_v_max - self._fixed_v_max) <= 1.0e-9
                and abs(applied.applied_v_max - self._fixed_v_max) <= 0.005
                and applied.header.stamp > action.header.stamp
            )

        if not self._wait(acknowledged, self._observation_timeout, "fixed action/applied identity"):
            raise RuntimeError("fixed action identity acknowledgement timeout")

    def _publish_sac_warmup_bootstrap(self):
        """Seed post-clear scalar state without using the formal SAC identity."""

        bootstrap_episode = "__sac_warmup_generation_{:06d}__".format(
            self._binding.reset_generation
        )
        message = SpeedRequestStamped()
        message.header.stamp = rospy.Time.now()
        message.version = "learning_speed_request_v1.0"
        message.episode_id = bootstrap_episode
        message.step_index = 0
        message.request_id = 1
        message.requested_v_max = self._fixed_v_max
        with self._lock:
            self._action = None
            self._applied = None
        self._request_pub.publish(message)
        self._event(
            "SAC_WARMUP_BOOTSTRAP_REQUEST",
            bootstrap_episode=bootstrap_episode,
            v_max=self._fixed_v_max,
        )

        def acknowledged():
            with self._lock:
                action = None if self._action is None else self._action[0]
                applied = None if self._applied is None else self._applied[0]
            return bool(
                action is not None and applied is not None
                and action.episode_id == bootstrap_episode
                and applied.episode_id == bootstrap_episode
                and int(action.step_index) == int(applied.step_index) == 0
                and int(action.request_id) == int(applied.request_id) == 1
                and abs(action.filtered_v_max - self._fixed_v_max) <= 1.0e-9
                and abs(applied.applied_v_max - self._fixed_v_max) <= 0.005
                and applied.header.stamp > action.header.stamp
            )

        if not self._wait(
            acknowledged,
            self._observation_timeout,
            "SAC warmup bootstrap action/applied identity",
        ):
            raise RuntimeError("SAC warmup bootstrap acknowledgement timeout")

    def _publish_entry_goal(self):
        if not self._wait(lambda: self._goal_pub.get_num_connections() > 0, self._startup_timeout, "EGO goal subscriber"):
            raise RuntimeError("EGO goal subscriber unavailable")
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self._entry
        goal.pose.orientation.w = 1.0
        with self._lock:
            self._goal_sequence = self._callback_sequence
            self._goal_stamp = goal.header.stamp.to_sec()
            self._trajectory = None
        self._goal_pub.publish(goal)
        self._event("ENTRY_GOAL_ACTIVE", x=self._entry[0], y=self._entry[1], z=self._entry[2])

    def _fresh_trajectory_ready(self):
        with self._lock:
            item = self._trajectory
            previous = self._previous_trajectory_id
            goal_sequence = self._goal_sequence
            goal_stamp = self._goal_stamp
        if item is None:
            return False
        message, sequence, _ = item
        return trajectory_matches(
            self._binding, message.traj_id, message.start_time.to_sec(),
            sequence, goal_sequence, goal_stamp, self._reset_barrier, previous,
        )

    def _current_observation_ready(self):
        with self._lock:
            message = self._observation
            accepted = self._accepted_trajectory_id
        return bool(
            message is not None and accepted > 0
            and observation_matches(
                self._binding, message.valid, message.temporal_generation,
                message.lidar_temporal_generation, message.header.stamp.to_sec(),
                self._reset_barrier, message.trajectory_id, accepted,
            )
        )

    def _prepare_episode(self, reset_result=None):
        self._state = "OBSERVATION_WARMUP"
        self._publish_identity()
        if not self._wait(
            self._forest_ready, self._forest_map_switch_timeout,
            "qualified Forest map ready",
        ):
            raise RuntimeError("Forest map readiness timeout")
        if not self._wait(self._physical_ready, self._startup_timeout, "truth/Mid360/EGO/Hector readiness"):
            raise RuntimeError("physical readiness timeout")
        if not self._wait(self._v2_ready, self._observation_timeout, "five-frame Observation v2 warmup"):
            raise RuntimeError("Observation v2 warmup timeout")
        if self._action_owner == "fixed":
            self._publish_fixed_action()
        else:
            self._publish_sac_warmup_bootstrap()
        self._publish_entry_goal()
        if not self._wait(self._fresh_trajectory_ready, self._trajectory_timeout, "fresh EGO B-spline"):
            raise RuntimeError("planner failure: fresh B-spline timeout")
        with self._lock:
            message = self._trajectory[0]
            self._accepted_trajectory_id = int(message.traj_id)
            self._accepted_trajectory_start = message.start_time.to_sec()
            self._accepted_trajectory_start_secs = int(message.start_time.secs)
            self._accepted_trajectory_start_nsecs = int(message.start_time.nsecs)
            self._previous_trajectory_id = max(self._previous_trajectory_id, self._accepted_trajectory_id)
        if not self._wait(self._current_observation_ready, self._observation_timeout, "current-generation Observation C"):
            raise RuntimeError("invalid observation: current-generation Observation C timeout")
        first_c_wall = time.monotonic()
        if reset_result is not None:
            reset_result["first_valid_observation_c_after_reset_wall_sec"] = first_c_wall - reset_result["reset_begin_monotonic"]
            reset_result["first_valid_observation_c_generation"] = self._binding.reset_generation
            reset_result["first_valid_observation_c_trajectory_id"] = self._accepted_trajectory_id
            self._event("RESET_READY", next_episode_id=self._binding.episode_id)
        self._publish_identity()

    def _wait_for_sac_closure(self):
        if self._action_owner != "sac":
            return None

        def closed():
            with self._lock:
                payload = self._sac_closure
            return sac_closure_matches(self._binding, payload)

        if not self._wait(
            closed,
            self._sac_closure_timeout,
            "SAC terminal transition closure",
        ):
            raise RuntimeError("SAC terminal transition closure timeout")
        with self._lock:
            payload = dict(self._sac_closure)
        self._state = "CLOSED"
        self._publish_identity()
        self._event(
            "SAC_EPISODE_CLOSED",
            transition_count=payload["transition_count"],
            last_step_index=payload["last_step_index"],
            last_request_id=payload["last_request_id"],
            active_action_interval_count=payload[
                "active_action_interval_count"
            ],
            terminal_row_count=payload["terminal_row_count"],
        )
        return payload

    def _activate_episode(self):
        response = self._adapter_activate()
        if not response.success:
            raise RuntimeError("controller activation failed: " + response.message)
        if not self._wait(
            lambda: bool(self._backend.get("mode") == "TRACK" and self._backend.get("ready", False)),
            self._trajectory_timeout, "Hector TRACK after episode activation",
        ):
            raise RuntimeError("controller failure: TRACK activation timeout")

    def _terminal_condition(self, start_sim, start_wall):
        now_sim = rospy.Time.now().to_sec()
        now_wall = time.monotonic()
        stale_snapshot = None
        with self._lock:
            state = self._actual_state_locked()
            planner = self._planner
            backend = dict(self._backend)
            observation = self._observation
            observation_wall = self._observation_receipt_wall
            last_valid_wall = self._last_valid_observation_wall
            trajectory = self._trajectory
            position_command = self._position_command
            reward_audit_failure = self._reward_runtime_audit_failure
            stale_candidate = bool(
                observation is None
                or now_wall - observation_wall > self._observation_freshness
            )
            if stale_candidate and not self._stale_gap_active:
                stale_snapshot = self._capture_stale_snapshot_locked(
                    now_sim, now_wall, time.time(), start_sim
                )
        if stale_snapshot is not None:
            self._stale_snapshot_pub.publish(
                String(data=json.dumps(stale_snapshot, sort_keys=True))
            )
        if reward_audit_failure:
            return "FAILURE", "reward_runtime_audit:" + reward_audit_failure
        if state is None:
            return "FAILURE", "truth_odometry_unavailable"
        position, velocity, _, _, _ = state
        speed = _norm3(*velocity)
        if position[2] < self._collision_min_z or (planner is not None and planner.current_position_in_collision):
            self._collision_count += 1
            return "FAILURE", "collision"
        if planner is not None and (planner.emergency_stop_active or planner.failure_reason in PLANNER_FAILURES):
            self._planner_failure_count += 1
            return "FAILURE", "planner_failure:" + (planner.failure_reason or "emergency_stop")
        if not backend.get("controllers_running", False) or backend.get("mode") in ("STALE_HOLD", "DISABLED", "RESET_PAUSED"):
            self._controller_failure_count += 1
            return "FAILURE", "controller_failure:" + str(backend.get("mode", "missing"))
        distance = _norm3(position[0] - self._entry[0], position[1] - self._entry[1], position[2] - self._entry[2])
        if observation is None or now_wall - observation_wall > self._observation_freshness:
            stale_terminal = self._commit_stale_terminal_if_current()
            if stale_terminal is not None:
                return stale_terminal
            # A fresh callback won the stale-candidate race.  Refresh every
            # observation-dependent value before evaluating later predicates.
            with self._lock:
                observation = self._observation
                observation_wall = self._observation_receipt_wall
                last_valid_wall = self._last_valid_observation_wall
                trajectory = self._trajectory
                position_command = self._position_command
            now_wall = time.monotonic()
        fixed_terminal_hold = False
        if (
            self._action_owner == "fixed"
            and observation is not None
            and trajectory is not None
            and position_command is not None
        ):
            trajectory_message = trajectory[0]
            command_message, _, command_receipt_wall = position_command
            trajectory_end = self._trajectory_end_sim_sec(
                trajectory_message
            )
            command_position = command_message.position
            command_velocity = command_message.velocity
            fixed_terminal_hold = fixed_terminal_hold_matches(
                observation_valid=observation.valid,
                observation_diagnostics=observation.diagnostics,
                trajectory_lookup_result=observation.trajectory_lookup_result,
                observation_stamp_sec=observation.header.stamp.to_sec(),
                observation_latest_trajectory_id=(
                    observation.latest_trajectory_id_at_lookup
                ),
                trajectory_id=trajectory_message.traj_id,
                trajectory_end_sec=trajectory_end,
                position_command_trajectory_id=(
                    command_message.trajectory_id
                ),
                position_command_flag=command_message.trajectory_flag,
                position_command_age_sec=now_wall - command_receipt_wall,
                position_command_distance_to_goal=_norm3(
                    command_position.x - self._entry[0],
                    command_position.y - self._entry[1],
                    command_position.z - self._entry[2],
                ),
                position_command_speed=_norm3(
                    command_velocity.x,
                    command_velocity.y,
                    command_velocity.z,
                ),
                actual_distance_to_goal=distance,
                goal_position_tolerance=self._goal_position_tolerance,
                goal_speed_tolerance=self._goal_speed_tolerance,
                freshness_limit_sec=self._observation_freshness,
                ready_flag=PositionCommand.TRAJECTORY_STATUS_READY,
            )
            if fixed_terminal_hold and not self._fixed_terminal_hold_active:
                self._fixed_terminal_hold_active = True
                self._event(
                    "FIXED_TERMINAL_HOLD",
                    trajectory_id=int(trajectory_message.traj_id),
                    trajectory_end_sim=trajectory_end,
                    observation_stamp_sim=observation.header.stamp.to_sec(),
                    distance_to_entry_m=distance,
                    actual_speed_mps=speed,
                )
        if (
            not observation.valid
            and now_wall - last_valid_wall > self._invalid_observation_grace
            and not fixed_terminal_hold
        ):
            return "FAILURE", "invalid_observation:continuous"
        if distance <= self._goal_position_tolerance and speed <= self._goal_speed_tolerance:
            if self._goal_ready_since_sim is None:
                self._goal_ready_since_sim = now_sim
            elif now_sim - self._goal_ready_since_sim >= self._goal_sustain_time:
                return "SUCCESS", "entry_gate_reached"
        else:
            self._goal_ready_since_sim = None
        if now_sim - start_sim >= self._max_episode_time:
            return "TRUNCATED", "max_episode_time"
        if now_wall - start_wall >= max(self._max_episode_time * 5.0, 30.0):
            return "FAILURE", "simulation_time_stalled"
        return None

    def _run_episode(self):
        self._state = "EPISODE_ACTIVE"
        self._terminal_latched = False
        self._terminal_outcome = ""
        self._terminal_reason = ""
        self._goal_ready_since_sim = None
        self._fixed_terminal_hold_active = False
        self._episode_observation_total = 0
        self._episode_observation_valid = 0
        self._episode_positions = []
        self._episode_tracking_errors = []
        self._episode_actual_speeds = []
        self._episode_reward_steps = []
        self._active_stale_snapshot_id = None
        self._stale_gap_active = False
        self._active_episode = True
        start_sim = rospy.Time.now().to_sec()
        start_wall = time.monotonic()
        self._publish_identity()
        self._event("EPISODE_START", id=self._binding.episode_id, trajectory_id=self._accepted_trajectory_id)
        terminal = None
        while not rospy.is_shutdown() and terminal is None:
            terminal = self._terminal_condition(start_sim, start_wall)
            time.sleep(0.02)
        self._active_episode = False
        if terminal is None:
            raise RuntimeError("ROS shutdown during active episode")
        outcome, reason = terminal
        reward_runtime_steps = self._finalize_reward_runtime_steps(
            outcome, reason
        )
        self._state = "TERMINAL_LATCHED"
        with self._lock:
            self._terminal_latched = True
            self._terminal_outcome = outcome
            self._terminal_reason = reason
        self._publish_identity()
        if reason in (
            "invalid_observation:source_stale",
            "infrastructure:observation_c_producer_stall",
        ):
            self._record_stale_terminal_latch(
                self._active_stale_snapshot_id or "missing_stale_candidate",
                reason,
            )
        self._event("EPISODE_" + outcome, reason=reason)
        with self._lock:
            state = self._actual_state_locked()
            positions = list(self._episode_positions)
            tracking_errors = list(self._episode_tracking_errors)
            actual_speeds = list(self._episode_actual_speeds)
            forest_map_state = dict(self._forest_map_state)
        path_length = sum(
            _norm3(
                positions[index][0] - positions[index - 1][0],
                positions[index][1] - positions[index - 1][1],
                positions[index][2] - positions[index - 1][2],
            )
            for index in range(1, len(positions))
        )
        start_xy = self._hover[:2]
        goal_xy = self._entry[:2]
        line_dx = goal_xy[0] - start_xy[0]
        line_dy = goal_xy[1] - start_xy[1]
        line_norm = math.hypot(line_dx, line_dy)
        deviations = (
            [
                abs(line_dx * (point[1] - start_xy[1]) - line_dy * (point[0] - start_xy[0])) / line_norm
                for point in positions
            ]
            if line_norm > 1.0e-9 else []
        )
        episode = {
            "episode_id": self._binding.episode_id,
            "episode_key": self._binding.episode_key,
            "reset_generation": self._binding.reset_generation,
            "outcome": outcome,
            "reason": reason,
            "start_sim_time": start_sim,
            "end_sim_time": rospy.Time.now().to_sec(),
            "sim_duration": rospy.Time.now().to_sec() - start_sim,
            "wall_duration": time.monotonic() - start_wall,
            "trajectory_id": self._accepted_trajectory_id,
            "trajectory_start_time": self._accepted_trajectory_start,
            "trajectory_start_time_secs": self._accepted_trajectory_start_secs,
            "trajectory_start_time_nsecs": self._accepted_trajectory_start_nsecs,
            "action_request_id": self._binding.episode_id,
            "start_reset": dict(self._current_episode_start),
            "observation_c_total": self._episode_observation_total,
            "observation_c_valid": self._episode_observation_valid,
            "observation_c_valid_rate": self._episode_observation_valid / float(max(1, self._episode_observation_total)),
            "terminal_position": None if state is None else list(state[0]),
            "terminal_speed": None if state is None else _norm3(*state[1]),
            "path_sample_count": len(positions),
            "actual_path_length": path_length,
            "direct_horizontal_distance": line_norm,
            "maximum_lateral_deviation_from_direct": max(deviations) if deviations else None,
            "tracking_error": _statistics(tracking_errors),
            "actual_speed": _statistics(actual_speeds),
            "fixed_terminal_hold_used": self._fixed_terminal_hold_active,
            "reward_runtime_step_count": reward_runtime_steps,
            "forest_seed": forest_map_state.get("logical_seed"),
            "raw_seed": forest_map_state.get("raw_seed"),
            "map_block_id": forest_map_state.get("map_block_id"),
            "map_round_id": forest_map_state.get("map_round_id"),
            "scheduler_order": forest_map_state.get("scheduler_order"),
            "forest_layout_sha256": forest_map_state.get("layout_sha256"),
        }
        self._episodes.append(episode)
        return episode

    @staticmethod
    def _parse_generation(response):
        values = {}
        for item in str(response.message).split(";"):
            if "=" in item:
                key, value = item.split("=", 1)
                values[key.strip()] = value.strip()
        return int(values["generation"]), float(values["barrier"])

    def _switch(self, start, stop):
        request = SwitchControllerRequest()
        request.start_controllers = list(start)
        request.stop_controllers = list(stop)
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = True
        request.timeout = 2.0
        response = self._switch_controller(request)
        if not response.ok:
            raise RuntimeError("controller switch failed start={} stop={}".format(start, stop))

    def _teleport(self):
        x, y, z, yaw = self._hover
        request = SetModelStateRequest()
        state = ModelState()
        state.model_name = "hector_uav1"
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.orientation.w = math.cos(0.5 * yaw)
        state.twist.linear.x = state.twist.linear.y = state.twist.linear.z = 0.0
        state.twist.angular.x = state.twist.angular.y = state.twist.angular.z = 0.0
        request.model_state = state
        response = self._set_model_state(request)
        if not response.success:
            raise RuntimeError("set_model_state failed: " + response.status_message)

    def _publish_reset_hover_target(self):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.pose.position.x = self._hover[0]
        message.pose.position.y = self._hover[1]
        message.pose.position.z = self._hover[2]
        message.pose.orientation.z = math.sin(0.5 * self._hover[3])
        message.pose.orientation.w = math.cos(0.5 * self._hover[3])
        self._reset_hover_pub.publish(message)

    def _adapter_has_reset_hover_target(self):
        with self._lock:
            backend = dict(self._backend)
        values = backend.get("configured_reset_hover")
        if not isinstance(values, list) or len(values) != 4:
            return False
        try:
            return bool(
                backend.get("mode") == "RESET_PAUSED"
                and all(
                    abs(float(actual) - expected) <= 1.0e-9
                    for actual, expected in zip(values, self._hover)
                )
            )
        except (TypeError, ValueError):
            return False

    def _low_speed(self):
        with self._lock:
            state = self._actual_state_locked()
        return state is not None and _norm3(*state[1]) <= self._goal_speed_tolerance

    def _switch_forest_map(self, request):
        if not self._require_forest_map_ready:
            raise RuntimeError("Forest switch requested outside Forest profile")
        request_id = str(request.get("request_id", ""))
        if not request_id:
            raise RuntimeError("Forest map request has no request_id")
        with self._lock:
            self._forest_map_state = {}
        self._event("FOREST_MAP_SWITCH_BEGIN", forest_request=request)
        self._forest_map_request_pub.publish(
            String(data=json.dumps(request, sort_keys=True))
        )

        def matching_ready():
            with self._lock:
                state = dict(self._forest_map_state)
            if state.get("request_id") != request_id:
                return False
            if state.get("status") == "failed":
                raise RuntimeError(
                    "Forest map manager failed: " + str(state.get("failure", ""))
                )
            return bool(
                state.get("status") == "ready"
                and state.get("map_ready", False)
                and state.get("passed", False)
                and int(state.get("loaded_obstacle_count", -1)) == 18
                and int(state.get("residual_obstacle_count", -1)) == 0
                and state.get("old_obstacles_absent_verified", False)
            )

        if not self._wait(
            matching_ready, self._forest_map_switch_timeout,
            "matching Forest map switch ready",
        ):
            raise RuntimeError("Forest map switch readiness timeout")
        with self._lock:
            state = dict(self._forest_map_state)
        result = {
            "request": dict(request),
            "ready_state": state,
        }
        self._forest_map_switches.append(result)
        self._event(
            "FOREST_MAP_SWITCH_READY",
            logical_seed=request["logical_seed"],
            raw_seed=request["raw_seed"],
            map_block_id=request["map_block_id"],
        )
        return result

    def _reset_once(self, reset_index, forest_map_request=None):
        self._state = "RESETTING"
        self._publish_identity()
        wall_start = time.monotonic()
        result = {
            "reset_index": reset_index,
            "from_episode_id": self._binding.episode_id,
            "from_generation": self._binding.reset_generation,
            "success": False,
            "failure": "",
            "reset_begin_monotonic": wall_start,
            "nominal_hover": list(self._nominal_hover),
            "reset_randomization_enabled": self._random_reset_config.enabled,
            "reset_random_seed": self._random_reset_config.seed,
        }
        self._event("RESET_BEGIN", index=reset_index)
        paused = False
        try:
            sample = self._reset_sampler.sample()
            candidate = sample.pop("candidate")
            self._hover = (candidate.x, candidate.y, candidate.z, candidate.yaw)
            next_episode_start = {
                "initial_spawn_nominal": False,
                "nominal_hover": list(self._nominal_hover),
                "sampled_reset_x": candidate.x,
                "sampled_reset_y": candidate.y,
                "sampled_reset_z": candidate.z,
                "sampled_yaw": candidate.yaw,
                "reset_random_seed": self._random_reset_config.seed,
                "sample_index": sample["sample_index"],
                "attempt_count": sample["attempt_count"],
                "candidate_validation_result": sample[
                    "candidate_validation_result"
                ],
            }
            result.update(
                sampled_reset_x=candidate.x,
                sampled_reset_y=candidate.y,
                sampled_reset_z=candidate.z,
                sampled_yaw=candidate.yaw,
                **sample
            )
            self._event(
                "RESET_CANDIDATE_ACCEPTED",
                index=reset_index,
                sampled_reset_x=candidate.x,
                sampled_reset_y=candidate.y,
                sampled_reset_z=candidate.z,
                sampled_yaw=candidate.yaw,
                reset_random_seed=self._random_reset_config.seed,
                sample_index=sample["sample_index"],
                attempt_count=sample["attempt_count"],
                candidate_validation_result=sample[
                    "candidate_validation_result"
                ],
            )
            self._cancel_pub.publish(EmptyMessage())
            response = self._adapter_hold()
            if not response.success:
                raise RuntimeError("adapter hold failed: " + response.message)
            if not self._wait(self._low_speed, self._hold_timeout, "terminal hold low speed"):
                raise RuntimeError("terminal hold did not settle")

            if forest_map_request is not None:
                result["forest_map_switch"] = self._switch_forest_map(
                    forest_map_request
                )

            map_response = self._clear_environment_map()
            if not map_response.success:
                raise RuntimeError(
                    "EGO environment map clear failed: " + map_response.message
                )
            result["ego_map_clear"] = map_response.message
            if "forest_map_switch" in result:
                result["forest_map_switch"]["ego_map_clear"] = (
                    map_response.message
                )
            self._event(
                "EGO_ENVIRONMENT_MAP_CLEARED",
                index=reset_index,
                response=map_response.message,
            )

            c_response = self._clear_c()
            cloud_response = (
                self._clear_truth_cloud()
                if self._clear_truth_cloud is not None else None
            )
            v2_response = self._clear_v2()
            if (
                not c_response.success or not v2_response.success
                or (cloud_response is not None and not cloud_response.success)
            ):
                raise RuntimeError("Observation temporal clear failed")
            c_generation, c_barrier = self._parse_generation(c_response)
            cloud_generation, cloud_barrier = (
                self._parse_generation(cloud_response)
                if cloud_response is not None else (None, None)
            )
            v2_generation, v2_barrier = self._parse_generation(v2_response)
            expected_generation = self._binding.reset_generation + 1
            self._event(
                "RESET_GENERATION_BARRIERS",
                expected_generation=expected_generation,
                observation_c_generation=c_generation,
                observation_c_barrier=c_barrier,
                truth_cloud_generation=cloud_generation,
                truth_cloud_barrier=cloud_barrier,
                observation_v2_generation=v2_generation,
                observation_v2_barrier=v2_barrier,
            )
            if (
                c_generation != expected_generation
                or v2_generation != expected_generation
                or (cloud_generation is not None and cloud_generation != expected_generation)
            ):
                raise RuntimeError("Observation generation did not advance exactly once")

            response = self._adapter_prepare()
            if not response.success:
                raise RuntimeError("adapter prepare reset failed: " + response.message)
            self._publish_reset_hover_target()
            if not self._wait(
                self._adapter_has_reset_hover_target,
                self._reset_target_timeout,
                "adapter random Hover target acknowledgement",
            ):
                raise RuntimeError("adapter did not acknowledge reset Hover target")
            self._switch([], self._controllers)
            self._pause()
            paused = True
            self._teleport()
            self._event("TELEPORT_TO_HOVER", x=self._hover[0], y=self._hover[1], z=self._hover[2])
            self._unpause()
            paused = False
            self._switch(self._controllers, [])
            response = self._adapter_resume()
            if not response.success:
                raise RuntimeError("fresh Hover command failed: " + response.message)
            response = self._adapter_engage()
            if not response.success:
                raise RuntimeError("explicit engage failed: " + response.message)

            self._reset_barrier = v2_barrier
            self._accepted_trajectory_id = 0
            self._accepted_trajectory_start = 0.0
            self._accepted_trajectory_start_secs = 0
            self._accepted_trajectory_start_nsecs = 0
            self._binding = self._ledger.advance_after_reset(v2_generation)
            self._current_episode_start = next_episode_start
            self._terminal_latched = False
            self._terminal_outcome = ""
            self._terminal_reason = ""
            self._publish_identity()

            def reset_physical_ready():
                if not self._physical_ready() or not self._v2_ready():
                    return False
                with self._lock:
                    state = self._actual_state_locked()
                position, velocity, roll, pitch, stamp = state
                return bool(
                    stamp > self._reset_barrier
                    and _norm3(position[0] - self._hover[0], position[1] - self._hover[1], position[2] - self._hover[2]) <= self._reset_position_tolerance
                    and _norm3(*velocity) <= self._reset_speed_tolerance
                    and max(abs(roll), abs(pitch)) <= self._reset_roll_pitch_tolerance
                )

            self._event("OBSERVATION_WARMUP")
            if not self._wait(reset_physical_ready, self._reset_timeout, "post-reset truth/Mid360 five-frame readiness"):
                raise RuntimeError("post-reset physical/Observation v2 readiness timeout")
            with self._lock:
                state = self._actual_state_locked()
                v2 = self._v2
            position, velocity, roll, pitch, stamp = state
            result.update(
                success=True,
                to_episode_id=self._binding.episode_id,
                to_generation=self._binding.reset_generation,
                reset_barrier_stamp=v2_barrier,
                observation_c_barrier_stamp=c_barrier,
                truth_cloud_barrier_stamp=cloud_barrier,
                reset_readiness_wall_sec=time.monotonic() - wall_start,
                first_valid_v2_stamp=v2.header.stamp.to_sec(),
                first_valid_v2_pose_stamp=v2.pose_source_stamp.to_sec(),
                first_valid_v2_history_frames=int(v2.history_frames),
                final_position_error=_norm3(position[0] - self._hover[0], position[1] - self._hover[1], position[2] - self._hover[2]),
                final_speed=_norm3(*velocity),
                final_roll_pitch=max(abs(roll), abs(pitch)),
                post_reset_truth_stamp=stamp,
            )
        except ResetSamplingError as error:
            result["sampling_attempts"] = list(error.attempts)
            result["candidate_validation_result"] = {
                "valid": False,
                "reasons": ["max_sampling_attempts_exhausted"],
            }
            result["failure"] = str(error)
            self._event(
                "RESET_FAILURE",
                index=reset_index,
                error=str(error),
                candidate_validation_result=result[
                    "candidate_validation_result"
                ],
            )
        except Exception as error:
            result["failure"] = str(error)
            self._event("RESET_FAILURE", index=reset_index, error=str(error))
            if paused:
                try:
                    self._unpause()
                except Exception:
                    pass
        self._resets.append(result)
        if not result["success"]:
            raise RuntimeError("reset {} failed: {}".format(reset_index, result["failure"]))
        return result

    def _forest_preflight_requests(self):
        config_path = Path(
            str(rospy.get_param("~forest_config", "")).strip()
        ).expanduser().resolve()
        if not config_path.is_file():
            raise RuntimeError(
                "Forest preflight config does not exist: {}".format(config_path)
            )
        config = json.loads(config_path.read_text(encoding="utf-8"))
        schedule = config["scheduler"]
        mapping = {
            int(item["logical_seed"]): int(item["raw_seed"])
            for item in config["seed_pool"]
        }
        switch_count = int(
            rospy.get_param("~forest_preflight_switch_count", 3)
        )
        block_size = int(
            rospy.get_param(
                "~forest_preflight_episodes_per_map_block", 10
            )
        )
        if switch_count != 3:
            raise RuntimeError(
                "Forest map-switch preflight must cover exactly three maps"
            )
        scheduler = BalancedForestMapScheduler(
            schedule["training_logical_seeds"],
            schedule["evaluation_logical_seeds"],
            mapping,
            episodes_per_map_block=block_size,
            rng_seed=int(
                rospy.get_param(
                    "~forest_scheduler_rng_seed", schedule["rng_seed"]
                )
            ),
            smoke_test=True,
        )
        requests = []
        for block_id in range(switch_count):
            assignment = scheduler.training_assignment(block_id * block_size)
            requests.append(
                {
                    "request_id": "forest_preflight:block:{}".format(block_id),
                    "logical_seed": assignment.logical_seed,
                    "raw_seed": assignment.raw_seed,
                    "mode": assignment.mode,
                    "map_block_id": assignment.map_block_id,
                    "map_round_id": assignment.map_round_id,
                    "scheduler_order": list(assignment.scheduler_order),
                }
            )
        return requests

    def _run_forest_map_switch_preflight(self):
        previous_generation = self._binding.reset_generation
        previous_trajectory_id = self._accepted_trajectory_id
        for index, request in enumerate(self._forest_preflight_requests(), 1):
            contamination_before = self._old_generation_valid_contamination
            reset_result = self._reset_once(index, forest_map_request=request)
            self._prepare_episode(reset_result=reset_result)
            with self._lock:
                forest_state = dict(self._forest_map_state)
                observation = self._observation
                v2 = self._v2
                registered = dict(self._registered_cloud_state)
            record = {
                "index": index,
                "request": dict(request),
                "map_state": forest_state,
                "old_generation": previous_generation,
                "new_generation": self._binding.reset_generation,
                "ego_map_clear": reset_result.get("ego_map_clear"),
                "first_valid_v2_stamp": reset_result.get("first_valid_v2_stamp"),
                "warmup_frames": reset_result.get("first_valid_v2_history_frames"),
                "first_valid_observation_c_stamp": (
                    None if observation is None
                    else observation.header.stamp.to_sec()
                ),
                "first_valid_observation_c_generation": (
                    None if observation is None
                    else int(observation.temporal_generation)
                ),
                "first_fresh_trajectory_id": self._accepted_trajectory_id,
                "first_fresh_trajectory_start": self._accepted_trajectory_start,
                "registered_cloud_state": registered,
                "old_generation_contamination_delta": (
                    self._old_generation_valid_contamination
                    - contamination_before
                ),
            }
            failures = []
            if self._binding.reset_generation != previous_generation + 1:
                failures.append("generation_increment")
            if record["old_generation_contamination_delta"] != 0:
                failures.append("old_generation_contamination")
            if observation is None or not observation.valid:
                failures.append("fresh_observation_c")
            elif int(observation.temporal_generation) != self._binding.reset_generation:
                failures.append("observation_c_generation")
            if v2 is None or int(v2.history_frames) < 5:
                failures.append("five_frame_warmup")
            if self._accepted_trajectory_id <= previous_trajectory_id:
                failures.append("fresh_ego_trajectory")
            if not forest_state.get("old_obstacles_absent_verified", False):
                failures.append("old_obstacle_removal")
            if int(forest_state.get("loaded_obstacle_count", -1)) != 18:
                failures.append("loaded_obstacle_count")
            record["failures"] = failures
            record["passed"] = not failures
            self._forest_preflight_records.append(record)
            self._event("FOREST_PREFLIGHT_MAP_READY", **record)
            if failures:
                raise RuntimeError(
                    "Forest preflight map {} failed: {}".format(
                        index, failures
                    )
                )
            previous_generation = self._binding.reset_generation
            previous_trajectory_id = self._accepted_trajectory_id
        self._cancel_pub.publish(EmptyMessage())
        self._state = "FOREST_MAP_SWITCH_PREFLIGHT_COMPLETE"
        self._publish_identity()

    def _write_json(self, name, payload):
        path = os.path.join(self._output_dir, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")

    def _summary(self, failure=""):
        outcomes = {name: sum(1 for item in self._episodes if item["outcome"] == name) for name in ("SUCCESS", "FAILURE", "TRUNCATED")}
        reset_successes = sum(1 for item in self._resets if item.get("success"))
        reset_times = [item["reset_readiness_wall_sec"] for item in self._resets if item.get("success")]
        first_c_times = [item["first_valid_observation_c_after_reset_wall_sec"] for item in self._resets if "first_valid_observation_c_after_reset_wall_sec" in item]
        observation_total = sum(item["observation_c_total"] for item in self._episodes)
        observation_valid = sum(item["observation_c_valid"] for item in self._episodes)
        sim_duration = max(0.0, rospy.Time.now().to_sec() - self._overall_sim_start)
        wall_duration = max(1.0e-9, time.monotonic() - self._overall_wall_start)
        acceptance_failures = []
        preflight = self._runner_mode == "map_switch_preflight"
        if not preflight and len(self._episodes) != self._episode_count:
            acceptance_failures.append("episode_count")
        if (
            self._action_owner == "fixed"
            and outcomes["SUCCESS"] != self._episode_count
        ):
            acceptance_failures.append("fixed_entry_episode_success")
        expected_resets = (
            self._episode_count
            if preflight
            else
            max(0, len(self._episodes) - 1)
            if self._runner_mode == "training"
            else self._episode_count
        )
        if reset_successes != expected_resets:
            acceptance_failures.append("reset_success")
        if self._old_generation_valid_contamination != 0:
            acceptance_failures.append("old_generation_valid_contamination")
        if self._trajectory_generation_mismatch != 0:
            acceptance_failures.append("trajectory_generation_mismatch")
        if self._controller_failure_count != 0:
            acceptance_failures.append("controller_failure")
        if self._runner_mode != "training" and self._planner_failure_count != 0:
            acceptance_failures.append("planner_failure")
        if self._runner_mode != "training" and self._collision_count != 0:
            acceptance_failures.append("collision")
        if self._surrogate_all_zero_count != 0:
            acceptance_failures.append("surrogate_all_zero")
        if self._surrogate_all_unknown_count != 0:
            acceptance_failures.append("surrogate_all_unknown")
        if (
            self._reward_runtime_audit_enabled
            and self._reward_runtime_invalid_count != 0
        ):
            acceptance_failures.append("reward_runtime_invalid")
        if (
            self._reward_runtime_audit_enabled
            and self._episodes
            and self._reward_runtime_step_count == 0
        ):
            acceptance_failures.append("reward_runtime_steps_missing")
        if (
            self._require_registered_ego_cloud
            and route_acceptance_required(self._runner_mode)
        ):
            if not self._registered_cloud_state.get("ready", False):
                acceptance_failures.append("registered_ego_cloud_not_ready")
            if not self._registered_corridor_counts or max(self._registered_corridor_counts) <= 0:
                acceptance_failures.append("worksite_entry_corridor_not_observed")
            try:
                accepted_replans = int(self._c_diagnostics.get("accepted_replans", "0"))
            except (TypeError, ValueError):
                accepted_replans = 0
            if accepted_replans <= 0:
                acceptance_failures.append("worksite_fresh_ego_replans_missing")
        if failure:
            acceptance_failures.append("runtime_exception")
        if self._require_acceptance and self._episode_count < 20:
            acceptance_failures.append("less_than_20_episodes")
        passed = not acceptance_failures
        if preflight and len(self._forest_preflight_records) != 3:
            acceptance_failures.append("forest_preflight_map_count")
        if preflight and any(
            not item.get("passed", False)
            for item in self._forest_preflight_records
        ):
            acceptance_failures.append("forest_preflight_map_failure")
        passed = not acceptance_failures
        verdict = (
            "FOREST MAP-SWITCH PREFLIGHT PASS"
            if passed and preflight
            else
            "GO FOR SAC TRAINING LOOP INTEGRATION"
            if passed and self._require_acceptance
            else (
                "SAC TRAINING EPISODE COUNT COMPLETE"
                if passed and self._runner_mode == "training"
                else (
                    "SAC COORDINATOR QUALIFICATION PASS"
                    if passed and self._action_owner == "sac"
                    else ("GUI QUALIFICATION PASS" if passed else "NO-GO")
                )
            )
        )
        with self._lock:
            diagnostics = {
                "observation_v2": dict(self._v2_diagnostics),
                "observation_c": dict(self._c_diagnostics),
                "registered_ego_cloud": dict(self._registered_cloud_state),
            }
        return {
            "version": VERSION,
            "qualification_only": True,
            "reward_runtime_audit": {
                "enabled": self._reward_runtime_audit_enabled,
                "schema_version": (
                    "learning_speed_reward_runtime_step_v1.0"
                    if self._reward_runtime_audit_enabled else None
                ),
                "formal_sac_transition": False,
                "reward_mode": (
                    self._reward.config.mode if self._reward is not None else None
                ),
                "reward_version": (
                    self._reward.config.version
                    if self._reward is not None else None
                ),
                "runtime_step_count": self._reward_runtime_step_count,
                "invalid_count": self._reward_runtime_invalid_count,
                "failure": self._reward_runtime_audit_failure,
            },
            "action_owner": self._action_owner,
            "runner_mode": self._runner_mode,
            "require_forest_map_ready": self._require_forest_map_ready,
            "forest_map_switch_count": len(self._forest_map_switches),
            "forest_map_switches": self._forest_map_switches,
            "forest_map_switch_preflight_records": self._forest_preflight_records,
            "nominal_hover": list(self._nominal_hover),
            "random_start": {
                "enabled": self._random_reset_config.enabled,
                "center_x": self._random_reset_config.center_x,
                "center_y": self._random_reset_config.center_y,
                "x_min_offset": self._random_reset_config.x_min_offset,
                "x_max_offset": self._random_reset_config.x_max_offset,
                "y_min_offset": self._random_reset_config.y_min_offset,
                "y_max_offset": self._random_reset_config.y_max_offset,
                "z": self._random_reset_config.z,
                "yaw": self._random_reset_config.yaw,
                "seed": self._random_reset_config.seed,
                "max_sampling_attempts": (
                    self._random_reset_config.max_sampling_attempts
                ),
            },
            "require_automated_acceptance": self._require_acceptance,
            "verdict": verdict,
            "acceptance_failures": acceptance_failures,
            "runtime_failure": failure,
            "episode_count": len(self._episodes),
            "configured_episode_count": self._episode_count,
            "completion_reason": (
                "forest_map_switch_preflight_complete"
                if preflight
                else "training_episode_count_reached"
                if self._runner_mode == "training"
                else "episode_count_reached"
            ),
            "success": outcomes["SUCCESS"],
            "failure": outcomes["FAILURE"],
            "truncated": outcomes["TRUNCATED"],
            "reset_count": len(self._resets),
            "reset_success_count": reset_successes,
            "reset_success_rate": reset_successes / float(max(1, len(self._resets))),
            "reset_readiness_wall_sec": {
                "min": min(reset_times) if reset_times else None,
                "median": statistics.median(reset_times) if reset_times else None,
                "p95": _percentile(reset_times, 0.95),
                "max": max(reset_times) if reset_times else None,
            },
            "first_valid_observation_c_after_reset_wall_sec": {
                "count": len(first_c_times),
                "min": min(first_c_times) if first_c_times else None,
                "median": statistics.median(first_c_times) if first_c_times else None,
                "p95": _percentile(first_c_times, 0.95),
                "max": max(first_c_times) if first_c_times else None,
            },
            "old_generation_valid_contamination_count": self._old_generation_valid_contamination,
            "stale_cloud_state_count": self._stale_cloud_state_count,
            "stale_observation_snapshot_count": len(self._stale_snapshots),
            "stale_observation_snapshot_file": os.path.join(
                self._output_dir, "stale_observation_snapshots.jsonl"
            ),
            "trajectory_generation_mismatch_count": self._trajectory_generation_mismatch,
            "controller_failure_count": self._controller_failure_count,
            "planner_failure_count": self._planner_failure_count,
            "collision_count": self._collision_count,
            "observation_c_valid_count": observation_valid,
            "observation_c_total_count": observation_total,
            "observation_c_valid_rate": observation_valid / float(max(1, observation_total)),
            "surrogate_obstacle_bins": _statistics(self._surrogate_obstacle_counts),
            "surrogate_free_bins": _statistics(self._surrogate_free_counts),
            "surrogate_unknown_bins": _statistics(self._surrogate_unknown_counts),
            "surrogate_all_zero_count": self._surrogate_all_zero_count,
            "surrogate_all_unknown_count": self._surrogate_all_unknown_count,
            "tracking_error": _statistics(self._tracking_errors),
            "actual_speed": _statistics(self._actual_speeds),
            "registered_entry_corridor_points": _statistics(self._registered_corridor_counts),
            "registered_entry_corridor_minimum_distance": _statistics(self._registered_corridor_minimum_distances),
            "sim_duration": sim_duration,
            "wall_duration": wall_duration,
            "rtf": sim_duration / wall_duration,
            "final_ready_episode_id": self._binding.episode_id,
            "final_reset_generation": self._binding.reset_generation,
            "diagnostics": diagnostics,
        }

    def run(self):
        failure = ""
        try:
            self._wait_services()
            self._overall_wall_start = time.monotonic()
            self._overall_sim_start = rospy.Time.now().to_sec()
            if self._runner_mode == "map_switch_preflight":
                self._run_forest_map_switch_preflight()
            else:
                self._state = "WAIT_INITIAL_READY"
                self._prepare_episode()
            index = 1
            while (
                self._runner_mode != "map_switch_preflight"
                and not rospy.is_shutdown()
            ):
                self._activate_episode()
                self._run_episode()
                closure = self._wait_for_sac_closure()
                if not sac_closure_reset_allowed(
                    self._binding,
                    closure,
                    self._terminal_reason,
                ):
                    self._event(
                        "SAC_INFRASTRUCTURE_FAIL_CLOSED",
                        terminal_reason=self._terminal_reason,
                        reset_forbidden=True,
                        next_episode_forbidden=True,
                    )
                    raise RuntimeError(
                        "infrastructure terminal closed; reset and next "
                        "Episode are forbidden: {}".format(
                            self._terminal_reason
                        )
                    )
                stop_due = episode_count_stop_due(
                    index,
                    self._episode_count,
                )
                if self._runner_mode == "training" and stop_due:
                    self._cancel_pub.publish(EmptyMessage())
                    self._state = "QUALIFICATION_COMPLETE_HOVER"
                    self._publish_identity()
                    break
                forest_map_request = (
                    None if closure is None
                    else closure.get("next_forest_map_request")
                )
                reset_result = self._reset_once(
                    index, forest_map_request=forest_map_request
                )
                self._prepare_episode(reset_result=reset_result)
                if stop_due:
                    self._cancel_pub.publish(EmptyMessage())
                    self._state = "QUALIFICATION_COMPLETE_HOVER"
                    self._publish_identity()
                    break
                index += 1
        except Exception as error:
            failure = str(error)
            self._state = "QUALIFICATION_FAILED"
            self._terminal_latched = True
            self._terminal_outcome = "FAILURE"
            self._terminal_reason = failure
            self._publish_identity()
            self._event("QUALIFICATION_FAILURE", error=failure)
        summary = self._summary(failure)
        self._write_json("episode_results.json", self._episodes)
        self._write_json("reset_results.json", self._resets)
        if self._runner_mode == "map_switch_preflight":
            self._write_json(
                "map_switch_preflight_results.json",
                self._forest_preflight_records,
            )
        self._write_json("qualification_summary.json", summary)
        self._event("QUALIFICATION_COMPLETE", verdict=summary["verdict"])
        self._event_file.close()
        self._stale_snapshot_file.close()
        if self._reward_step_file is not None:
            self._reward_step_file.close()
        rospy.logwarn("[TRAINING EPISODE] %s", summary["verdict"])
        return summary


def main():
    rospy.init_node("training_episode_reset_coordinator")
    coordinator = TrainingEpisodeResetCoordinator()
    coordinator.run()


if __name__ == "__main__":
    main()
