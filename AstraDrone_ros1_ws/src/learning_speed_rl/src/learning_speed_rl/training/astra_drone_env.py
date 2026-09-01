"""UAV1 Learning Speed causal environment and sequential Episode scheduler.

The nominal policy grid remains 0.1 s in ROS/simulation time, while at most one
ACTION INTERVAL may be active.  Once its causal transition is formed, the
immutable record leaves action ownership and may wait for ordered persistence
without owning the next Actor admission.  Gazebo, sensors, mapping, EGO and
control remain continuous and asynchronous.  This module does not reset the
stack, train a policy, or bypass the stamped SpeedSafetyFilter action chain.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Dict, List, Mapping, Optional, Tuple

from .calibration import TrackingSafetyMirror, lidar_clutter_metrics
from .data_contract import (
    LIDAR_BINS,
    AppliedSpeedAction,
    OfficialTrajectoryIdentity,
    PolicyStateV1,
    ProgressContextState,
    ProgressRewardContext,
    RunEpisodeProvenance,
    SacTransitionV1,
    causal_observation_receipt_time,
    is_causal_next_policy_state,
    official_trajectory_identity_from_bspline,
    policy_state_from_observation_c,
)
from .environment_interface import SpeedTrainingEnvironment
from .reward import (
    LearningSpeedReward,
    actual_speed_mps_from_body_velocity,
    reward_from_config,
    reward_input_from_signals,
    tracking_error_m_from_body_error,
)
from .transition_persistence import ImmutableTransitionRecord


EPISODE_VERSION = "astra_drone_episode_v0.1"
FORCE_REPLAN_DECREASE_MPS = 0.3
FORCE_REPLAN_INCREASE_MPS = 0.5
TRAJECTORY_END_FUTURE_TOLERANCE_SEC = 1.0e-6


class AstraDroneStepError(RuntimeError):
    """Base error for a step that cannot form a valid transition."""


class AstraDroneStepTimeout(AstraDroneStepError):
    def __init__(self, phase: str, timeout_sec: float):
        self.phase = str(phase)
        self.timeout_sec = float(timeout_sec)
        super().__init__(
            "AstraDroneEnv step timed out in {} after {:.3f} wall seconds".format(
                self.phase, self.timeout_sec
            )
        )


class AstraDroneStepCausalityError(AstraDroneStepError):
    """Raised instead of emitting a transition with mixed causal provenance."""


class AstraDroneActionIneligible(RuntimeError):
    """A policy tick that must not cross the action acceptance boundary."""

    def __init__(self, reason: str):
        self.reason = str(reason)
        super().__init__(self.reason)


class AstraDroneEpisodeError(RuntimeError):
    """Raised when Episode v0.1 cannot preserve its configured contract."""


@dataclass(frozen=True)
class AstraDroneStepConfig:
    action_timeout_sec: float = 2.0
    applied_timeout_sec: float = 3.0
    next_observation_timeout_sec: float = 2.0
    requested_match_tolerance_mps: float = 1.0e-6
    applied_pair_tolerance_mps: float = 0.005

    def __post_init__(self):
        values = (
            self.action_timeout_sec,
            self.applied_timeout_sec,
            self.next_observation_timeout_sec,
            self.requested_match_tolerance_mps,
            self.applied_pair_tolerance_mps,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("step configuration must be finite")
        if min(
            self.action_timeout_sec,
            self.applied_timeout_sec,
            self.next_observation_timeout_sec,
        ) <= 0.0:
            raise ValueError("step timeouts must be positive")
        if min(
            self.requested_match_tolerance_mps,
            self.applied_pair_tolerance_mps,
        ) < 0.0:
            raise ValueError("step match tolerances must be non-negative")


@dataclass(frozen=True)
class AstraDroneEpisodeConfig:
    policy_period_sec: float = 0.1
    max_steps: Optional[int] = 100
    max_duration_sec: Optional[float] = None
    start_timeout_sec: float = 600.0
    completion_timeout_sec: float = 10.0
    deadline_tolerance_sec: float = 0.02
    minimum_active_speed_mps: float = 0.2
    max_steps_reason: str = "max_episode_steps"

    def __post_init__(self):
        numeric = (
            self.policy_period_sec,
            self.start_timeout_sec,
            self.completion_timeout_sec,
            self.deadline_tolerance_sec,
            self.minimum_active_speed_mps,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("episode timing configuration must be finite")
        if abs(float(self.policy_period_sec) - 0.1) > 1.0e-12:
            raise ValueError("Episode v0.1 policy_period_sec must be 0.1")
        if self.start_timeout_sec <= 0.0 or self.completion_timeout_sec <= 0.0:
            raise ValueError("episode wall-time timeouts must be positive")
        if self.deadline_tolerance_sec < 0.0:
            raise ValueError("deadline tolerance must be non-negative")
        if self.minimum_active_speed_mps < 0.0:
            raise ValueError("minimum active speed must be non-negative")
        if self.max_steps is None and self.max_duration_sec is None:
            raise ValueError("Episode v0.1 requires a max step or duration limit")
        if self.max_steps is not None and int(self.max_steps) <= 0:
            raise ValueError("max_steps must be positive when configured")
        if not str(self.max_steps_reason).strip():
            raise ValueError("max_steps_reason must be non-empty")
        if self.max_duration_sec is not None:
            duration = float(self.max_duration_sec)
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError("max_duration_sec must be positive and finite")


@dataclass
class ActiveActionInterval:
    episode_id: str
    step_index: int
    scheduled_ros_time_sec: float
    policy_tick_index: int
    request: "ActionRequestToken"
    state_t: "StepObservation"
    created_wall_time_sec: float
    action_event: Optional["ActionStampedEvent"] = None
    applied_event: Optional["AppliedVMaxEvent"] = None
    policy_trajectory: Optional[OfficialTrajectoryIdentity] = None
    planner_diagnostic_trajectory: Optional[OfficialTrajectoryIdentity] = None
    force_replan_expected: bool = False
    planner_diagnostic_status: str = "not_expected"
    transition_record: Optional[ImmutableTransitionRecord] = None
    reward: Optional[float] = None
    terminated: bool = False
    terminal_reason: str = ""
    next_state: Optional["StepObservation"] = None
    actor_provider_wall_sec: float = 0.0
    request_publish_wall_time_sec: Optional[float] = None
    action_ack_wall_time_sec: Optional[float] = None
    applied_ack_wall_time_sec: Optional[float] = None
    planner_diagnostic_ready_wall_time_sec: Optional[float] = None
    close_policy_tick_index: Optional[int] = None
    close_scheduled_ros_time_sec: Optional[float] = None
    close_trigger: str = ""
    completed_wall_time_sec: Optional[float] = None


@dataclass(frozen=True)
class EpisodeBoundary:
    episode_id: str
    step_index: int
    episode_start_time: float
    episode_elapsed: float
    episode_return: float
    terminated: bool
    truncated: bool
    terminal_reason: str

    def to_record(self) -> Dict[str, object]:
        return {
            "version": EPISODE_VERSION,
            "episode_id": self.episode_id,
            "step_index": int(self.step_index),
            "episode_start_time": float(self.episode_start_time),
            "episode_elapsed": float(self.episode_elapsed),
            "episode_return": float(self.episode_return),
            "terminated": bool(self.terminated),
            "truncated": bool(self.truncated),
            "terminal_reason": self.terminal_reason,
        }


@dataclass(frozen=True)
class StepObservation:
    sequence: int
    state: PolicyStateV1
    reward_context: ProgressContextState
    reward_observation: Mapping[str, object]
    receive_wall_time_sec: float
    telemetry: Mapping[str, object]


@dataclass(frozen=True)
class ActionStampedEvent:
    sequence: int
    receive_sec: float
    stamp_sec: float
    episode_id: str
    step_index: int
    request_id: int
    source_mode: str
    requested_v_max: float
    filtered_v_max: float
    receive_wall_time_sec: float


@dataclass(frozen=True)
class AppliedVMaxEvent:
    sequence: int
    receive_sec: float
    stamp_sec: float
    episode_id: str
    step_index: int
    request_id: int
    applied_v_max: float
    receive_wall_time_sec: float


@dataclass(frozen=True)
class ActionRequestToken:
    episode_id: str
    step_index: int
    request_id: int
    publish_ros_time_sec: float
    requested_v_max: float
    publish_ros_secs: Optional[int] = None
    publish_ros_nsecs: Optional[int] = None


@dataclass(frozen=True)
class TerminalSnapshot:
    terminated: bool
    dangerous_terminal: bool
    terminal_reason: str
    event_ros_time_sec: Optional[float] = None


class AstraDroneEnv(SpeedTrainingEnvironment):
    """Single-UAV asynchronous causal environment with one Episode owner."""

    def __init__(
        self,
        publish_requested_v_max: Callable[[float], None],
        ros_clock: Callable[[], float],
        reward: LearningSpeedReward,
        config: Optional[AstraDroneStepConfig] = None,
        run_id: str = "astra_drone_env_paper_aligned",
        episode_id: str = "single_runtime",
        wall_clock: Callable[[], float] = time.monotonic,
        tracking_limit_m: float = 1.0,
        tracking_duration_sec: float = 1.0,
        ros_stamp_clock: Optional[Callable[[], object]] = None,
    ):
        if not callable(publish_requested_v_max) or not callable(ros_clock):
            raise ValueError("publisher and ROS clock must be callable")
        self._publish_requested_v_max = publish_requested_v_max
        self._ros_clock = ros_clock
        self._wall_clock = wall_clock
        self._ros_stamp_clock = ros_stamp_clock
        self._reward = reward
        self.config = config or AstraDroneStepConfig()
        self._run_episode = RunEpisodeProvenance(run_id, episode_id)

        self._condition = threading.Condition(threading.RLock())
        self._episode_lock = threading.Lock()
        self._sequence = 0
        self._observations = deque(maxlen=2048)
        self._actions = {}
        self._applied = {}
        self._seen_action_keys = set()
        self._seen_applied_keys = set()
        self._last_action_id_by_episode = {}
        self._last_applied_id_by_episode = {}
        self._accepted_action_count = 0
        self._accepted_applied_count = 0
        self._duplicate_action_count = 0
        self._duplicate_applied_count = 0
        self._identity_violation = ""
        self._trajectory_history = deque(maxlen=2048)
        self._trajectory_receive_wall = {}
        self._latest_official_trajectory = None
        self._episode_trajectory_floor = None
        self._episode_initial_trajectory_ready = False
        self._valid_observations = 0
        self._invalid_observations = 0

        self._mission_state = ""
        self._mission_state_receipt_sec = None
        self._latest_progress = None
        self._mission_started = False
        self._mission_done = False
        self._mission_done_receipt_sec = None
        self._mission_success = False
        self._mission_success_receipt_sec = None
        self._mission_failure = False
        self._mission_failure_receipt_sec = None
        self._bridge_state = ""
        self._planner_state = ""
        self._collision_terminal = False
        self._collision_terminal_receipt_sec = None
        self._emergency_terminal = False
        self._emergency_terminal_receipt_sec = None
        self._external_truncated = False
        self._external_truncated_receipt_sec = None
        self._external_truncated_reason = ""
        self._expected_reset_generation = None
        self._generation_mismatch_observations = 0
        self._tracking_mirror = TrackingSafetyMirror(
            tracking_limit_m, tracking_duration_sec
        )
        self._shutdown_requested = False
        self._ros_handles = []

    def reset(self):
        raise NotImplementedError("AstraDroneEnv does not implement reset()")

    def request_shutdown(self):
        with self._condition:
            self._shutdown_requested = True
            self._condition.notify_all()

    def begin_external_episode(
        self,
        run_id,
        episode_id,
        reset_generation,
        *,
        official_trajectory_id,
        official_trajectory_start_time,
        official_trajectory_start_secs=None,
        official_trajectory_start_nsecs=None,
    ):
        """Rebind one coordinator-owned Episode without implementing reset here."""

        generation = int(reset_generation)
        trajectory_id = int(official_trajectory_id)
        trajectory_start = float(official_trajectory_start_time)
        if (
            not str(run_id)
            or not str(episode_id)
            or generation < 0
            or trajectory_id <= 0
            or not math.isfinite(trajectory_start)
            or trajectory_start <= 0.0
        ):
            raise ValueError("external Episode identity is invalid")
        trajectory_identity = OfficialTrajectoryIdentity(
            trajectory_id=trajectory_id,
            start_time_sec=trajectory_start,
            source_frame="episode_identity",
            start_time_secs=official_trajectory_start_secs,
            start_time_nsecs=official_trajectory_start_nsecs,
        )
        with self._episode_lock:
            with self._condition:
                trajectory_floor = trajectory_identity.identity_key
                accepted = None
                for receipt, identity in self._trajectory_history:
                    if identity.identity_key >= trajectory_floor:
                        accepted = (receipt, identity)
                self._run_episode = RunEpisodeProvenance(
                    str(run_id), str(episode_id)
                )
                self._expected_reset_generation = generation
                self._observations.clear()
                self._actions.clear()
                self._applied.clear()
                self._seen_action_keys.clear()
                self._seen_applied_keys.clear()
                self._last_action_id_by_episode.clear()
                self._last_applied_id_by_episode.clear()
                self._trajectory_history.clear()
                self._episode_trajectory_floor = trajectory_floor
                self._episode_initial_trajectory_ready = accepted is not None
                self._latest_official_trajectory = (
                    None if accepted is None else accepted[1]
                )
                if accepted is not None:
                    self._trajectory_history.append(accepted)
                self._identity_violation = ""
                self._mission_state = ""
                self._mission_state_receipt_sec = None
                self._latest_progress = None
                self._mission_started = False
                self._mission_done = False
                self._mission_done_receipt_sec = None
                self._mission_success = False
                self._mission_success_receipt_sec = None
                self._mission_failure = False
                self._mission_failure_receipt_sec = None
                self._bridge_state = ""
                self._planner_state = ""
                self._collision_terminal = False
                self._collision_terminal_receipt_sec = None
                self._emergency_terminal = False
                self._emergency_terminal_receipt_sec = None
                self._external_truncated = False
                self._external_truncated_receipt_sec = None
                self._external_truncated_reason = ""
                self._generation_mismatch_observations = 0
                self._tracking_mirror = TrackingSafetyMirror(
                    self._tracking_mirror.limit_m,
                    self._tracking_mirror.duration_sec,
                )
                self._condition.notify_all()

    def record_external_truncation(self, reason, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        if not math.isfinite(receipt) or receipt <= 0.0 or not str(reason):
            raise ValueError("external truncation is invalid")
        with self._condition:
            if not self._external_truncated:
                self._external_truncated = True
                self._external_truncated_receipt_sec = receipt
                self._external_truncated_reason = str(reason)
            self._condition.notify_all()

    @staticmethod
    def _positive_finite(value, name):
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError("{} must be positive and finite".format(name))
        return result

    def _next_sequence_locked(self):
        self._sequence += 1
        return self._sequence

    def _now_ros(self):
        value = float(self._ros_clock())
        if not math.isfinite(value) or value <= 0.0:
            raise AstraDroneStepCausalityError(
                "ROS/simulation time is not positive and finite"
            )
        return value

    def record_official_trajectory(self, identity, receive_sec=None):
        if not isinstance(identity, OfficialTrajectoryIdentity):
            raise TypeError("official trajectory identity has the wrong type")
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        if not math.isfinite(receipt) or receipt < 0.0:
            raise ValueError("trajectory receipt time is invalid")
        with self._condition:
            receive_wall = self._wall_clock()
            identity_key = identity.identity_key
            if (
                self._episode_trajectory_floor is not None
                and identity_key < self._episode_trajectory_floor
            ):
                return None
            current = self._latest_official_trajectory
            if current is not None and identity_key < current.identity_key:
                return None
            self._latest_official_trajectory = identity
            self._episode_initial_trajectory_ready = True
            self._next_sequence_locked()
            self._trajectory_history.append((receipt, identity))
            self._trajectory_receive_wall[identity_key] = receive_wall
            self._condition.notify_all()
            return identity

    def record_observation(
        self,
        state: PolicyStateV1,
        reward_context: ProgressContextState,
        reward_observation: Mapping[str, object],
        telemetry: Optional[Mapping[str, object]] = None,
    ):
        if not isinstance(state, PolicyStateV1):
            raise TypeError("state must be PolicyStateV1")
        if not isinstance(reward_context, ProgressContextState):
            raise TypeError("reward_context must be ProgressContextState")
        if (
            abs(
                state.provenance.observation_receive_sec
                - reward_context.observation_receive_sec
            )
            > 1.0e-9
        ):
            raise ValueError("reward context is not bound to its observation")
        required = {
            "nearest_obstacle_distance_m",
            "known_obstacle_bin_fraction",
            "unknown_bin_count",
        }
        if not required.issubset(reward_observation):
            raise ValueError("reward observation is incomplete")
        with self._condition:
            sequence = self._next_sequence_locked()
            packet = StepObservation(
                sequence=sequence,
                state=state,
                reward_context=reward_context,
                reward_observation=dict(reward_observation),
                receive_wall_time_sec=self._wall_clock(),
                telemetry={} if telemetry is None else dict(telemetry),
            )
            self._observations.append(packet)
            self._valid_observations += 1
            self._condition.notify_all()
            return packet

    def record_invalid_observation(self):
        with self._condition:
            self._invalid_observations += 1
            self._next_sequence_locked()
            self._condition.notify_all()

    def record_action_stamped(
        self,
        *,
        stamp_sec,
        episode_id,
        step_index,
        request_id,
        source_mode,
        requested_v_max,
        filtered_v_max,
        receive_sec=None,
    ):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        values = (
            receipt,
            float(stamp_sec),
            float(requested_v_max),
            float(filtered_v_max),
        )
        if (
            not all(math.isfinite(value) for value in values)
            or min(values) <= 0.0
            or not str(episode_id)
            or int(request_id) <= 0
        ):
            return None
        with self._condition:
            key = (str(episode_id), int(request_id))
            if key in self._seen_action_keys:
                self._duplicate_action_count += 1
                return None
            previous = self._last_action_id_by_episode.get(key[0])
            if previous is not None and key[1] <= previous:
                self._identity_violation = (
                    "action request_id is not strictly monotonic for episode {}"
                ).format(key[0])
                self._condition.notify_all()
                return None
            if float(stamp_sec) > receipt + 1.0e-6:
                self._identity_violation = "action timestamp is in the future"
                self._condition.notify_all()
                return None
            event = ActionStampedEvent(
                sequence=self._next_sequence_locked(),
                receive_sec=receipt,
                stamp_sec=float(stamp_sec),
                episode_id=key[0],
                step_index=int(step_index),
                request_id=key[1],
                source_mode=str(source_mode),
                requested_v_max=float(requested_v_max),
                filtered_v_max=float(filtered_v_max),
                receive_wall_time_sec=self._wall_clock(),
            )
            self._actions[key] = event
            self._seen_action_keys.add(key)
            self._last_action_id_by_episode[key[0]] = key[1]
            self._accepted_action_count += 1
            self._condition.notify_all()
            return event

    def record_applied_stamped(
        self,
        *,
        stamp_sec,
        episode_id,
        step_index,
        request_id,
        applied_v_max,
        receive_sec=None,
    ):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        value = float(applied_v_max)
        if (
            not math.isfinite(receipt)
            or receipt <= 0.0
            or not math.isfinite(float(stamp_sec))
            or float(stamp_sec) <= 0.0
            or not math.isfinite(value)
            or value <= 0.0
            or not str(episode_id)
            or int(request_id) <= 0
        ):
            return None
        with self._condition:
            key = (str(episode_id), int(request_id))
            if key in self._seen_applied_keys:
                self._duplicate_applied_count += 1
                return None
            previous = self._last_applied_id_by_episode.get(key[0])
            if previous is not None and key[1] <= previous:
                self._identity_violation = (
                    "applied request_id is not strictly monotonic for episode {}"
                ).format(key[0])
                self._condition.notify_all()
                return None
            if float(stamp_sec) > receipt + 1.0e-6:
                self._identity_violation = (
                    "applied acknowledgement timestamp is in the future"
                )
                self._condition.notify_all()
                return None
            event = AppliedVMaxEvent(
                sequence=self._next_sequence_locked(),
                receive_sec=receipt,
                stamp_sec=float(stamp_sec),
                episode_id=key[0],
                step_index=int(step_index),
                request_id=key[1],
                applied_v_max=value,
                receive_wall_time_sec=self._wall_clock(),
            )
            self._applied[key] = event
            self._seen_applied_keys.add(key)
            self._last_applied_id_by_episode[key[0]] = key[1]
            self._accepted_applied_count += 1
            self._condition.notify_all()
            return event

    def record_mission_state(self, mission_state, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        with self._condition:
            self._mission_state = str(mission_state)
            self._mission_state_receipt_sec = receipt
            if self._mission_state not in ("", "WAIT_INPUTS"):
                self._mission_started = True
            self._condition.notify_all()

    def record_mission_progress(self, progress, receive_sec=None):
        value = float(progress)
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            return
        with self._condition:
            self._latest_progress = (receipt, value)
            self._condition.notify_all()

    def record_mission_done(self, done, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        with self._condition:
            if bool(done) and not self._mission_done:
                self._mission_done = True
                self._mission_done_receipt_sec = receipt
            self._condition.notify_all()

    def record_mission_success(self, success, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        with self._condition:
            if bool(success) and not self._mission_success:
                self._mission_success = True
                self._mission_success_receipt_sec = receipt
            self._condition.notify_all()

    def record_mission_failure(self, failure, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        with self._condition:
            if bool(failure) and not self._mission_failure:
                self._mission_failure = True
                self._mission_failure_receipt_sec = receipt
            self._condition.notify_all()

    def record_bridge_state(self, bridge_state):
        with self._condition:
            self._bridge_state = str(bridge_state)
            self._condition.notify_all()

    def record_tracking_error(self, error_m, receive_sec=None):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        value = float(error_m)
        if not math.isfinite(value):
            return
        with self._condition:
            self._tracking_mirror.update(receipt, value, self._bridge_state)
            self._condition.notify_all()

    def record_planner_safety(
        self,
        *,
        current_position_in_collision,
        emergency_stop_active,
        planner_state=None,
        receive_sec=None
    ):
        receipt = self._now_ros() if receive_sec is None else float(receive_sec)
        with self._condition:
            if planner_state is not None:
                self._planner_state = str(planner_state)
            if self._mission_started and not self._mission_done:
                collision = bool(
                    current_position_in_collision
                    and self._bridge_state == "TRACK_EGO"
                )
                if collision and not self._collision_terminal:
                    self._collision_terminal = True
                    self._collision_terminal_receipt_sec = receipt
                if bool(emergency_stop_active) and not self._emergency_terminal:
                    self._emergency_terminal = True
                    self._emergency_terminal_receipt_sec = receipt
            self._condition.notify_all()

    def _progress_context_locked(self, observation_receive_sec):
        if (
            self._latest_progress is None
            or not self._mission_state
            or self._mission_state_receipt_sec is None
        ):
            return None
        progress_receipt, progress = self._latest_progress
        if (
            progress_receipt > observation_receive_sec
            or self._mission_state_receipt_sec > observation_receive_sec
        ):
            return None
        return ProgressContextState(
            mission_progress=progress,
            progress_receipt_ros_time_sec=progress_receipt,
            mission_state=self._mission_state,
            mission_state_receipt_ros_time_sec=self._mission_state_receipt_sec,
            observation_receive_sec=observation_receive_sec,
        )

    def ingest_observation_c(self, message):
        callback_ros_time = self._now_ros()
        receive_sec = causal_observation_receipt_time(
            callback_ros_time,
            message.lookup_receipt_time.to_sec(),
            message.header.stamp.to_sec(),
        )
        if self._expected_reset_generation is not None and (
            int(message.temporal_generation) != self._expected_reset_generation
            or int(message.lidar_temporal_generation)
            != self._expected_reset_generation
        ):
            with self._condition:
                self._generation_mismatch_observations += 1
            self.record_invalid_observation()
            return None
        try:
            state = policy_state_from_observation_c(message, receive_sec)
            if state is None:
                self.record_invalid_observation()
                return None
            clutter = lidar_clutter_metrics(
                message.lidar_surrogate, message.lidar_semantic
            )
        except (TypeError, ValueError):
            self.record_invalid_observation()
            return None
        with self._condition:
            reward_context = self._progress_context_locked(receive_sec)
        if reward_context is None:
            self.record_invalid_observation()
            return None
        return self.record_observation(
            state,
            reward_context,
            clutter,
            telemetry={
                "observation_header_stamp_sec": float(
                    message.header.stamp.to_sec()
                ),
                "lookup_receipt_stamp_sec": float(
                    message.lookup_receipt_time.to_sec()
                ),
                "source_to_receipt_latency_sec": float(
                    message.source_to_receipt_latency_sec
                ),
                "lidar_pose_source_stamp_sec": float(
                    message.lidar_pose_source_stamp.to_sec()
                ),
                "lidar_build_duration_ms": float(
                    message.lidar_build_duration_ms
                ),
                "producer_input_queue_lag_wall_sec": float(
                    message.producer_input_queue_lag_wall_sec
                ),
                "producer_source_to_callback_sec": float(
                    message.producer_source_to_callback_sec
                ),
                "producer_fusion_duration_wall_ms": float(
                    message.producer_fusion_duration_wall_ms
                ),
                "producer_input_count": int(message.producer_input_count),
                "producer_processed_count": int(
                    message.producer_processed_count
                ),
                "producer_coalesced_drop_count": int(
                    message.producer_coalesced_drop_count
                ),
                "temporal_generation": int(message.temporal_generation),
                "lidar_temporal_generation": int(
                    message.lidar_temporal_generation
                ),
                "selected_trajectory_id": int(
                    message.selected_trajectory_id
                ),
                "selected_trajectory_start_stamp_sec": float(
                    message.selected_trajectory_start_stamp.to_sec()
                ),
                "selected_trajectory_start_stamp_secs": int(
                    message.selected_trajectory_start_stamp.secs
                ),
                "selected_trajectory_start_stamp_nsecs": int(
                    message.selected_trajectory_start_stamp.nsecs
                ),
                "selected_trajectory_end_stamp_sec": float(
                    message.selected_trajectory_end_stamp.to_sec()
                ),
                "latest_trajectory_id_at_lookup": int(
                    message.latest_trajectory_id_at_lookup
                ),
                "latest_trajectory_start_stamp_sec": float(
                    message.latest_trajectory_start_stamp.to_sec()
                ),
                "latest_trajectory_start_stamp_secs": int(
                    message.latest_trajectory_start_stamp.secs
                ),
                "latest_trajectory_start_stamp_nsecs": int(
                    message.latest_trajectory_start_stamp.nsecs
                ),
                "trajectory_lookup_result": str(
                    message.trajectory_lookup_result
                ),
                "kinematic_lookup_result": str(
                    message.kinematic_lookup_result
                ),
            },
        )

    def ingest_official_bspline(self, message):
        try:
            identity = official_trajectory_identity_from_bspline(message)
        except (TypeError, ValueError):
            return None
        return self.record_official_trajectory(identity)

    def ingest_action_stamped(self, message):
        if message.version != "learning_speed_action_v1.1":
            return None
        stamp_sec = message.header.stamp.to_sec()
        return self.record_action_stamped(
            stamp_sec=stamp_sec,
            episode_id=message.episode_id,
            step_index=message.step_index,
            request_id=message.request_id,
            source_mode=message.source_mode,
            requested_v_max=message.requested_v_max,
            filtered_v_max=message.filtered_v_max,
            receive_sec=max(self._now_ros(), stamp_sec),
        )

    def ingest_applied_stamped(self, message):
        if message.version != "learning_speed_applied_v1.0":
            return None
        stamp_sec = message.header.stamp.to_sec()
        return self.record_applied_stamped(
            stamp_sec=stamp_sec,
            episode_id=message.episode_id,
            step_index=message.step_index,
            request_id=message.request_id,
            applied_v_max=message.applied_v_max,
            receive_sec=max(self._now_ros(), stamp_sec),
        )

    def _terminal_snapshot(self, not_after_ros_time_sec=None):
        with self._condition:
            cutoff = (
                math.inf
                if not_after_ros_time_sec is None
                else float(not_after_ros_time_sec)
            )
            events: List[Tuple[float, str, bool]] = []

            def add(receipt, reason, dangerous=False):
                if receipt is not None and receipt <= cutoff + 1.0e-9:
                    events.append((float(receipt), reason, bool(dangerous)))

            if self._collision_terminal:
                add(self._collision_terminal_receipt_sec, "collision_proxy", True)
            if self._emergency_terminal:
                add(self._emergency_terminal_receipt_sec, "ego_emergency_stop", True)
            if self._tracking_mirror.triggered:
                add(
                    self._tracking_mirror.trigger_stamp_sec,
                    "tracking_safety_gate",
                    True,
                )
            if self._mission_failure:
                add(self._mission_failure_receipt_sec, "mission_failure")
            if self._mission_success:
                add(self._mission_success_receipt_sec, "mission_success")
            have_mission_result = any(
                item[1] in ("mission_success", "mission_failure")
                for item in events
            )
            if self._mission_done and not have_mission_result and not events:
                add(self._mission_done_receipt_sec, "mission_terminal")

            events.sort(key=lambda item: (item[0], item[1]))
            return TerminalSnapshot(
                terminated=bool(events),
                dangerous_terminal=any(item[2] for item in events),
                terminal_reason=";".join(item[1] for item in events),
                event_ros_time_sec=(events[0][0] if events else None),
            )

    def _episode_ready_locked(self, minimum_active_speed_mps=0.2):
        terminal = self._terminal_snapshot()
        actual_speed = (
            actual_speed_mps_from_body_velocity(
                self._observations[-1].state.actual_velocity_body
            )
            if self._observations
            else 0.0
        )
        return bool(
            self._observations
            and self._episode_initial_trajectory_ready
            and self._latest_official_trajectory is not None
            and self._mission_started
            and self._bridge_state == "TRACK_EGO"
            and self._planner_state == "EXEC_TRAJ"
            and actual_speed + 1.0e-12 >= float(minimum_active_speed_mps)
            and not terminal.terminated
        )

    def episode_readiness(self) -> Dict[str, object]:
        with self._condition:
            terminal = self._terminal_snapshot()
            return {
                "ready": self._episode_ready_locked(),
                "valid_observation_available": bool(self._observations),
                "fresh_official_trajectory_available": bool(
                    self._episode_initial_trajectory_ready
                    and self._latest_official_trajectory is not None
                ),
                "mission_started": bool(self._mission_started),
                "mission_state": self._mission_state,
                "bridge_state": self._bridge_state,
                "planner_state": self._planner_state,
                "actual_speed_mps": (
                    actual_speed_mps_from_body_velocity(
                        self._observations[-1].state.actual_velocity_body
                    )
                    if self._observations
                    else None
                ),
                "terminated": terminal.terminated,
                "terminal_reason": terminal.terminal_reason,
            }

    @staticmethod
    def _requires_force_replan(previous_v_max, requested_v_max):
        previous = float(previous_v_max)
        requested = float(requested_v_max)
        return bool(
            requested < previous - FORCE_REPLAN_DECREASE_MPS
            or requested > previous + FORCE_REPLAN_INCREASE_MPS
        )

    def _first_force_replan_trajectory_locked(self, active_interval):
        """Return post-action planner provenance for diagnostics only."""

        if (
            active_interval.action_event is None
            or active_interval.policy_trajectory is None
        ):
            return None
        action_stamp = active_interval.action_event.stamp_sec
        baseline = active_interval.policy_trajectory.identity_key
        for receive_sec, identity in self._trajectory_history:
            if (
                receive_sec + 1.0e-9 >= action_stamp
                and identity.start_time_sec > action_stamp + 1.0e-9
                and identity.identity_key > baseline
            ):
                return identity
        return None

    def _latest_strictly_causal_observation_locked(self, decision_ros_time_sec):
        decision = float(decision_ros_time_sec)
        for packet in reversed(self._observations):
            if packet.state.provenance.observation_receive_sec <= decision:
                return packet
        return None

    @staticmethod
    def _action_trajectory_lifetime_eligibility_locked(
        state_t, decision_ros_time_sec, minimum_lifetime_sec
    ):
        """Gate acceptance on one real next-state-capable official trajectory.

        The selected trajectory is the official trajectory that actually
        formed ``state_t``.  Its end is therefore causal and does not guess
        the lifetime of a newer trajectory that may not yet be active at the
        Observation C source stamp.
        """

        minimum = float(minimum_lifetime_sec)
        decision = float(decision_ros_time_sec)
        if (
            not math.isfinite(minimum)
            or minimum < 0.0
            or not math.isfinite(decision)
            or decision <= 0.0
        ):
            raise ValueError("action trajectory lifetime gate is invalid")
        if minimum == 0.0:
            return True, "not_required"

        telemetry = state_t.telemetry
        provenance = state_t.state.provenance.official_trajectory
        try:
            selected_id = int(telemetry["selected_trajectory_id"])
            selected_secs = int(
                telemetry["selected_trajectory_start_stamp_secs"]
            )
            selected_nsecs = int(
                telemetry["selected_trajectory_start_stamp_nsecs"]
            )
            selected_start = float(
                telemetry["selected_trajectory_start_stamp_sec"]
            )
            selected_end = float(
                telemetry["selected_trajectory_end_stamp_sec"]
            )
        except (KeyError, TypeError, ValueError):
            return False, "official_trajectory_lifetime_unavailable"
        values = (selected_start, selected_end)
        if (
            selected_id <= 0
            or not all(math.isfinite(value) for value in values)
            or selected_end <= selected_start
        ):
            return False, "official_trajectory_lifetime_unavailable"
        if (
            selected_id != provenance.trajectory_id
            or (selected_secs, selected_nsecs, selected_id)
            != provenance.identity_key
        ):
            return False, "official_trajectory_lifetime_provenance"
        required_end = (
            decision + minimum + TRAJECTORY_END_FUTURE_TOLERANCE_SEC
        )
        if selected_end + 1.0e-12 < required_end:
            return False, "official_trajectory_lifetime_insufficient"
        return True, "eligible"

    def _begin_active_action_interval(
        self, requested_v_max, scheduled_sec, step_index, state_t,
        actor_provider_wall_sec=0.0, policy_tick_index=None,
        minimum_trajectory_lifetime_sec=0.0,
    ):
        requested = self._positive_finite(requested_v_max, "requested_v_max")
        publish_stamp = (
            None if self._ros_stamp_clock is None else self._ros_stamp_clock()
        )
        publish_sec = (
            self._now_ros()
            if publish_stamp is None
            else float(publish_stamp.to_sec())
        )
        with self._condition:
            if (
                state_t.state.provenance.observation_receive_sec
                > publish_sec
            ):
                raise AstraDroneStepCausalityError(
                    "state_t was received after the policy request"
                )
        request_id = int(step_index) + 1
        if request_id <= 0 or request_id > (2**64 - 1):
            raise AstraDroneStepCausalityError("request_id is outside uint64 range")
        token = ActionRequestToken(
            episode_id=self._run_episode.episode_id,
            step_index=int(step_index),
            request_id=request_id,
            publish_ros_time_sec=publish_sec,
            requested_v_max=requested,
            publish_ros_secs=(
                None if publish_stamp is None else int(publish_stamp.secs)
            ),
            publish_ros_nsecs=(
                None if publish_stamp is None else int(publish_stamp.nsecs)
            ),
        )
        with self._condition:
            if self._terminal_snapshot().terminated or self._external_truncated:
                return None
            eligible, ineligible_reason = (
                self._action_trajectory_lifetime_eligibility_locked(
                    state_t,
                    publish_sec,
                    minimum_trajectory_lifetime_sec,
                )
            )
            if not eligible:
                raise AstraDroneActionIneligible(ineligible_reason)
            # This lock is also held by terminal callbacks.  Publishing here is
            # the action/terminal linearization point: a latched terminal wins;
            # otherwise this one request is formally before that terminal.
            request_publish_wall = self._wall_clock()
            self._publish_requested_v_max(token)
            return ActiveActionInterval(
                episode_id=self._run_episode.episode_id,
                step_index=int(step_index),
                scheduled_ros_time_sec=float(scheduled_sec),
                policy_tick_index=(
                    int(step_index)
                    if policy_tick_index is None
                    else int(policy_tick_index)
                ),
                request=token,
                state_t=state_t,
                created_wall_time_sec=request_publish_wall,
                actor_provider_wall_sec=float(actor_provider_wall_sec),
                request_publish_wall_time_sec=request_publish_wall,
            )

    def _build_transition_record(
        self, active_interval, state_t_plus_1, terminal, truncation_reason=""
    ) -> Dict[str, object]:
        if (
            active_interval.action_event is None
            or active_interval.applied_event is None
            or active_interval.policy_trajectory is None
        ):
            raise AstraDroneStepCausalityError(
                "active action interval is incomplete at transition construction"
            )
        action_event = active_interval.action_event
        applied_event = active_interval.applied_event
        state_t = active_interval.state_t
        action = AppliedSpeedAction(
            requested_v_max=action_event.requested_v_max,
            requested_stamp_sec=action_event.stamp_sec,
            filtered_v_max=action_event.filtered_v_max,
            filtered_stamp_sec=action_event.stamp_sec,
            applied_v_max=applied_event.applied_v_max,
            applied_stamp_sec=applied_event.receive_sec,
            latest_official_trajectory=active_interval.policy_trajectory,
        )
        truncated = bool(truncation_reason) and not terminal.terminated
        boundary_reason = (
            terminal.terminal_reason
            if terminal.terminated
            else (str(truncation_reason) if truncated else "")
        )
        transition = SacTransitionV1(
            state_t=state_t.state,
            action_t=action,
            state_t_plus_1=state_t_plus_1.state,
            reward_context=ProgressRewardContext(
                provenance=self._run_episode,
                state_t=state_t.reward_context,
                state_t_plus_1=state_t_plus_1.reward_context,
            ),
            reward=None,
            reward_defined=False,
            terminated=terminal.terminated,
            # Frozen v1.3 Replay payload keeps legal Episode truncation in the
            # outer transition record; Reward continues to evaluate only a
            # valid, non-truncated causal environment step.
            truncated=False,
            terminal_reason=(
                terminal.terminal_reason if terminal.terminated else ""
            ),
            dangerous_terminal=terminal.dangerous_terminal,
        )
        reward_input = reward_input_from_signals(
            nearest_obstacle_distance_m=state_t.reward_observation[
                "nearest_obstacle_distance_m"
            ],
            known_obstacle_bin_fraction=state_t.reward_observation[
                "known_obstacle_bin_fraction"
            ],
            unknown_bin_count=state_t.reward_observation["unknown_bin_count"],
            lidar_bin_count=LIDAR_BINS,
            applied_v_max_mps=action.applied_v_max,
            previous_applied_v_max_mps=state_t.state.previous_applied_v_max,
            actual_speed_mps=actual_speed_mps_from_body_velocity(
                state_t.state.actual_velocity_body
            ),
            tracking_error_m=tracking_error_m_from_body_error(
                state_t.state.tracking_error_body
            ),
            dangerous_terminal=transition.dangerous_terminal,
            terminated=transition.terminated,
            observation_valid=True,
            same_episode=True,
            truncated=False,
        )
        evaluation = self._reward.evaluate(reward_input)
        if not evaluation.reward_valid:
            raise AstraDroneStepCausalityError(
                "Stage 1 reward rejected causal step: {}".format(
                    evaluation.invalid_reason
                )
            )
        transition = transition.with_defined_reward(evaluation)
        request_wall = float(
            active_interval.request_publish_wall_time_sec
            if active_interval.request_publish_wall_time_sec is not None
            else active_interval.created_wall_time_sec
        )
        action_wall = float(
            active_interval.action_ack_wall_time_sec
            if active_interval.action_ack_wall_time_sec is not None
            else action_event.receive_wall_time_sec
        )
        applied_wall = float(
            active_interval.applied_ack_wall_time_sec
            if active_interval.applied_ack_wall_time_sec is not None
            else applied_event.receive_wall_time_sec
        )
        planner_diagnostic_wall = float(
            active_interval.planner_diagnostic_ready_wall_time_sec
            if active_interval.planner_diagnostic_ready_wall_time_sec is not None
            else action_wall
        )
        causal_gate_wall = applied_wall
        completed_wall = float(
            active_interval.completed_wall_time_sec
            if active_interval.completed_wall_time_sec is not None
            else self._wall_clock()
        )

        def wall_delta(later, earlier):
            return max(0.0, float(later) - float(earlier))

        return {
            "episode_id": active_interval.episode_id,
            "step_index": active_interval.step_index,
            "request_id": active_interval.request.request_id,
            "scheduled_ros_time_sec": active_interval.scheduled_ros_time_sec,
            "policy_tick_index": active_interval.policy_tick_index,
            "transition": transition.to_record(),
            "reward": transition.reward,
            "terminated": transition.terminated,
            "truncated": truncated,
            "transition_truncated": transition.truncated,
            "terminal_reason": boundary_reason,
            "timing": {
                "timing_contract": "absolute_action_interval_async_replay_v2",
                "scheduled_ros_time_sec": active_interval.scheduled_ros_time_sec,
                "policy_tick_index": active_interval.policy_tick_index,
                "close_policy_tick_index": active_interval.close_policy_tick_index,
                "close_scheduled_ros_time_sec": (
                    active_interval.close_scheduled_ros_time_sec
                ),
                "close_trigger": active_interval.close_trigger,
                "requested_publish_ros_time_sec": active_interval.request.publish_ros_time_sec,
                "request_id": active_interval.request.request_id,
                "action_request_id": action_event.request_id,
                "applied_request_id": applied_event.request_id,
                "request_deadline_error_sec": (
                    active_interval.request.publish_ros_time_sec
                    - active_interval.scheduled_ros_time_sec
                ),
                "action_stamp_sec": action_event.stamp_sec,
                "action_receive_ros_time_sec": action_event.receive_sec,
                "applied_receive_ros_time_sec": applied_event.receive_sec,
                "applied_stamp_sec": applied_event.stamp_sec,
                "force_replan_expected": bool(
                    active_interval.force_replan_expected
                ),
                "policy_trajectory_role": "POLICY_INPUT",
                "planner_provenance_role": "DIAGNOSTIC",
                "planner_diagnostic_status": (
                    active_interval.planner_diagnostic_status
                ),
                "policy_trajectory_id": (
                    None
                    if active_interval.policy_trajectory is None
                    else active_interval.policy_trajectory.trajectory_id
                ),
                "policy_trajectory_start_time_sec": (
                    None
                    if active_interval.policy_trajectory is None
                    else active_interval.policy_trajectory.start_time_sec
                ),
                "policy_trajectory_start_time_secs": (
                    None
                    if active_interval.policy_trajectory is None
                    else active_interval.policy_trajectory.start_time_secs
                ),
                "policy_trajectory_start_time_nsecs": (
                    None
                    if active_interval.policy_trajectory is None
                    else active_interval.policy_trajectory.start_time_nsecs
                ),
                "planner_diagnostic_trajectory_id": (
                    None
                    if active_interval.planner_diagnostic_trajectory is None
                    else active_interval.planner_diagnostic_trajectory.trajectory_id
                ),
                "planner_diagnostic_trajectory_start_time_sec": (
                    None
                    if active_interval.planner_diagnostic_trajectory is None
                    else active_interval.planner_diagnostic_trajectory.start_time_sec
                ),
                "action_period_start_ros_time_sec": (
                    active_interval.request.publish_ros_time_sec
                ),
                "action_period_end_ros_time_sec": (
                    active_interval.close_scheduled_ros_time_sec
                ),
                "actual_action_period_ros_sec": (
                    None
                    if active_interval.close_scheduled_ros_time_sec is None
                    else active_interval.close_scheduled_ros_time_sec
                    - active_interval.scheduled_ros_time_sec
                ),
                "state_t_stamp_sec": state_t.state.provenance.observation_stamp_sec,
                "state_t_receive_sec": state_t.state.provenance.observation_receive_sec,
                "state_t_plus_1_stamp_sec": (
                    state_t_plus_1.state.provenance.observation_stamp_sec
                ),
                "state_t_plus_1_receive_sec": (
                    state_t_plus_1.state.provenance.observation_receive_sec
                ),
                "request_publish_wall_time_sec": request_wall,
                "action_ack_wall_time_sec": action_wall,
                "applied_ack_wall_time_sec": applied_wall,
                "planner_diagnostic_ready_wall_time_sec": planner_diagnostic_wall,
                "state_t_plus_1_receive_wall_time_sec": float(
                    state_t_plus_1.receive_wall_time_sec
                ),
                "transition_completed_wall_time_sec": completed_wall,
                "state_t_telemetry": dict(state_t.telemetry),
                "state_t_plus_1_telemetry": dict(state_t_plus_1.telemetry),
                "phase_wall_sec": {
                    "actor_provider_total": max(
                        0.0, float(active_interval.actor_provider_wall_sec)
                    ),
                    "action_ack_wait": wall_delta(action_wall, request_wall),
                    "applied_ack_wait": wall_delta(applied_wall, action_wall),
                    "force_replan": (
                        wall_delta(planner_diagnostic_wall, action_wall)
                        if active_interval.force_replan_expected else 0.0
                    ),
                    "planner_diagnostic_ready_after_applied": wall_delta(
                        planner_diagnostic_wall, applied_wall
                    ),
                    "policy_grid_wait_after_causal_gate": wall_delta(
                        completed_wall, causal_gate_wall
                    ),
                    "snapshot_available_after_causal_gate": wall_delta(
                        state_t_plus_1.receive_wall_time_sec, causal_gate_wall
                    ),
                    "request_to_transition_total": wall_delta(
                        completed_wall, request_wall
                    ),
                    "decision_to_transition_total": (
                        max(0.0, float(active_interval.actor_provider_wall_sec))
                        + wall_delta(completed_wall, request_wall)
                    ),
                },
            },
        }

    def _advance_active_action_interval_locked(
        self,
        active_interval,
        terminal_floor_sec=None,
        next_state_candidate=None,
        close_policy_tick_index=None,
        close_scheduled_ros_time_sec=None,
        close_trigger="",
        truncation_reason="",
    ):
        """Form the unique active ACTION INTERVAL transition exactly once."""

        if self._identity_violation:
            raise AstraDroneStepCausalityError(self._identity_violation)
        if active_interval is None or active_interval.transition_record is not None:
            return None
        key = (active_interval.request.episode_id, active_interval.request.request_id)
        if active_interval.action_event is None:
            event = self._actions.get(key)
            if event is not None:
                if (
                    event.step_index != active_interval.request.step_index
                    or event.source_mode != "mock"
                    or abs(event.requested_v_max - active_interval.request.requested_v_max)
                    > self.config.requested_match_tolerance_mps
                ):
                    raise AstraDroneStepCausalityError(
                        "action identity matched but action provenance/value did not"
                    )
                if (
                    event.stamp_sec + 1.0e-9
                    < active_interval.request.publish_ros_time_sec
                    or event.receive_sec + 1.0e-9 < event.stamp_sec
                    or event.stamp_sec + 1.0e-9
                    < active_interval.state_t.state.provenance.observation_receive_sec
                ):
                    raise AstraDroneStepCausalityError(
                        "action identity matched but timestamp order is invalid: "
                        "episode={} step={} request={} request_publish={:.9f} "
                        "action_stamp={:.9f} action_receive={:.9f} "
                        "state_receive={:.9f}".format(
                            active_interval.request.episode_id,
                            active_interval.request.step_index,
                            active_interval.request.request_id,
                            active_interval.request.publish_ros_time_sec,
                            event.stamp_sec,
                            event.receive_sec,
                            active_interval.state_t.state.provenance.observation_receive_sec,
                        )
                    )
                active_interval.action_event = self._actions.pop(key)
                active_interval.action_ack_wall_time_sec = (
                    active_interval.action_event.receive_wall_time_sec
                )
                active_interval.policy_trajectory = (
                    active_interval.state_t.state.provenance.official_trajectory
                )
                active_interval.force_replan_expected = self._requires_force_replan(
                    active_interval.state_t.state.previous_applied_v_max,
                    event.filtered_v_max,
                )
                # The trajectory selected in state_t remains the causal policy
                # input and the action record's official trajectory identity.
                # A later force-replan publication is action-effect diagnostic,
                # not a minimum condition for forming this interval.
                active_interval.planner_diagnostic_status = (
                    "pending" if active_interval.force_replan_expected
                    else "not_expected"
                )
                active_interval.planner_diagnostic_ready_wall_time_sec = (
                    active_interval.action_ack_wall_time_sec
                )
        if (
            active_interval.action_event is not None
            and active_interval.force_replan_expected
            and active_interval.planner_diagnostic_trajectory is None
        ):
            candidate = self._first_force_replan_trajectory_locked(active_interval)
            if candidate is not None:
                active_interval.planner_diagnostic_trajectory = candidate
                active_interval.planner_diagnostic_status = "observed"
                active_interval.planner_diagnostic_ready_wall_time_sec = (
                    self._trajectory_receive_wall.get(
                        candidate.identity_key,
                        self._wall_clock(),
                    )
                )
        if active_interval.action_event is not None and active_interval.applied_event is None:
            action = active_interval.action_event
            event = self._applied.get(key)
            if event is not None:
                if event.step_index != active_interval.request.step_index:
                    raise AstraDroneStepCausalityError(
                        "applied acknowledgement identity disagrees with step_index"
                    )
                if (
                    event.stamp_sec <= action.stamp_sec
                    or event.receive_sec + 1.0e-9 < event.stamp_sec
                ):
                    raise AstraDroneStepCausalityError(
                        "applied acknowledgement timestamp order is invalid"
                    )
                if (
                    abs(event.applied_v_max - action.filtered_v_max)
                    > self.config.applied_pair_tolerance_mps
                ):
                    raise AstraDroneStepCausalityError(
                        "applied acknowledgement value disagrees with filtered action"
                    )
                active_interval.applied_event = self._applied.pop(key)
                active_interval.applied_ack_wall_time_sec = (
                    active_interval.applied_event.receive_wall_time_sec
                )
        if active_interval.applied_event is None:
            return None
        candidates = (
            () if next_state_candidate is None else (next_state_candidate,)
        )
        for packet in candidates:
            if (
                terminal_floor_sec is not None
                and packet.state.provenance.observation_receive_sec + 1.0e-9
                < float(terminal_floor_sec)
            ):
                continue
            if is_causal_next_policy_state(
                active_interval.state_t.state,
                packet.state,
                active_interval.applied_event.receive_sec,
            ):
                terminal = self._terminal_snapshot(
                    packet.state.provenance.observation_receive_sec
                )
                active_interval.close_policy_tick_index = close_policy_tick_index
                active_interval.close_scheduled_ros_time_sec = (
                    close_scheduled_ros_time_sec
                )
                active_interval.close_trigger = str(close_trigger)
                active_interval.completed_wall_time_sec = self._wall_clock()
                record = self._build_transition_record(
                    active_interval,
                    packet,
                    terminal,
                    truncation_reason=truncation_reason,
                )
                active_interval.transition_record = ImmutableTransitionRecord(record)
                active_interval.reward = float(record["reward"])
                active_interval.terminated = bool(record["terminated"])
                active_interval.terminal_reason = str(record["terminal_reason"])
                active_interval.next_state = packet
                return active_interval
        return None

    def _active_action_interval_timeout(self, active_interval):
        elapsed = self._wall_clock() - active_interval.created_wall_time_sec
        if active_interval.action_event is None and elapsed > self.config.action_timeout_sec:
            return "action_identity_ack", self.config.action_timeout_sec
        if (
            active_interval.action_event is not None
            and active_interval.applied_event is None
            and elapsed
            > self.config.action_timeout_sec + self.config.applied_timeout_sec
        ):
            return "applied_identity_ack", self.config.applied_timeout_sec
        total = (
            self.config.action_timeout_sec
            + self.config.applied_timeout_sec
            + self.config.next_observation_timeout_sec
        )
        if active_interval.applied_event is not None and elapsed > total:
            return "state_t_plus_1", self.config.next_observation_timeout_sec
        return None

    def run_episode(
        self,
        action_provider,
        config=None,
        transition_submitter=None,
        action_acceptance_recorder=None,
    ):
        """Run the sole Episode v0.1 scheduler over one continuous mission."""

        if not callable(action_provider):
            raise ValueError("action_provider must be callable")
        if transition_submitter is not None and not callable(transition_submitter):
            raise ValueError("transition_submitter must be callable")
        if (
            action_acceptance_recorder is not None
            and not callable(action_acceptance_recorder)
        ):
            raise ValueError("action_acceptance_recorder must be callable")
        episode_config = config or AstraDroneEpisodeConfig()
        if not isinstance(episode_config, AstraDroneEpisodeConfig):
            raise TypeError("config must be AstraDroneEpisodeConfig")
        with self._episode_lock:
            start_wait_deadline = (
                self._wall_clock() + episode_config.start_timeout_sec
            )
            with self._condition:
                while not self._episode_ready_locked(
                    episode_config.minimum_active_speed_mps
                ):
                    if self._shutdown_requested:
                        raise AstraDroneEpisodeError(
                            "shutdown requested before Episode start"
                        )
                    terminal = self._terminal_snapshot()
                    if terminal.terminated:
                        raise AstraDroneEpisodeError(
                            "terminal before Episode start: {}".format(
                                terminal.terminal_reason
                            )
                        )
                    remaining = start_wait_deadline - self._wall_clock()
                    if remaining <= 0.0:
                        raise AstraDroneStepTimeout(
                            "episode_start", episode_config.start_timeout_sec
                        )
                    self._condition.wait(min(remaining, 0.05))

            episode_start = self._now_ros()
            previous_ros_time = episode_start
            next_schedule = episode_start
            active_action_interval = None
            completed_records: List[ImmutableTransitionRecord] = []
            actor_action_count = 0
            decision_tick_count = 0
            decision_tick_misses = 0
            observation_wait_count = 0
            deadline_misses = 0
            timeouts = 0
            causal_mismatches = 0
            maximum_active_action_intervals = 0
            episode_return = 0.0
            stop_scheduling = False
            stop_reason = ""
            terminal_result = None
            observed_terminal = None
            scheduler_state = "ACTIVE"
            scheduler_state_history = ["IDLE", "ACTIVE"]
            next_policy_tick_index = 0
            tick_miss_reasons: Dict[str, int] = {}
            completion_deadline = None
            status = "running"
            error = ""
            invalid_start = self._invalid_observations
            valid_start = self._valid_observations
            accepted_actions_start = self._accepted_action_count
            accepted_applied_start = self._accepted_applied_count
            duplicate_actions_start = self._duplicate_action_count
            duplicate_applied_start = self._duplicate_applied_count

            def count_tick_miss(reason, count=1):
                nonlocal decision_tick_misses, observation_wait_count
                amount = int(count)
                decision_tick_misses += amount
                key = str(reason)
                tick_miss_reasons[key] = tick_miss_reasons.get(key, 0) + amount
                if key in (
                    "causal_snapshot_unavailable",
                    "initial_snapshot_unavailable",
                ):
                    observation_wait_count += amount

            def attempt_action(
                state, scheduled_sec, step_index, policy_tick_index
            ):
                actor_started = self._wall_clock()
                requested = action_provider(state.state, step_index)
                actor_provider_wall_sec = self._wall_clock() - actor_started
                ineligible_reason = ""
                try:
                    active_interval = self._begin_active_action_interval(
                        requested,
                        scheduled_sec,
                        step_index,
                        state,
                        actor_provider_wall_sec=actor_provider_wall_sec,
                        policy_tick_index=policy_tick_index,
                        minimum_trajectory_lifetime_sec=(
                            episode_config.policy_period_sec
                        ),
                    )
                except AstraDroneActionIneligible as skipped:
                    active_interval = None
                    ineligible_reason = skipped.reason
                if action_acceptance_recorder is not None:
                    # _begin_active_action_interval() decides under the terminal
                    # condition lock and publishes before returning an interval.
                    # Actor ownership transfers only after that linearization.
                    action_acceptance_recorder(
                        int(step_index), active_interval is not None
                    )
                return active_interval, ineligible_reason

            def submit_transition(active_interval, truncation_reason=""):
                nonlocal episode_return, terminal_result
                record = active_interval.transition_record
                expected_truncated = bool(
                    truncation_reason and not active_interval.terminated
                )
                if expected_truncated != bool(record["truncated"]):
                    raise AstraDroneStepCausalityError(
                        "formed transition truncation disagrees with boundary"
                    )
                completed_records.append(record)
                episode_return += float(active_interval.reward)
                if transition_submitter is not None:
                    transition_submitter(record)
                if active_interval.terminated and terminal_result is None:
                    terminal_result = active_interval

            try:
                while True:
                    if self._shutdown_requested:
                        status = "interrupted"
                        error = "shutdown requested"
                        break
                    now_ros = self._now_ros()
                    if now_ros + 1.0e-12 < previous_ros_time:
                        raise AstraDroneStepCausalityError(
                            "ROS/simulation time moved backwards during Episode"
                        )
                    previous_ros_time = now_ros

                    terminal_now = self._terminal_snapshot()
                    if terminal_now.terminated:
                        observed_terminal = terminal_now
                        if scheduler_state == "ACTIVE":
                            scheduler_state = "TERMINATING"
                            scheduler_state_history.append(scheduler_state)
                        stop_scheduling = True
                        stop_reason = terminal_now.terminal_reason
                        if completion_deadline is None:
                            completion_deadline = (
                                self._wall_clock()
                                + episode_config.completion_timeout_sec
                            )

                    with self._condition:
                        external_truncated = self._external_truncated
                        external_truncated_reason = (
                            self._external_truncated_reason
                        )
                    if external_truncated and not stop_scheduling:
                        stop_scheduling = True
                        stop_reason = external_truncated_reason
                        scheduler_state = "TERMINATING"
                        scheduler_state_history.append(scheduler_state)
                        completion_deadline = (
                            self._wall_clock()
                            + episode_config.completion_timeout_sec
                        )

                    # ACK/provenance may progress continuously, but a normal
                    # transition may close only on an absolute policy tick.
                    if active_action_interval is not None:
                        with self._condition:
                            self._advance_active_action_interval_locked(
                                active_action_interval
                            )

                    if stop_scheduling and active_action_interval is not None:
                        # A true environment terminal must be represented by a
                        # packet received at or after its latch.  An external
                        # time-limit truncation is only an Episode boundary:
                        # its final real causal next state may already have
                        # arrived before the coordinator callback.  Requiring
                        # a post-truncation Observation C incorrectly deadlocks
                        # when the final EGO trajectory has ended and C
                        # correctly reports trajectory_unavailable.
                        terminal_floor = (
                            observed_terminal.event_ros_time_sec
                            if observed_terminal is not None
                            else None
                        )
                        close_trigger = (
                            "terminal"
                            if observed_terminal is not None
                            else "external_truncation"
                        )
                        decision_ros_time = self._now_ros()
                        with self._condition:
                            state = self._latest_strictly_causal_observation_locked(
                                decision_ros_time
                            )
                            newly_completed = self._advance_active_action_interval_locked(
                                active_action_interval,
                                terminal_floor_sec=terminal_floor,
                                next_state_candidate=state,
                                close_policy_tick_index=None,
                                close_scheduled_ros_time_sec=decision_ros_time,
                                close_trigger=close_trigger,
                                truncation_reason=(
                                    external_truncated_reason
                                    if external_truncated else ""
                                ),
                            )
                        if newly_completed is not None:
                            submit_transition(
                                newly_completed,
                                external_truncated_reason
                                if external_truncated else "",
                            )
                            active_action_interval = None

                    if active_action_interval is not None:
                        timeout = self._active_action_interval_timeout(
                            active_action_interval
                        )
                        if timeout is not None:
                            phase, timeout_sec = timeout
                            timeouts += 1
                            raise AstraDroneStepTimeout(phase, timeout_sec)

                    if stop_scheduling and active_action_interval is None:
                        status = "completed"
                        scheduler_state = "CLOSED"
                        scheduler_state_history.append(scheduler_state)
                        break
                    if (
                        stop_scheduling
                        and completion_deadline is not None
                        and self._wall_clock() >= completion_deadline
                    ):
                        timeouts += 1
                        raise AstraDroneStepTimeout(
                            "episode_completion",
                            episode_config.completion_timeout_sec,
                        )

                    if not stop_scheduling and now_ros + 1.0e-12 >= next_schedule:
                        due_ticks = 1 + int(
                            math.floor(
                                max(0.0, now_ros - next_schedule)
                                / episode_config.policy_period_sec
                            )
                        )
                        decision_tick_count += due_ticks
                        current_tick_index = (
                            next_policy_tick_index + due_ticks - 1
                        )
                        next_policy_tick_index += due_ticks
                        scheduled_boundary = next_schedule + (
                            (due_ticks - 1) * episode_config.policy_period_sec
                        )
                        lateness = max(0.0, now_ros - scheduled_boundary)
                        next_schedule += due_ticks * episode_config.policy_period_sec
                        if due_ticks > 1:
                            count_tick_miss("scheduler_overrun", due_ticks - 1)
                        if lateness > episode_config.deadline_tolerance_sec:
                            deadline_misses += 1
                        decision_ros_time = self._now_ros()
                        with self._condition:
                            state = self._latest_strictly_causal_observation_locked(
                                decision_ros_time
                            )

                        truncation_reason = ""
                        if (
                            episode_config.max_steps is not None
                            and actor_action_count >= episode_config.max_steps
                        ):
                            truncation_reason = str(
                                episode_config.max_steps_reason
                            )
                        elif (
                            episode_config.max_duration_sec is not None
                            and scheduled_boundary + 1.0e-12
                            >= episode_start + episode_config.max_duration_sec
                        ):
                            truncation_reason = "max_episode_duration"

                        if active_action_interval is not None:
                            with self._condition:
                                newly_completed = (
                                    self._advance_active_action_interval_locked(
                                        active_action_interval,
                                        next_state_candidate=state,
                                        close_policy_tick_index=(
                                            current_tick_index
                                        ),
                                        close_scheduled_ros_time_sec=(
                                            scheduled_boundary
                                        ),
                                        close_trigger="policy_tick",
                                        truncation_reason=truncation_reason,
                                    )
                                )
                            if newly_completed is None:
                                if active_action_interval.action_event is None:
                                    miss_reason = "action_ack_unavailable"
                                elif active_action_interval.applied_event is None:
                                    miss_reason = "applied_ack_unavailable"
                                else:
                                    miss_reason = "causal_snapshot_unavailable"
                                count_tick_miss(miss_reason)
                            else:
                                submit_transition(
                                    newly_completed, truncation_reason
                                )
                                state = newly_completed.next_state
                                active_action_interval = None
                                if truncation_reason:
                                    stop_scheduling = True
                                    stop_reason = truncation_reason
                                    scheduler_state = "TERMINATING"
                                    scheduler_state_history.append(
                                        scheduler_state
                                    )
                                    completion_deadline = (
                                        self._wall_clock()
                                        + episode_config.completion_timeout_sec
                                    )

                        # A terminal racing close/commit wins before Actor.
                        terminal_now = self._terminal_snapshot()
                        if terminal_now.terminated and not stop_scheduling:
                            observed_terminal = terminal_now
                            stop_scheduling = True
                            stop_reason = terminal_now.terminal_reason
                            scheduler_state = "TERMINATING"
                            scheduler_state_history.append(scheduler_state)
                            completion_deadline = (
                                self._wall_clock()
                                + episode_config.completion_timeout_sec
                            )

                        if (
                            not stop_scheduling
                            and active_action_interval is None
                        ):
                            if state is None:
                                count_tick_miss("initial_snapshot_unavailable")
                            else:
                                with self._condition:
                                    eligible, ineligible_reason = (
                                        self._action_trajectory_lifetime_eligibility_locked(
                                            state,
                                            decision_ros_time,
                                            episode_config.policy_period_sec,
                                        )
                                    )
                                if not eligible:
                                    count_tick_miss(ineligible_reason)
                                else:
                                    active_action_interval, ineligible_reason = attempt_action(
                                        state,
                                        scheduled_boundary,
                                        actor_action_count,
                                        current_tick_index,
                                    )
                                    if active_action_interval is not None:
                                        actor_action_count += 1
                                        maximum_active_action_intervals = 1
                                    elif ineligible_reason:
                                        count_tick_miss(ineligible_reason)
                                    else:
                                        terminal_now = self._terminal_snapshot()
                                        if terminal_now.terminated:
                                            observed_terminal = terminal_now
                                            stop_reason = (
                                                terminal_now.terminal_reason
                                            )
                                        else:
                                            with self._condition:
                                                external_now = (
                                                    self._external_truncated
                                                )
                                                external_reason_now = (
                                                    self._external_truncated_reason
                                                )
                                            if not external_now:
                                                raise AstraDroneStepCausalityError(
                                                    "action rejected without terminal or truncation"
                                                )
                                            stop_reason = external_reason_now
                                        stop_scheduling = True
                                        scheduler_state = "TERMINATING"
                                        scheduler_state_history.append(
                                            scheduler_state
                                        )
                                        completion_deadline = (
                                            self._wall_clock()
                                            + episode_config.completion_timeout_sec
                                        )

                    with self._condition:
                        self._condition.wait(0.002)
            except AstraDroneStepTimeout as caught:
                status = "timeout"
                error = str(caught)
            except (AstraDroneStepCausalityError, ValueError) as caught:
                causal_mismatches += 1
                status = "causal_mismatch"
                error = str(caught)

            episode_end = self._now_ros()
            completed_records.sort(key=lambda record: record["step_index"])
            episode_return = sum(
                float(record["reward"]) for record in completed_records
            )
            terminated = bool(
                terminal_result is not None or observed_terminal is not None
            )
            truncated = bool(
                status == "completed"
                and not terminated
                and (
                    self._external_truncated
                    or stop_reason
                    in (
                        "max_episode_duration",
                        "max_episode_steps",
                    )
                )
            )
            terminal_reason = (
                terminal_result.terminal_reason
                if terminal_result is not None
                else (
                    observed_terminal.terminal_reason
                    if observed_terminal is not None
                    else stop_reason
                )
            )
            boundary = EpisodeBoundary(
                episode_id=self._run_episode.episode_id,
                step_index=(
                    int(completed_records[-1]["step_index"])
                    if completed_records
                    else -1
                ),
                episode_start_time=episode_start,
                episode_elapsed=max(0.0, episode_end - episode_start),
                episode_return=episode_return,
                terminated=terminated,
                truncated=truncated,
                terminal_reason=terminal_reason,
            )
            # Result rows are presentation copies.  The records handed to the
            # ordered writer remain deeply immutable; the retained terminal
            # boundary special case is an Episode overlay, not a mutation of a
            # formed transition waiting for persistence.
            result_records = [record.to_mutable() for record in completed_records]
            if result_records:
                result_records[-1]["truncated"] = boundary.truncated
                if (
                    boundary.terminated
                    and not any(
                        bool(record.get("terminated", False))
                        for record in result_records
                    )
                ):
                    # A terminal may win after the previous causal transition
                    # committed but before the next candidate is accepted.  No
                    # new action/step exists; promote only the last real row to
                    # the Episode boundary, as already done for truncation.
                    result_records[-1]["terminated"] = True
                    result_records[-1]["terminal_reason"] = (
                        boundary.terminal_reason
                    )
                result_records[-1]["episode_boundary"] = boundary.to_record()
            action_stamps = [
                float(record["timing"]["action_stamp_sec"])
                for record in result_records
            ]
            action_intervals = [
                later - earlier
                for earlier, later in zip(action_stamps, action_stamps[1:])
            ]
            sorted_intervals = sorted(action_intervals)
            action_stamps_strictly_monotonic = all(
                later > earlier
                for earlier, later in zip(action_stamps, action_stamps[1:])
            )

            def percentile(values, fraction):
                if not values:
                    return None
                position = (len(values) - 1) * float(fraction)
                lower = int(math.floor(position))
                upper = int(math.ceil(position))
                if lower == upper:
                    return values[lower]
                weight = position - lower
                return values[lower] * (1.0 - weight) + values[upper] * weight

            valid_observations = self._valid_observations - valid_start
            invalid_observations = self._invalid_observations - invalid_start
            observation_total = valid_observations + invalid_observations
            finite_rewards = sum(
                1
                for record in result_records
                if math.isfinite(float(record["reward"]))
            )
            request_ids = [
                int(record["request_id"]) for record in completed_records
            ]
            identity_chain_strict = all(
                int(record["timing"]["request_id"])
                == int(record["timing"]["action_request_id"])
                == int(record["timing"]["applied_request_id"])
                for record in completed_records
            ) and len(request_ids) == len(set(request_ids))
            return {
                "version": EPISODE_VERSION,
                "status": status,
                "error": error,
                "episode": boundary.to_record(),
                "steps": result_records,
                "metrics": {
                    "nominal_decision_period_sec": (
                        episode_config.policy_period_sec
                    ),
                    "decision_tick_count": decision_tick_count,
                    "policy_tick_count": next_policy_tick_index,
                    "policy_tick_miss_reasons": dict(tick_miss_reasons),
                    "trajectory_lifetime_skip_count": sum(
                        int(count)
                        for reason, count in tick_miss_reasons.items()
                        if str(reason).startswith(
                            "official_trajectory_lifetime_"
                        )
                    ),
                    "actor_action_count": actor_action_count,
                    "scheduled_action_count": actor_action_count,
                    "decision_tick_miss_count": decision_tick_misses,
                    "observation_wait_count": observation_wait_count,
                    "accepted_action_count": (
                        self._accepted_action_count - accepted_actions_start
                    ),
                    "accepted_applied_ack_count": (
                        self._accepted_applied_count - accepted_applied_start
                    ),
                    "transition_count": len(result_records),
                    "identity_chain_strict_1_to_1": identity_chain_strict,
                    "closed_request_ids_strictly_monotonic": all(
                        later > earlier
                        for earlier, later in zip(request_ids, request_ids[1:])
                    ),
                    "duplicate_action_count": (
                        self._duplicate_action_count - duplicate_actions_start
                    ),
                    "duplicate_applied_ack_count": (
                        self._duplicate_applied_count - duplicate_applied_start
                    ),
                    "transition_success_rate": (
                        float(len(result_records)) / actor_action_count
                        if actor_action_count
                        else 0.0
                    ),
                    "deadline_miss_count": deadline_misses,
                    "deadline_miss_rate": (
                        float(deadline_misses) / decision_tick_count
                        if decision_tick_count
                        else 0.0
                    ),
                    "timeout_count": timeouts,
                    "causal_mismatch_count": causal_mismatches,
                    "maximum_active_action_interval_count": (
                        maximum_active_action_intervals
                    ),
                    "active_action_interval_count_at_closure": int(
                        active_action_interval is not None
                    ),
                    "scheduler_state": scheduler_state,
                    "scheduler_state_history": scheduler_state_history,
                    "valid_observation_count": valid_observations,
                    "invalid_observation_count": invalid_observations,
                    "generation_mismatch_observation_count": (
                        self._generation_mismatch_observations
                    ),
                    "observation_valid_rate": (
                        float(valid_observations) / observation_total
                        if observation_total
                        else 0.0
                    ),
                    "finite_reward_count": finite_rewards,
                    "reward_finite_rate": (
                        float(finite_rewards) / len(completed_records)
                        if result_records
                        else 0.0
                    ),
                    "action_interval_ros_sec": action_intervals,
                    "action_stamps_strictly_monotonic": (
                        action_stamps_strictly_monotonic
                    ),
                    "median_action_interval_ros_sec": percentile(
                        sorted_intervals, 0.5
                    ),
                    "p95_action_interval_ros_sec": percentile(
                        sorted_intervals, 0.95
                    ),
                    "max_action_interval_ros_sec": (
                        max(sorted_intervals) if sorted_intervals else None
                    ),
                    "action_rate_hz": (
                        float(len(action_stamps) - 1)
                        / (action_stamps[-1] - action_stamps[0])
                        if len(action_stamps) >= 2
                        and action_stamps[-1] > action_stamps[0]
                        else 0.0
                    ),
                    "effective_policy_rate_hz": (
                        float(len(action_stamps) - 1)
                        / (action_stamps[-1] - action_stamps[0])
                        if len(action_stamps) >= 2
                        and action_stamps[-1] > action_stamps[0]
                        else 0.0
                    ),
                    "accepted_policy_tick_indices": [
                        int(record["policy_tick_index"])
                        for record in result_records
                    ],
                    "policy_tick_indices_strictly_monotonic": all(
                        int(later["policy_tick_index"])
                        > int(earlier["policy_tick_index"])
                        for earlier, later in zip(
                            result_records, result_records[1:]
                        )
                    ),
                },
            }

    @classmethod
    def from_ros_params(cls):
        """Bind the paper-aligned step to UAV1 ROS topics."""

        import rospy
        from astra_custom_msgs.msg import PlannerStatus
        from learning_speed_rl.msg import (
            ObservationC,
            SpeedActionStamped,
            SpeedAppliedStamped,
            SpeedRequestStamped,
        )
        from std_msgs.msg import Bool, Float64, String
        from traj_utils.msg import Bspline

        namespace = str(rospy.get_param("~namespace", "uav1")).strip("/")
        if namespace != "uav1":
            raise rospy.ROSInitException("AstraDroneEnv supports only uav1")
        try:
            reward = reward_from_config({"reward": rospy.get_param("~reward")})
            config = AstraDroneStepConfig(
                action_timeout_sec=float(
                    rospy.get_param("~step/action_timeout_sec", 2.0)
                ),
                applied_timeout_sec=float(
                    rospy.get_param("~step/applied_timeout_sec", 3.0)
                ),
                next_observation_timeout_sec=float(
                    rospy.get_param("~step/next_observation_timeout_sec", 2.0)
                ),
                requested_match_tolerance_mps=float(
                    rospy.get_param("~step/requested_match_tolerance_mps", 1.0e-6)
                ),
                applied_pair_tolerance_mps=float(
                    rospy.get_param("~step/applied_pair_tolerance_mps", 0.005)
                ),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise rospy.ROSInitException(
                "invalid AstraDroneEnv parameters: {}".format(error)
            )

        topic = lambda suffix: "/{}/{}".format(namespace, suffix)
        publisher = rospy.Publisher(
            topic("learning_speed/mock_v_max"),
            SpeedRequestStamped,
            queue_size=100,
        )

        def publish_request(token):
            message = SpeedRequestStamped()
            message.header.stamp = (
                rospy.Time.from_sec(token.publish_ros_time_sec)
                if token.publish_ros_secs is None
                else rospy.Time(
                    token.publish_ros_secs, token.publish_ros_nsecs
                )
            )
            message.version = "learning_speed_request_v1.0"
            message.episode_id = token.episode_id
            message.step_index = token.step_index
            message.request_id = token.request_id
            message.requested_v_max = token.requested_v_max
            publisher.publish(message)

        env = cls(
            publish_requested_v_max=publish_request,
            ros_clock=lambda: rospy.Time.now().to_sec(),
            reward=reward,
            config=config,
            run_id=str(
                rospy.get_param("~run_id", "astra_drone_env_paper_aligned")
            ),
            episode_id=str(rospy.get_param("~episode_id", "single_runtime")),
            tracking_limit_m=float(
                rospy.get_param("~tracking_safety/limit_m", 1.0)
            ),
            tracking_duration_sec=float(
                rospy.get_param("~tracking_safety/duration_sec", 1.0)
            ),
            ros_stamp_clock=rospy.Time.now,
        )
        env._ros_handles.extend(
            [
                publisher,
                rospy.Subscriber(
                    topic("learning_speed/observation_c"),
                    ObservationC,
                    env.ingest_observation_c,
                    queue_size=50,
                ),
                rospy.Subscriber(
                    topic("planning/bspline"),
                    Bspline,
                    env.ingest_official_bspline,
                    queue_size=20,
                ),
                rospy.Subscriber(
                    topic("learning_speed/action_stamped"),
                    SpeedActionStamped,
                    env.ingest_action_stamped,
                    queue_size=50,
                ),
                rospy.Subscriber(
                    topic("learning_speed/applied_v_max_stamped"),
                    SpeedAppliedStamped,
                    env.ingest_applied_stamped,
                    queue_size=50,
                ),
                rospy.Subscriber(
                    topic("tower_mission/state"),
                    String,
                    lambda message: env.record_mission_state(message.data),
                    queue_size=20,
                ),
                rospy.Subscriber(
                    topic("tower_mission/progress"),
                    Float64,
                    lambda message: env.record_mission_progress(message.data),
                    queue_size=50,
                ),
                rospy.Subscriber(
                    topic("tower_mission/mission_done"),
                    Bool,
                    lambda message: env.record_mission_done(message.data),
                    queue_size=5,
                ),
                rospy.Subscriber(
                    topic("tower_mission/mission_success"),
                    Bool,
                    lambda message: env.record_mission_success(message.data),
                    queue_size=5,
                ),
                rospy.Subscriber(
                    topic("tower_mission/mission_failure"),
                    Bool,
                    lambda message: env.record_mission_failure(message.data),
                    queue_size=5,
                ),
                rospy.Subscriber(
                    topic("ego_mavros_bridge/state"),
                    String,
                    lambda message: env.record_bridge_state(message.data),
                    queue_size=20,
                ),
                rospy.Subscriber(
                    topic("ego_mavros_bridge/tracking_error"),
                    Float64,
                    lambda message: env.record_tracking_error(message.data),
                    queue_size=50,
                ),
                rospy.Subscriber(
                    topic("planner/status"),
                    PlannerStatus,
                    lambda message: env.record_planner_safety(
                        current_position_in_collision=(
                            message.current_position_in_collision
                        ),
                        emergency_stop_active=message.emergency_stop_active,
                        planner_state=message.planner_state,
                    ),
                    queue_size=50,
                ),
            ]
        )
        rospy.on_shutdown(env.request_shutdown)
        return env
