"""UAV1 Learning Speed causal environment and Episode v0.1 scheduler.

The scheduler publishes policy requests on a 0.1 s ROS/simulation-time grid
without waiting for the previous transition to close.  Pending transitions
continue to use the reviewed Observation C, SpeedActionStamped, applied-v-max,
trajectory-provenance and Stage 1 reward contracts.  This module does not reset
Gazebo/PX4/FAST-LIO/EGO state, train a policy, or bypass the existing
mock-policy -> SpeedSafetyFilter -> EGO applied-v-max chain.
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
    latest_official_trajectory_before,
    official_trajectory_identity_from_bspline,
    policy_state_from_observation_c,
)
from .environment_interface import SpeedTrainingEnvironment
from .reward import (
    Stage1Reward,
    actual_speed_mps_from_body_velocity,
    reward_from_config,
    stage1_reward_input_from_signals,
)


SUPPORTED_STEP_DURATIONS_SEC = (0.1,)
EPISODE_VERSION = "astra_drone_episode_v0.1"


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


class AstraDroneEpisodeError(RuntimeError):
    """Raised when Episode v0.1 cannot preserve its configured contract."""


@dataclass(frozen=True)
class AstraDroneStepConfig:
    duration_sec: float = 0.1
    action_timeout_sec: float = 2.0
    applied_timeout_sec: float = 3.0
    hold_timeout_sec: float = 10.0
    next_observation_timeout_sec: float = 2.0
    requested_match_tolerance_mps: float = 1.0e-6
    applied_pair_tolerance_mps: float = 0.005

    def __post_init__(self):
        values = (
            self.duration_sec,
            self.action_timeout_sec,
            self.applied_timeout_sec,
            self.hold_timeout_sec,
            self.next_observation_timeout_sec,
            self.requested_match_tolerance_mps,
            self.applied_pair_tolerance_mps,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("step configuration must be finite")
        if not any(
            abs(float(self.duration_sec) - supported) <= 1.0e-12
            for supported in SUPPORTED_STEP_DURATIONS_SEC
        ):
            raise ValueError("duration_sec must be 0.1")
        if min(
            self.action_timeout_sec,
            self.applied_timeout_sec,
            self.hold_timeout_sec,
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
    maximum_in_flight: int = 16
    minimum_active_speed_mps: float = 0.2

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
        if self.maximum_in_flight <= 0:
            raise ValueError("maximum_in_flight must be positive")
        if self.max_steps is None and self.max_duration_sec is None:
            raise ValueError("Episode v0.1 requires a max step or duration limit")
        if self.max_steps is not None and int(self.max_steps) <= 0:
            raise ValueError("max_steps must be positive when configured")
        if self.max_duration_sec is not None:
            duration = float(self.max_duration_sec)
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError("max_duration_sec must be positive and finite")


@dataclass
class PendingCausalStep:
    episode_id: str
    step_index: int
    scheduled_ros_time_sec: float
    request: "ActionRequestToken"
    state_t: "StepObservation"
    created_wall_time_sec: float
    action_event: Optional["ActionStampedEvent"] = None
    applied_event: Optional["AppliedVMaxEvent"] = None
    latest_trajectory: Optional[OfficialTrajectoryIdentity] = None
    transition_record: Optional[Dict[str, object]] = None
    reward: Optional[float] = None
    terminated: bool = False
    terminal_reason: str = ""


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


@dataclass(frozen=True)
class AppliedVMaxEvent:
    sequence: int
    receive_sec: float
    stamp_sec: float
    episode_id: str
    step_index: int
    request_id: int
    applied_v_max: float


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
        reward: Stage1Reward,
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
        self._latest_official_trajectory = None
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

    def begin_external_episode(self, run_id, episode_id, reset_generation):
        """Rebind one coordinator-owned Episode without implementing reset here."""

        generation = int(reset_generation)
        if not str(run_id) or not str(episode_id) or generation < 0:
            raise ValueError("external Episode identity is invalid")
        with self._episode_lock:
            with self._condition:
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
                self._latest_official_trajectory = None
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
            current = self._latest_official_trajectory
            if current is not None and (
                identity.start_time_sec,
                identity.trajectory_id,
            ) < (current.start_time_sec, current.trajectory_id):
                return None
            self._latest_official_trajectory = identity
            self._next_sequence_locked()
            self._trajectory_history.append((receipt, identity))
            self._condition.notify_all()
            return identity

    def record_observation(
        self,
        state: PolicyStateV1,
        reward_context: ProgressContextState,
        reward_observation: Mapping[str, object],
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
        return self.record_observation(state, reward_context, clutter)

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

    def _latest_strictly_causal_observation_locked(self, decision_ros_time_sec):
        decision = float(decision_ros_time_sec)
        for packet in reversed(self._observations):
            if packet.state.provenance.observation_receive_sec <= decision:
                return packet
        return None

    def _begin_pending_step(
        self, requested_v_max, scheduled_sec, step_index, state_t
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
        self._publish_requested_v_max(token)
        return PendingCausalStep(
            episode_id=self._run_episode.episode_id,
            step_index=int(step_index),
            scheduled_ros_time_sec=float(scheduled_sec),
            request=token,
            state_t=state_t,
            created_wall_time_sec=self._wall_clock(),
        )

    def _build_transition_record(
        self, pending, state_t_plus_1, terminal
    ) -> Dict[str, object]:
        if (
            pending.action_event is None
            or pending.applied_event is None
            or pending.latest_trajectory is None
        ):
            raise AstraDroneStepCausalityError(
                "pending step is incomplete at transition construction"
            )
        action_event = pending.action_event
        applied_event = pending.applied_event
        state_t = pending.state_t
        action = AppliedSpeedAction(
            requested_v_max=action_event.requested_v_max,
            requested_stamp_sec=action_event.stamp_sec,
            filtered_v_max=action_event.filtered_v_max,
            filtered_stamp_sec=action_event.stamp_sec,
            applied_v_max=applied_event.applied_v_max,
            applied_stamp_sec=applied_event.receive_sec,
            latest_official_trajectory=pending.latest_trajectory,
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
            truncated=False,
            terminal_reason=terminal.terminal_reason,
            dangerous_terminal=terminal.dangerous_terminal,
        )
        reward_input = stage1_reward_input_from_signals(
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
        hold_target = applied_event.receive_sec + self.config.duration_sec
        return {
            "episode_id": pending.episode_id,
            "step_index": pending.step_index,
            "request_id": pending.request.request_id,
            "scheduled_ros_time_sec": pending.scheduled_ros_time_sec,
            "transition": transition.to_record(),
            "reward": transition.reward,
            "terminated": transition.terminated,
            "truncated": False,
            "transition_truncated": transition.truncated,
            "terminal_reason": transition.terminal_reason,
            "timing": {
                "step_duration_config_sec": self.config.duration_sec,
                "scheduled_ros_time_sec": pending.scheduled_ros_time_sec,
                "requested_publish_ros_time_sec": pending.request.publish_ros_time_sec,
                "request_id": pending.request.request_id,
                "action_request_id": action_event.request_id,
                "applied_request_id": applied_event.request_id,
                "request_deadline_error_sec": (
                    pending.request.publish_ros_time_sec
                    - pending.scheduled_ros_time_sec
                ),
                "action_stamp_sec": action_event.stamp_sec,
                "action_receive_ros_time_sec": action_event.receive_sec,
                "applied_receive_ros_time_sec": applied_event.receive_sec,
                "applied_stamp_sec": applied_event.stamp_sec,
                "hold_start_ros_time_sec": applied_event.receive_sec,
                "hold_target_ros_time_sec": hold_target,
                "state_t_stamp_sec": state_t.state.provenance.observation_stamp_sec,
                "state_t_receive_sec": state_t.state.provenance.observation_receive_sec,
                "state_t_plus_1_stamp_sec": (
                    state_t_plus_1.state.provenance.observation_stamp_sec
                ),
                "state_t_plus_1_receive_sec": (
                    state_t_plus_1.state.provenance.observation_receive_sec
                ),
            },
        }

    def _advance_pending_locked(self, pending_steps, used_next_observations):
        completed = []
        if self._identity_violation:
            raise AstraDroneStepCausalityError(self._identity_violation)
        for pending in pending_steps:
            if pending.transition_record is not None:
                continue
            key = (pending.request.episode_id, pending.request.request_id)
            if pending.action_event is None:
                event = self._actions.get(key)
                if event is not None:
                    if (
                        event.step_index != pending.request.step_index
                        or event.source_mode != "mock"
                        or abs(
                            event.requested_v_max
                            - pending.request.requested_v_max
                        )
                        > self.config.requested_match_tolerance_mps
                    ):
                        raise AstraDroneStepCausalityError(
                            "action identity matched but action provenance/value did not"
                        )
                    if (
                        event.stamp_sec + 1.0e-9
                        < pending.request.publish_ros_time_sec
                        or event.receive_sec + 1.0e-9 < event.stamp_sec
                        or event.stamp_sec + 1.0e-9
                        < pending.state_t.state.provenance.observation_receive_sec
                    ):
                        raise AstraDroneStepCausalityError(
                            "action identity matched but timestamp order is invalid: "
                            "episode={} step={} request={} request_publish={:.9f} "
                            "action_stamp={:.9f} action_receive={:.9f} "
                            "state_receive={:.9f}".format(
                                pending.request.episode_id,
                                pending.request.step_index,
                                pending.request.request_id,
                                pending.request.publish_ros_time_sec,
                                event.stamp_sec,
                                event.receive_sec,
                                pending.state_t.state.provenance.observation_receive_sec,
                            )
                        )
                    pending.action_event = self._actions.pop(key)
                    pending.latest_trajectory = latest_official_trajectory_before(
                        self._trajectory_history, event.stamp_sec
                    )
                    if pending.latest_trajectory is None:
                        raise AstraDroneStepCausalityError(
                            "action has no official trajectory provenance"
                        )
            if pending.action_event is not None and pending.applied_event is None:
                action = pending.action_event
                event = self._applied.get(key)
                if event is not None:
                    if event.step_index != pending.request.step_index:
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
                    pending.applied_event = self._applied.pop(key)
            if pending.applied_event is None:
                continue
            hold_target = (
                pending.applied_event.receive_sec + self.config.duration_sec
            )
            for packet in self._observations:
                if packet.sequence in used_next_observations:
                    continue
                if is_causal_next_policy_state(
                    pending.state_t.state,
                    packet.state,
                    pending.applied_event.receive_sec,
                    hold_target,
                ):
                    terminal = self._terminal_snapshot(
                        packet.state.provenance.observation_receive_sec
                    )
                    record = self._build_transition_record(
                        pending, packet, terminal
                    )
                    pending.transition_record = record
                    pending.reward = float(record["reward"])
                    pending.terminated = bool(record["terminated"])
                    pending.terminal_reason = str(record["terminal_reason"])
                    used_next_observations.add(packet.sequence)
                    completed.append(pending)
                    break
        return completed

    def _pending_timeout(self, pending):
        elapsed = self._wall_clock() - pending.created_wall_time_sec
        if pending.action_event is None and elapsed > self.config.action_timeout_sec:
            return "action_identity_ack", self.config.action_timeout_sec
        if (
            pending.action_event is not None
            and pending.applied_event is None
            and elapsed
            > self.config.action_timeout_sec + self.config.applied_timeout_sec
        ):
            return "applied_identity_ack", self.config.applied_timeout_sec
        total = (
            self.config.action_timeout_sec
            + self.config.applied_timeout_sec
            + self.config.hold_timeout_sec
            + self.config.next_observation_timeout_sec
        )
        if pending.applied_event is not None and elapsed > total:
            return "state_t_plus_1", self.config.next_observation_timeout_sec
        return None

    def run_episode(self, action_provider, config=None, transition_consumer=None):
        """Run the sole Episode v0.1 scheduler over one continuous mission."""

        if not callable(action_provider):
            raise ValueError("action_provider must be callable")
        if transition_consumer is not None and not callable(transition_consumer):
            raise ValueError("transition_consumer must be callable")
        episode_config = config or AstraDroneEpisodeConfig()
        if not isinstance(episode_config, AstraDroneEpisodeConfig):
            raise TypeError("config must be AstraDroneEpisodeConfig")
        if abs(self.config.duration_sec - episode_config.policy_period_sec) > 1.0e-12:
            raise ValueError("policy period must equal the reviewed causal duration")

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
            pending_steps: List[PendingCausalStep] = []
            completed_records: List[Dict[str, object]] = []
            used_next_observations = set()
            scheduled_actions = 0
            deadline_misses = 0
            dropped_deadlines = 0
            timeouts = 0
            causal_mismatches = 0
            cancelled_in_flight = 0
            maximum_observed_in_flight = 0
            episode_return = 0.0
            stop_scheduling = False
            stop_reason = ""
            terminal_result = None
            observed_terminal = None
            completion_deadline = None
            status = "running"
            error = ""
            invalid_start = self._invalid_observations
            valid_start = self._valid_observations
            accepted_actions_start = self._accepted_action_count
            accepted_applied_start = self._accepted_applied_count
            duplicate_actions_start = self._duplicate_action_count
            duplicate_applied_start = self._duplicate_applied_count

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

                    with self._condition:
                        newly_completed = self._advance_pending_locked(
                            pending_steps,
                            used_next_observations,
                        )
                    for pending in sorted(
                        newly_completed, key=lambda value: value.step_index
                    ):
                        completed_records.append(pending.transition_record)
                        episode_return += float(pending.reward)
                        if transition_consumer is not None:
                            transition_consumer(pending.transition_record)
                        if pending.terminated and terminal_result is None:
                            terminal_result = pending
                            stop_scheduling = True
                            stop_reason = pending.terminal_reason

                    if terminal_result is not None:
                        survivors = []
                        for pending in pending_steps:
                            if pending.step_index > terminal_result.step_index:
                                cancelled_in_flight += 1
                                continue
                            survivors.append(pending)
                        pending_steps = survivors
                        completed_records = [
                            record
                            for record in completed_records
                            if record["step_index"] <= terminal_result.step_index
                        ]
                        if all(
                            pending.transition_record is not None
                            for pending in pending_steps
                        ):
                            status = "completed"
                            break

                    terminal_now = self._terminal_snapshot()
                    if terminal_now.terminated:
                        observed_terminal = terminal_now
                    if terminal_now.terminated and not stop_scheduling:
                        stop_scheduling = True
                        stop_reason = terminal_now.terminal_reason
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
                        completion_deadline = (
                            self._wall_clock()
                            + episode_config.completion_timeout_sec
                        )

                    duration_limit = (
                        episode_config.max_duration_sec is not None
                        and now_ros + 1.0e-12
                        >= episode_start + episode_config.max_duration_sec
                    )
                    step_limit = (
                        episode_config.max_steps is not None
                        and scheduled_actions >= episode_config.max_steps
                    )
                    if (duration_limit or step_limit) and not stop_scheduling:
                        stop_scheduling = True
                        stop_reason = (
                            "max_episode_duration"
                            if duration_limit
                            else "max_episode_steps"
                        )
                        completion_deadline = (
                            self._wall_clock()
                            + episode_config.completion_timeout_sec
                        )

                    active_pending = [
                        pending
                        for pending in pending_steps
                        if pending.transition_record is None
                    ]
                    for pending in active_pending:
                        timeout = self._pending_timeout(pending)
                        if timeout is not None:
                            phase, timeout_sec = timeout
                            timeouts += 1
                            raise AstraDroneStepTimeout(phase, timeout_sec)

                    if stop_scheduling and not active_pending:
                        status = "completed"
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
                        lateness = max(0.0, now_ros - next_schedule)
                        if lateness >= episode_config.policy_period_sec:
                            skipped = int(
                                math.floor(
                                    lateness / episode_config.policy_period_sec
                                )
                            )
                            dropped_deadlines += skipped
                            next_schedule += (
                                skipped * episode_config.policy_period_sec
                            )
                            lateness = max(0.0, now_ros - next_schedule)
                        if lateness > episode_config.deadline_tolerance_sec:
                            deadline_misses += 1
                        active_count = len(active_pending)
                        if active_count >= episode_config.maximum_in_flight:
                            dropped_deadlines += 1
                        else:
                            decision_ros_time = self._now_ros()
                            with self._condition:
                                state = (
                                    self._latest_strictly_causal_observation_locked(
                                        decision_ros_time
                                    )
                                )
                                if state is None:
                                    raise AstraDroneStepCausalityError(
                                        "policy tick has no strictly causal valid state"
                                    )
                            requested = action_provider(
                                state.state, scheduled_actions
                            )
                            pending = self._begin_pending_step(
                                requested,
                                next_schedule,
                                scheduled_actions,
                                state,
                            )
                            pending_steps.append(pending)
                            scheduled_actions += 1
                            maximum_observed_in_flight = max(
                                maximum_observed_in_flight,
                                active_count + 1,
                            )
                        next_schedule += episode_config.policy_period_sec

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
                    in ("max_episode_duration", "max_episode_steps")
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
            if completed_records:
                completed_records[-1]["truncated"] = boundary.truncated
                completed_records[-1]["episode_boundary"] = boundary.to_record()
            action_stamps = [
                float(record["timing"]["action_stamp_sec"])
                for record in completed_records
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
                for record in completed_records
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
                "steps": completed_records,
                "metrics": {
                    "scheduled_action_count": scheduled_actions,
                    "accepted_action_count": (
                        self._accepted_action_count - accepted_actions_start
                    ),
                    "accepted_applied_ack_count": (
                        self._accepted_applied_count - accepted_applied_start
                    ),
                    "transition_count": len(completed_records),
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
                        float(len(completed_records)) / scheduled_actions
                        if scheduled_actions
                        else 0.0
                    ),
                    "deadline_miss_count": deadline_misses,
                    "deadline_miss_rate": (
                        float(deadline_misses) / scheduled_actions
                        if scheduled_actions
                        else 0.0
                    ),
                    "dropped_deadline_count": dropped_deadlines,
                    "timeout_count": timeouts,
                    "causal_mismatch_count": causal_mismatches,
                    "cancelled_in_flight_after_terminal": cancelled_in_flight,
                    "maximum_observed_in_flight": maximum_observed_in_flight,
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
                        if completed_records
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
                    "action_rate_hz": (
                        float(len(action_stamps) - 1)
                        / (action_stamps[-1] - action_stamps[0])
                        if len(action_stamps) >= 2
                        and action_stamps[-1] > action_stamps[0]
                        else 0.0
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
                duration_sec=float(rospy.get_param("~step/duration_sec", 0.1)),
                action_timeout_sec=float(
                    rospy.get_param("~step/action_timeout_sec", 2.0)
                ),
                applied_timeout_sec=float(
                    rospy.get_param("~step/applied_timeout_sec", 3.0)
                ),
                hold_timeout_sec=float(
                    rospy.get_param("~step/hold_timeout_sec", 10.0)
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
