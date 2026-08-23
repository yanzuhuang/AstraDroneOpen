"""Bounded replay storage for the frozen 3267-D Learning Speed contract."""

from copy import deepcopy
import math
import threading

import numpy as np


OBSERVATION_DIM = 3267
ACTION_DIM = 1


def flatten_policy_input(value):
    """Flatten exactly the five frozen Observation C policy inputs."""

    if hasattr(value, "policy_input"):
        value = value.policy_input()
    lidar = np.asarray(value["lidar_surrogate"], dtype=np.float32).reshape(-1)
    future = np.asarray(
        value["future_positions_body"], dtype=np.float32
    ).reshape(-1)
    velocity = np.asarray(
        value["actual_velocity_body"], dtype=np.float32
    ).reshape(-1)
    tracking = np.asarray(
        value["tracking_error_body"], dtype=np.float32
    ).reshape(-1)
    previous = np.asarray(
        [value["previous_applied_v_max"]], dtype=np.float32
    )
    result = np.concatenate((lidar, future, velocity, tracking, previous))
    if result.shape != (OBSERVATION_DIM,) or not np.all(np.isfinite(result)):
        raise ValueError("policy input must be finite with dimension 3267")
    return result


class ActionMapping:
    """Reviewed normalized SAC action to live SpeedSafetyFilter bounds."""

    def __init__(self, v_max_min, v_max_max):
        self.v_max_min = float(v_max_min)
        self.v_max_max = float(v_max_max)
        if (
            not math.isfinite(self.v_max_min)
            or not math.isfinite(self.v_max_max)
            or self.v_max_min <= 0.0
            or self.v_max_max <= self.v_max_min
        ):
            raise ValueError("v_max action bounds are invalid")

    def to_v_max(self, normalized_action):
        value = float(normalized_action)
        if not math.isfinite(value) or value < -1.0 or value > 1.0:
            raise ValueError("normalized action must be finite in [-1, 1]")
        return self.v_max_min + 0.5 * (value + 1.0) * (
            self.v_max_max - self.v_max_min
        )

    def to_normalized(self, v_max):
        value = float(v_max)
        if (
            not math.isfinite(value)
            or value < self.v_max_min - 1.0e-9
            or value > self.v_max_max + 1.0e-9
        ):
            raise ValueError("v_max is outside the reviewed action range")
        normalized = (
            2.0 * (value - self.v_max_min)
            / (self.v_max_max - self.v_max_min)
            - 1.0
        )
        return min(1.0, max(-1.0, normalized))


class SacReplayBuffer:
    """Thread-safe replay with explicit identity, reset and terminal metadata."""

    def __init__(
        self,
        capacity,
        mapping,
        intervention_tolerance_mps=0.005,
        initial_allocation=None,
    ):
        self.capacity = int(capacity)
        if self.capacity <= 0:
            raise ValueError("replay capacity must be positive")
        if not isinstance(mapping, ActionMapping):
            raise TypeError("mapping must be ActionMapping")
        self.mapping = mapping
        self.intervention_tolerance_mps = float(intervention_tolerance_mps)
        if self.intervention_tolerance_mps < 0.0:
            raise ValueError("intervention tolerance must be non-negative")
        requested_allocation = (
            self.capacity
            if initial_allocation is None
            else int(initial_allocation)
        )
        if requested_allocation <= 0:
            raise ValueError("initial replay allocation must be positive")
        self._allocated_capacity = min(self.capacity, requested_allocation)
        self._lock = threading.RLock()
        self._size = 0
        self._position = 0
        self.observations = np.empty(
            (self._allocated_capacity, OBSERVATION_DIM), dtype=np.float32
        )
        self.next_observations = np.empty_like(self.observations)
        self.policy_actions = np.empty(
            (self._allocated_capacity, 1), dtype=np.float32
        )
        self.critic_actions = np.empty(
            (self._allocated_capacity, 1), dtype=np.float32
        )
        self.requested_v_max = np.empty(self._allocated_capacity, dtype=np.float32)
        self.filtered_v_max = np.empty(self._allocated_capacity, dtype=np.float32)
        self.applied_v_max = np.empty(self._allocated_capacity, dtype=np.float32)
        self.rewards = np.empty(self._allocated_capacity, dtype=np.float32)
        self.terminated = np.zeros(self._allocated_capacity, dtype=np.bool_)
        self.truncated = np.zeros(self._allocated_capacity, dtype=np.bool_)
        self.step_index = np.empty(self._allocated_capacity, dtype=np.int64)
        self.request_id = np.empty(self._allocated_capacity, dtype=np.uint64)
        self.reset_generation = np.empty(self._allocated_capacity, dtype=np.uint64)
        self.safety_intervention = np.zeros(self._allocated_capacity, dtype=np.bool_)
        self.episode_ids = [None] * self._allocated_capacity
        self.reward_components = [None] * self._allocated_capacity
        self.terminal_reasons = [None] * self._allocated_capacity
        self._key_to_index = {}

    @staticmethod
    def _grow_array(value, new_capacity):
        shape = (new_capacity,) + value.shape[1:]
        result = np.empty(shape, dtype=value.dtype)
        result[: value.shape[0]] = value
        return result

    def _ensure_storage_for_add(self):
        if self._size < self._allocated_capacity or self._size >= self.capacity:
            return
        new_capacity = min(
            self.capacity, max(self._allocated_capacity + 1, self._allocated_capacity * 2)
        )
        self.observations = self._grow_array(self.observations, new_capacity)
        self.next_observations = self._grow_array(
            self.next_observations, new_capacity
        )
        self.policy_actions = self._grow_array(self.policy_actions, new_capacity)
        self.critic_actions = self._grow_array(self.critic_actions, new_capacity)
        for name in (
            "requested_v_max", "filtered_v_max", "applied_v_max", "rewards",
            "terminated", "truncated", "step_index", "request_id",
            "reset_generation", "safety_intervention",
        ):
            setattr(self, name, self._grow_array(getattr(self, name), new_capacity))
        extension = [None] * (new_capacity - self._allocated_capacity)
        self.episode_ids.extend(extension)
        self.reward_components.extend(extension)
        self.terminal_reasons.extend(extension)
        self._allocated_capacity = new_capacity

    def __len__(self):
        with self._lock:
            return self._size

    def add(self, transition_record, normalized_policy_action, reset_generation):
        transition = transition_record["transition"]
        episode_id = str(transition_record["episode_id"])
        step_index = int(transition_record["step_index"])
        request_id = int(transition_record["request_id"])
        generation = int(reset_generation)
        if not episode_id or step_index < 0 or request_id != step_index + 1:
            raise ValueError("replay transition identity is invalid")
        if generation < 0:
            raise ValueError("reset_generation is invalid")
        provenance = transition["reward_context"]["provenance"]
        if str(provenance["episode_id"]) != episode_id:
            raise ValueError("transition crosses Episode provenance")

        observation = flatten_policy_input(transition["state_t"])
        next_observation = flatten_policy_input(transition["state_t_plus_1"])
        policy_action = float(normalized_policy_action)
        requested = float(transition["action_t"]["requested_v_max"])
        filtered = float(transition["action_t"]["filtered_v_max"])
        applied = float(transition["action_t"]["applied_v_max"])
        reward = float(transition["reward"])
        values = (policy_action, requested, filtered, applied, reward)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("replay transition contains non-finite values")
        expected_request = self.mapping.to_v_max(policy_action)
        if abs(expected_request - requested) > 1.0e-6:
            raise ValueError("normalized policy action does not map to request")
        critic_action = self.mapping.to_normalized(applied)
        intervention = bool(
            abs(requested - filtered) > self.intervention_tolerance_mps
            or abs(filtered - applied) > self.intervention_tolerance_mps
        )
        key = (episode_id, request_id)

        with self._lock:
            if key in self._key_to_index:
                raise ValueError("duplicate replay request identity")
            self._ensure_storage_for_add()
            index = self._position
            if self._size == self.capacity:
                old_key = (self.episode_ids[index], int(self.request_id[index]))
                self._key_to_index.pop(old_key, None)
            self.observations[index] = observation
            self.next_observations[index] = next_observation
            self.policy_actions[index, 0] = policy_action
            self.critic_actions[index, 0] = critic_action
            self.requested_v_max[index] = requested
            self.filtered_v_max[index] = filtered
            self.applied_v_max[index] = applied
            self.rewards[index] = reward
            self.terminated[index] = bool(transition["terminated"])
            self.truncated[index] = bool(transition_record.get("truncated", False))
            self.step_index[index] = step_index
            self.request_id[index] = request_id
            self.reset_generation[index] = generation
            self.safety_intervention[index] = intervention
            self.episode_ids[index] = episode_id
            self.reward_components[index] = deepcopy(
                transition["reward_components"]
            )
            self.terminal_reasons[index] = str(
                transition_record.get("terminal_reason", "")
            )
            self._key_to_index[key] = index
            self._position = (index + 1) % self.capacity
            self._size = min(self.capacity, self._size + 1)
            return index

    def mark_episode_boundary(self, episode_id, truncated, terminal_reason):
        with self._lock:
            candidates = [
                index
                for index in range(self._size)
                if self.episode_ids[index] == str(episode_id)
            ]
            if not candidates:
                raise ValueError("Episode has no replay transition")
            index = max(candidates, key=lambda item: int(self.step_index[item]))
            self.truncated[index] = bool(truncated)
            self.terminal_reasons[index] = str(terminal_reason)
            if self.terminated[index] and self.truncated[index]:
                raise ValueError("replay transition cannot terminate and truncate")

    def sample(self, batch_size, rng):
        count = int(batch_size)
        with self._lock:
            if count <= 0 or self._size < count:
                raise ValueError("replay does not contain a full batch")
            indices = rng.integers(0, self._size, size=count)
            return {
                "observations": self.observations[indices].copy(),
                "next_observations": self.next_observations[indices].copy(),
                "actions": self.critic_actions[indices].copy(),
                "rewards": self.rewards[indices].copy(),
                "terminated": self.terminated[indices].astype(np.float32),
                "truncated": self.truncated[indices].astype(np.float32),
            }

    def audit(self):
        with self._lock:
            size = self._size
            if size == 0:
                return {"size": 0, "passed": False, "failures": ["empty"]}
            sl = slice(0, size)
            failures = []
            arrays = (
                self.observations[sl], self.next_observations[sl],
                self.policy_actions[sl], self.critic_actions[sl],
                self.requested_v_max[sl], self.filtered_v_max[sl],
                self.applied_v_max[sl], self.rewards[sl],
            )
            if not all(np.all(np.isfinite(value)) for value in arrays):
                failures.append("non_finite")
            if np.any(self.policy_actions[sl] < -1.0) or np.any(
                self.policy_actions[sl] > 1.0
            ):
                failures.append("normalized_action_range")
            if np.any(self.requested_v_max[sl] < self.mapping.v_max_min) or np.any(
                self.requested_v_max[sl] > self.mapping.v_max_max
            ):
                failures.append("v_max_range")
            if np.any(self.safety_intervention[sl]):
                failures.append("safety_intervention")
            if any(
                int(self.request_id[index]) != int(self.step_index[index]) + 1
                for index in range(size)
            ):
                failures.append("request_step_identity")
            episode_sequences = {}
            for index in range(size):
                episode_sequences.setdefault(self.episode_ids[index], []).append(
                    (int(self.step_index[index]), int(self.request_id[index]))
                )
            if any(
                sorted(values) != [(step, step + 1) for step in range(len(values))]
                for values in episode_sequences.values()
            ):
                failures.append("episode_step_sequence")
            return {
                "size": size,
                "logical_capacity": self.capacity,
                "allocated_capacity": self._allocated_capacity,
                "passed": not failures,
                "failures": failures,
                "observation_dimension": int(self.observations.shape[1]),
                "finite_reward_count": int(np.isfinite(self.rewards[sl]).sum()),
                "safety_intervention_count": int(
                    self.safety_intervention[sl].sum()
                ),
                "terminated_count": int(self.terminated[sl].sum()),
                "truncated_count": int(self.truncated[sl].sum()),
                "episode_count": len(episode_sequences),
                "normalized_action": {
                    "mean": float(self.policy_actions[sl].mean()),
                    "std": float(self.policy_actions[sl].std()),
                    "min": float(self.policy_actions[sl].min()),
                    "max": float(self.policy_actions[sl].max()),
                },
                "requested_v_max": {
                    "min": float(self.requested_v_max[sl].min()),
                    "max": float(self.requested_v_max[sl].max()),
                },
                "raw_observation": {
                    "min": float(self.observations[sl].min()),
                    "max": float(self.observations[sl].max()),
                    "mean": float(self.observations[sl].mean()),
                    "std": float(self.observations[sl].std()),
                },
            }

    def snapshot(self):
        with self._lock:
            size = self._size
            sl = slice(0, size)
            return {
                "observations": self.observations[sl].copy(),
                "next_observations": self.next_observations[sl].copy(),
                "policy_actions": self.policy_actions[sl].copy(),
                "critic_actions": self.critic_actions[sl].copy(),
                "requested_v_max": self.requested_v_max[sl].copy(),
                "filtered_v_max": self.filtered_v_max[sl].copy(),
                "applied_v_max": self.applied_v_max[sl].copy(),
                "rewards": self.rewards[sl].copy(),
                "terminated": self.terminated[sl].copy(),
                "truncated": self.truncated[sl].copy(),
                "step_index": self.step_index[sl].copy(),
                "request_id": self.request_id[sl].copy(),
                "reset_generation": self.reset_generation[sl].copy(),
                "safety_intervention": self.safety_intervention[sl].copy(),
                "episode_ids": np.asarray(self.episode_ids[:size], dtype=str),
            }
