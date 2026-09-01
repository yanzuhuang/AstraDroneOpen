"""Runner-local ownership for one candidate or active SAC ACTION INTERVAL."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActorActionCandidate:
    step_index: int
    normalized_action: float


class ActorActionOwnership:
    """Separate Actor inference from environment-accepted action ownership."""

    def __init__(self):
        self._candidate = None
        self._active_interval = None

    @property
    def candidate_count(self):
        return int(self._candidate is not None)

    @property
    def active_interval_count(self):
        return int(self._active_interval is not None)

    def propose(self, step_index, normalized_action):
        step = int(step_index)
        action = float(normalized_action)
        if step < 0:
            raise ValueError("Actor candidate step_index must be non-negative")
        if not math.isfinite(action) or action < -1.0 or action > 1.0:
            raise ValueError("Actor candidate action must be finite in [-1, 1]")
        if self._candidate is not None:
            raise RuntimeError("Actor inference produced a second candidate")
        if self._active_interval is not None:
            raise RuntimeError(
                "Actor inference requested while an ACTION INTERVAL is active"
            )
        self._candidate = ActorActionCandidate(step, action)
        return action

    def resolve_acceptance(self, step_index, accepted):
        step = int(step_index)
        if self._candidate is None:
            raise RuntimeError("environment resolved a missing Actor candidate")
        if self._candidate.step_index != step:
            raise RuntimeError("environment acceptance disagrees with Actor candidate")
        candidate = self._candidate
        self._candidate = None
        if not bool(accepted):
            return None
        if self._active_interval is not None:
            raise RuntimeError("environment accepted a second ACTION INTERVAL")
        self._active_interval = candidate
        return candidate

    def consume_transition(self, step_index):
        step = int(step_index)
        if self._candidate is not None:
            raise RuntimeError("transition closed before candidate acceptance resolved")
        if self._active_interval is None:
            raise RuntimeError("closed transition has no accepted Actor action")
        if self._active_interval.step_index != step:
            raise RuntimeError("closed transition disagrees with Actor action")
        normalized_action = self._active_interval.normalized_action
        self._active_interval = None
        return normalized_action

    def require_closed(self):
        if self._candidate is not None:
            raise RuntimeError("Episode closed with one Actor candidate unresolved")
        if self._active_interval is not None:
            raise RuntimeError(
                "Episode closed with one accepted ACTION INTERVAL still active"
            )
